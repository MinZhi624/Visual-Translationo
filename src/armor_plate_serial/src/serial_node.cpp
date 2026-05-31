#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include "armor_plate_serial/packet.hpp"

#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <thread>
#include <mutex>

using armor_plate_interfaces::msg::AimCommand;
using armor_plate_interfaces::msg::GimbalAngle;


uint16_t crc16_modbus_bit(const uint8_t * data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001u)
            {
                crc = (crc >> 1) ^ 0xA001u;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}


class SerialDriver : public rclcpp::Node
{
private:
    // 发送数据
    float latest_yaw_ = 0.0f;
    float latest_pitch_ = 0.0f;
    uint8_t latest_seq_ = 0;
    std::mutex data_mutex_;
    rclcpp::Subscription<AimCommand>::SharedPtr aim_command_sub_;
    // 串口
    std::unique_ptr<IoContext> owned_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    // 接受相关
    rclcpp::Publisher<GimbalAngle>::SharedPtr gimbal_angle_pub_;
    std::atomic<bool> running_{true};
    std::thread recv_thread_;
    std::vector<uint8_t> recv_payload_;
    // 缓存
    std::vector<uint8_t> recv_temp_buf_;
    // 时间补偿
    double timestamp_offset_ = 0.0;
    
    void publishFrame(const std::array<uint8_t, 13> & frame)
    {
        const auto * packet = reinterpret_cast<const EcToVisionFrame_t *>(frame.data());
        GimbalAngle msg;
        // 补偿串口通信延迟：把时间戳往前推
        msg.stamp = this->now() - rclcpp::Duration::from_seconds(timestamp_offset_);
        msg.yaw_abs   = static_cast<float>(packet->yaw_actual_1e4rad) / 10000.0f;
        msg.pitch_abs = static_cast<float>(packet->pitch_actual_1e4rad) / 10000.0f;
        gimbal_angle_pub_->publish(msg);
    }

    void recvLoop()
    {
        std::vector<uint8_t> header(1);
        std::array<uint8_t, 13> frame;

        while (rclcpp::ok()) {
            try {
                // 等 SOF1
                serial_driver_->port()->receive(header);
                if (header[0] != 0x5A) continue;

                //等 SOF2
                while (true) {
                    serial_driver_->port()->receive(header);
                    if (header[0] == 0xA5) {
                        break;                 // 找到完整帧头
                    } else if (header[0] == 0x5A) {
                        continue;              // 5A 5A A5：继续等 SOF2
                    } else {
                        break;                 // 帧头断裂，回到外层
                    }
                }
                if (header[0] != 0xA5) continue;

                size_t received = 0;
                while (received < 11) {
                    std::vector<uint8_t> tmp(11 - received);
                    size_t n = serial_driver_->port()->receive(tmp);
                    std::copy(tmp.begin(), tmp.begin() + n, frame.begin() + 2 + received);
                    received += n;
                }
                frame[0] = 0x5A;
                frame[1] = 0xA5;

                // CRC 校验
                uint16_t calc_crc = crc16_modbus_bit(frame.data(), 11);
                uint16_t recv_crc = static_cast<uint16_t>(frame[11]) |
                                (static_cast<uint16_t>(frame[12]) << 8);
                if (calc_crc != recv_crc) {
                    RCLCPP_WARN(this->get_logger(), "CRC校验失败");
                    continue;
                }

                publishFrame(frame);

            } catch (const std::exception & ex) {
                RCLCPP_ERROR(this->get_logger(), "接收异常: %s", ex.what());
            }
        }
    }
    // 发送
    bool sendAll(const std::vector<uint8_t>& data)
    {
        size_t total = 0;
        while (total < data.size()) {
            std::vector<uint8_t> remain(data.begin() + total, data.end());
            size_t sent = serial_driver_->port()->send(remain);
            if (sent == 0) {
                return false;
            }
            total += sent;
        }
        return true;
    }
    void sendData(const AimCommand::SharedPtr msg)
    {
        VisionToEcFrame_t frame;
        latest_yaw_ = msg->delta_yaw;
        latest_pitch_ = msg->delta_pitch;
        if(latest_pitch_ == 0.0f && latest_yaw_ == 0.0f) return;
        frame.sof1 = 0xA5;
        frame.sof2 = 0x5A;
        frame.seq = latest_seq_++;
        frame.target_valid = 1;
        frame.delta_yaw_1e4rad = static_cast<int16_t>(latest_yaw_ * 10000.0f);
        frame.delta_pitch_1e4rad = static_cast<int16_t>(latest_pitch_ * 10000.0f);
        frame.crc16 = crc16_modbus_bit(reinterpret_cast<uint8_t *>(&frame), 8);
        std::vector<uint8_t> data(
            reinterpret_cast<uint8_t *>(&frame),
            reinterpret_cast<uint8_t *>(&frame) + sizeof(frame));
        try {
            if(!sendAll(data)) {
                RCLCPP_ERROR(this->get_logger(), "发送失败");
            } else {
                // RCLCPP_INFO(this->get_logger(), "发送数据： vaild = %d, yaw = %f, pitch = %f",target_valid, yaw, pitch);   
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "发送错误: %s", e.what());
        }
    } 
    void init()
    {
        // ===== 串口初始化 =====
        std::string device_name = this->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
        uint32_t baud_rate = static_cast<uint32_t>(this->declare_parameter<int>("baud_rate", 115200));
        timestamp_offset_ = this->declare_parameter<double>("timestamp_offset", 0.0);

        using FC = drivers::serial_driver::FlowControl;
        using PT = drivers::serial_driver::Parity;
        using SB = drivers::serial_driver::StopBits;
        device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
        baud_rate, FC::NONE, PT::NONE, SB::ONE
        );
        owned_ctx_ = std::make_unique<IoContext>(2);
        serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
        serial_driver_->init_port(device_name, *device_config_);
        serial_driver_->port()->open();
        RCLCPP_INFO(this->get_logger(), "Serial打开成功: %s @ %d", device_name.c_str(), baud_rate);
        // ===== 接受信息 =====
        aim_command_sub_ = this->create_subscription<AimCommand>(
            "aim_command", rclcpp::SensorDataQoS(),
            std::bind(&SerialDriver::sendData, this, std::placeholders::_1)
        );
        // ===== 启动接收线程 =====
        gimbal_angle_pub_ = this->create_publisher<GimbalAngle>("gimbal_angle", rclcpp::SensorDataQoS());
        recv_thread_ = std::thread(&SerialDriver::recvLoop, this);
    }

public:
    SerialDriver() : Node("serial_driver_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "SerialDriver节点创建成功！");
        init();
    }

    ~SerialDriver()
    {
        running_.store(false);
        // 依赖关系 send_thread_ -> serial_driver_ -> owned_ctx_
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        if (serial_driver_ && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
        if (owned_ctx_) {
            owned_ctx_->waitForExit();
        }
    }

};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialDriver>());
    rclcpp::shutdown();
    return 0;
}
