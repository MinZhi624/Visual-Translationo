#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_interfaces/msg/gimbal_angle.hpp"
#include "armor_plate_serial/packet.hpp"

#include <armor_plate_interfaces/msg/detail/aim_command__struct.hpp>
#include <armor_plate_interfaces/msg/detail/gimbal_angle__struct.hpp>
#include <chrono>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <atomic>
#include <thread>
#include <mutex>
#include <sstream>
#include <iomanip>

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
    std::atomic<bool> running_{true};
    std::thread recv_thread_;
    enum class ParseState { WAIT_SOF1, WAIT_SOF2, READ_PAYLOAD };
    ParseState parse_state_ = ParseState::WAIT_SOF1;
    std::vector<uint8_t> recv_payload_;
    rclcpp::Publisher<GimbalAngle>::SharedPtr gimbal_angle_pub_;
    // 缓存
    std::vector<uint8_t> recv_temp_buf_;
    /////// DEBUG ///////
    uint64_t debug_total_frames_ = 0;
    uint64_t debug_crc_failures_ = 0;
    uint64_t debug_header_syncs_ = 0;
    
    // 接受
    void parseByte(uint8_t byte)
    {
        switch(parse_state_) {
        case ParseState::WAIT_SOF1:
            if (byte == 0x5A) {
                parse_state_ = ParseState::WAIT_SOF2;
            }
            break;
        case ParseState::WAIT_SOF2:
            if (byte == 0xA5) {
                parse_state_ = ParseState::READ_PAYLOAD;
                recv_payload_.clear(); // 准备读取
                /////// DEBUG ///////
                debug_header_syncs_++;
            } else if(byte == 0x5A) {
                parse_state_ = ParseState::WAIT_SOF2;
            } else {
                parse_state_ = ParseState::WAIT_SOF1;
            }
            break;
        case ParseState::READ_PAYLOAD:
            recv_payload_.push_back(byte);
            if (recv_payload_.size() == 11) {
                // 填充数据
                processFrame();
                parse_state_ = ParseState::WAIT_SOF1;
            }
            break;

        }
    }
    void processFrame()
    {
        uint8_t crc_input[11] = {
            0X5A, 0XA5, recv_payload_[0],
            recv_payload_[1], recv_payload_[2], recv_payload_[3], recv_payload_[4],
            recv_payload_[5], recv_payload_[6], recv_payload_[7], recv_payload_[8]
        };
        uint16_t calc_crc = crc16_modbus_bit(crc_input, 11);
        uint16_t recv_crc = static_cast<uint16_t>(recv_payload_[9] | 
                            static_cast<uint16_t>(recv_payload_[10]) << 8);
        /////// DEBUG ///////
        debug_total_frames_++;
        if(calc_crc != recv_crc) {
            debug_crc_failures_++;
            std::ostringstream oss;
            oss << std::hex << std::uppercase << std::setfill('0');
            oss << "RX raw:";
            oss << " 5A A5";
            for (size_t i = 0; i < recv_payload_.size(); ++i) {
                oss << " " << std::setw(2) << static_cast<int>(recv_payload_[i]);
            }
            oss << " | calc=0x" << std::setw(4) << calc_crc
                << " recv=0x" << std::setw(4) << recv_crc;
            double fail_rate = (debug_total_frames_ > 0)
                                   ? (100.0 * static_cast<double>(debug_crc_failures_) / static_cast<double>(debug_total_frames_))
                                   : 0.0;
            RCLCPP_WARN(
                this->get_logger(),
                "%s | [STATS] total=%llu, fail=%llu, sync=%llu, fail_rate=%.2f%%",
                oss.str().c_str(),
                static_cast<unsigned long long>(debug_total_frames_),
                static_cast<unsigned long long>(debug_crc_failures_),
                static_cast<unsigned long long>(debug_header_syncs_),
                fail_rate);
            return;
        }
        // 提取数据
        int32_t yaw_raw, pitch_raw;
        std::memcpy(&yaw_raw, recv_payload_.data() + 1, sizeof(yaw_raw));
        std::memcpy(&pitch_raw, recv_payload_.data() + 5, sizeof(pitch_raw));
        float yaw_abs = static_cast<float>(yaw_raw) / 10000.0f;
        float pitch_abs = static_cast<float>(pitch_raw) / 10000.0f;
        GimbalAngle msg;
        msg.stamp = this->now();
        msg.pitch_abs = pitch_abs;
        msg.yaw_abs = yaw_abs;
        gimbal_angle_pub_->publish(msg);
        
        // RCLCPP_INFO(this->get_logger(), "收到数据: yaw=%4f, pitch=%4f", yaw_abs, pitch_abs);
    }
    void recvLoop()
    {
        recv_temp_buf_.resize(64);
        while(running_.load() && rclcpp::ok()) {
            try {
                size_t n = serial_driver_->port()->receive(recv_temp_buf_);
                for (size_t i = 0; i < n; ++i) {
                    parseByte(recv_temp_buf_[i]);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "接收错误: %s", e.what());
            }
            // 避免过度占用CPU资源
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
            "aim_command", 10,
            std::bind(&SerialDriver::sendData, this, std::placeholders::_1)
        );
        // ===== 启动接收线程 =====
        gimbal_angle_pub_ = this->gimbal_angle_pub_ = this->create_publisher<GimbalAngle>("gimbal_angle", 10);
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
