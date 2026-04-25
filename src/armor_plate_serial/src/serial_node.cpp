#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_serial/packet.hpp"
#include "armor_plate_serial/crc.hpp"

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <atomic>
#include <thread>
#include <vector>
#include <cmath>
#include <mutex>

using armor_plate_interfaces::msg::AimCommand;


class SerialDriver : public rclcpp::Node
{
private:
  // 核心数据（受 mutex 保护）
  float latest_yaw_{0.0f};
  float latest_pitch_{0.0f};
  bool has_target_{false};
  std::mutex data_mutex_;
  rclcpp::Subscription<AimCommand>::SharedPtr aim_command_sub_;
  // 串口
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  // 发送线程
  std::thread send_thread_;
  std::atomic<bool> running_{true};

  void sendLoop()
  {
    rclcpp::Rate rate(100);  // 100 Hz
    while (running_.load() && rclcpp::ok()) {
      float yaw = 0.0f;
      float pitch = 0.0f;
      bool do_send = false;

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_target_ && latest_yaw_ != 0.0f && latest_pitch_ != 0.0f) {
          has_target_ = false;
          yaw = latest_yaw_;
          pitch = latest_pitch_;
          do_send = true;
        }
      }
      
      if (do_send) {
        // 数据打包（此时已有局部副本，不受回调影响）-- 实现数据读取与处理分离
        VisionToEcFrame_t frame;
        frame.sof1 = 0xA5;
        frame.sof2 = 0x5A;
        frame.delta_yaw_1e4rad = static_cast<int16_t>(yaw * 10000.0f);
        frame.delta_pitch_1e4rad = static_cast<int16_t>(pitch * 10000.0f);
        frame.crc16 = crc16_modbus(reinterpret_cast<uint8_t *>(&frame), 6);
        // 数据发送
        std::vector<uint8_t> data(
          reinterpret_cast<uint8_t *>(&frame),
          reinterpret_cast<uint8_t *>(&frame) + sizeof(frame));
        try {
          size_t sent = serial_driver_->port()->send(data);
          if (sent != data.size()) {
            RCLCPP_ERROR(
              this->get_logger(), "发送不完整: 期望 %zu, 实际 %zu", data.size(), sent);
          } else {
            RCLCPP_INFO(this->get_logger(), "发送数据: yaw=%f, pitch=%f", yaw, pitch);
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "发送错误: %s", e.what());
        }
      }
      rate.sleep();
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
      [this](const AimCommand::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_yaw_ = msg->delta_yaw;
        latest_pitch_ = msg->delta_pitch;
        has_target_ = true;
      }
    );
    // ===== 启动发送线程 =====
    send_thread_ = std::thread(&SerialDriver::sendLoop, this);
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
    if (send_thread_.joinable()) {
      send_thread_.join();
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
