#include "armor_plate_interfaces/msg/aim_command.hpp"
#include "armor_plate_serial/packet.hpp"
#include "armor_plate_serial/crc.hpp"

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>

#include <atomic>
#include <thread>
#include <vector>
#include <cmath>

using armor_plate_interfaces::msg::AimCommand;


class SerialDriver : public rclcpp::Node
{
private:
  // 核心数据
  std::atomic<float> latest_yaw_{0.0f};
  std::atomic<float> latest_pitch_{0.0f};
  std::atomic<bool> has_target_{false};
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
      if (has_target_.load()) {
        float yaw = latest_yaw_.load();
        float pitch = latest_pitch_.load();
        VisionToEcFrame_t frame;
        frame.sof1 = 0xA5;
        frame.sof2 = 0x5A;
        frame.delta_yaw_1e4rad = static_cast<int16_t>(yaw * 10000.0f);
        frame.delta_pitch_1e4rad = static_cast<int16_t>(pitch * 10000.0f);
        frame.crc16 = crc16_modbus(reinterpret_cast<uint8_t *>(&frame), 6);

        std::vector<uint8_t> data(
          reinterpret_cast<uint8_t *>(&frame),
          reinterpret_cast<uint8_t *>(&frame) + sizeof(frame));
        try {
          serial_driver_->port()->send(data);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "Send error: %s", e.what());
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
        latest_yaw_.store(msg->delta_yaw);
        latest_pitch_.store(msg->delta_pitch);
        has_target_.store(true);
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
