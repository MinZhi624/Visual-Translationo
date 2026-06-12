#include "armor_plate_serial/crc16.hpp"
#include "armor_plate_serial/packet.hpp"

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

enum class FrameStatus
{
  PENDING,
  OK,
  TIMEOUT,
  SEND_ERROR
};

enum class RecvState
{
  WAIT_SOF1,
  WAIT_SOF2,
  READ_PAYLOAD
};

struct FrameRecord
{
  size_t frame_index = 0;
  uint8_t seq = 0;
  std::chrono::steady_clock::time_point send_time;
  FrameStatus status = FrameStatus::PENDING;
  double rtt_ms = -1.0;
  bool warmup = false;
};

class SerialLatencyTestNode : public rclcpp::Node
{
public:
  SerialLatencyTestNode() : Node("serial_latency_test")
  {
    declareParameters();
    initSerial();
    runTest();
    finalize();
    writeCsv();
    printStats();
  }

private:
  std::string device_name_;
  int baud_rate_ = 115200;
  double send_rate_hz_ = 100.0;
  double duration_sec_ = 60.0;
  double reply_timeout_ms_ = 100.0;
  int warmup_count_ = 100;
  std::string output_csv_;

  std::unique_ptr<IoContext> io_context_;
  std::unique_ptr<SerialPortConfig> port_config_;
  std::unique_ptr<SerialDriver> serial_driver_;

  std::vector<FrameRecord> records_;
  std::mutex records_mutex_;

  uint8_t next_seq_ = 0;
  std::chrono::steady_clock::time_point test_start_time_;

  size_t crc_error_count_ = 0;
  size_t unmatched_echo_count_ = 0;
  size_t send_error_count_ = 0;

  RecvState recv_state_ = RecvState::WAIT_SOF1;
  std::array<uint8_t, 13> recv_frame_{};
  size_t recv_payload_index_ = 0;

  void declareParameters()
  {
    device_name_ = this->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);
    send_rate_hz_ = this->declare_parameter<double>("send_rate_hz", 100.0);
    duration_sec_ = this->declare_parameter<double>("duration_sec", 60.0);
    reply_timeout_ms_ = this->declare_parameter<double>("reply_timeout_ms", 100.0);
    warmup_count_ = this->declare_parameter<int>("warmup_count", 100);
    output_csv_ = this->declare_parameter<std::string>(
      "output_csv", "/tmp/armor_plate_serial_latency.csv");

    if (send_rate_hz_ <= 0.0) {
      throw std::invalid_argument("send_rate_hz 必须大于 0");
    }
    if (duration_sec_ <= 0.0) {
      throw std::invalid_argument("duration_sec 必须大于 0");
    }
    if (reply_timeout_ms_ <= 0.0) {
      throw std::invalid_argument("reply_timeout_ms 必须大于 0");
    }
    if (warmup_count_ < 0) {
      throw std::invalid_argument("warmup_count 必须大于等于 0");
    }

    const int total_frames = static_cast<int>(std::ceil(duration_sec_ * send_rate_hz_));
    if (warmup_count_ >= total_frames) {
      throw std::invalid_argument("warmup_count 必须小于总帧数");
    }
  }

  void initSerial()
  {
    port_config_ = std::make_unique<SerialPortConfig>(
      static_cast<uint32_t>(baud_rate_),
      FlowControl::NONE,
      Parity::NONE,
      StopBits::ONE);

    io_context_ = std::make_unique<IoContext>(2);
    serial_driver_ = std::make_unique<SerialDriver>(*io_context_);
    serial_driver_->init_port(device_name_, *port_config_);
    serial_driver_->port()->open();

    RCLCPP_INFO(
      this->get_logger(),
      "串口已打开: %s @ %d", device_name_.c_str(), baud_rate_);

    // 启动异步接收
    serial_driver_->port()->async_receive(
      std::bind(
        &SerialLatencyTestNode::onReceive, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  bool sendAll(const std::vector<uint8_t> & data)
  {
    size_t total = 0;
    while (total < data.size()) {
      std::vector<uint8_t> remain(data.begin() + total, data.end());
      size_t sent = 0;
      try {
        sent = serial_driver_->port()->send(remain);
      } catch (const std::exception &) {
        return false;
      }
      if (sent == 0) {
        return false;
      }
      total += sent;
    }
    return true;
  }

  void onReceive(std::vector<uint8_t> & buf, const size_t & bytes_transferred)
  {
    if (bytes_transferred == 0 || !serial_driver_->port()->is_open()) {
      return;
    }
    for (size_t i = 0; i < bytes_transferred; ++i) {
      processByte(buf[i]);
    }
  }

  void processByte(uint8_t byte)
  {
    switch (recv_state_) {
      case RecvState::WAIT_SOF1:
        if (byte == 0x5A) {
          recv_state_ = RecvState::WAIT_SOF2;
        }
        break;

      case RecvState::WAIT_SOF2:
        if (byte == 0xA5) {
          recv_state_ = RecvState::READ_PAYLOAD;
          recv_payload_index_ = 0;
        } else if (byte != 0x5A) {
          recv_state_ = RecvState::WAIT_SOF1;
        }
        break;

      case RecvState::READ_PAYLOAD:
        recv_frame_[2 + recv_payload_index_] = byte;
        ++recv_payload_index_;
        if (recv_payload_index_ >= 11) {
          recv_frame_[0] = 0x5A;
          recv_frame_[1] = 0xA5;
          handleFrame();
          recv_state_ = RecvState::WAIT_SOF1;
        }
        break;
    }
  }

  void handleFrame()
  {
    if (!crc16::checkCrc16(recv_frame_)) {
      std::lock_guard<std::mutex> lock(records_mutex_);
      ++crc_error_count_;
      return;
    }

    const auto * packet = reinterpret_cast<const EcToVisionFrame_t *>(recv_frame_.data());
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(records_mutex_);
    bool matched = false;
    for (auto it = records_.rbegin(); it != records_.rend(); ++it) {
      if (it->status != FrameStatus::PENDING) {
        continue;
      }
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
        now - it->send_time).count();
      if (it->seq == packet->seq_echo && elapsed_ms <= reply_timeout_ms_) {
        it->status = FrameStatus::OK;
        it->rtt_ms = elapsed_ms;
        matched = true;
        break;
      }
    }
    if (!matched) {
      ++unmatched_echo_count_;
    }
  }

  void runTest()
  {
    test_start_time_ = std::chrono::steady_clock::now();
    const int total_frames = static_cast<int>(std::ceil(duration_sec_ * send_rate_hz_));
    records_.reserve(total_frames);

    rclcpp::WallRate rate(send_rate_hz_);

    for (int i = 0; i < total_frames && rclcpp::ok(); ++i) {
      VisionToEcFrame_t frame{};
      frame.sof1 = 0xA5;
      frame.sof2 = 0x5A;
      frame.seq = next_seq_++;
      frame.target_valid = 0;
      frame.delta_yaw_1e4rad = 0;
      frame.delta_pitch_1e4rad = 0;
      crc16::appendCrc16(frame);

      std::vector<uint8_t> data(sizeof(frame));
      std::memcpy(data.data(), &frame, sizeof(frame));

      // 生成帧并计算 CRC 后，记录发送时间并立即发送
      const auto send_time = std::chrono::steady_clock::now();
      const bool send_ok = sendAll(data);

      FrameRecord record;
      record.frame_index = static_cast<size_t>(i);
      record.seq = frame.seq;
      record.send_time = send_time;
      record.status = send_ok ? FrameStatus::PENDING : FrameStatus::SEND_ERROR;
      record.rtt_ms = -1.0;
      record.warmup = (i < warmup_count_);

      {
        std::lock_guard<std::mutex> lock(records_mutex_);
        records_.push_back(record);
        if (!send_ok) {
          ++send_error_count_;
        }
      }

      rate.sleep();
    }

    // 最后等待一个超时窗口，让末尾帧有机会收到回传
    std::this_thread::sleep_for(
      std::chrono::milliseconds(static_cast<int>(reply_timeout_ms_)));
  }

  void stop()
  {
    if (serial_driver_ && serial_driver_->port()->is_open()) {
      try {
        serial_driver_->port()->close();
      } catch (const std::exception &) {
      }
    }
    if (io_context_) {
      io_context_->waitForExit();
    }
  }

  void finalize()
  {
    stop();

    std::lock_guard<std::mutex> lock(records_mutex_);
    for (auto & record : records_) {
      if (record.status == FrameStatus::PENDING) {
        record.status = FrameStatus::TIMEOUT;
      }
    }
  }

  static const char * statusToString(FrameStatus status)
  {
    switch (status) {
      case FrameStatus::PENDING:
        return "PENDING";
      case FrameStatus::OK:
        return "OK";
      case FrameStatus::TIMEOUT:
        return "TIMEOUT";
      case FrameStatus::SEND_ERROR:
        return "SEND_ERROR";
    }
    return "UNKNOWN";
  }

  void writeCsv()
  {
    std::lock_guard<std::mutex> lock(records_mutex_);
    std::ofstream ofs(output_csv_);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 CSV 文件: %s", output_csv_.c_str());
      return;
    }

    ofs << "frame_index,seq,send_time_ms,rtt_ms,status,warmup\n";
    ofs << std::fixed << std::setprecision(3);

    for (const auto & record : records_) {
      const double send_time_ms = std::chrono::duration<double, std::milli>(
        record.send_time - test_start_time_).count();
      ofs << record.frame_index << ','
          << static_cast<int>(record.seq) << ','
          << send_time_ms << ','
          << (record.status == FrameStatus::OK ? record.rtt_ms : -1.0) << ','
          << statusToString(record.status) << ','
          << (record.warmup ? 1 : 0) << '\n';
    }

    ofs.close();
    RCLCPP_INFO(this->get_logger(), "CSV 已写入: %s", output_csv_.c_str());
  }

  void printStats()
  {
    std::vector<double> rtt_values;
    size_t ok_count = 0;
    size_t timeout_count = 0;
    size_t send_error_count = 0;

    {
      std::lock_guard<std::mutex> lock(records_mutex_);
      for (const auto & record : records_) {
        if (record.warmup) {
          continue;
        }
        switch (record.status) {
          case FrameStatus::OK:
            ++ok_count;
            rtt_values.push_back(record.rtt_ms);
            break;
          case FrameStatus::TIMEOUT:
            ++timeout_count;
            break;
          case FrameStatus::SEND_ERROR:
            ++send_error_count;
            break;
          default:
            break;
        }
      }
    }

    const size_t total_participated = ok_count + timeout_count;
    const double loss_rate = (total_participated > 0)
                               ? (100.0 * static_cast<double>(timeout_count) /
                                  static_cast<double>(total_participated))
                               : 0.0;

    std::sort(rtt_values.begin(), rtt_values.end());

    const double mean = [&]() {
      if (rtt_values.empty()) {
        return 0.0;
      }
      double sum = 0.0;
      for (double v : rtt_values) {
        sum += v;
      }
      return sum / static_cast<double>(rtt_values.size());
    }();

    auto percentile = [&](double p) -> double {
      if (rtt_values.empty()) {
        return 0.0;
      }
      const size_t n = rtt_values.size();
      const size_t idx = static_cast<size_t>(std::ceil(p / 100.0 * n)) - 1;
      return rtt_values[std::min(idx, n - 1)];
    };

    const double median = percentile(50.0);
    const double p95 = percentile(95.0);
    const double p99 = percentile(99.0);
    const double max_rtt = rtt_values.empty() ? 0.0 : rtt_values.back();

    std::cout << "\n========== 串口延迟测试结果 ==========\n";
    std::cout << "总发送帧数: " << records_.size() << "\n";
    std::cout << "预热帧数:   " << warmup_count_ << "\n";
    std::cout << "参与统计帧数: " << total_participated << "\n";
    std::cout << "有效回复数: " << ok_count << "\n";
    std::cout << "超时丢包数: " << timeout_count << "\n";
    std::cout << "发送失败数: " << send_error_count << "\n";
    std::cout << "CRC 错误数: " << crc_error_count_ << "\n";
    std::cout << "无法匹配回复数: " << unmatched_echo_count_ << "\n";
    std::cout << "丢包率:     " << std::fixed << std::setprecision(3) << loss_rate << "%\n";
    std::cout << "平均 RTT:   " << mean << " ms\n";
    std::cout << "中位数 RTT: " << median << " ms\n";
    std::cout << "P95 RTT:    " << p95 << " ms\n";
    std::cout << "P99 RTT:    " << p99 << " ms\n";
    std::cout << "最大 RTT:   " << max_rtt << " ms\n";
    if (rtt_values.empty()) {
      std::cout << "timestamp_offset 粗略初始估计: 无法估算（无有效回复）\n";
    } else {
      const double timestamp_offset_estimate = median / 2.0 / 1000.0;
      std::cout << "timestamp_offset 粗略初始估计: "
                << std::scientific << timestamp_offset_estimate << " s\n";
    }
    std::cout << "CSV 路径:   " << output_csv_ << "\n";
    std::cout << "======================================\n" << std::endl;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialLatencyTestNode>();
  rclcpp::shutdown();
  return 0;
}
