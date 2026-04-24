#ifndef MYAGV_PLUS_ESP32_INTERFACE_H
#define MYAGV_PLUS_ESP32_INTERFACE_H

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <vector>

#define SEND_DATA_SIZE 14                               // Total bytes in a command frame to ESP32(version>=V1.0.8)
#define RECEIVE_FRAME_SIZE 31                           // Total bytes in a frame from ESP32(version>=V1.0.8)
#define RECEIVE_PAYLOAD_SIZE (RECEIVE_FRAME_SIZE - 3)   // Payload length (excluding header)

namespace myagvplus_esp32_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyAGVPlusEsp32Interface : public hardware_interface::SystemInterface
{
public:
  // LifecycleNodeInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // SystemInterface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  bool initSerial(const std::string & port, int baud);
  void closeSerial();

  uint16_t crc16_ibm(const uint8_t * data, size_t length);
  std::vector<uint8_t> buildFrame(uint8_t cmd, const std::vector<uint8_t> & payload);

  bool readFrame();
  void parseImu(const std::vector<uint8_t> & frame);

  void sendLedCommand();

private:
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;

  std::string port_;
  int baudrate_{0};

  double ori_[4] = {0};   // quaternion
  double ang_[3] = {0};   // gyro
  double lin_[3] = {0};   // accel

  double cmd_r_{0};
  double cmd_g_{0};
  double cmd_b_{0};

  std::mutex mtx_;
};

}  // namespace myagvplus_esp32_interfaces

#endif  // MYAGV_PLUS_ESP32_INTERFACE_H