#ifndef MYAGV_PLUS_ESP32_INTERFACE_H
#define MYAGV_PLUS_ESP32_INTERFACE_H

#include <sensor_msgs/msg/imu.hpp>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <boost/asio.hpp>
#include <memory>
#include <mutex>
#include <vector>

#define SEND_DATA_SIZE 14                               // Total bytes in a command frame to ESP32(version>=V1.0.8)
#define RECEIVE_FRAME_SIZE 31                           // Total bytes in a frame from ESP32(version>=V1.0.8)
#define RECEIVE_PAYLOAD_SIZE (RECEIVE_FRAME_SIZE - 3)   // Payload length (excluding header)

#define GET_POWER_STATE 0x12
#define SET_AUTO_REPORT_STATE 0x23
#define SET_LED_COLOR 0x34
#define SET_LED_MODE 0x3A

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

  std::vector<double> imu_sensor_state_;

  double cmd_r_{0};
  double cmd_g_{0};
  double cmd_b_{0};

  double linearX = 0.0;
  double linearY = 0.0;
  double angularZ = 0.0;

  double ax= 0.0;
  double ay= 0.0;
  double az= 0.0;

  double wx= 0.0;
  double wy= 0.0;
  double wz= 0.0;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  uint8_t imu_status = 0;
  uint8_t battery_status = 0;
  uint8_t battery_rating = 0;
  uint8_t battery_charging_status = 0;
  
  float battery_voltage = 0.0f;
  float battery_backup_voltage = 0.0f;

  sensor_msgs::msg::Imu imu_data;

  std::mutex mtx_;
};

}  // namespace myagvplus_esp32_interfaces

#endif  // MYAGV_PLUS_ESP32_INTERFACE_H