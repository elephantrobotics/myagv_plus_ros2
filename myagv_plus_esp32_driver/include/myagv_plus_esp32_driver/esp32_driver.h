#ifndef ESP32_NODE_HPP
#define ESP32_NODE_HPP

#include <algorithm> 
#include <iostream>
#include <boost/asio.hpp>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "myagv_plus_msgs/srv/set_led_color.hpp"
#include "myagv_plus_msgs/srv/set_led_mode.hpp"
#include "myagv_plus_msgs/srv/query_device.hpp"

#define SEND_DATA_SIZE 14                               // Total bytes in a command frame to ESP32(version>=V1.0.8)
#define RECEIVE_FRAME_SIZE 31                           // Total bytes in a frame from ESP32(version>=V1.0.8)
#define RECEIVE_PAYLOAD_SIZE (RECEIVE_FRAME_SIZE - 3)   // Payload length (excluding header)

#define GET_MODIFY_VERSION 0x01
#define GET_SYSTEM_VERSION 0x02
#define GET_ROBOT_STATUS 0x05
#define SET_AUTO_REPORT_STATE 0x23
#define GET_AUTO_REPORT 0x24
#define SET_LED_COLOR 0x34
#define SET_LED_MODE 0x3A

class MyAGV_Plus : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   */
  MyAGV_Plus(std::string node_name);

  /**
   * @brief Destructor
   */
  ~MyAGV_Plus();
private:
  /**
   * @brief Main control loop for the AGV.
   */
  void Control();

  /**
   * @brief Compute the CRC-16-IBM checksum for a byte array.
   *
   * This function calculates the CRC using the standard IBM polynomial 0xA001.
   *
   * @param[in] data Pointer to the byte array.
   * @param[in] length Number of bytes to include in the CRC calculation.
   * @return The computed 16-bit CRC value.
   */
  uint16_t crc16_ibm(const uint8_t* data, size_t length);

  /**
   * @brief Build a standard AGV serial frame with header, payload, and CRC.
   *
   * The frame has a fixed size of RECEIVE_DATA_SIZE, starts with 0xFE 0xFE 0x0B,
   * includes a command ID and up to 8 payload bytes, and ends with a 16-bit CRC.
   *
   * @param[in] cmd_id The command ID for the serial frame.
   * @param[in] payload The payload bytes to include (up to 8 bytes).
   * @return A vector containing the complete serial frame ready to transmit.
   */
  std::vector<uint8_t> build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload);

  /**
   * @brief Print a vector of bytes in hexadecimal format to the ROS logger.
   *
   * @param[in] label A label to prepend to the printed data.
   * @param[in] data The byte vector to print.
   * @param[in] override_size Optional size to display instead of the full data length.
   */
  void print_hex(const std::string& label,
    const std::vector<uint8_t>& data,
    std::optional<size_t> override_size = std::nullopt);

  /**
   * @brief Send a serial frame to the AGV and optionally print it in hex.
   *
   * @param[in] frame The byte vector representing the serial frame to send.
   * @param[in] debug If true, prints the transmitted frame using print_hex().
   */
  void send_serial_frame(const std::vector<uint8_t>& frame, bool debug);

  /**
   * @brief Read a serial response from the AGV device, waiting for a specific header.
   *
   * This function reads bytes from the serial port until the expected header
   * sequence is detected or the timeout expires. After detecting the header,
   * it reads the remaining payload bytes along with a 2-byte CRC.
   *
   * @param[in] expected_header The byte sequence to identify the start of a valid frame.
   * @param[in] payload_size The expected number of payload bytes following the header.
   * @param[in] timeout_sec Maximum time (in seconds) to wait for the header.
   * @return A vector containing the complete frame (header + payload + CRC).
   *         Returns an empty vector if a timeout occurs or the full payload is not received.
   */
  std::vector<uint8_t> read_serial_response(
    const std::vector<uint8_t>& expected_header,
    size_t payload_size,
    double timeout_sec);

  /**
   * @brief Enable or disable AGV auto-reporting
   * @param[in] enable 0 = disable, 1 = enable
   */
  void set_auto_report(bool enable);

  /**
   * @brief Clear the serial port input and output buffers
   * @param[in] fd File descriptor of the serial port
   */
  void clearSerialBuffer(int fd);

  /**
   * @brief Disable the DTR (Data Terminal Ready) and RTS (Request To Send) lines of the serial port
   * @param[in] fd File descriptor of the serial port
   */
  void disableDTR_RTS(int fd);

  /**
   * @brief Read sensor and motor data from the AGV via serial port.
   * @return true if data is successfully read and verified; false otherwise.
   */
  bool readData();

  /**
   * @brief Voltage publisher
   */
  void publisherVoltage();

  /**
   * @brief ImuSensor publisher
   */
  void publisherImuSensor();

  std::vector<uint8_t> send_and_wait(
    uint8_t cmd_id,
    const std::vector<uint8_t>& payload,
    size_t resp_payload_size,
    double timeout_sec);

  void handleSetLedColor(
    const std::shared_ptr<myagv_plus_msgs::srv::SetLedColor::Request> request,
    std::shared_ptr<myagv_plus_msgs::srv::SetLedColor::Response> response);

  void handleSetLedMode(
    const std::shared_ptr<myagv_plus_msgs::srv::SetLedMode::Request> request,
    std::shared_ptr<myagv_plus_msgs::srv::SetLedMode::Response> response);

  void handleQueryDevice(
    const std::shared_ptr<myagv_plus_msgs::srv::QueryDevice::Request> request,
    std::shared_ptr<myagv_plus_msgs::srv::QueryDevice::Response> response);

private:
  boost::asio::io_service io_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::mutex serial_mutex_;

  std::string frame_id_of_imu_;
  std::string name_space_;
  std::string device_name_;

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

  rclcpp::Time currentTime, lastTime;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_voltage,pub_voltage_backup;

  rclcpp::Service<myagv_plus_msgs::srv::SetLedColor>::SharedPtr set_led_service;
  rclcpp::Service<myagv_plus_msgs::srv::SetLedMode>::SharedPtr set_led_mode_service;
  rclcpp::Service<myagv_plus_msgs::srv::QueryDevice>::SharedPtr query_service_;

};

#endif // ESP32_NODE_HPP