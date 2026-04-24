#include "myagv_plus_hardware_interfaces/myagvplus_esp32_interface.hpp"

namespace myagvplus_esp32_interface
{

CallbackReturn MyAGVPlusEsp32Interface::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  port_ = info_.hardware_parameters.at("port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  RCLCPP_INFO(
    rclcpp::get_logger("MyAGVPlusEsp32_Interface"),
    "Using port=%s baudrate=%d", port_.c_str(), baudrate_);

  if (!initSerial(port_, baudrate_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("MyAGVPlusEsp32Interface"), "Serial init failed");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusEsp32Interface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusEsp32Interface"), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusEsp32Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
  closeSerial();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MyAGVPlusEsp32Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("imu", "orientation.x", &ori_[0]);
  state_interfaces.emplace_back("imu", "orientation.y", &ori_[1]);
  state_interfaces.emplace_back("imu", "orientation.z", &ori_[2]);
  state_interfaces.emplace_back("imu", "orientation.w", &ori_[3]);

  state_interfaces.emplace_back("imu", "angular_velocity.x", &ang_[0]);
  state_interfaces.emplace_back("imu", "angular_velocity.y", &ang_[1]);
  state_interfaces.emplace_back("imu", "angular_velocity.z", &ang_[2]);

  state_interfaces.emplace_back("imu", "linear_acceleration.x", &lin_[0]);
  state_interfaces.emplace_back("imu", "linear_acceleration.y", &lin_[1]);
  state_interfaces.emplace_back("imu", "linear_acceleration.z", &lin_[2]);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyAGVPlusEsp32Interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back("rgb_led", "r", &cmd_r_);
  command_interfaces.emplace_back("rgb_led", "g", &cmd_g_);
  command_interfaces.emplace_back("rgb_led", "b", &cmd_b_);

  return command_interfaces;
}

hardware_interface::return_type
MyAGVPlusEsp32Interface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  readFrame();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MyAGVPlusEsp32Interface::write(const rclcpp::Time &, const rclcpp::Duration &)
{

  sendLedCommand();

  return hardware_interface::return_type::OK;
}

bool MyAGVPlusEsp32Interface::initSerial(const std::string & port, int baud)
{
  try
  {
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_);
    serial_port_->open(port);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Failed to initialize serial port: %s"),  ex.what());
    return false;
  }

  return true;
}

void MyAGVPlusEsp32Interface::closeSerial()
{
  if (serial_port_ && serial_port_->is_open())
  {
    serial_port_->close();
  }
}

uint16_t MyAGVPlusEsp32Interface::crc16_ibm(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc = crc >> 1;
    }
  }
  return crc;
}

std::vector<uint8_t>
MyAGVPlusEsp32Interface::buildFrame(uint8_t cmd_id, const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> frame(SEND_DATA_SIZE, 0x00);
  frame[0] = 0xFE;
  frame[1] = 0xFE;
  frame[2] = 0x0B;
  frame[3] = cmd_id;

  for (size_t i = 0; i < payload.size() && i < 8; ++i) {
    frame[4 + i] = payload[i];
  }

  uint16_t crc = this->crc16_ibm(frame.data(), 12);
  frame[12] = (crc >> 8) & 0xff;
  frame[13] = crc & 0xff;

  return frame;
}

bool MyAGVPlusEsp32Interface::readFrame()
{
  std::vector<uint8_t> buf_length(1);
  std::vector<uint8_t> data_buf(RECEIVE_PAYLOAD_SIZE);

  uint8_t byte = 0;
  boost::system::error_code ec;
  
  while (true)
  {
    size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("Serial read error: %s"), ec.message().c_str());
      return false;
    }
    if (ret != 1 || byte != 0xfe) {
      continue;
    }

    ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("Serial read error: %s"), ec.message().c_str());
      return false;
    }
    if (ret == 1 && byte == 0xfe) {
      break; 
    }
  }

  size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(buf_length), ec);
  if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("Serial read error: %s"), ec.message().c_str());
      return false;
  }

  if (buf_length[0] != RECEIVE_PAYLOAD_SIZE) {
    //RCLCPP_ERROR(this->get_logger(), "The received length is incorrect:%u", buf_length[0]);
    return false;
  }

  ret = boost::asio::read(*serial_port_, boost::asio::buffer(data_buf), ec);
  if (ec || ret != data_buf.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger("Failed to receive full payload"), ec.message().c_str());
    return false;
  }

  std::vector<uint8_t> recv_buf;
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0x1C);
  recv_buf.insert(recv_buf.end(), data_buf.begin(), data_buf.end());
  
  // print_hex("recv_buf", recv_buf); //debug

  if (recv_buf[3] != 0x25) {
    //RCLCPP_WARN(this->get_logger("Command error:0x%02X"), , recv_buf[2]); //debug
    return false;
  }

  uint16_t received_crc = recv_buf[RECEIVE_FRAME_SIZE-1] | (recv_buf[RECEIVE_FRAME_SIZE-2] << 8);
  uint16_t computed_crc = crc16_ibm(recv_buf.data(), RECEIVE_FRAME_SIZE-2);

  if (received_crc != computed_crc) {
    RCLCPP_WARN(this->get_logger(), "CRC error: received 0x%04X, calculated 0x%04X", received_crc, computed_crc);
    return false;
  }

  vx = static_cast<double>(static_cast<int8_t>(recv_buf[4])) * 0.01;
  vy = static_cast<double>(static_cast<int8_t>(recv_buf[5])) * 0.01;
  vtheta = static_cast<double>(static_cast<int8_t>(recv_buf[6])) * 0.01;

  motor_status = recv_buf[7];
  motor_error  = recv_buf[8];
  battery_voltage = static_cast<float>(recv_buf[9]) / 10.0f;
  enable_status = recv_buf[10];

  imu_data.linear_acceleration.x = static_cast<double>(static_cast<int16_t>((recv_buf[11] << 8) | recv_buf[12])) * 0.01;
  imu_data.linear_acceleration.y = static_cast<double>(static_cast<int16_t>((recv_buf[13] << 8) | recv_buf[14])) * 0.01;
  imu_data.linear_acceleration.z = static_cast<double>(static_cast<int16_t>((recv_buf[15] << 8) | recv_buf[16])) * 0.01;

  imu_data.angular_velocity.x = static_cast<double>(static_cast<int16_t>((recv_buf[17] << 8) | recv_buf[18])) * 0.01;
  imu_data.angular_velocity.y = static_cast<double>(static_cast<int16_t>((recv_buf[19] << 8) | recv_buf[20])) * 0.01;
  imu_data.angular_velocity.z = static_cast<double>(static_cast<int16_t>((recv_buf[21] << 8) | recv_buf[22])) * 0.01;

  roll  = static_cast<double>(static_cast<int16_t>((recv_buf[23] << 8) | recv_buf[24])) * 0.01;
  pitch = static_cast<double>(static_cast<int16_t>((recv_buf[25] << 8) | recv_buf[26])) * 0.01;
  yaw   = static_cast<double>(static_cast<int16_t>((recv_buf[27] << 8) | recv_buf[28])) * 0.01;

  // RCLCPP_INFO(this->get_logger(),
  // "IMU Data - Accel[x: %.2f, y: %.2f, z: %.2f], "
  // "Gyro[x: %.2f, y: %.2f, z: %.2f], "
  // "RPY[roll: %.2f, pitch: %.2f, yaw: %.2f]",
  // imu_data.linear_acceleration.x,
  // imu_data.linear_acceleration.y,
  // imu_data.linear_acceleration.z,
  // imu_data.angular_velocity.x,
  // imu_data.angular_velocity.y,
  // imu_data.angular_velocity.z,
  // roll, pitch, yaw);

  return true;
}

void MyAGVPlusEsp32Interface::parseImu(const std::vector<uint8_t> & frame)
{
  std::lock_guard<std::mutex> lock(mtx_);

  // TODO: 按你协议解析
}

// ================= WRITE =================
void MyAGVPlusEsp32Interface::sendLedCommand()
{
  // TODO:
  // buildFrame()
  // boost::asio::write()
}

}  // namespace myagvplus_hardware_interfaces
