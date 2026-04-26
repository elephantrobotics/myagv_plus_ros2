#include "myagv_plus_esp32_driver/esp32_driver.h"

uint16_t MyAGV_Plus::crc16_ibm(const uint8_t* data, size_t length) {
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

std::vector<uint8_t> MyAGV_Plus::build_serial_frame(uint8_t cmd_id, const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> frame(SEND_DATA_SIZE, 0x00);
  frame[0] = 0xFE;
  frame[1] = 0xFE;
  frame[2] = 0x0B;
  frame[3] = cmd_id;

  for (size_t i = 0; i < payload.size() && i < 8; ++i) {
    frame[4 + i] = payload[i];
  }

  uint16_t crc = crc16_ibm(frame.data(), 12);
  frame[12] = (crc >> 8) & 0xff;
  frame[13] = crc & 0xff;

  return frame;
}

void MyAGV_Plus::print_hex(const std::string& label, const std::vector<uint8_t>& data, std::optional<size_t> override_size) {
  std::stringstream ss;
  for (auto b : data) {
    ss << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
       << static_cast<int>(b) << " ";
  }
  size_t len = override_size.value_or(data.size());
  RCLCPP_INFO(this->get_logger(), "%s (%zu bytes): [%s]", label.c_str(), len, ss.str().c_str());
}

void MyAGV_Plus::send_serial_frame(const std::vector<uint8_t>& frame, bool debug)
{
  try {
    size_t bytes_transmit_size = boost::asio::write(*serial_port_, boost::asio::buffer(frame));
    if (debug) {
      print_hex("Sent", frame, bytes_transmit_size);
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error Transmiting from serial port: %s", ex.what());
  }
}

std::vector<uint8_t> MyAGV_Plus::read_serial_response(
  const std::vector<uint8_t>& expected_header,
  size_t payload_size,
  double timeout_sec)
{
  std::vector<uint8_t> sliding_buf;
  uint8_t byte = 0;

  rclcpp::Time start_time = this->now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(timeout_sec);

  while ((this->now() - start_time) < timeout) {
    boost::asio::mutable_buffers_1 buf(&byte, 1);
    boost::system::error_code ec;
    size_t n = serial_port_->read_some(buf, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
        return {};
    }
    if (n == 1) {
      sliding_buf.push_back(byte);
      if (sliding_buf.size() > expected_header.size()) {
          sliding_buf.erase(sliding_buf.begin());
      }
      if (sliding_buf == expected_header) {
          break;
      }
    }
  }
  
  if (sliding_buf != expected_header) {
    RCLCPP_WARN(this->get_logger(), "Timeout waiting for header");
    return {};
  }

  size_t remain_len = payload_size + 2;
  std::vector<uint8_t> remain_buf(remain_len);
  size_t total_read = 0;

  while (total_read < remain_len && (this->now() - start_time) < timeout) {
    boost::asio::mutable_buffers_1 buf(&remain_buf[total_read], remain_len - total_read);
    boost::system::error_code ec;
    size_t n = serial_port_->read_some(buf, ec);
    if (ec) {
        RCLCPP_WARN(this->get_logger(), "Serial read error: %s", ec.message().c_str());
        return {};
    }
    total_read += n;
  }

  if (total_read != remain_len) {
    RCLCPP_WARN(this->get_logger(), "Timeout or incomplete data payload");
    return {};
  }

  std::vector<uint8_t> full_buf = expected_header;
  full_buf.insert(full_buf.end(), remain_buf.begin(), remain_buf.end());

  return full_buf;
}

void MyAGV_Plus::set_auto_report(bool enable){
  auto frame = build_serial_frame(0x23, {static_cast<uint8_t>(enable)});
  send_serial_frame(frame,true);
}

void MyAGV_Plus::clearSerialBuffer(int fd) {
  if (tcflush(fd, TCIOFLUSH) < 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to flush serial buffer: %s", std::strerror(errno));
  } else {
    RCLCPP_INFO(this->get_logger(), "Serial buffer flushed.");
  }
}

void MyAGV_Plus::disableDTR_RTS(int fd) {
  int status;
  if (::ioctl(fd, TIOCMGET, &status) == 0) {
    status &= ~(TIOCM_DTR | TIOCM_RTS);
    if (::ioctl(fd, TIOCMSET, &status) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to clear DTR and RTS: %s", std::strerror(errno));
    } else {
      RCLCPP_INFO(this->get_logger(), "DTR and RTS lines disabled successfully.");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to read modem status: %s", std::strerror(errno));
  }
}

void MyAGV_Plus::handleSetLedColor(
  const std::shared_ptr<myagv_plus_msgs::srv::SetLedColor::Request> request,
  std::shared_ptr<myagv_plus_msgs::srv::SetLedColor::Response> response)
{
  if (request->position < 0 || request->position > 1) {
    RCLCPP_ERROR(this->get_logger(), "Invalid LED position: %d", request->position);
    response->success = false;
    response->message = "Invalid LED position";
    return;
  }

  if (request->brightness < 0 || request->brightness > 255) {
    RCLCPP_ERROR(this->get_logger(), "Invalid brightness: %d", request->brightness);
    response->success = false;
    response->message = "Invalid brightness";
    return;
  }

  if (request->r < 0 || request->r > 255 ||
      request->g < 0 || request->g > 255 ||
      request->b < 0 || request->b > 255) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid RGB value: r=%d g=%d b=%d",
      request->r, request->g, request->b
    );
    response->success = false;
    response->message = "Invalid RGB value";
    return;
  }

  uint8_t position   = static_cast<uint8_t>(request->position);
  uint8_t brightness = static_cast<uint8_t>(request->brightness);
  uint8_t r          = static_cast<uint8_t>(request->r);
  uint8_t g          = static_cast<uint8_t>(request->g);
  uint8_t b          = static_cast<uint8_t>(request->b);

  auto frame = build_serial_frame(SET_LED_COLOR, {position, brightness, r, g, b});
  send_serial_frame(frame, true);

  const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, SET_LED_COLOR};
  auto response_frame = read_serial_response(expected_header, 8, 5.0);

  // print_hex("recv_buf", response_frame); //debug

  uint8_t status = response_frame[4];
  if (status == 0x01) {
    RCLCPP_DEBUG(this->get_logger(), "SetLedColor succeeded");
    response->success = true;
    response->message = "Success";
  } else {
    RCLCPP_ERROR(this->get_logger(), "SetLedColor failed with status: 0x%02X", status);
    response->success = false;
    response->message = "Failed with status code";
  }
}

void MyAGV_Plus::handleSetLedMode(
  const std::shared_ptr<myagv_plus_msgs::srv::SetLedMode::Request> request,
  std::shared_ptr<myagv_plus_msgs::srv::SetLedMode::Response> response)
{
  uint8_t mode = request->mode ? 0x01 : 0x00;

  auto frame = build_serial_frame(SET_LED_MODE, {mode});
  send_serial_frame(frame, true);

  const std::vector<uint8_t> expected_header = {0xFE, 0xFE, 0x0B, SET_LED_MODE};
  auto response_frame = read_serial_response(expected_header, 8, 5.0);

  // print_hex("recv_buf", response_frame); //debug

  uint8_t status = response_frame[4];
  if (status == 0x01) {
    RCLCPP_DEBUG(this->get_logger(), "SetLedMode succeeded");
    response->success = true;
    response->message = "Success";
  } else {
    RCLCPP_ERROR(this->get_logger(), "SetLedMode failed with status: 0x%02X", status);
    response->success = false;
    response->message = "Failed with status code";
  }
}

bool MyAGV_Plus::readData()
{
  std::vector<uint8_t> buf_length(1);
  std::vector<uint8_t> data_buf(RECEIVE_PAYLOAD_SIZE);

  uint8_t byte = 0;
  boost::system::error_code ec;
  
  while (true)
  {
    size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
    }
    if (ret != 1 || byte != 0xfe) {
      continue;
    }

    ret = boost::asio::read(*serial_port_, boost::asio::buffer(&byte, 1), ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
    }
    if (ret == 1 && byte == 0xfe) {
      break; 
    }
  }

  size_t ret = boost::asio::read(*serial_port_, boost::asio::buffer(buf_length), ec);
  if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
      return false;
  }

  if (buf_length[0] != RECEIVE_PAYLOAD_SIZE) {
    //RCLCPP_ERROR(this->get_logger(), "The received length is incorrect:%u", buf_length[0]);
    return false;
  }

  ret = boost::asio::read(*serial_port_, boost::asio::buffer(data_buf), ec);
  if (ec || ret != data_buf.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive full payload");
    return false;
  }

  std::vector<uint8_t> recv_buf;
  recv_buf.push_back(0xFE);
  recv_buf.push_back(0xFE);
  recv_buf.push_back(data_buf.size());
  recv_buf.insert(recv_buf.end(), data_buf.begin(), data_buf.end());
  
  // print_hex("recv_buf", recv_buf); //debug

  if (recv_buf[3] != 0x25) {
    //RCLCPP_WARN(this->get_logger(), "Command error:0x%02X", recv_buf[2]); //debug
    return false;
  }

  uint16_t received_crc = recv_buf[RECEIVE_FRAME_SIZE-1] | (recv_buf[RECEIVE_FRAME_SIZE-2] << 8);
  uint16_t computed_crc = crc16_ibm(recv_buf.data(), RECEIVE_FRAME_SIZE-2);

  if (received_crc != computed_crc) {
    RCLCPP_WARN(this->get_logger(), "CRC error: received 0x%04X, calculated 0x%04X", received_crc, computed_crc);
    return false;
  }

  battery_status = recv_buf[1];
  imu_status     = recv_buf[2];
  battery_rating = recv_buf[3];
  battery_charging_status = recv_buf[4];
  battery_voltage = static_cast<float>(recv_buf[5]) / 10.0f;
  battery_backup_voltage = static_cast<float>(recv_buf[6]) / 10.0f;

  imu_data.linear_acceleration.x = static_cast<double>(static_cast<int16_t>((recv_buf[7] << 8) | recv_buf[8])) * 0.01;
  imu_data.linear_acceleration.y = static_cast<double>(static_cast<int16_t>((recv_buf[9] << 8) | recv_buf[10])) * 0.01;
  imu_data.linear_acceleration.z = static_cast<double>(static_cast<int16_t>((recv_buf[11] << 8) | recv_buf[12])) * 0.01;

  imu_data.angular_velocity.x = static_cast<double>(static_cast<int16_t>((recv_buf[13] << 8) | recv_buf[14])) * 0.01;
  imu_data.angular_velocity.y = static_cast<double>(static_cast<int16_t>((recv_buf[15] << 8) | recv_buf[16])) * 0.01;
  imu_data.angular_velocity.z = static_cast<double>(static_cast<int16_t>((recv_buf[17] << 8) | recv_buf[18])) * 0.01;

  roll  = static_cast<double>(static_cast<int16_t>((recv_buf[19] << 8) | recv_buf[20])) * 0.01;
  pitch = static_cast<double>(static_cast<int16_t>((recv_buf[21] << 8) | recv_buf[22])) * 0.01;
  yaw   = static_cast<double>(static_cast<int16_t>((recv_buf[23] << 8) | recv_buf[24])) * 0.01;

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

void MyAGV_Plus::publisherVoltage()
{
  std_msgs::msg::Float32 voltage_msg,voltage_backup_msg;
  voltage_msg.data = battery_voltage;
  voltage_backup_msg.data = battery_backup_voltage;
  pub_voltage->publish(voltage_msg);
  pub_voltage_backup->publish(voltage_backup_msg);
}

void MyAGV_Plus::publisherImuSensor() 
{
  sensor_msgs::msg::Imu ImuSensor;

  ImuSensor.header.stamp = this->get_clock()->now();
  ImuSensor.header.frame_id = "imu_link";

  tf2::Quaternion qua;
  qua.setRPY(0, 0, yaw * M_PI / 180.0);

  ImuSensor.orientation.x = qua[0];
  ImuSensor.orientation.y = qua[1];
  ImuSensor.orientation.z = qua[2];
  ImuSensor.orientation.w = qua[3];

  ImuSensor.angular_velocity.x = imu_data.angular_velocity.x;
  ImuSensor.angular_velocity.y = imu_data.angular_velocity.y;
  ImuSensor.angular_velocity.z = imu_data.angular_velocity.z;

  ImuSensor.linear_acceleration.x = imu_data.linear_acceleration.x;
  ImuSensor.linear_acceleration.y = imu_data.linear_acceleration.y;
  ImuSensor.linear_acceleration.z = imu_data.linear_acceleration.z;

  ImuSensor.orientation_covariance[0] = 1e6;
  ImuSensor.orientation_covariance[4] = 1e6;
  ImuSensor.orientation_covariance[8] = 1e-6;

  ImuSensor.angular_velocity_covariance[0] = 1e6;
  ImuSensor.angular_velocity_covariance[4] = 1e6;
  ImuSensor.angular_velocity_covariance[8] = 1e-6;

  pub_imu->publish(ImuSensor);
}

void MyAGV_Plus::Control()
{
  if (true == readData())
  {
    currentTime = this->get_clock()->now();
    double dt = 0.0;
    if (lastTime.nanoseconds() != 0) {
      dt = (currentTime - lastTime).seconds();
    }

    lastTime = currentTime;
    // RCLCPP_INFO(this->get_logger(), "dt:%f", dt);
    publisherVoltage();
    publisherImuSensor();
  }
}

MyAGV_Plus::MyAGV_Plus(std::string node_name):rclcpp::Node(node_name)
{
  this->declare_parameter<std::string>("port_name","/dev/myagv_plus_esp32");
  this->declare_parameter<std::string>("odometry.frame_id", "odom");
  this->declare_parameter<std::string>("odometry.child_frame_id", "base_footprint");
  this->declare_parameter<std::string>("imu.frame_id", "imu_link");
  this->declare_parameter<std::string>("namespace", "");

  this->get_parameter_or<std::string>("port_name",device_name_,std::string("/dev/myagv_plus_esp32"));
  this->get_parameter_or<std::string>("imu.frame_id",frame_id_of_imu_,std::string("imu_link"));        
  this->get_parameter_or<std::string>("namespace",name_space_,std::string(""));

  if (name_space_ != "") {
    frame_id_of_imu_ = name_space_ + "/" + frame_id_of_imu_;
  }

  pub_imu =  this->create_publisher<sensor_msgs::msg::Imu>("imu", 20);
  pub_voltage = create_publisher<std_msgs::msg::Float32>("voltage", 10);
  pub_voltage_backup = create_publisher<std_msgs::msg::Float32>("voltage_backup", 10);

  set_led_service = this->create_service<myagv_plus_msgs::srv::SetLedColor>(
    "set_led_color",
    std::bind(&MyAGV_Plus::handleSetLedColor, this, std::placeholders::_1, std::placeholders::_2)
  );

  set_led_mode_service = this->create_service<myagv_plus_msgs::srv::SetLedMode>(
    "set_led_mode",
    std::bind(&MyAGV_Plus::handleSetLedMode, this, std::placeholders::_1, std::placeholders::_2)
  );

  lastTime = this->get_clock()->now();
      
  try{
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_);

    serial_port_->open(device_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    // int fd = serial_port_->native_handle();
    // this->clearSerialBuffer(fd);
    // this->disableDTR_RTS(fd);
    // rclcpp::sleep_for(std::chrono::milliseconds(3000));//esp32 Restart time

    RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Using device: %s", device_name_.c_str());

    boost::asio::serial_port_base::baud_rate baud_option;
    serial_port_->get_option(baud_option);
    unsigned int current_baud = baud_option.value();
    RCLCPP_INFO(this->get_logger(), "Baud_rate: %u", current_baud);
  }
  catch (const std::exception &ex){
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port: %s", ex.what());
    return;
  }

  this->set_auto_report(1);

  control_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(20),
  std::bind(&MyAGV_Plus::Control, this)
  );
  RCLCPP_INFO(this->get_logger(), "Control timer started");
}

MyAGV_Plus::~MyAGV_Plus()
{
  if (serial_port_ && serial_port_->is_open()) {
    this->set_auto_report(0);
    serial_port_->cancel();
    serial_port_->close();
  } 
}