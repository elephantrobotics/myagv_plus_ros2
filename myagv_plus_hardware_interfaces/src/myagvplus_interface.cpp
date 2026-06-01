#include "myagv_plus_hardware_interfaces/myagvplus_interface.hpp"

namespace myagvplus_hardware_interfaces
{

CallbackReturn MyAGVPlusInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  port_ = info_.hardware_parameters.at("port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  RCLCPP_INFO(
    rclcpp::get_logger("MyAGVPlusInterface"),
    "Using port=%s baudrate=%d", port_.c_str(), baudrate_);

  motors_.clear();
  for (const auto & joint : info_.joints)
  {
    MotorDesc motor;
    motor.joint_name = joint.name;
    motor.can_id = std::stoi(joint.parameters.at("can_id"), nullptr, 0);
    motor.mst_id = std::stoi(joint.parameters.at("mst_id"), nullptr, 0);

    RCLCPP_INFO(
      rclcpp::get_logger("MyAGVPlusInterface"),
      "Joint %s -> CAN:0x%X MST:0x%X",
      motor.joint_name.c_str(), motor.can_id, motor.mst_id);
    
    motors_.emplace_back(std::move(motor));
  }

  size_t num_joints = info_.joints.size();

  position_states_.resize(num_joints, 0.0);
  velocity_commands_.resize(num_joints, 0.0);
  velocity_states_.resize(num_joints, 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("MyAGVPlusInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MyAGVPlusInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("MyAGVPlusInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MyAGVPlusInterface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
          rclcpp::get_logger("MyAGVPlusInterface"),
          "Joint '%s' have '%s' as second state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Configuring MyAGVPlus hardware interface...");
  try {
    serial_ = std::make_shared<SerialPort>(port_, baudrate_);

    motor_ctrl_ = std::make_shared<damiao::Motor_Control>(serial_);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyAGVPlusInterface"), "Failed to create Motor_Control: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Activating MyAGVPlus hardware interface...");
  
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);

  for (auto & m : motors_) {
    m.motor = std::make_unique<damiao::Motor>(
      damiao::DM_Motor_Type::DM2325,  
      m.can_id,
      m.mst_id);

    motor_ctrl_->addMotor(m.motor.get());
    motor_ctrl_->disable(*m.motor);
    motor_ctrl_->switchControlMode(*m.motor, damiao::VEL_MODE);
    motor_ctrl_->enable(*m.motor);
    motor_ctrl_->set_zero_position(*m.motor);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Deactivating MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Shutting down MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Cleaning up MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Error in MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyAGVPlusInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size();i++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyAGVPlusInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size();i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &velocity_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MyAGVPlusInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  constexpr double VEL_DEADBAND = 0.13; // rad/s
  static constexpr int DIR[4] = { -1, +1, -1, +1 };

  for (size_t i = 0; i < motors_.size(); ++i){
    auto & m = motors_[i];
    motor_ctrl_->refresh_motor_status(*m.motor);
    position_states_[i] = DIR[i] * m.motor->Get_Position();
    double vel = m.motor->Get_Velocity();
    // Apply deadband to velocity
    if (std::abs(vel) < VEL_DEADBAND)
    {
      vel = 0.0;
    }
    velocity_states_[i] = DIR[i] * vel;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyAGVPlusInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  static constexpr int DIR[4] = { -1, +1, -1, +1 };
  for (size_t i = 0; i < motors_.size(); ++i)
  {
    motor_ctrl_->control_vel(*motors_[i].motor, DIR[i] * velocity_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

} // namespace myagvplus_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(myagvplus_hardware_interfaces::MyAGVPlusInterface,
                       hardware_interface::SystemInterface)