#include "myagv_plus_hardware_interfaces/myagvplus_interface.hpp"

namespace myagvplus_hardware_interfaces
{

MyAGVPlusInterface::MyAGVPlusInterface()
{
}

MyAGVPlusInterface::~MyAGVPlusInterface()
{
}

CallbackReturn MyAGVPlusInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
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

CallbackReturn MyAGVPlusInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Configuring MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Activating MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Deactivating MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Shutting down MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("MyAGVPlusInterface"), "Cleaning up MyAGVPlus hardware interface...");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_error(const rclcpp_lifecycle::State& previous_state)
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

hardware_interface::return_type MyAGVPlusInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyAGVPlusInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

} // namespace myagvplus_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(myagvplus_hardware_interfaces::MyAGVPlusInterface,
                       hardware_interface::SystemInterface)