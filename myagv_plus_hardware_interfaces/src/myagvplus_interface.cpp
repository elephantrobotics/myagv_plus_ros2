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
    return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn MyAGVPlusInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyAGVPlusInterface::export_state_interfaces()
{
    return {};
}

std::vector<hardware_interface::CommandInterface> MyAGVPlusInterface::export_command_interfaces()
{
    return {};
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
