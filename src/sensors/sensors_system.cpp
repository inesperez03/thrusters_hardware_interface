#include "sura_hardware_interface/sensors/sensors_system.hpp"

#include <algorithm>

#include <hardware_interface/handle.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace sura_hardware_interface
{

hardware_interface::CallbackReturn SensorsSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Failed to initialize base SystemInterface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;

  has_imu_ = std::any_of(
    info_.sensors.begin(),
    info_.sensors.end(),
    [this](const auto & sensor) {
      return sensor.name == imu_sensor_name_;
    });

  if (!has_imu_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Sensor '%s' not found in ros2_control description",
      imu_sensor_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorsSystem"),
    "SensorsSystem initialized. IMU sensor '%s' detected.",
    imu_sensor_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!has_imu_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Cannot configure SensorsSystem because no IMU sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!imu_.initialize(info_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Failed to initialize IMU interface");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorsSystem"),
    "SensorsSystem configured successfully");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SensorsSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  if (!has_imu_) {
    return interfaces;
  }

  interfaces.emplace_back(imu_sensor_name_, "orientation.x", &orientation_x_);
  interfaces.emplace_back(imu_sensor_name_, "orientation.y", &orientation_y_);
  interfaces.emplace_back(imu_sensor_name_, "orientation.z", &orientation_z_);
  interfaces.emplace_back(imu_sensor_name_, "orientation.w", &orientation_w_);

  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.x", &angular_velocity_x_);
  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.y", &angular_velocity_y_);
  interfaces.emplace_back(imu_sensor_name_, "angular_velocity.z", &angular_velocity_z_);

  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.x", &linear_acceleration_x_);
  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.y", &linear_acceleration_y_);
  interfaces.emplace_back(imu_sensor_name_, "linear_acceleration.z", &linear_acceleration_z_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> SensorsSystem::export_command_interfaces()
{
  return {};
}

hardware_interface::CallbackReturn SensorsSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!has_imu_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Cannot activate SensorsSystem because no IMU sensor was detected");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!imu_.activate()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Failed to activate IMU");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorsSystem"),
    "SensorsSystem activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorsSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (has_imu_ && !imu_.deactivate()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SensorsSystem"),
      "Failed to deactivate IMU");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorsSystem"),
    "SensorsSystem deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorsSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!has_imu_) {
    return hardware_interface::return_type::OK;
  }

  const bool ok = imu_.read(
    orientation_x_, orientation_y_, orientation_z_, orientation_w_,
    angular_velocity_x_, angular_velocity_y_, angular_velocity_z_,
    linear_acceleration_x_, linear_acceleration_y_, linear_acceleration_z_);

    if (!ok) {
    RCLCPP_ERROR(
        rclcpp::get_logger("SensorsSystem"),
        "Failed to read IMU data");
    return hardware_interface::return_type::ERROR;
    }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorsSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace sura_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  sura_hardware_interface::SensorsSystem,
  hardware_interface::SystemInterface)