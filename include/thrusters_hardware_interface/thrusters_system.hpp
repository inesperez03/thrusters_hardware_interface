#pragma once

#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "thrusters_hardware_interface/thruster_mapper.hpp"

namespace thrusters_hardware_interface
{

class ThrustersSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ThrustersSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void publish_zero_command();

  std::string environment_;
  std::string lookup_csv_path_;

  bool is_active_{false};

  double left_force_cmd_{0.0};
  double right_force_cmd_{0.0};

  double left_force_state_{0.0};
  double right_force_state_{0.0};

  double last_left_force_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double last_right_force_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double last_left_output_{std::numeric_limits<double>::quiet_NaN()};
  double last_right_output_{std::numeric_limits<double>::quiet_NaN()};

  ThrusterMapper mapper_;

  rclcpp::Node::SharedPtr internal_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr thruster_stonefish_pub_;

  bool navigator_initialized_{false};
  bool pwm_enabled_{false};
  double pwm_frequency_hz_{50.0};

  int left_pwm_channel_index_{3};
  int right_pwm_channel_index_{4};
};

}  // namespace thrusters_hardware_interface