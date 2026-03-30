#include "thrusters_hardware_interface/thrusters_system.hpp"
#include "bindings.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace thrusters_hardware_interface
{

static uint16_t pulse_us_to_counts(double pulse_us, double freq_hz)
{
  const double period_us = 1e6 / freq_hz;  // 50 Hz -> 20000 us
  const double counts = pulse_us * 4096.0 / period_us;

  const long rounded = std::lround(counts);
  return static_cast<uint16_t>(std::clamp(rounded, 0L, 4095L));
}

void ThrustersSystem::publish_zero_command()
{
  if (environment_ == "sim" && thruster_stonefish_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {0.0, 0.0};
    thruster_stonefish_pub_->publish(msg);
  } else if (environment_ == "real" && navigator_initialized_ && pwm_enabled_) {
    const double neutral_pulse_us = mapper_.forceToPwm(0.0);
    const uint16_t neutral_counts = pulse_us_to_counts(neutral_pulse_us, pwm_frequency_hz_);

    try {
      set_pwm_channel_value(static_cast<uintptr_t>(left_pwm_channel_index_), neutral_counts);
      set_pwm_channel_value(static_cast<uintptr_t>(right_pwm_channel_index_), neutral_counts);
    } catch (const std::exception & e) {
      std::cerr << "Failed to send neutral PWM command: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Failed to send neutral PWM command: unknown error" << std::endl;
    }
  }
}

hardware_interface::CallbackReturn ThrustersSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    environment_ = info_.hardware_parameters.at("environment");
    lookup_csv_path_ = info_.hardware_parameters.at("lookup_csv");
  } catch (const std::out_of_range & e) {
    std::cerr << "Missing hardware parameter: " << e.what() << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    std::cerr << "Expected exactly 2 joints, got " << info_.joints.size() << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != "effort") {
      std::cerr << "Joint " << joint.name
                << " must have exactly one command interface: effort" << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != "effort") {
      std::cerr << "Joint " << joint.name
                << " must have exactly one state interface: effort" << std::endl;
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  is_active_ = false;
  navigator_initialized_ = false;
  pwm_enabled_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!mapper_.loadCsv(lookup_csv_path_)) {
    std::cerr << "Failed to load thruster lookup CSV: " << lookup_csv_path_ << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  is_active_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  set_raspberry_pi_version(Raspberry::Pi4);
  set_navigator_version(NavigatorVersion::Version1);

  navigator_initialized_ = false;
  pwm_enabled_ = false;

  if (environment_ == "sim") {
    internal_node_ = std::make_shared<rclcpp::Node>("thrusters_hardware_interface_pub");

    thruster_stonefish_pub_ =
      internal_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/catamaran/controller/thruster_setpoints_sim", 10);

    std::cout << "Stonefish publisher created successfully" << std::endl;
  } else if (environment_ == "real") {
    try {
      init();
      navigator_initialized_ = true;

      set_pwm_freq_hz(pwm_frequency_hz_);
      set_pwm_enable(true);
      pwm_enabled_ = true;

      const double neutral_pulse_us = mapper_.forceToPwm(0.0);
      const uint16_t neutral_counts = pulse_us_to_counts(neutral_pulse_us, pwm_frequency_hz_);

      set_pwm_channel_value(static_cast<uintptr_t>(left_pwm_channel_index_), neutral_counts);
      set_pwm_channel_value(static_cast<uintptr_t>(right_pwm_channel_index_), neutral_counts);

      std::cout << "Navigator initialized successfully" << std::endl;
      std::cout << "PWM enabled at " << pwm_frequency_hz_ << " Hz" << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "Failed to initialize Navigator: " << e.what() << std::endl;
      navigator_initialized_ = false;
      pwm_enabled_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    } catch (...) {
      std::cerr << "Failed to initialize Navigator: unknown error" << std::endl;
      navigator_initialized_ = false;
      pwm_enabled_ = false;
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    std::cerr << "Unknown environment: " << environment_ << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout << "ThrustersSystem configured successfully" << std::endl;
  std::cout << "Environment: " << environment_ << std::endl;
  std::cout << "Loaded CSV samples: " << mapper_.size() << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  if (environment_ == "real" && navigator_initialized_) {
    try {
      set_pwm_enable(false);
    } catch (const std::exception & e) {
      std::cerr << "Failed to disable PWM during cleanup: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Failed to disable PWM during cleanup: unknown error" << std::endl;
    }
  }

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  std::cout << "ThrustersSystem cleaned up" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  if (environment_ == "real" && navigator_initialized_) {
    try {
      set_pwm_enable(false);
    } catch (const std::exception & e) {
      std::cerr << "Failed to disable PWM during shutdown: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Failed to disable PWM during shutdown: unknown error" << std::endl;
    }
  }

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  std::cout << "ThrustersSystem shutdown completed" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = true;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  last_left_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_right_force_cmd_ = std::numeric_limits<double>::quiet_NaN();
  last_left_output_ = std::numeric_limits<double>::quiet_NaN();
  last_right_output_ = std::numeric_limits<double>::quiet_NaN();

  publish_zero_command();

  std::cout << "ThrustersSystem activated" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  if (environment_ == "real" && navigator_initialized_) {
    try {
      set_pwm_enable(false);
      pwm_enabled_ = false;
    } catch (const std::exception & e) {
      std::cerr << "Failed to disable PWM during deactivation: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Failed to disable PWM during deactivation: unknown error" << std::endl;
    }
  }

  std::cout << "ThrustersSystem deactivated" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ThrustersSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  is_active_ = false;

  left_force_cmd_ = 0.0;
  right_force_cmd_ = 0.0;
  left_force_state_ = 0.0;
  right_force_state_ = 0.0;

  publish_zero_command();

  if (environment_ == "real" && navigator_initialized_) {
    try {
      set_pwm_enable(false);
    } catch (const std::exception & e) {
      std::cerr << "Failed to disable PWM during error handling: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "Failed to disable PWM during error handling: unknown error" << std::endl;
    }
  }

  pwm_enabled_ = false;
  navigator_initialized_ = false;

  internal_node_.reset();
  thruster_stonefish_pub_.reset();

  std::cerr << "ThrustersSystem entered error state" << std::endl;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ThrustersSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[0].name, "effort", &left_force_state_));

  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.joints[1].name, "effort", &right_force_state_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ThrustersSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[0].name, "effort", &left_force_cmd_));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      info_.joints[1].name, "effort", &right_force_cmd_));

  return command_interfaces;
}

hardware_interface::return_type ThrustersSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  left_force_state_ = left_force_cmd_;
  right_force_state_ = right_force_cmd_;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ThrustersSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!is_active_) {
    left_force_state_ = 0.0;
    right_force_state_ = 0.0;
    publish_zero_command();
    return hardware_interface::return_type::OK;
  }

  const double left_stonefish = mapper_.forceToStonefish(left_force_cmd_);
  const double right_stonefish = mapper_.forceToStonefish(right_force_cmd_);	
  const double left_pulse_us = mapper_.forceToPwm(left_force_cmd_);
  const double right_pulse_us = mapper_.forceToPwm(right_force_cmd_);
 
  const uint16_t left_counts = pulse_us_to_counts(left_pulse_us, pwm_frequency_hz_);
  const uint16_t right_counts = pulse_us_to_counts(right_pulse_us, pwm_frequency_hz_);

  if (environment_ == "real") {
    if (!navigator_initialized_ || !pwm_enabled_) {
      std::cerr << "Navigator PWM not initialized" << std::endl;
      return hardware_interface::return_type::ERROR;
    }

    try {
      set_pwm_channel_value(
        static_cast<uintptr_t>(left_pwm_channel_index_),
        left_counts);

      set_pwm_channel_value(
        static_cast<uintptr_t>(right_pwm_channel_index_),
        right_counts);
    } catch (const std::exception & e) {
      std::cerr << "Failed to write PWM to Navigator: " << e.what() << std::endl;
      return hardware_interface::return_type::ERROR;
    } catch (...) {
      std::cerr << "Failed to write PWM to Navigator: unknown error" << std::endl;
      return hardware_interface::return_type::ERROR;
    }

    const bool changed =
      std::isnan(last_left_force_cmd_) ||
      std::isnan(last_right_force_cmd_) ||
      std::fabs(left_force_cmd_ - last_left_force_cmd_) > 1e-6 ||
      std::fabs(right_force_cmd_ - last_right_force_cmd_) > 1e-6 ||
      std::fabs(static_cast<double>(left_counts) - last_left_output_) > 1e-6 ||
      std::fabs(static_cast<double>(right_counts) - last_right_output_) > 1e-6;

    if (changed) {
      std::cout
      << "[REAL] "
      << "left_force=" << left_force_cmd_
      << " N -> left_pulse_us=" << left_pulse_us
      << " -> left_counts=" << left_counts
      << " | right_force=" << right_force_cmd_
      << " N -> right_pulse_us=" << right_pulse_us
      << " -> right_counts=" << right_counts
      << std::endl;

      last_left_force_cmd_ = left_force_cmd_;
      last_right_force_cmd_ = right_force_cmd_;
      last_left_output_ = static_cast<double>(left_counts);
      last_right_output_ = static_cast<double>(right_counts);
    }
  } else if (environment_ == "sim") {
    if (!thruster_stonefish_pub_) {
      std::cerr << "Stonefish publisher not initialized" << std::endl;
      return hardware_interface::return_type::ERROR;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {left_stonefish, right_stonefish};

    thruster_stonefish_pub_->publish(msg);

    const bool changed =
      std::isnan(last_left_force_cmd_) ||
      std::isnan(last_right_force_cmd_) ||
      std::fabs(left_force_cmd_ - last_left_force_cmd_) > 1e-6 ||
      std::fabs(right_force_cmd_ - last_right_force_cmd_) > 1e-6 ||
      std::fabs(left_stonefish - last_left_output_) > 1e-6 ||
      std::fabs(right_stonefish - last_right_output_) > 1e-6;

    if (changed) {
      std::cout
        << "[SIM] "
        << "left_force=" << left_force_cmd_ << " N -> left_stonefish=" << left_stonefish
        << " | right_force=" << right_force_cmd_ << " N -> right_stonefish=" << right_stonefish
        << std::endl;

      last_left_force_cmd_ = left_force_cmd_;
      last_right_force_cmd_ = right_force_cmd_;
      last_left_output_ = left_stonefish;
      last_right_output_ = right_stonefish;
    }
  } else {
    std::cerr << "Unknown environment: " << environment_ << std::endl;
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace thrusters_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  thrusters_hardware_interface::ThrustersSystem,
  hardware_interface::SystemInterface)
