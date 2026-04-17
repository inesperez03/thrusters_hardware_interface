#pragma once

#include <hardware_interface/hardware_info.hpp>

namespace sura_hardware_interface
{

class ImuInterface
{
public:
  bool initialize(const hardware_interface::HardwareInfo & info);
  bool activate();
  bool deactivate();

  bool read(
    double & orientation_x,
    double & orientation_y,
    double & orientation_z,
    double & orientation_w,
    double & angular_velocity_x,
    double & angular_velocity_y,
    double & angular_velocity_z,
    double & linear_acceleration_x,
    double & linear_acceleration_y,
    double & linear_acceleration_z);

private:
  bool initialized_{false};
  bool active_{false};
};

}  // namespace sura_hardware_interface