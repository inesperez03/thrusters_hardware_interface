#include "sura_hardware_interface/sensors/imu_interface.hpp"

#include "bindings.h"

namespace sura_hardware_interface
{

bool ImuInterface::initialize(const hardware_interface::HardwareInfo &)
{
  init();
  initialized_ = true;
  return true;
}

bool ImuInterface::activate()
{
  if (!initialized_) {
    return false;
  }

  active_ = true;
  return true;
}

bool ImuInterface::deactivate()
{
  active_ = false;
  return true;
}

bool ImuInterface::read(
  double & orientation_x,
  double & orientation_y,
  double & orientation_z,
  double & orientation_w,
  double & angular_velocity_x,
  double & angular_velocity_y,
  double & angular_velocity_z,
  double & linear_acceleration_x,
  double & linear_acceleration_y,
  double & linear_acceleration_z)
{
  if (!initialized_ || !active_) {
    return false;
  }

  AxisData accel = read_accel();
  AxisData gyro = read_gyro();

  // Placeholder: la orientación real la calculará el filtro después
  orientation_x = 0.0;
  orientation_y = 0.0;
  orientation_z = 0.0;
  orientation_w = 1.0;

  angular_velocity_x = static_cast<double>(gyro.x);
  angular_velocity_y = static_cast<double>(gyro.y);
  angular_velocity_z = static_cast<double>(gyro.z);

  linear_acceleration_x = static_cast<double>(accel.x);
  linear_acceleration_y = static_cast<double>(accel.y);
  linear_acceleration_z = static_cast<double>(accel.z);

  return true;
}

}  // namespace sura_hardware_interface