#include "sura_hardware_interface/sensors/magnetometer_interface.hpp"

#include "bindings.h"

namespace sura_hardware_interface
{

bool MagnetometerInterface::read(
  double & magnetic_field_x,
  double & magnetic_field_y,
  double & magnetic_field_z)
{
  const AxisData mag = read_mag();

  magnetic_field_x = static_cast<double>(mag.x);
  magnetic_field_y = static_cast<double>(mag.y);
  magnetic_field_z = static_cast<double>(mag.z);

  return true;
}

}  // namespace sura_hardware_interface