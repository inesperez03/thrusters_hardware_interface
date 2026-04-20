#pragma once

namespace sura_hardware_interface
{

class MagnetometerInterface
{
public:
  bool read(
    double & magnetic_field_x,
    double & magnetic_field_y,
    double & magnetic_field_z);
};

}  // namespace sura_hardware_interface