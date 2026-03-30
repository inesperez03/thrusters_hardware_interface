#pragma once

#include <string>
#include <vector>

namespace thrusters_hardware_interface
{

struct ThrusterSample
{
  double force_n;
  double pwm_us;
  double stonefish;
};

class ThrusterMapper
{
public:
  bool loadCsv(const std::string & file_path);

  double forceToPwm(double force_n) const;
  double forceToStonefish(double force_n) const;

  bool isLoaded() const;
  std::size_t size() const;

private:
  double interpolatePwm(double force_n) const;
  double interpolateStonefish(double force_n) const;

  std::vector<ThrusterSample> samples_;
};

}
