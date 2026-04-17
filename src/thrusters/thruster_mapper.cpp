#include "sura_hardware_interface/thrusters/thruster_mapper.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sura_hardware_interface
{

bool ThrusterMapper::loadCsv(const std::string & file_path)
{
  samples_.clear();

  std::ifstream file(file_path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  bool first_line = true;

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    if (first_line) {
      first_line = false;
      continue;  // saltar cabecera
    }

    std::stringstream ss(line);
    std::string pwm_str, force_str, stonefish_str;

    if (!std::getline(ss, pwm_str, ',')) {
      continue;
    }
    if (!std::getline(ss, force_str, ',')) {
      continue;
    }
    if (!std::getline(ss, stonefish_str, ',')) {
      continue;
    }

    ThrusterSample sample;
    sample.pwm_us = std::stod(pwm_str);
    sample.force_n = std::stod(force_str);
    sample.stonefish = std::stod(stonefish_str);

    samples_.push_back(sample);
  }

  if (samples_.empty()) {
    return false;
  }

  std::sort(samples_.begin(), samples_.end(),
    [](const ThrusterSample & a, const ThrusterSample & b) {
      return a.force_n < b.force_n;
    });

  return true;
}

double ThrusterMapper::forceToPwm(double force_n) const
{
  return interpolatePwm(force_n);
}

double ThrusterMapper::forceToStonefish(double force_n) const
{
  return interpolateStonefish(force_n);
}

bool ThrusterMapper::isLoaded() const
{
  return !samples_.empty();
}

std::size_t ThrusterMapper::size() const
{
  return samples_.size();
}

double ThrusterMapper::interpolatePwm(double force_n) const
{
  if (samples_.empty()) {
    throw std::runtime_error("ThrusterMapper: CSV not loaded");
  }

  if (force_n <= samples_.front().force_n) {
    return samples_.front().pwm_us;
  }

  if (force_n >= samples_.back().force_n) {
    return samples_.back().pwm_us;
  }

  for (std::size_t i = 0; i < samples_.size() - 1; ++i) {
    const auto & p1 = samples_[i];
    const auto & p2 = samples_[i + 1];

    if (force_n >= p1.force_n && force_n <= p2.force_n) {
      const double ratio = (force_n - p1.force_n) / (p2.force_n - p1.force_n);
      return p1.pwm_us + ratio * (p2.pwm_us - p1.pwm_us);
    }
  }

  return samples_.back().pwm_us;
}

double ThrusterMapper::interpolateStonefish(double force_n) const
{
  if (samples_.empty()) {
    throw std::runtime_error("ThrusterMapper: CSV not loaded");
  }

  if (force_n <= samples_.front().force_n) {
    return samples_.front().stonefish;
  }

  if (force_n >= samples_.back().force_n) {
    return samples_.back().stonefish;
  }

  for (std::size_t i = 0; i < samples_.size() - 1; ++i) {
    const auto & p1 = samples_[i];
    const auto & p2 = samples_[i + 1];

    if (force_n >= p1.force_n && force_n <= p2.force_n) {
      const double ratio = (force_n - p1.force_n) / (p2.force_n - p1.force_n);
      return p1.stonefish + ratio * (p2.stonefish - p1.stonefish);
    }
  }

  return samples_.back().stonefish;
}

}
