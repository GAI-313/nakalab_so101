#ifndef NAKALAB_SO101__POWER_MONITOR_HPP_
#define NAKALAB_SO101__POWER_MONITOR_HPP_

#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace nakalab_so101
{

struct PowerMonitorConfig
{
  std::size_t expected_motor_count = 6;
  double min_voltage = 5.5;
  int miss_threshold = 2;
};

struct PowerMonitorUpdate
{
  bool estop_active = false;
  bool entered_estop = false;
  bool ready_for_recovery = false;
  double min_voltage = 0.0;
  std::string reason;
};

class PowerMonitor
{
public:
  explicit PowerMonitor(const PowerMonitorConfig & config = PowerMonitorConfig{})
  : config_(config)
  {
  }

  PowerMonitorUpdate update_from_voltages(
    const std::vector<double> & voltages,
    bool has_voltage_readings = true)
  {
    PowerMonitorUpdate result;
    const std::size_t count = voltages.size();

    double min_v = 0.0;
    if (!voltages.empty()) {
      min_v = *std::min_element(voltages.begin(), voltages.end());
    }
    last_min_voltage_ = min_v;
    result.min_voltage = min_v;

    if (has_voltage_readings && !voltages.empty() && config_.min_voltage > 0.0 &&
      min_v < config_.min_voltage)
    {
      const bool was_active = estop_active_;
      estop_active_ = true;
      consecutive_misses_ = 0;
      reason_ = "low_voltage";
      result.estop_active = true;
      result.entered_estop = !was_active;
      result.reason = reason_;
      return result;
    }

    if (count < config_.expected_motor_count) {
      ++consecutive_misses_;
      if (consecutive_misses_ >= std::max(1, config_.miss_threshold)) {
        const bool was_active = estop_active_;
        estop_active_ = true;
        reason_ = "communication_lost";
        result.estop_active = true;
        result.entered_estop = !was_active;
        result.reason = reason_;
        return result;
      }
      result.estop_active = estop_active_;
      result.reason = reason_;
      return result;
    }

    consecutive_misses_ = 0;
    result.estop_active = estop_active_;
    result.reason = reason_;
    return result;
  }

  PowerMonitorUpdate update_from_probe(std::optional<double> probe_voltage)
  {
    PowerMonitorUpdate result;
    if (!probe_voltage.has_value()) {
      last_min_voltage_ = 0.0;
      result.estop_active = estop_active_;
      result.min_voltage = 0.0;
      result.reason = reason_;
      return result;
    }

    last_min_voltage_ = *probe_voltage;
    result.min_voltage = *probe_voltage;
    const bool voltage_ok = config_.min_voltage <= 0.0 || *probe_voltage >= config_.min_voltage;
    result.ready_for_recovery = voltage_ok;
    result.estop_active = estop_active_;
    result.reason = reason_;
    return result;
  }

  void complete_recovery()
  {
    estop_active_ = false;
    consecutive_misses_ = 0;
    reason_.clear();
  }

  bool is_estop_active() const
  {
    return estop_active_;
  }

  int consecutive_misses() const
  {
    return consecutive_misses_;
  }

  double last_min_voltage() const
  {
    return last_min_voltage_;
  }

  const std::string & reason() const
  {
    return reason_;
  }

private:
  PowerMonitorConfig config_;
  bool estop_active_ = false;
  int consecutive_misses_ = 0;
  double last_min_voltage_ = 0.0;
  std::string reason_;
};

}  // namespace nakalab_so101

#endif  // NAKALAB_SO101__POWER_MONITOR_HPP_
