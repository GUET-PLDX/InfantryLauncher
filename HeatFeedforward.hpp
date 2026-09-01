#pragma once

#include <algorithm>
#include <cmath>

namespace launcher {

constexpr float HEAT_SETTLEMENT_PERIOD_SEC = 0.1f;
constexpr float PERIODS_PER_SECOND = 10.0f;

struct HeatFeedforwardConfig {
  float single_heat = 10.0f;
  float max_frequency = 15.0f;
  float burst_duration = 2.0f;
  float heat_margin = 10.0f;
};

struct HeatFeedforwardObservation {
  float heat_limit = 0.0f;
  float current_heat = 0.0f;
  float cooling_rate = 0.0f;
  bool data_valid = false;
};

struct HeatFeedforwardResult {
  bool allow_fire = false;
  float target_frequency = 0.0f;
};

// PDF model: each 100 ms period settles shots first, then cooling.
class HeatFeedforward {
 public:
  static float AdvanceBudget(const HeatFeedforwardConfig& config,
                             float current_heat, float cooling_rate,
                             unsigned int launched_shots,
                             unsigned int completed_periods) {
    if (!std::isfinite(current_heat) || !std::isfinite(cooling_rate) ||
        !std::isfinite(config.single_heat) || config.single_heat <= 0.0f ||
        cooling_rate < 0.0f) {
      return current_heat;
    }
    const float settled_heat =
        current_heat + config.single_heat * static_cast<float>(launched_shots);
    return std::max(0.0f,
                    settled_heat - cooling_rate * HEAT_SETTLEMENT_PERIOD_SEC *
                                       static_cast<float>(completed_periods));
  }

  static HeatFeedforwardResult Calculate(
      const HeatFeedforwardConfig& config,
      const HeatFeedforwardObservation& observation,
      unsigned int shot_count = 1U) {
    const float d = config.single_heat;
    const float n =
        std::ceil(config.burst_duration / HEAT_SETTLEMENT_PERIOD_SEC);
    if (!observation.data_valid || !std::isfinite(d) || d <= 0.0f ||
        !std::isfinite(n) || n <= 0.0f ||
        !std::isfinite(config.max_frequency) || config.max_frequency <= 0.0f ||
        !std::isfinite(config.heat_margin) || config.heat_margin < 0.0f ||
        !std::isfinite(observation.heat_limit) ||
        !std::isfinite(observation.current_heat) ||
        !std::isfinite(observation.cooling_rate) ||
        observation.heat_limit <= 0.0f || observation.cooling_rate < 0.0f ||
        shot_count == 0U) {
      return {};
    }
    const float remaining =
        observation.heat_limit - observation.current_heat - config.heat_margin;
    if (remaining < d * static_cast<float>(shot_count)) {
      return {};
    }
    const float sustainable = observation.cooling_rate / d;
    const float burst =
        (PERIODS_PER_SECOND * remaining - observation.cooling_rate) / (d * n) +
        sustainable;
    const float lower_frequency = std::min(sustainable, config.max_frequency);
    return {true, std::clamp(burst, lower_frequency, config.max_frequency)};
  }
};

}  // namespace launcher
