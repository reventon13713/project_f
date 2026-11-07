#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "vins_fusion_ros2/motor_input_adapter_core.h"

namespace vins_fusion_ros2 {

struct MotorFactorEndpoint {
  int64_t sample_timestamp_ns{0};
  uint64_t source_sequence{0};
  std::string source_hash;
  MotorQualityState quality_state{MotorQualityState::kRejected};
  double factor_weight_scale{0.0};
};

struct MotorFactorPairDecision {
  bool admitted{false};
  double pair_scale{0.0};
  double sigma_effective{0.0};
};

inline bool IsValidMotorFactorEndpoint(const MotorFactorEndpoint &endpoint) {
  if (endpoint.sample_timestamp_ns <= 0 || endpoint.source_hash.size() != 16 ||
      !std::isfinite(endpoint.factor_weight_scale))
    return false;
  if (endpoint.quality_state == MotorQualityState::kNormal)
    return endpoint.factor_weight_scale == 1.0;
  if (endpoint.quality_state == MotorQualityState::kDegraded)
    return endpoint.factor_weight_scale == 0.25;
  return false;
}

inline MotorFactorPairDecision EvaluateMotorFactorPair(
    double base_sigma, const MotorFactorEndpoint &first,
    const MotorFactorEndpoint &second) {
  MotorFactorPairDecision result;
  if (!std::isfinite(base_sigma) || base_sigma <= 0.0 ||
      !IsValidMotorFactorEndpoint(first) ||
      !IsValidMotorFactorEndpoint(second))
    return result;
  result.pair_scale =
      std::min(first.factor_weight_scale, second.factor_weight_scale);
  result.sigma_effective = base_sigma / std::sqrt(result.pair_scale);
  result.admitted = std::isfinite(result.sigma_effective);
  return result;
}

inline std::string MotorFactorTimelineJson(
    uint64_t solve_sequence, int first_window_index, int second_window_index,
    const MotorFactorEndpoint &first, const MotorFactorEndpoint &second,
    const MotorFactorPairDecision &decision) {
  std::ostringstream stream;
  stream << std::setprecision(17)
         << "{\"schema\":\"project_f.motor_factor_timeline.v1\""
         << ",\"solve_sequence\":" << solve_sequence
         << ",\"first_window_index\":" << first_window_index
         << ",\"second_window_index\":" << second_window_index
         << ",\"first_sample_timestamp_ns\":" << first.sample_timestamp_ns
         << ",\"first_source_sequence\":" << first.source_sequence
         << ",\"first_source_hash_fnv1a64\":\"" << first.source_hash << "\""
         << ",\"first_quality_state\":\"" << ToString(first.quality_state) << "\""
         << ",\"first_factor_weight_scale\":" << first.factor_weight_scale
         << ",\"second_sample_timestamp_ns\":" << second.sample_timestamp_ns
         << ",\"second_source_sequence\":" << second.source_sequence
         << ",\"second_source_hash_fnv1a64\":\"" << second.source_hash << "\""
         << ",\"second_quality_state\":\"" << ToString(second.quality_state) << "\""
         << ",\"second_factor_weight_scale\":" << second.factor_weight_scale
         << ",\"pair_scale\":" << decision.pair_scale
         << ",\"sigma_effective\":" << decision.sigma_effective
         << ",\"factor_participation\":"
         << (decision.admitted ? "true" : "false")
         << ",\"old_prior_participation\":false}";
  return stream.str();
}

}  // namespace vins_fusion_ros2

