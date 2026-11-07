#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <regex>
#include <stdexcept>
#include <string>

#include "vins_fusion_ros2/motor_input_adapter_core.h"

namespace vins_fusion_ros2 {

struct EffectiveMotorQualityRecord {
  int64_t sample_timestamp_ns{0};
  uint64_t source_sequence{0};
  std::string source_hash;
  MotorInputMode mode{MotorInputMode::kOff};
  MotorQualityState state{MotorQualityState::kRejected};
  double factor_weight_scale{0.0};
  bool factor_participation_allowed{false};
  bool old_prior_participation_allowed{false};
};

struct VerifiedEffectiveMotorInput {
  int64_t sample_timestamp_ns{0};
  uint64_t source_sequence{0};
  std::string source_hash;
  std::array<double, 4> rad_s{{0.0, 0.0, 0.0, 0.0}};
  MotorQualityState quality_state{MotorQualityState::kRejected};
  double factor_weight_scale{0.0};
};

inline std::string JsonToken(const std::string &json, const std::string &key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (!std::regex_search(json, match, expression))
    throw std::invalid_argument("quality JSON missing string field: " + key);
  return match[1];
}

inline int64_t JsonInt64(const std::string &json, const std::string &key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(json, match, expression))
    throw std::invalid_argument("quality JSON missing integer field: " + key);
  return std::stoll(match[1]);
}

inline double JsonDouble(const std::string &json, const std::string &key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(json, match, expression))
    throw std::invalid_argument("quality JSON missing numeric field: " + key);
  return std::stod(match[1]);
}

inline bool JsonBool(const std::string &json, const std::string &key) {
  const std::regex expression("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (!std::regex_search(json, match, expression))
    throw std::invalid_argument("quality JSON missing boolean field: " + key);
  return match[1] == "true";
}

inline EffectiveMotorQualityRecord ParseEffectiveMotorQuality(const std::string &json) {
  if (JsonToken(json, "schema") != "project_f.effective_motor_quality.v1")
    throw std::invalid_argument("effective motor quality schema mismatch");
  EffectiveMotorQualityRecord result;
  result.sample_timestamp_ns = JsonInt64(json, "sample_timestamp_ns");
  result.source_sequence = static_cast<uint64_t>(JsonInt64(json, "source_sequence"));
  result.source_hash = JsonToken(json, "source_hash_fnv1a64");
  result.mode = ParseMotorInputMode(JsonToken(json, "mode"));
  const std::string quality = JsonToken(json, "quality_state");
  if (quality == "NORMAL") result.state = MotorQualityState::kNormal;
  else if (quality == "DEGRADED") result.state = MotorQualityState::kDegraded;
  else if (quality == "REJECTED") result.state = MotorQualityState::kRejected;
  else throw std::invalid_argument("unknown motor quality state");
  result.factor_weight_scale = JsonDouble(json, "factor_weight_scale");
  result.factor_participation_allowed = JsonBool(json, "factor_participation_allowed");
  result.old_prior_participation_allowed = JsonBool(json, "old_prior_participation_allowed");
  if (result.sample_timestamp_ns <= 0 || result.source_hash.size() != 16 ||
      result.old_prior_participation_allowed)
    throw std::invalid_argument("invalid motor quality provenance or prior policy");
  if (result.state == MotorQualityState::kRejected) {
    if (result.factor_participation_allowed || result.factor_weight_scale != 0.0)
      throw std::invalid_argument("REJECTED motor quality must have zero participation");
  } else {
    if (!result.factor_participation_allowed || !(result.factor_weight_scale > 0.0 &&
                                                  result.factor_weight_scale <= 1.0))
      throw std::invalid_argument("admitted motor quality has invalid weight");
  }
  return result;
}

class EffectiveMotorQualityJoin {
 public:
  explicit EffectiveMotorQualityJoin(MotorInputMode configured_mode,
                                     size_t maximum_pending = 1024)
      : configured_mode_(configured_mode), maximum_pending_(maximum_pending) {
    if (maximum_pending_ == 0) throw std::invalid_argument("pending cap must be positive");
  }

  EffectiveMotorQualityRecord PushQuality(const std::string &json) {
    EffectiveMotorQualityRecord quality = ParseEffectiveMotorQuality(json);
    if (quality.mode != configured_mode_)
      throw std::invalid_argument("quality mode does not match configured estimator mode");
    if (qualities_.count(quality.sample_timestamp_ns))
      throw std::invalid_argument("duplicate motor quality timestamp");
    qualities_.emplace(quality.sample_timestamp_ns, quality);
    Trim(&qualities_);
    return quality;
  }

  void PushState(int64_t timestamp_ns, const std::array<double, 4> &rad_s) {
    if (timestamp_ns <= 0 || !Finite4(rad_s))
      throw std::invalid_argument("effective motor state is invalid");
    if (states_.count(timestamp_ns))
      throw std::invalid_argument("duplicate effective motor state timestamp");
    states_.emplace(timestamp_ns, rad_s);
    Trim(&states_);
  }

  bool TryPop(int64_t timestamp_ns, VerifiedEffectiveMotorInput *output) {
    const auto quality_iterator = qualities_.find(timestamp_ns);
    const auto state_iterator = states_.find(timestamp_ns);
    if (quality_iterator == qualities_.end() || state_iterator == states_.end()) return false;
    const auto quality = quality_iterator->second;
    const auto state = state_iterator->second;
    qualities_.erase(quality_iterator);
    states_.erase(state_iterator);
    if (!quality.factor_participation_allowed || quality.state == MotorQualityState::kRejected)
      throw std::invalid_argument("REJECTED quality unexpectedly has an effective state");
    EffectiveMotorSample sample;
    sample.sample_timestamp_ns = timestamp_ns;
    sample.source_sequence = quality.source_sequence;
    sample.mode = quality.mode;
    sample.rad_s = state;
    const std::string expected_hash = Fnv1a64(StableSamplePayload(sample));
    if (expected_hash != quality.source_hash)
      throw std::invalid_argument("effective state/quality source hash mismatch");
    output->sample_timestamp_ns = timestamp_ns;
    output->source_sequence = quality.source_sequence;
    output->source_hash = quality.source_hash;
    output->rad_s = state;
    output->quality_state = quality.state;
    output->factor_weight_scale = quality.factor_weight_scale;
    return true;
  }

 private:
  template <typename Map>
  void Trim(Map *values) {
    while (values->size() > maximum_pending_) values->erase(values->begin());
  }

  MotorInputMode configured_mode_;
  size_t maximum_pending_;
  std::map<int64_t, EffectiveMotorQualityRecord> qualities_;
  std::map<int64_t, std::array<double, 4>> states_;
};

}  // namespace vins_fusion_ros2
