#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace vins_fusion_ros2 {

enum class MotorInputMode {
  kOff,
  kSimExact,
  kDeployableCommandBattery,
  kMildContaminatedCommandBattery,
};

enum class MotorQualityState { kNormal, kDegraded, kRejected };

inline const char *ToString(MotorInputMode mode) {
  switch (mode) {
    case MotorInputMode::kOff: return "MOTOR_OFF";
    case MotorInputMode::kSimExact: return "SIM_EXACT_MOTOR";
    case MotorInputMode::kDeployableCommandBattery: return "DEPLOYABLE_COMMAND_BATTERY";
    case MotorInputMode::kMildContaminatedCommandBattery: return "MILD_CONTAMINATED_COMMAND_BATTERY";
  }
  return "UNKNOWN";
}

inline const char *ToString(MotorQualityState state) {
  switch (state) {
    case MotorQualityState::kNormal: return "NORMAL";
    case MotorQualityState::kDegraded: return "DEGRADED";
    case MotorQualityState::kRejected: return "REJECTED";
  }
  return "REJECTED";
}

inline MotorInputMode ParseMotorInputMode(const std::string &value) {
  if (value == "MOTOR_OFF") return MotorInputMode::kOff;
  if (value == "SIM_EXACT_MOTOR") return MotorInputMode::kSimExact;
  if (value == "DEPLOYABLE_COMMAND_BATTERY") return MotorInputMode::kDeployableCommandBattery;
  if (value == "MILD_CONTAMINATED_COMMAND_BATTERY") return MotorInputMode::kMildContaminatedCommandBattery;
  throw std::invalid_argument("unknown motor input mode: " + value);
}

struct MotorQualityPolicy {
  double gap_degraded_s{0.15};
  double gap_rejected_s{0.30};
  double battery_age_degraded_s{0.25};
  double battery_age_rejected_s{1.00};
  double degraded_weight_scale{0.25};
  double exact_slowdown{10.0};
  double maximum_rad_s{1100.0};
};

struct Cb0Parameters {
  bool frozen{false};
  double gain_rad_s_per_command{0.0};
  double idle_rad_s{0.0};
  double delay_s{0.0};
  double tau_s{0.0};
  double voltage_exponent{0.0};
  double reference_voltage_v{0.0};
};

struct MotorQuality {
  MotorQualityState state{MotorQualityState::kRejected};
  double factor_weight_scale{0.0};
  std::string reasons;

  bool admitted() const { return state != MotorQualityState::kRejected; }
};

struct EffectiveMotorSample {
  int64_t sample_timestamp_ns{0};
  uint64_t source_sequence{0};
  std::array<double, 4> rad_s{{0.0, 0.0, 0.0, 0.0}};
  MotorInputMode mode{MotorInputMode::kOff};
  MotorQuality quality;
  std::string source_hash;
};

inline bool Finite4(const std::array<double, 4> &values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

inline bool MatchesScopedJointName(const std::string &name,
                                   const std::string &joint_name) {
  if (name == joint_name) return true;
  if (name.size() <= joint_name.size()) return false;
  const size_t offset = name.size() - joint_name.size();
  if (name.compare(offset, joint_name.size(), joint_name) != 0) return false;
  return name[offset - 1] == ':' || name[offset - 1] == '/';
}

inline std::string Fnv1a64(const std::string &value) {
  uint64_t hash = 14695981039346656037ULL;
  for (unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << hash;
  return stream.str();
}

inline std::string StableSamplePayload(const EffectiveMotorSample &sample) {
  std::ostringstream stream;
  stream << sample.sample_timestamp_ns << '|' << sample.source_sequence << '|'
         << ToString(sample.mode) << std::setprecision(17);
  for (double value : sample.rad_s) stream << '|' << value;
  return stream.str();
}

inline void AppendReason(std::string *reasons, const std::string &reason) {
  if (!reasons->empty()) *reasons += ',';
  *reasons += reason;
}

inline MotorQuality EvaluateCommonQuality(
    const MotorQualityPolicy &policy, double gap_s, double battery_age_s,
    bool needs_battery, bool valid, bool active) {
  MotorQuality result;
  if (!valid) AppendReason(&result.reasons, "invalid_source");
  if (!active) AppendReason(&result.reasons, "motor_inactive_or_disarmed");
  if (!std::isfinite(gap_s) || gap_s < 0.0 || gap_s > policy.gap_rejected_s)
    AppendReason(&result.reasons, "source_gap_rejected");
  if (needs_battery && (!std::isfinite(battery_age_s) || battery_age_s < 0.0 ||
                        battery_age_s > policy.battery_age_rejected_s))
    AppendReason(&result.reasons, "battery_age_rejected");
  if (!result.reasons.empty()) {
    result.state = MotorQualityState::kRejected;
    result.factor_weight_scale = 0.0;
    return result;
  }
  const bool degraded = gap_s > policy.gap_degraded_s ||
      (needs_battery && battery_age_s > policy.battery_age_degraded_s);
  result.state = degraded ? MotorQualityState::kDegraded : MotorQualityState::kNormal;
  result.factor_weight_scale = degraded ? policy.degraded_weight_scale : 1.0;
  return result;
}

class MotorInputAdapterCore {
 public:
  MotorInputAdapterCore(MotorInputMode mode, MotorQualityPolicy policy,
                        Cb0Parameters cb0)
      : mode_(mode), policy_(policy), cb0_(cb0) {
    if (!(policy_.degraded_weight_scale > 0.0 && policy_.degraded_weight_scale < 1.0))
      throw std::invalid_argument("degraded weight must be strictly between zero and one");
    if ((mode_ == MotorInputMode::kDeployableCommandBattery ||
         mode_ == MotorInputMode::kMildContaminatedCommandBattery) &&
        (!cb0_.frozen || cb0_.gain_rad_s_per_command <= 0.0 || cb0_.tau_s <= 0.0 ||
         cb0_.reference_voltage_v <= 0.0))
      throw std::invalid_argument("command+battery modes require a frozen valid MODEL-CB0");
  }

  EffectiveMotorSample AdaptExact(int64_t timestamp_ns, uint64_t sequence,
                                  const std::array<double, 4> &joint_rad_s) {
    EffectiveMotorSample sample;
    sample.sample_timestamp_ns = timestamp_ns;
    sample.source_sequence = sequence;
    sample.mode = mode_;
    for (size_t index = 0; index < 4; ++index)
      sample.rad_s[index] = std::abs(joint_rad_s[index]) * policy_.exact_slowdown;
    const double gap_s = PreviousGap(timestamp_ns);
    const bool valid = mode_ == MotorInputMode::kSimExact && timestamp_ns > 0 &&
        Finite4(sample.rad_s) && std::all_of(sample.rad_s.begin(), sample.rad_s.end(),
          [this](double value) { return value <= policy_.maximum_rad_s; });
    const bool active = std::any_of(sample.rad_s.begin(), sample.rad_s.end(),
                                    [](double value) { return value > 1e-6; });
    sample.quality = EvaluateCommonQuality(policy_, gap_s, 0.0, false, valid, active);
    sample.source_hash = Fnv1a64(StableSamplePayload(sample));
    return sample;
  }

  EffectiveMotorSample AdaptCommandBattery(
      int64_t timestamp_ns, uint64_t sequence,
      const std::array<double, 4> &command, double battery_voltage_v,
      int64_t battery_timestamp_ns) {
    EffectiveMotorSample sample;
    sample.sample_timestamp_ns = timestamp_ns;
    sample.source_sequence = sequence;
    sample.mode = mode_;
    const double gap_s = PreviousGap(timestamp_ns);
    const double battery_age_s = static_cast<double>(timestamp_ns - battery_timestamp_ns) * 1e-9;
    bool valid = timestamp_ns > 0 && Finite4(command) && std::isfinite(battery_voltage_v) &&
        battery_voltage_v > 0.0 && battery_timestamp_ns > 0;
    for (double value : command) valid = valid && value >= 0.0 && value <= 1.0;
    const bool command_mode = mode_ == MotorInputMode::kDeployableCommandBattery ||
        mode_ == MotorInputMode::kMildContaminatedCommandBattery;
    valid = valid && command_mode && cb0_.frozen;
    const bool active = std::any_of(command.begin(), command.end(),
                                    [](double value) { return value > 1e-6; });
    command_history_.push_back({timestamp_ns, command, battery_voltage_v});
    const int64_t delayed_timestamp_ns = timestamp_ns -
        static_cast<int64_t>(std::llround(cb0_.delay_s * 1e9));
    while (command_history_.size() > 2 && command_history_[1].timestamp_ns <= delayed_timestamp_ns)
      command_history_.pop_front();
    const CommandState *delayed = nullptr;
    for (auto iterator = command_history_.rbegin(); iterator != command_history_.rend(); ++iterator) {
      if (iterator->timestamp_ns <= delayed_timestamp_ns) {
        delayed = &*iterator;
        break;
      }
    }
    if (valid && delayed == nullptr) valid = false;
    if (valid) {
      const double voltage_scale = std::pow(delayed->battery_voltage_v / cb0_.reference_voltage_v,
                                            cb0_.voltage_exponent);
      std::array<double, 4> target;
      for (size_t index = 0; index < 4; ++index)
        target[index] = std::max(0.0, cb0_.idle_rad_s +
          cb0_.gain_rad_s_per_command * delayed->command[index] * voltage_scale);
      if (!have_filter_state_) {
        filtered_rad_s_ = target;
        have_filter_state_ = true;
      } else if (gap_s > 0.0 && std::isfinite(gap_s)) {
        const double alpha = 1.0 - std::exp(-gap_s / cb0_.tau_s);
        for (size_t index = 0; index < 4; ++index)
          filtered_rad_s_[index] += alpha * (target[index] - filtered_rad_s_[index]);
      }
      sample.rad_s = filtered_rad_s_;
      valid = Finite4(sample.rad_s) && std::all_of(sample.rad_s.begin(), sample.rad_s.end(),
        [this](double value) { return value <= policy_.maximum_rad_s; });
    }
    sample.quality = EvaluateCommonQuality(policy_, gap_s, battery_age_s, true, valid, active);
    sample.source_hash = Fnv1a64(StableSamplePayload(sample));
    return sample;
  }

 private:
  struct CommandState {
    int64_t timestamp_ns;
    std::array<double, 4> command;
    double battery_voltage_v;
  };

  double PreviousGap(int64_t timestamp_ns) {
    double gap = 0.0;
    if (last_timestamp_ns_ > 0)
      gap = static_cast<double>(timestamp_ns - last_timestamp_ns_) * 1e-9;
    last_timestamp_ns_ = std::max(last_timestamp_ns_, timestamp_ns);
    return gap;
  }

  MotorInputMode mode_;
  MotorQualityPolicy policy_;
  Cb0Parameters cb0_;
  int64_t last_timestamp_ns_{0};
  bool have_filter_state_{false};
  std::array<double, 4> filtered_rad_s_{{0.0, 0.0, 0.0, 0.0}};
  std::deque<CommandState> command_history_;
};

}  // namespace vins_fusion_ros2
