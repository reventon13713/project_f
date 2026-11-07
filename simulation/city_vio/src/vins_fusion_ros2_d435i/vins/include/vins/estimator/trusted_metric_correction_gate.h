#pragma once

#include <cmath>

struct TrustedMetricCorrectionEvidence {
  bool gps_fresh = false;
  bool range_fresh = false;
  double gps_position_error_m = 0.0;
  double gps_velocity_error_mps = 0.0;
  double range_position_error_m = 0.0;
  double output_step_m = 0.0;
};

struct TrustedGpsDropoutCorrectionEvidence {
  bool anchored_epoch = false;
  bool gps_position_factor_active = false;
  bool range_factor_active = false;
  double horizontal_speed_mps = 0.0;
  double maximum_speed_mps = 0.0;
  double optimization_position_correction_m = 0.0;
  double horizontal_output_step_m = 0.0;
  double vertical_output_step_m = 0.0;
  double vertical_output_innovation_m = 0.0;
  double range_position_error_m = 0.0;
};

inline bool IsTrustedMetricCorrection(
    const TrustedMetricCorrectionEvidence &evidence) {
  return evidence.gps_fresh && evidence.range_fresh &&
         std::isfinite(evidence.gps_position_error_m) &&
         std::isfinite(evidence.gps_velocity_error_mps) &&
         std::isfinite(evidence.range_position_error_m) &&
         std::isfinite(evidence.output_step_m) &&
         evidence.gps_position_error_m <= 0.10 &&
         evidence.gps_velocity_error_mps <= 0.25 &&
         evidence.range_position_error_m <= 0.20 &&
         evidence.output_step_m <= 0.50;
}

inline bool IsTrustedGpsDropoutCorrection(
    const TrustedGpsDropoutCorrectionEvidence &evidence) {
  return evidence.anchored_epoch &&
         !evidence.gps_position_factor_active &&
         evidence.range_factor_active &&
         std::isfinite(evidence.horizontal_speed_mps) &&
         std::isfinite(evidence.maximum_speed_mps) &&
         std::isfinite(evidence.optimization_position_correction_m) &&
         std::isfinite(evidence.horizontal_output_step_m) &&
         std::isfinite(evidence.vertical_output_step_m) &&
         std::isfinite(evidence.vertical_output_innovation_m) &&
         std::isfinite(evidence.range_position_error_m) &&
         evidence.maximum_speed_mps > 0.0 &&
         evidence.horizontal_speed_mps <=
             evidence.maximum_speed_mps + 0.05 &&
         evidence.optimization_position_correction_m <= 0.15 &&
         evidence.horizontal_output_step_m <= 0.50 &&
         evidence.vertical_output_step_m <= 0.15 &&
         evidence.range_position_error_m <= 0.15;
}

inline bool ShouldUseAnchoredMetricGauge(
    bool has_imu, bool gps_enabled, bool anchor_initialized,
    bool position_factor_active_this_solve) {
  return has_imu && gps_enabled && anchor_initialized &&
         position_factor_active_this_solve;
}
