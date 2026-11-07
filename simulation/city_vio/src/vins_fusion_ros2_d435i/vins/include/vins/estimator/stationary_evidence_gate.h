#pragma once

#include <cmath>

namespace vins {

struct StationaryEvidence {
  int visual_tracks = 0;
  double median_flow_px_s = INFINITY;
  double angular_rate_rad_s = INFINITY;
  double acceleration_norm_error_mps2 = INFINITY;
  bool range_required = false;
  bool range_pair_valid = false;
  double range_delta_m = INFINITY;
};

class StationaryEvidenceGate {
 public:
  bool evaluate(const StationaryEvidence &evidence) const {
    return evidence.visual_tracks >= 30 &&
           std::isfinite(evidence.median_flow_px_s) &&
           evidence.median_flow_px_s <= 15.0 &&
           std::isfinite(evidence.angular_rate_rad_s) &&
           evidence.angular_rate_rad_s <= 0.03 &&
           std::isfinite(evidence.acceleration_norm_error_mps2) &&
           evidence.acceleration_norm_error_mps2 <= 0.20 &&
           (!evidence.range_required ||
            (evidence.range_pair_valid &&
             std::isfinite(evidence.range_delta_m) &&
             evidence.range_delta_m <= 0.05));
  }
};

}  // namespace vins
