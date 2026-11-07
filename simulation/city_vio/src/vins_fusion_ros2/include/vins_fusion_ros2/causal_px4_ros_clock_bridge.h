#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdexcept>

namespace vins_fusion_ros2 {

// Live fail-closed mapping from a PX4 timestamp domain into ROS simulation
// time.  Each anchor is an observed pair of PX4 publication timestamp and ROS
// callback-receipt clock.  A sample maps only when two retained anchors bracket
// it.  There is no nearest association, staircase reuse, or extrapolation.
class CausalPx4RosClockBridge {
 public:
  explicit CausalPx4RosClockBridge(size_t maximum_anchors = 128)
      : maximum_anchors_(maximum_anchors) {
    if (maximum_anchors_ < 2) throw std::invalid_argument("clock bridge needs at least two anchors");
  }

  bool PushAnchor(uint64_t px4_publication_timestamp_us, int64_t ros_receipt_timestamp_ns) {
    if (px4_publication_timestamp_us == 0 || ros_receipt_timestamp_ns <= 0) return false;
    if (!anchors_.empty()) {
      if (px4_publication_timestamp_us <= anchors_.back().px4_us ||
          ros_receipt_timestamp_ns <= anchors_.back().ros_ns)
        return false;
    }
    anchors_.push_back({px4_publication_timestamp_us, ros_receipt_timestamp_ns});
    while (anchors_.size() > maximum_anchors_) anchors_.pop_front();
    return true;
  }

  bool MapBracketed(uint64_t px4_sample_timestamp_us, int64_t *ros_timestamp_ns) const {
    if (px4_sample_timestamp_us == 0 || anchors_.size() < 2 || ros_timestamp_ns == nullptr)
      return false;
    const auto upper = std::lower_bound(
        anchors_.begin(), anchors_.end(), px4_sample_timestamp_us,
        [](const Anchor &anchor, uint64_t timestamp_us) { return anchor.px4_us < timestamp_us; });
    if (upper == anchors_.end()) return false;
    if (upper->px4_us == px4_sample_timestamp_us) {
      if (upper == anchors_.begin()) return false;  // still require a causal lower bracket
      *ros_timestamp_ns = upper->ros_ns;
      return true;
    }
    if (upper == anchors_.begin()) return false;
    const auto lower = std::prev(upper);
    const long double numerator = static_cast<long double>(px4_sample_timestamp_us - lower->px4_us);
    const long double denominator = static_cast<long double>(upper->px4_us - lower->px4_us);
    const long double mapped = static_cast<long double>(lower->ros_ns) +
        numerator / denominator * static_cast<long double>(upper->ros_ns - lower->ros_ns);
    const int64_t rounded = static_cast<int64_t>(mapped + 0.5L);
    if (rounded < lower->ros_ns || rounded > upper->ros_ns) return false;
    *ros_timestamp_ns = rounded;
    return true;
  }

  size_t anchor_count() const { return anchors_.size(); }

 private:
  struct Anchor { uint64_t px4_us; int64_t ros_ns; };
  size_t maximum_anchors_;
  std::deque<Anchor> anchors_;
};

}  // namespace vins_fusion_ros2

