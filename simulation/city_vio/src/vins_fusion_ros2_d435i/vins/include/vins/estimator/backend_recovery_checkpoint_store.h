/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 * and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0.
 *******************************************************/

#pragma once

#include <vins/estimator/backend_state.h>
#include <vins/featureTracker/feature_tracker.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace vins {

struct BackendRecoveryCheckpoint {
  std::uint64_t feature_sequence = 0;
  std::uint64_t raw_image_sequence = 0;
  double timestamp = std::numeric_limits<double>::quiet_NaN();
  BackendState::Ptr backend;
  std::unique_ptr<FeatureTracker> tracker;
};

class BackendRecoveryCheckpointStore {
 public:
  explicit BackendRecoveryCheckpointStore(std::size_t capacity)
      : capacity_(capacity) {}

  bool valid() const { return capacity_ > 0; }

  bool push(std::uint64_t feature_sequence,
            std::uint64_t raw_image_sequence, double timestamp,
            BackendState::Ptr backend,
            std::unique_ptr<FeatureTracker> tracker,
            std::string *error = nullptr) {
    if (!valid() || feature_sequence == 0 || raw_image_sequence == 0 ||
        !std::isfinite(timestamp) || !backend || !tracker ||
        !backend->priorAddressesAreOwned()) {
      return fail("invalid recovery checkpoint", error);
    }
    if (!checkpoints_.empty() &&
        (feature_sequence <= checkpoints_.back().feature_sequence ||
         raw_image_sequence <= checkpoints_.back().raw_image_sequence ||
         timestamp <= checkpoints_.back().timestamp)) {
      return fail("recovery checkpoints must be strictly monotonic", error);
    }
    BackendRecoveryCheckpoint checkpoint;
    checkpoint.feature_sequence = feature_sequence;
    checkpoint.raw_image_sequence = raw_image_sequence;
    checkpoint.timestamp = timestamp;
    checkpoint.backend = std::move(backend);
    checkpoint.tracker = std::move(tracker);
    checkpoints_.push_back(std::move(checkpoint));
    while (checkpoints_.size() > capacity_) {
      attempted_.erase(checkpoints_.front().feature_sequence);
      checkpoints_.pop_front();
    }
    if (error) error->clear();
    return true;
  }

  const BackendRecoveryCheckpoint *selectForRewind(
      std::uint64_t first_suspect_sequence,
      std::uint64_t minimum_rewind_frames) const {
    if (first_suspect_sequence <= minimum_rewind_frames) return nullptr;
    const std::uint64_t latest_eligible =
        first_suspect_sequence - minimum_rewind_frames;
    for (auto it = checkpoints_.rbegin(); it != checkpoints_.rend(); ++it) {
      if (it->feature_sequence <= latest_eligible &&
          attempted_.count(it->feature_sequence) == 0) {
        return &*it;
      }
    }
    return nullptr;
  }

  bool markAttempted(std::uint64_t feature_sequence) {
    for (const auto &checkpoint : checkpoints_) {
      if (checkpoint.feature_sequence == feature_sequence) {
        return attempted_.insert(feature_sequence).second;
      }
    }
    return false;
  }

  std::size_t size() const { return checkpoints_.size(); }
  bool empty() const { return checkpoints_.empty(); }
  void clear() {
    checkpoints_.clear();
    attempted_.clear();
  }
  double oldestTimestamp() const {
    return checkpoints_.empty()
               ? std::numeric_limits<double>::quiet_NaN()
               : checkpoints_.front().timestamp;
  }

 private:
  static bool fail(const char *message, std::string *error) {
    if (error) *error = message;
    return false;
  }

  std::size_t capacity_;
  std::deque<BackendRecoveryCheckpoint> checkpoints_;
  std::set<std::uint64_t> attempted_;
};

struct BackendRecoverySensorSlice {
  std::vector<ImageData> images;
  std::vector<IMUData> imu;
  std::vector<RangeData> ranges;
};

class BackendRecoverySensorRing {
 public:
  bool appendImage(const ImageData &image) {
    if (image.sequence == 0 || !std::isfinite(image.timestamp) ||
        (!images_.empty() &&
         (image.sequence <= images_.back().sequence ||
          image.timestamp <= images_.back().timestamp))) {
      return false;
    }
    images_.push_back(image);
    return true;
  }

  bool appendImu(const IMUData &imu) {
    if (!std::isfinite(imu.timestamp) ||
        (!imu_.empty() && imu.timestamp <= imu_.back().timestamp)) {
      return false;
    }
    imu_.push_back(imu);
    return true;
  }

  bool appendRange(const RangeData &range) {
    if (!std::isfinite(range.timestamp) ||
        (!ranges_.empty() &&
         range.timestamp <= ranges_.back().timestamp)) {
      return false;
    }
    ranges_.push_back(range);
    return true;
  }

  void trimBefore(double timestamp) {
    while (!images_.empty() && images_.front().timestamp < timestamp)
      images_.pop_front();
    // Preserve the final sample before the boundary for exact interpolation.
    while (imu_.size() > 1 && imu_[1].timestamp < timestamp)
      imu_.pop_front();
    while (!ranges_.empty() && ranges_.front().timestamp < timestamp)
      ranges_.pop_front();
  }

  BackendRecoverySensorSlice slice(double start, double end) const {
    BackendRecoverySensorSlice result;
    if (!std::isfinite(start) || !std::isfinite(end) || end < start)
      return result;
    for (const auto &image : images_)
      if (image.timestamp > start && image.timestamp <= end)
        result.images.push_back(image);
    for (const auto &imu : imu_)
      if (imu.timestamp >= start && imu.timestamp <= end)
        result.imu.push_back(imu);
    for (const auto &range : ranges_)
      if (range.timestamp > start && range.timestamp <= end)
        result.ranges.push_back(range);
    return result;
  }

  std::size_t imageCount() const { return images_.size(); }
  std::size_t imuCount() const { return imu_.size(); }
  std::size_t rangeCount() const { return ranges_.size(); }
  double newestImageTimestamp() const {
    return images_.empty()
               ? std::numeric_limits<double>::quiet_NaN()
               : images_.back().timestamp;
  }
  std::uint64_t newestImageSequence() const {
    return images_.empty() ? 0 : images_.back().sequence;
  }
  void clear() {
    images_.clear();
    imu_.clear();
    ranges_.clear();
  }

 private:
  std::deque<ImageData> images_;
  std::deque<IMUData> imu_;
  std::deque<RangeData> ranges_;
};

}  // namespace vins
