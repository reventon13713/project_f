#include <vins/estimator/backend_recovery_checkpoint_store.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

bool expect(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
    return false;
  }
  return true;
}

vins::BackendState::Ptr backend(std::uint64_t generation, double x) {
  auto result = vins::BackendState::Ptr(new vins::BackendState);
  result->generation = generation;
  result->states[0].position.x() = x;
  result->pose_parameters[0][0] = x;
  result->options = std::make_shared<VINSOptions>();
  result->options->RIC = {Eigen::Matrix3d::Identity(),
                          Eigen::Matrix3d::Identity()};
  result->options->TIC = {Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero()};
  result->options->num_of_camera = 1;
  result->feature_manager.setOptions(result->options);
  return result;
}

std::unique_ptr<FeatureTracker> tracker(int id, unsigned char pixel) {
  FeatureTracker source;
  auto options = std::make_shared<VINSOptions>();
  source.setOptions(options);
  source.ids.push_back(id);
  source.track_cnt.push_back(3);
  source.prev_img = cv::Mat(2, 2, CV_8UC1, cv::Scalar(pixel));
  source.cur_img = cv::Mat(2, 2, CV_8UC1, cv::Scalar(pixel + 1));
  source.n_id = id + 1;
  return source.cloneWithOptions(
      std::make_shared<VINSOptions>(*options));
}

bool trackerCloneIsDeep() {
  FeatureTracker source;
  auto options = std::make_shared<VINSOptions>();
  source.setOptions(options);
  source.prev_img = cv::Mat(2, 2, CV_8UC1, cv::Scalar(7));
  source.cur_img = cv::Mat(2, 2, CV_8UC1, cv::Scalar(8));
  source.ids = {4, 5};
  source.track_cnt = {2, 3};
  source.prev_pts = {{1.0F, 2.0F}};
  source.n_id = 9;
  auto clone = source.cloneWithOptions(
      std::make_shared<VINSOptions>(*options));
  if (!expect(clone != nullptr, "tracker clone exists") ||
      !expect(clone->prev_img.data != source.prev_img.data,
              "previous image storage is deep-cloned") ||
      !expect(clone->cur_img.data != source.cur_img.data,
              "current image storage is deep-cloned") ||
      !expect(clone->ids == source.ids &&
                  clone->track_cnt == source.track_cnt &&
                  clone->n_id == source.n_id,
              "tracker identity and age state are preserved")) {
    return false;
  }
  clone->prev_img.at<unsigned char>(0, 0) = 99;
  clone->ids[0] = 77;
  return expect(source.prev_img.at<unsigned char>(0, 0) == 7,
                "clone image mutation cannot reach source") &&
         expect(source.ids[0] == 4,
                "clone feature-id mutation cannot reach source");
}

bool rewindEscalatesToOlderCheckpoint() {
  vins::BackendRecoveryCheckpointStore store(3);
  std::string error;
  auto backend100 = backend(1, 1.0);
  auto tracker100 = tracker(10, 10);
  bool pushed = store.push(100, 200, 10.0, std::move(backend100),
                           std::move(tracker100), &error);
  if (!pushed) {
    std::cerr << error << '\n';
    return false;
  }
  pushed = store.push(110, 220, 11.0, backend(2, 2.0),
                      tracker(20, 20), &error);
  if (!pushed) {
    std::cerr << error << '\n';
    return false;
  }
  pushed = store.push(120, 240, 12.0, backend(3, 3.0),
                      tracker(30, 30), &error);
  if (!pushed) {
    std::cerr << error << '\n';
    return false;
  }
  const auto *selected = store.selectForRewind(130, 5);
  if (!expect(selected && selected->feature_sequence == 120,
              "newest checkpoint outside rewind margin is selected") ||
      !expect(selected->backend->states[0].position.x() == 3.0,
              "selected checkpoint owns its complete backend") ||
      !expect(selected->tracker->ids.front() == 30,
              "selected checkpoint owns synchronized tracker") ||
      !expect(store.markAttempted(120),
              "first checkpoint attempt is recorded")) {
    return false;
  }
  selected = store.selectForRewind(130, 5);
  if (!expect(selected && selected->feature_sequence == 110,
              "failed recovery moves strictly older") ||
      !expect(store.markAttempted(110),
              "second checkpoint attempt is recorded")) {
    return false;
  }
  selected = store.selectForRewind(130, 5);
  if (!expect(selected && selected->feature_sequence == 100,
              "third recovery reaches oldest retained checkpoint") ||
      !expect(store.markAttempted(100),
              "oldest checkpoint attempt is recorded") ||
      !expect(store.selectForRewind(130, 5) == nullptr,
              "identical checkpoint tuple is never retried")) {
    return false;
  }

  return expect(!store.push(119, 250, 13.0, backend(4, 4.0),
                            tracker(40, 40), &error),
                "sequence regression fails closed");
}

ImageData image(std::uint64_t sequence, double timestamp,
                unsigned char pixel) {
  ImageData result;
  result.sequence = sequence;
  result.timestamp = timestamp;
  result.image0 = cv::Mat(2, 2, CV_8UC1, cv::Scalar(pixel));
  return result;
}

IMUData imu(double timestamp) {
  IMUData result;
  result.timestamp = timestamp;
  result.linear_acceleration.x() = timestamp;
  return result;
}

RangeData range(double timestamp) {
  RangeData result;
  result.timestamp = timestamp;
  result.range = timestamp;
  result.valid = true;
  return result;
}

bool rawSensorRingPreservesReplayBoundary() {
  vins::BackendRecoverySensorRing ring;
  if (!expect(ring.appendImage(image(1, 1.0, 1)),
              "first raw image is retained") ||
      !expect(ring.appendImage(image(2, 2.0, 2)),
              "second raw image is retained") ||
      !expect(!ring.appendImage(image(2, 3.0, 3)),
              "duplicate raw image sequence is rejected") ||
      !expect(ring.appendImu(imu(0.9)) &&
                  ring.appendImu(imu(1.1)) &&
                  ring.appendImu(imu(1.9)) &&
                  ring.appendImu(imu(2.1)),
              "ordered IMU ring is retained") ||
      !expect(!ring.appendImu(imu(2.1)),
              "duplicate IMU timestamp is rejected") ||
      !expect(ring.appendRange(range(1.0)) &&
                  ring.appendRange(range(2.0)),
              "ordered range ring is retained")) {
    return false;
  }
  ring.trimBefore(1.5);
  if (!expect(ring.imageCount() == 1,
              "images older than checkpoint are released") ||
      !expect(ring.imuCount() == 3,
              "last pre-boundary IMU is retained for interpolation") ||
      !expect(ring.rangeCount() == 1,
              "old range samples are released")) {
    return false;
  }
  auto slice = ring.slice(1.5, 2.1);
  if (!expect(slice.images.size() == 1 &&
                  slice.images.front().sequence == 2,
              "camera replay slice preserves raw identity") ||
      !expect(slice.imu.size() == 2 &&
                  slice.imu.front().timestamp == 1.9 &&
                  slice.imu.back().timestamp == 2.1,
              "IMU replay slice preserves exact order") ||
      !expect(slice.ranges.size() == 1 &&
                  slice.ranges.front().timestamp == 2.0,
              "range replay slice preserves timestamp")) {
    return false;
  }
  slice.images.front().image0.at<unsigned char>(0, 0) = 99;
  auto second_slice = ring.slice(1.5, 2.1);
  return expect(
      second_slice.images.front().image0.at<unsigned char>(0, 0) == 2,
      "returned raw image slice cannot mutate ring storage");
}

bool capacityDropsOldestCheckpoint() {
  vins::BackendRecoveryCheckpointStore store(2);
  std::string error;
  if (!store.push(1, 2, 1.0, backend(1, 1.0), tracker(1, 1), &error) ||
      !store.push(2, 4, 2.0, backend(2, 2.0), tracker(2, 2), &error) ||
      !store.push(3, 6, 3.0, backend(3, 3.0), tracker(3, 3), &error)) {
    return expect(false, error.c_str());
  }
  return expect(store.size() == 2,
                "checkpoint capacity is bounded") &&
         expect(store.oldestTimestamp() == 2.0,
                "capacity eviction releases oldest checkpoint");
}

}  // namespace

int main() {
  if (!trackerCloneIsDeep()) return EXIT_FAILURE;
  if (!rewindEscalatesToOlderCheckpoint()) return EXIT_FAILURE;
  if (!rawSensorRingPreservesReplayBoundary()) return EXIT_FAILURE;
  if (!capacityDropsOldestCheckpoint()) {
    return EXIT_FAILURE;
  }
  std::cout << "backend_recovery_checkpoint_store_test: PASS\n";
  return EXIT_SUCCESS;
}
