/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include <vins/estimator/estimator.h>
#include <vins/estimator/stationary_evidence_gate.h>
#include <vins/estimator/trusted_metric_correction_gate.h>
#include <vins/factor/horizontal_speed_upper_bound_factor.h>
#include <vins/factor/rotor_drag_dynamics_factor.h>
#include "vins_fusion_ros2/motor_factor_policy.h"
#include <vins/factor/gyro_bias_z_prior_factor.h>
#include <vins/factor/pose_only_visual_factor.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <sys/stat.h>
#include <sstream>
#include <tuple>

namespace {
constexpr double kMaxAccelBiasNorm = 2.5;
constexpr double kMaxGyroBiasNorm = 1.0;

uint64_t featureFingerprint(const FeatureFrame &features) {
  uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](const void *data, size_t size) {
    const auto *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ULL;
    }
  };
  for (const auto &feature : features) {
    mix(&feature.first, sizeof(feature.first));
    for (const auto &observation : feature.second) {
      mix(&observation.first, sizeof(observation.first));
      for (int i = 0; i < observation.second.size(); ++i) {
        const double value = observation.second(i);
        mix(&value, sizeof(value));
      }
    }
  }
  return hash;
}

uint64_t fnvFingerprint(const void *data, size_t size,
                        uint64_t hash = 1469598103934665603ULL) {
  const auto *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

uint64_t imuIntervalFingerprint(const std::vector<IMUData> &values) {
  uint64_t hash = 1469598103934665603ULL;
  for (const auto &value : values) {
    hash = fnvFingerprint(&value.source_sequence,
                          sizeof(value.source_sequence), hash);
    hash = fnvFingerprint(&value.timestamp, sizeof(value.timestamp), hash);
    hash = fnvFingerprint(&value.payload_fingerprint,
                          sizeof(value.payload_fingerprint), hash);
  }
  return hash;
}

uint64_t stateWindowFingerprint(const StateData *states, int frame_count) {
  uint64_t hash = 1469598103934665603ULL;
  for (int index = 0; index <= frame_count; ++index) {
    const StateData &state = states[index];
    hash = fnvFingerprint(&state.timestamp, sizeof(state.timestamp), hash);
    hash = fnvFingerprint(state.position.data(), 3 * sizeof(double), hash);
    hash = fnvFingerprint(state.velocity.data(), 3 * sizeof(double), hash);
    hash = fnvFingerprint(state.rotation.data(), 9 * sizeof(double), hash);
    hash = fnvFingerprint(state.accel_bias.data(), 3 * sizeof(double), hash);
    hash = fnvFingerprint(state.gyro_bias.data(), 3 * sizeof(double), hash);
  }
  return hash;
}

uint64_t priorFingerprint(const MarginalizationInfo::Ptr &prior) {
  // Timing-free diagnostic hash; pointer values and runtime timing are
  // intentionally excluded from this representation.
  if (!prior) return 0;
  uint64_t hash = 1469598103934665603ULL;
  hash = fnvFingerprint(&prior->valid, sizeof(prior->valid), hash);
  hash = fnvFingerprint(&prior->m, sizeof(prior->m), hash);
  hash = fnvFingerprint(&prior->n, sizeof(prior->n), hash);
  if (prior->linearized_jacobians.size() > 0) {
    hash = fnvFingerprint(prior->linearized_jacobians.data(),
                          prior->linearized_jacobians.size() * sizeof(double),
                          hash);
  }
  if (prior->linearized_residuals.size() > 0) {
    hash = fnvFingerprint(prior->linearized_residuals.data(),
                          prior->linearized_residuals.size() * sizeof(double),
                          hash);
  }
  for (std::size_t index = 0; index < prior->keep_block_size.size(); ++index) {
    const int size = prior->keep_block_size[index];
    const int local_index = prior->keep_block_idx[index];
    hash = fnvFingerprint(&size, sizeof(size), hash);
    hash = fnvFingerprint(&local_index, sizeof(local_index), hash);
    if (index < prior->keep_block_data.size() &&
        prior->keep_block_data[index] && size > 0) {
      hash = fnvFingerprint(prior->keep_block_data[index],
                            static_cast<std::size_t>(size) * sizeof(double),
                            hash);
    }
  }
  return hash;
}

std::vector<double> observabilityExportTimestamps() {
  std::vector<double> result;
  const char *value = std::getenv("VINS_OBSERVABILITY_EXPORT_TIMESTAMPS");
  if (!value || !*value) return result;
  std::istringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    std::size_t consumed = 0;
    const double timestamp = std::stod(item, &consumed);
    if (consumed != item.size() || !std::isfinite(timestamp)) {
      throw std::runtime_error(
          "invalid VINS_OBSERVABILITY_EXPORT_TIMESTAMPS value");
    }
    result.push_back(timestamp);
  }
  return result;
}

bool shouldExportObservability(double timestamp) {
  static const std::vector<double> targets = observabilityExportTimestamps();
  for (const double target : targets) {
    if (std::abs(timestamp - target) <= 0.005) return true;
  }
  return false;
}

bool shouldExportMarginalizationTransport(double timestamp) {
  const char *start_value =
      std::getenv("VINS_MARGINALIZATION_TRANSPORT_START_TIMESTAMP");
  const char *end_value =
      std::getenv("VINS_MARGINALIZATION_TRANSPORT_END_TIMESTAMP");
  if ((!start_value || !*start_value) && (!end_value || !*end_value)) {
    return false;
  }
  if (!start_value || !*start_value || !end_value || !*end_value) {
    throw std::runtime_error(
        "both marginalization transport timestamps are required");
  }
  std::size_t start_consumed = 0;
  std::size_t end_consumed = 0;
  const double start = std::stod(start_value, &start_consumed);
  const double end = std::stod(end_value, &end_consumed);
  if (start_consumed != std::strlen(start_value) ||
      end_consumed != std::strlen(end_value) || !std::isfinite(start) ||
      !std::isfinite(end) || end < start) {
    throw std::runtime_error("invalid marginalization transport range");
  }
  return timestamp >= start && timestamp <= end;
}

bool shouldRunPreCommitConsistencyAudit(double timestamp) {
  const char *start_value =
      std::getenv("VINS_PRECOMMIT_AUDIT_START_TIMESTAMP");
  const char *end_value =
      std::getenv("VINS_PRECOMMIT_AUDIT_END_TIMESTAMP");
  if ((!start_value || !*start_value) && (!end_value || !*end_value)) {
    return false;
  }
  if (!start_value || !*start_value || !end_value || !*end_value) {
    throw std::runtime_error(
        "both pre-commit audit timestamps are required");
  }
  std::size_t start_consumed = 0;
  std::size_t end_consumed = 0;
  const double start = std::stod(start_value, &start_consumed);
  const double end = std::stod(end_value, &end_consumed);
  if (start_consumed != std::strlen(start_value) ||
      end_consumed != std::strlen(end_value) || !std::isfinite(start) ||
      !std::isfinite(end) || end < start) {
    throw std::runtime_error("invalid pre-commit audit range");
  }
  return timestamp >= start && timestamp <= end;
}

bool shouldForceReplayOldMarginalization(double timestamp) {
  const char *start_value =
      std::getenv("VINS_REPLAY_FORCE_OLD_MARGINALIZATION_START_TIMESTAMP");
  const char *end_value =
      std::getenv("VINS_REPLAY_FORCE_OLD_MARGINALIZATION_END_TIMESTAMP");
  if ((!start_value || !*start_value) && (!end_value || !*end_value)) {
    return false;
  }
  if (!start_value || !*start_value || !end_value || !*end_value) {
    throw std::runtime_error(
        "both replay force-old marginalization timestamps are required");
  }
  std::size_t start_consumed = 0;
  std::size_t end_consumed = 0;
  const double start = std::stod(start_value, &start_consumed);
  const double end = std::stod(end_value, &end_consumed);
  if (start_consumed != std::strlen(start_value) ||
      end_consumed != std::strlen(end_value) || !std::isfinite(start) ||
      !std::isfinite(end) || end < start) {
    throw std::runtime_error(
        "invalid replay force-old marginalization range");
  }
  return timestamp >= start - 0.005 && timestamp <= end + 0.005;
}

struct ReplayGtInterventionConfig {
  double target = std::numeric_limits<double>::quiet_NaN();
  std::set<std::string> families;
  std::string family_text;
};

const ReplayGtInterventionConfig &replayGtInterventionConfig() {
  static const ReplayGtInterventionConfig config = []() {
    ReplayGtInterventionConfig result;
    const char *target_value =
        std::getenv("VINS_REPLAY_GT_INTERVENTION_TARGET_TIMESTAMP");
    const char *families_value =
        std::getenv("VINS_REPLAY_GT_INTERVENTION_FAMILIES");
    if ((!target_value || !*target_value) &&
        (!families_value || !*families_value)) {
      return result;
    }
    if (!target_value || !*target_value || !families_value || !*families_value) {
      throw std::runtime_error(
          "both replay GT intervention target and families are required");
    }
    std::size_t consumed = 0;
    result.target = std::stod(target_value, &consumed);
    if (consumed != std::strlen(target_value) ||
        !std::isfinite(result.target)) {
      throw std::runtime_error("invalid replay GT intervention target");
    }
    result.family_text = families_value;
    std::istringstream stream(result.family_text);
    std::string family;
    const std::set<std::string> allowed{
        "accel_bias", "velocity", "roll_pitch", "landmark_depth",
        "imu_preintegration", "orientation", "gyro_bias", "position",
        "full"};
    while (std::getline(stream, family, ',')) {
      if (family.empty() || !allowed.count(family) ||
          !result.families.insert(family).second) {
        throw std::runtime_error("invalid or duplicate GT intervention family");
      }
    }
    if (result.families.empty()) {
      throw std::runtime_error("empty replay GT intervention family set");
    }
    return result;
  }();
  return config;
}

const std::deque<OdomData> &replayGtInterventionGroundTruth() {
  static const std::deque<OdomData> history = []() {
    std::deque<OdomData> result;
    const char *path =
        std::getenv("VINS_REPLAY_GT_INTERVENTION_GROUND_TRUTH_FILE");
    if (!path || !*path) return result;
    std::ifstream stream(path);
    if (!stream) {
      throw std::runtime_error("cannot open intervention ground-truth CSV");
    }
    std::string line;
    if (!std::getline(stream, line)) {
      throw std::runtime_error("missing intervention ground-truth header");
    }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "timestamp_s,px,py,pz,qx,qy,qz,qw") {
      throw std::runtime_error("invalid intervention ground-truth header");
    }
    int line_number = 1;
    while (std::getline(stream, line)) {
      ++line_number;
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      OdomData sample;
      double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
      if (!(row >> sample.timestamp >> sample.position.x() >>
            sample.position.y() >> sample.position.z() >> qx >> qy >> qz >>
            qw) || !row.eof() || !std::isfinite(sample.timestamp) ||
          !sample.position.allFinite()) {
        throw std::runtime_error(
            "invalid intervention ground-truth row " +
            std::to_string(line_number));
      }
      sample.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
      if (!sample.orientation.coeffs().allFinite() ||
          (!result.empty() && sample.timestamp <= result.back().timestamp)) {
        throw std::runtime_error(
            "invalid intervention ground-truth order or quaternion");
      }
      result.push_back(sample);
    }
    if (result.size() < 2) {
      throw std::runtime_error("empty intervention ground-truth CSV");
    }
    return result;
  }();
  return history;
}

struct ReplayVisualMeasurement {
  Eigen::Vector3d point;
  Eigen::Vector2d velocity;
};

using ReplayVisualMeasurementKey = std::tuple<int64_t, int, int>;

int64_t replayVisualTimestampKey(double timestamp) {
  return static_cast<int64_t>(std::llround(timestamp * 1e9));
}

double replayVisualGtTargetTimestamp() {
  const char *target_value =
      std::getenv("VINS_REPLAY_VISUAL_GT_TARGET_TIMESTAMP");
  const char *file_value =
      std::getenv("VINS_REPLAY_VISUAL_GT_MEASUREMENT_FILE");
  if ((!target_value || !*target_value) && (!file_value || !*file_value)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!target_value || !*target_value || !file_value || !*file_value) {
    throw std::runtime_error(
        "both replay visual-GT target and measurement file are required");
  }
  std::size_t consumed = 0;
  const double target = std::stod(target_value, &consumed);
  if (consumed != std::strlen(target_value) || !std::isfinite(target)) {
    throw std::runtime_error("invalid replay visual-GT target timestamp");
  }
  return target;
}

const std::map<ReplayVisualMeasurementKey, ReplayVisualMeasurement> &
replayVisualGtMeasurements() {
  static const auto measurements = []() {
    std::map<ReplayVisualMeasurementKey, ReplayVisualMeasurement> result;
    const char *path =
        std::getenv("VINS_REPLAY_VISUAL_GT_MEASUREMENT_FILE");
    if (!path || !*path) return result;
    std::ifstream stream(path);
    if (!stream) {
      throw std::runtime_error("cannot open replay visual-GT measurement file");
    }
    std::string line;
    if (!std::getline(stream, line)) {
      throw std::runtime_error("missing replay visual-GT CSV header");
    }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line !=
        "timestamp_s,feature_id,camera_id,x,y,z,velocity_x,velocity_y") {
      throw std::runtime_error("invalid replay visual-GT CSV header");
    }
    int line_number = 1;
    while (std::getline(stream, line)) {
      ++line_number;
      if (line.empty()) continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      double timestamp = 0.0;
      int feature_id = 0;
      int camera_id = 0;
      ReplayVisualMeasurement measurement;
      if (!(row >> timestamp >> feature_id >> camera_id
                >> measurement.point.x() >> measurement.point.y()
                >> measurement.point.z() >> measurement.velocity.x()
                >> measurement.velocity.y()) ||
          !row.eof() || !std::isfinite(timestamp) ||
          !measurement.point.allFinite() ||
          !measurement.velocity.allFinite() ||
          (camera_id != 0 && camera_id != 1)) {
        throw std::runtime_error(
            "invalid replay visual-GT CSV row " +
            std::to_string(line_number));
      }
      const ReplayVisualMeasurementKey key{
          replayVisualTimestampKey(timestamp), feature_id, camera_id};
      if (!result.emplace(key, measurement).second) {
        throw std::runtime_error(
            "duplicate replay visual-GT CSV key at row " +
            std::to_string(line_number));
      }
    }
    if (result.empty()) {
      throw std::runtime_error("empty replay visual-GT measurement file");
    }
    return result;
  }();
  return measurements;
}

void writeDenseCsv(const std::string &path, const Eigen::MatrixXd &matrix) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write " + path);
  stream << std::setprecision(17);
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int column = 0; column < matrix.cols(); ++column) {
      if (column) stream << ',';
      stream << matrix(row, column);
    }
    stream << '\n';
  }
}

void writeVectorCsv(const std::string &path, const Eigen::VectorXd &vector) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write " + path);
  stream << std::setprecision(17) << "index,value\n";
  for (int i = 0; i < vector.size(); ++i) stream << i << ',' << vector(i) << '\n';
}
}  // namespace

Estimator::Estimator() {
  const char *ledger_path = std::getenv("VINS_REPLAY_ROTOR_FACTOR_LEDGER");
  if (ledger_path && ledger_path[0] != '\0') {
    replayRotorFactorLedger.open(
        ledger_path, std::ios::out | std::ios::trunc);
    if (!replayRotorFactorLedger.is_open())
      throw std::runtime_error(
          "failed to open VINS_REPLAY_ROTOR_FACTOR_LEDGER output");
    replayRotorFactorLedger
        << "solve_sequence,current_timestamp_ns,frame_count,factor_count\n";
  }
  const char *factor_timeline = std::getenv("VINS_MOTOR_FACTOR_TIMELINE");
  if (factor_timeline && factor_timeline[0] != '\0') {
    motorFactorTimeline.open(
        factor_timeline, std::ios::out | std::ios::trunc);
    if (!motorFactorTimeline.is_open())
      throw std::runtime_error("failed to open VINS_MOTOR_FACTOR_TIMELINE");
  }
}

Estimator::~Estimator() {
  if (isRunning.load()) {
    isRunning.store(false);
    imuCondition.notify_all();
    featureCondition.notify_all();
    if (processThread.joinable()) {
      processThread.join();
    }
  }
  if (replayRotorFactorLedger.is_open()) replayRotorFactorLedger.close();
  if (motorFactorTimeline.is_open()) motorFactorTimeline.close();
}

void Estimator::resetState() {
  std::lock_guard<std::mutex> lock(processingMutex);
  resetStateLocked();
}

void Estimator::restart() {
  std::lock_guard<std::mutex> lock(processingMutex);
  resetStateLocked();
  initializeCamerasFromOptions();
}

void Estimator::resetStateLocked(bool request_warm_reinitialization) {
  // A failed speculative solve must not clear global sensor queues or the
  // protected committed epoch.  The processing loop will discard this live
  // workspace and restore the owning committed BackendState.
  if (backendCandidateWorkspaceActive) {
    backendCandidateProcessingFailed = true;
    VINS_ERROR << "Candidate backend requested reset; deferring to "
                  "generation discard.";
    return;
  }
  bool warm_reinitialization_started = false;
  if (request_warm_reinitialization && replayWarmReinitialization &&
      solver_flag == SolverState::NON_LINEAR &&
      latest_state.position.allFinite() &&
      latest_state.velocity.allFinite() &&
      latest_state.accel_bias.allFinite() &&
      latest_state.gyro_bias.allFinite() && latest_Q.coeffs().allFinite() &&
      std::isfinite(latestImuData.timestamp)) {
    OdomData anchor;
    anchor.timestamp = latestImuData.timestamp;
    anchor.position = latest_state.position;
    anchor.velocity = latest_state.velocity;
    anchor.orientation = latest_Q.normalized();
    warm_reinitialization_started = warmReinitializationBridge.begin(
        anchor, latestImuData, latest_state.accel_bias,
        latest_state.gyro_bias, gravity, 5.0);
  }
  if (!warm_reinitialization_started) {
    warmReinitializationBridge.clear();
  }
  motionOnsetFrameHistory.clear();
  {
    std::lock_guard<std::mutex> history_lock(
        motionOnsetImuHistoryMutex);
    motionOnsetImuHistory.clear();
  }
  replayMotionOnsetWarmReinitializationConsumed = false;
  motionOnsetBiasAnchorThisSolve = false;
  motionOnsetBiasAnchorActive = false;
  motionOnsetReleasePending = false;
  motionOnsetReleaseThisSolve = false;
  motionOnsetBiasAnchorSolveCount = 0;
  motionOnsetExcitingMotionCount = 0;
  // IMU propagation runs directly from the subscription callback.  Serialize
  // it with reset so it cannot dereference a half-cleared latest state.
  std::lock_guard<std::mutex> propagate_lock(propagateMutex);
  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex);
    clearBuffer(imuBuffer);
  }
  {
    std::lock_guard<std::mutex> feature_lock(featureBufferMutex);
    clearBuffer(featureBuffer);
  }
  {
    std::lock_guard<std::mutex> range_lock(rangeMutex);
    clearBuffer(rangeBuffer);
  }
  {
    std::lock_guard<std::mutex> velocity_lock(metricVelocityMutex);
    clearBuffer(metricVelocityBuffer);
  }
  {
    std::lock_guard<std::mutex> dynamics_lock(rotorDynamicsMutex);
    clearBuffer(rotorDynamicsBuffer);
  }
  previousTimestamp = -1;
  currentTimestamp = 0;
  openExEstimation = 0;
  inputImageCount.store(0);
  lastEnqueuedImageTimestamp =
      -std::numeric_limits<double>::infinity();
  isFirstPoseInitialized = false;

  for (int i = 0; i < WINDOW_SIZE + 1; i++) {
    estimator_state[i].clear();
    std::vector<IMUData>().swap(deltaTimeImuBuffer[i]);
    pre_integrations[i] = nullptr;
    rangeMeasurements[i] = RangeData();
    metricVelocityMeasurements[i] = MetricVelocityData();
    rotorDynamicsMeasurements[i] = RotorDynamicsData();
  }

  for (int i = 0; i < options->getNumCameras(); i++) {
    cameraTranslation[i] = Vector3d::Zero();
    cameraRotation[i] = Matrix3d::Identity();
    updateCameraPose(i);
  }

  isFirstIMUReceived = false, backCount = 0;
  frontCount = 0;
  frameCount = 0;
  solver_flag = SolverState::INITIAL;
  initialTimestamp = 0;
  std::map<double, ImageFrame>().swap(all_image_frame);
  tmp_pre_integration = nullptr;
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();

  featureManager.clearState();
  metricVelocityFactorActivationLogged = false;
  gpsDropoutSpeedFactorActivationLogged = false;
  gpsDropoutGyroBiasPriorActivationLogged = false;
  rotorDragDynamicsFactorActivationLogged = false;
  poseOnlyVisualFactorActivationLogged = false;
  metricPositionFactorActiveThisSolve = false;
  metricDisplacementAnchorInitialized = false;
  metricDisplacementAnchorSegment = 0;
  metricDisplacementAnchorMeasurement.setZero();
  metricDisplacementAnchorPosition.setZero();
  gpsDropoutGyroBiasAnchorInitialized = false;
  gpsDropoutGyroBiasAnchorZ = 0.0;
  {
    std::lock_guard<std::mutex> tracker_lock(featureTrackerMutex);
    featureTracker.reset();
  }
  if (!warm_reinitialization_started) {
    safe_vio_odom.clear();
    safe_imu_pre_odom.clear();
  }
  safe_key_poses.clear();
  safe_main_cloud.clear();
  safe_margin_cloud.clear();
  safe_point_cloud.clear();
  safe_keyframe_pose.clear();
  failure_occur = 0;
  nonLinearUpdateCount = 0;
  lastSolvedFeatureCount = 0;
  lastSolvedParametersValid = false;
  replayQuarantineUpdatesRemaining = 0;
  replayQuarantineConsecutiveHealthy = 0;
  preCommitConsistencyAuditShadowActive = false;
  auditTriggeredHistoryExclusionActive = false;
  auditTriggeredHistoryExclusionConsumed = false;
  auditTriggeredDropAllVisualActive = false;
  auditTriggeredTransactionRejectConsumed = false;
  auditTriggeredQuarantineRejectConsumed = false;
  auditTriggeredPriorInformationCapConsumed = false;
  auditTriggeredRecentHistoryRetryConsumed = false;
  auditTriggeredRelinearizedHistoryRetryConsumed = false;
  auditTriggeredMarginOldDelayCount = 0;
  lastPreCommitAuditSuspect = false;
  previousPreCommitAuditAvbaInformationRatio =
      std::numeric_limits<double>::quiet_NaN();
  rangeFactorActivationLogged = false;
  rangeGroundInitialized = false;
  rangeGroundZ = 0.0;
  currentAlignmentScale = -1.0;
  currentFeatureFlowMedianPxS = std::numeric_limits<double>::infinity();
  currentFeatureFlowCount = 0;
  stationaryVisualUpdateCount = 0;
  consecutivePositionInnovationRejects = 0;
  lastMarginalizationSourceTimestamp =
      std::numeric_limits<double>::quiet_NaN();
  lastMarginalizationSourceType = "none";
  backendGeneration = 0;
  backendCandidateStore.reset();
  backendCandidateInputBuffer.reset();
  backendCandidateProcessingFailed = false;
  lastBackendHealthAvailable = false;
  lastBackendSolverUsable = false;
  lastBackendTransactionRejected = false;
  {
    std::lock_guard<std::mutex> recovery_lock(backendRecoveryMutex);
    backendRecoveryCheckpoints.clear();
    backendRecoverySensorRing.clear();
    lastBackendRecoveryCheckpointSequence = 0;
    backendRecoveryFirstSuspectSequence = 0;
    backendRecoveryFirstSuspectRawImageSequence = 0;
    backendRecoverySelectionLogged = false;
    backendRecoveryEvaluationComplete = false;
  }
  {
    std::lock_guard<std::mutex> feature_lock(featureBufferMutex);
    backendFeatureTrackerSnapshots.clear();
    backendFeatureRawImageSequences.clear();
  }
  VINS_INFO << "reset state successfully; warm_reinitialization="
            << warm_reinitialization_started;
}

void Estimator::initialize(std::shared_ptr<VINSOptions> options_) {
  options = options_;

  // resetState() initializes several scalar flags and pre-integration
  // pointers.  Starting the processing thread without it leaves those values
  // indeterminate and can cause intermittent SIGSEGV/SIGABRT on first data.
  resetState();

  std::lock_guard<std::mutex> lock(processingMutex);
  initializeCamerasFromOptions();
  isRunning.store(true);
  processThread = std::thread(&Estimator::processMeasurements, this);
}

void Estimator::setDeterministicCommitOutputCallback(
    std::function<void(const OdomData &)> callback) {
  std::lock_guard<std::mutex> lock(deterministicCommitOutputMutex);
  deterministicCommitOutputCallback = std::move(callback);
}

void Estimator::emitDeterministicCommitOutput() {
  if (solver_flag != SolverState::NON_LINEAR || nonLinearUpdateCount < 3 ||
      frameCount < 0 || frameCount > WINDOW_SIZE) {
    return;
  }
  OdomData output;
  output.timestamp = estimator_state[frameCount].timestamp;
  output.position = estimator_state[frameCount].position;
  output.orientation = Quaterniond(estimator_state[frameCount].rotation);
  output.velocity = estimator_state[frameCount].velocity;
  std::lock_guard<std::mutex> lock(deterministicCommitOutputMutex);
  if (deterministicCommitOutputCallback) {
    deterministicCommitOutputCallback(output);
  }
}

void Estimator::initializeCamerasFromOptions() {
  if (backendCandidateWorkspaceActive &&
      backendCandidateProcessingFailed) {
    return;
  }
  for (int i = 0; i < options->getNumCameras(); i++) {
    cameraTranslation[i] = options->TIC[i];
    cameraRotation[i] = options->RIC[i];
    updateCameraPose(i);
    VINS_INFO << " exitrinsic cam[" << i << "]: \n"
              << cameraRotation[i] << "\n"
              << cameraTranslation[i].transpose();
  }
  featureManager.setOptions(options);
  featureTracker.setOptions(options);
  ProjectionTwoFrameOneCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionTwoFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  ProjectionOneFrameTwoCamFactor::sqrt_info =
      FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
  gravity = options->imu.gravity();
  featureTracker.readIntrinsicParameter(options->camera_names);
}

void Estimator::inputImage(const ImageData &image) {
  const int input_image_count = inputImageCount.fetch_add(1) + 1;
  FeatureFrame featureFrame;
  std::shared_ptr<const FeatureTracker> tracker_snapshot;
  TicToc featureTrackerTime;

  {
    std::lock_guard<std::mutex> tracker_lock(featureTrackerMutex);
    if (image.image1.empty()) {
      featureFrame = featureTracker.trackImage(image.timestamp, image.image0);
    } else {
      featureFrame =
          featureTracker.trackImage(image.timestamp, image.image0, image.image1);
    }
    if (options->shouldShowTrack()) {
      track_image.image0 = featureTracker.getTrackImage();
      track_image.timestamp = image.timestamp;
      safe_track_image.set(track_image);
    }
    if (replayMultiCheckpointRecovery) {
      auto cloned_options =
          std::make_shared<VINSOptions>(*options);
      auto cloned_tracker =
          featureTracker.cloneWithOptions(cloned_options);
      tracker_snapshot =
          std::shared_ptr<const FeatureTracker>(
              std::move(cloned_tracker));
    }
  }
  if (replayMultiCheckpointRecovery) {
    std::lock_guard<std::mutex> recovery_lock(backendRecoveryMutex);
    if (!backendRecoverySensorRing.appendImage(image)) {
      VINS_ERROR << "Multi-checkpoint raw image ring protocol failed: "
                 << "sequence=" << image.sequence
                 << ", timestamp=" << image.timestamp;
      replayMultiCheckpointRecovery = false;
    }
  }

  // The D435i Stage-B sensor runs at 20 Hz. A two-frame cadence gives the
  // stereo backend a deterministic 10 Hz budget while keeping enough temporal
  // overlap for the 920 m endurance route. The previous 25/3 Hz candidate
  // stayed within its compute budget but accumulated large long-run
  // visual/inertial innovations before a fail-closed reset.
  constexpr std::uint64_t kInputDownsampleDivisor = 2;
  const bool scheduled_phase =
      input_image_count % kInputDownsampleDivisor ==
      static_cast<std::uint64_t>(inputDownsamplePhase);
  const double time_since_enqueue =
      image.timestamp - lastEnqueuedImageTimestamp;
  const bool scheduled_enqueue =
      scheduled_phase &&
      (!std::isfinite(lastEnqueuedImageTimestamp) ||
       time_since_enqueue >= 0.060);
  // Renderer jitter can otherwise turn one missing 25 Hz frame into a
  // 160--180 ms backend gap. Insert a catch-up frame before the strict
  // 150 ms boundary without returning to the overloaded 12.5 Hz cadence.
  const bool gap_catchup_enqueue =
      std::isfinite(lastEnqueuedImageTimestamp) &&
      time_since_enqueue >= 0.105;
  const bool should_enqueue = scheduled_enqueue || gap_catchup_enqueue;
  size_t queue_depth_after_push = 0;
  {
    std::lock_guard<std::mutex> lock(featureBufferMutex);
    // Downsample by callback sequence only: the former
    // "even frame or queue empty" rule depended on processing-thread timing,
    // while enqueueing every frame caused live backlog.  This fixed rule is
    // deterministic for a given input/reset boundary and independent of queue
    // scheduling.
    if (should_enqueue) {
      lastEnqueuedImageTimestamp = image.timestamp;
      const std::uint64_t sequence =
          featureInputSequence.fetch_add(1) + 1;
      featureBuffer.push(
          TimestampedFeatureFrame{sequence, image.sequence, image.timestamp,
                                  featureFrame});
      if (tracker_snapshot) {
        backendFeatureTrackerSnapshots.emplace(
            sequence, std::move(tracker_snapshot));
        backendFeatureRawImageSequences.emplace(
            sequence, image.sequence);
      }
    }
    queue_depth_after_push = featureBuffer.size();
  }
  VINS_INFO << "Camera estimator trace: sequence=" << image.sequence
            << ", input_count=" << input_image_count << ", timestamp="
            << std::fixed << std::setprecision(9) << image.timestamp
            << ", features=" << featureFrame.size()
            << ", feature_fingerprint=0x" << std::hex
            << featureFingerprint(featureFrame) << std::dec
            << ", raw_image_sequence=" << image.sequence
            << ", left_fingerprint=0x" << std::hex
            << image.left_fingerprint << ", right_fingerprint=0x"
            << image.right_fingerprint << std::dec
            << ", enqueued=" << should_enqueue
            << ", downsample_phase=" << inputDownsamplePhase
            << ", queue_depth_after_push=" << queue_depth_after_push;
  if (should_enqueue) featureCondition.notify_one();
}

void Estimator::setInputDownsamplePhase(int phase) {
  inputDownsamplePhase = phase == 0 ? 0 : 1;
}

void Estimator::setReplayPositionJumpQuarantine(bool enabled) {
  replayPositionJumpQuarantine = enabled;
  replayQuarantineUpdatesRemaining = 0;
  replayQuarantineConsecutiveHealthy = 0;
  if (enabled) {
    VINS_WARN << "Replay-only position-jump quarantine enabled; this mode "
                 "must not be used for live flight.";
  }
}

void Estimator::setReplayJumpFallbackIterations(int iterations) {
  replayJumpFallbackIterations = std::max(0, iterations);
  if (replayJumpFallbackIterations > 0) {
    VINS_WARN << "Replay-only jump fallback enabled: max_iterations="
              << replayJumpFallbackIterations
              << "; this mode must not be used for live flight.";
  }
}

void Estimator::setReplayJumpTransactionReject(bool enabled) {
  replayJumpTransactionReject = enabled;
  replayJumpTransactionRejectConsumed = false;
  if (enabled) {
    VINS_WARN << "Replay-only jump transaction reject enabled; this mode "
                 "must not be used for live flight.";
  }
}

void Estimator::setReplayJumpPriorReset(bool enabled) {
  replayJumpPriorReset = enabled;
  if (enabled) {
    VINS_WARN << "Replay-only jump prior reset enabled; this mode must not "
                 "be used for live flight.";
  }
}

void Estimator::setReplayRejectPersistentInnovationTimestamp(
    double timestamp) {
  replayRejectPersistentInnovationTimestamp = timestamp;
  replayRejectPersistentInnovationConsumed = false;
  if (timestamp >= 0.0) {
    VINS_WARN << "Replay-only persistent-innovation approval rejection "
                 "enabled at timestamp="
              << timestamp << "; this mode must not be used for live flight.";
  }
}

void Estimator::setReplayRejectAcceptedUpdateTimestamp(double timestamp) {
  replayRejectAcceptedUpdateTimestamp = timestamp;
  replayRejectAcceptedUpdateConsumed = false;
  if (timestamp >= 0.0) {
    VINS_WARN << "Replay-only accepted-update transaction rejection enabled "
              << "at timestamp=" << timestamp
              << "; this mode must not be used for live flight.";
  }
}

void Estimator::setReplayVelocityTraceWindow(double start_timestamp,
                                             double end_timestamp) {
  replayVelocityTraceStartTimestamp = start_timestamp;
  replayVelocityTraceEndTimestamp = end_timestamp;
  if (start_timestamp >= 0.0 && end_timestamp >= start_timestamp) {
    VINS_WARN << "Replay-only velocity/prior trace enabled: "
              << start_timestamp << ".." << end_timestamp
              << " s; this mode does not alter estimator decisions.";
  }
}

void Estimator::setReplayBackendHealthTraceWindow(
    double start_timestamp, double end_timestamp) {
  replayBackendHealthTraceStartTimestamp = start_timestamp;
  replayBackendHealthTraceEndTimestamp = end_timestamp;
  if (start_timestamp >= 0.0 && end_timestamp >= start_timestamp) {
    VINS_WARN << "Replay-only backend health metric trace enabled: "
              << start_timestamp << ".." << end_timestamp
              << " s; this mode does not alter estimator decisions.";
  }
}

void Estimator::setReplayDualStateBackend(
    bool enabled, int required_healthy_updates) {
  std::lock_guard<std::mutex> lock(processingMutex);
  if (enabled && required_healthy_updates <= 0) {
    throw std::invalid_argument(
        "dual-state backend requires a positive approval horizon");
  }
  replayDualStateBackend = enabled;
  replayDualStateRequiredHealthyUpdates =
      std::max(1, required_healthy_updates);
  backendCandidateStore.reset();
  backendCandidateInputBuffer.reset();
  backendCandidateProcessingFailed = false;
  lastBackendHealthAvailable = false;
  lastBackendSolverUsable = false;
  lastBackendTransactionRejected = false;
  if (enabled) {
    VINS_WARN << "Replay-only dual-state backend enabled: approval_horizon="
              << replayDualStateRequiredHealthyUpdates
              << "; candidate output remains withheld until commit.";
  }
}

void Estimator::setReplayMultiCheckpointRecovery(bool enabled) {
  std::lock_guard<std::mutex> processing_lock(processingMutex);
  if (enabled && !replayDualStateBackend) {
    throw std::invalid_argument(
        "multi-checkpoint recovery requires dual-state backend");
  }
  replayMultiCheckpointRecovery = enabled;
  {
    std::lock_guard<std::mutex> recovery_lock(backendRecoveryMutex);
    backendRecoveryCheckpoints.clear();
    backendRecoverySensorRing.clear();
    lastBackendRecoveryCheckpointSequence = 0;
    backendRecoveryFirstSuspectSequence = 0;
    backendRecoveryFirstSuspectRawImageSequence = 0;
    backendRecoverySelectionLogged = false;
    backendRecoveryEvaluationComplete = false;
  }
  {
    std::lock_guard<std::mutex> feature_lock(featureBufferMutex);
    backendFeatureTrackerSnapshots.clear();
    backendFeatureRawImageSequences.clear();
  }
  if (enabled) {
    VINS_WARN << "Replay-only multi-checkpoint storage enabled; "
                 "rewind execution remains fail-closed until paired-phase "
                 "replay is connected.";
  }
}

void Estimator::setReplayWarmReinitialization(bool enabled) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayWarmReinitialization = enabled;
  warmReinitializationBridge.clear();
  if (enabled) {
    VINS_WARN << "Replay-only warm reinitialization enabled; IMU bridge "
                 "handover remains limited to 5 s and 0.5 m.";
  }
}

void Estimator::setReplayMotionOnsetWarmReinitialization(bool enabled) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayMotionOnsetWarmReinitialization = enabled;
  replayMotionOnsetWarmReinitializationConsumed = false;
  motionOnsetFrameHistory.clear();
  {
    std::lock_guard<std::mutex> history_lock(
        motionOnsetImuHistoryMutex);
    motionOnsetImuHistory.clear();
  }
  if (enabled) {
    VINS_WARN
        << "Replay-only motion-onset warm reinitialization enabled; "
           "a stale prior/window is replaced once by a continuous "
           "11-state camera/IMU history at detected motion onset.";
  }
}

void Estimator::setReplayPriorScaleProjection(
    bool enabled, double start_timestamp,
    double minimum_directional_gradient, int maximum_projection_count) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayPriorScaleProjection = enabled;
  replayPriorScaleProjectionStartTimestamp = start_timestamp;
  replayPriorScaleProjectionMinimumDirectionalGradient =
      minimum_directional_gradient;
  replayPriorScaleProjectionMaximumCount =
      std::max(0, maximum_projection_count);
  replayPriorScaleProjectionCount = 0;
  if (enabled) {
    VINS_WARN << "Replay-only marginalization prior scale projection "
                 "enabled; one joint pose/velocity scale direction is "
                 "removed from eligible new priors; start_timestamp="
              << start_timestamp
              << ", minimum_directional_gradient="
              << minimum_directional_gradient
              << ", maximum_projection_count="
              << replayPriorScaleProjectionMaximumCount;
  }
}

void Estimator::setReplaySkipMarginalizationPriorTimestamp(
    double timestamp) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replaySkipMarginalizationPriorTimestamp = timestamp;
  replaySkipMarginalizationPriorConsumed = false;
  if (timestamp >= 0.0) {
    VINS_WARN << "Replay-only one-solve marginalization prior skip enabled "
              << "at timestamp=" << timestamp
              << "; the prior remains available for later marginalization; "
                 "this mode must not be used for live flight.";
  }
}

void Estimator::setReplayIncludeLongImuFactors(bool enabled) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayIncludeLongImuFactors = enabled;
  if (enabled) {
    VINS_WARN << "Replay-only long IMU factor inclusion enabled; "
                 "preintegrations over 10 s remain connected in solves and "
                 "marginalization; this mode must not be used for live "
                 "flight.";
  }
}

void Estimator::setReplayMaximumImuWindowInterval(
    double maximum_seconds) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayMaximumImuWindowInterval = maximum_seconds;
  if (maximum_seconds > 0.0) {
    VINS_WARN << "Replay-only maximum IMU window interval enabled: "
              << maximum_seconds
              << " s; low-parallax second-new marginalization is overridden "
                 "to flush stale keyframes; this mode must not be used for "
                 "live flight.";
  }
}

void Estimator::setReplayGtAccelBiasZeroOracle(bool enabled) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayGtAccelBiasZeroOracle = enabled;
  if (enabled) {
    VINS_WARN << "Replay-only GT accel-bias zero oracle enabled; simulator "
                 "accel bias coordinates are fixed to zero in every solve; "
                 "this mode must not be used for live flight.";
  }
}

void Estimator::setReplayGtInitialVelocityZeroOracle(bool enabled) {
  std::lock_guard<std::mutex> lock(processingMutex);
  replayGtInitialVelocityZeroOracle = enabled;
  if (enabled) {
    VINS_WARN << "Replay-only GT initial-velocity zero oracle enabled; "
                 "velocity coordinates are fixed to zero only in the stereo "
                 "initialization solve and released afterward; this mode "
                 "must not be used for live flight.";
  }
}

std::unique_ptr<vins::BackendState> Estimator::captureBackendState(
    std::string *error) const {
  const auto fail = [&](const char *message) {
    if (error) *error = message;
    return std::unique_ptr<vins::BackendState>{};
  };
  if (!options) return fail("estimator options are not initialized");

  auto snapshot = std::unique_ptr<vins::BackendState>(
      new vins::BackendState);
  snapshot->generation = backendGeneration;
  snapshot->previous_timestamp = previousTimestamp;
  snapshot->current_timestamp = currentTimestamp;
  snapshot->initial_timestamp = initialTimestamp;
  snapshot->open_ex_estimation = openExEstimation;
  snapshot->solver_flag = solver_flag;
  snapshot->marginalization_flag = marginalization_flag;
  snapshot->gravity = gravity;
  snapshot->frame_count = frameCount;
  snapshot->back_count = backCount;
  snapshot->front_count = frontCount;
  snapshot->nonlinear_update_count = nonLinearUpdateCount;
  snapshot->stationary_visual_update_count =
      stationaryVisualUpdateCount;
  snapshot->consecutive_position_innovation_rejects =
      consecutivePositionInnovationRejects;
  snapshot->is_first_imu_received = isFirstIMUReceived;
  snapshot->failure_occurred = failure_occur;
  snapshot->range_ground_initialized = rangeGroundInitialized;
  snapshot->range_ground_z = rangeGroundZ;
  snapshot->last_published_alignment_scale =
      lastPublishedAlignmentScale;
  snapshot->current_alignment_scale = currentAlignmentScale;
  snapshot->current_feature_flow_median_px_s =
      currentFeatureFlowMedianPxS;
  snapshot->current_feature_flow_count = currentFeatureFlowCount;
  snapshot->last_optimization_initial_cost =
      lastOptimizationInitialCost;
  snapshot->last_optimization_final_cost = lastOptimizationFinalCost;
  snapshot->last_optimization_time_seconds =
      lastOptimizationTimeSeconds;
  snapshot->last_optimization_successful_steps =
      lastOptimizationSuccessfulSteps;
  snapshot->last_optimization_unsuccessful_steps =
      lastOptimizationUnsuccessfulSteps;
  snapshot->last_optimization_residual_blocks =
      lastOptimizationResidualBlocks;
  snapshot->last_optimization_residuals = lastOptimizationResiduals;
  snapshot->last_optimization_termination =
      lastOptimizationTermination;
  snapshot->last_marginalization_source_timestamp =
      lastMarginalizationSourceTimestamp;
  snapshot->last_marginalization_source_type =
      lastMarginalizationSourceType;
  snapshot->latest_orientation = latest_Q;
  snapshot->is_first_pose_initialized = isFirstPoseInitialized;
  snapshot->back_state = back_state;
  snapshot->last_state = last_state;
  snapshot->last_state0 = last_state0;
  snapshot->latest_state = latest_state;
  snapshot->previous_imu_data = previousImuData;
  snapshot->latest_imu_data = latestImuData;
  snapshot->options = std::make_shared<VINSOptions>(*options);
  snapshot->feature_manager =
      featureManager.cloneWithOptions(snapshot->options);

  for (int i = 0; i <= WINDOW_SIZE; ++i) {
    snapshot->states[i] = estimator_state[i];
    snapshot->range_measurements[i] = rangeMeasurements[i];
    snapshot->metric_velocity_measurements[i] =
        metricVelocityMeasurements[i];
    snapshot->rotor_dynamics_measurements[i] =
        rotorDynamicsMeasurements[i];
    snapshot->delta_time_imu_buffer[i] = deltaTimeImuBuffer[i];
    if (pre_integrations[i]) {
      snapshot->preintegrations[i] = pre_integrations[i]->clone();
    }
    std::copy(poseArray[i], poseArray[i] + SIZE_POSE,
              snapshot->pose_parameters[i].begin());
    std::copy(speedBiasArray[i], speedBiasArray[i] + SIZE_SPEEDBIAS,
              snapshot->speed_bias_parameters[i].begin());
    std::copy(lastSolvedPoseArray[i],
              lastSolvedPoseArray[i] + SIZE_POSE,
              snapshot->last_solved_pose_parameters[i].begin());
    std::copy(lastSolvedSpeedBiasArray[i],
              lastSolvedSpeedBiasArray[i] + SIZE_SPEEDBIAS,
              snapshot->last_solved_speed_bias_parameters[i].begin());
  }
  snapshot->metric_displacement_anchor_initialized =
      metricDisplacementAnchorInitialized;
  snapshot->metric_displacement_anchor_segment =
      metricDisplacementAnchorSegment;
  snapshot->metric_displacement_anchor_measurement =
      metricDisplacementAnchorMeasurement;
  snapshot->metric_displacement_anchor_position =
      metricDisplacementAnchorPosition;
  snapshot->gps_dropout_gyro_bias_anchor_initialized =
      gpsDropoutGyroBiasAnchorInitialized;
  snapshot->gps_dropout_gyro_bias_anchor_z =
      gpsDropoutGyroBiasAnchorZ;
  for (int i = 0; i < 2; ++i) {
    snapshot->camera_rotation[i] = cameraRotation[i];
    snapshot->camera_translation[i] = cameraTranslation[i];
    std::copy(para_Ex_Pose[i], para_Ex_Pose[i] + SIZE_POSE,
              snapshot->extrinsic_parameters[i].begin());
    std::copy(lastSolvedExtrinsicArray[i],
              lastSolvedExtrinsicArray[i] + SIZE_POSE,
              snapshot->last_solved_extrinsic_parameters[i].begin());
  }
  for (int i = 0; i < NUM_OF_F; ++i) {
    snapshot->feature_parameters[i][0] = para_Feature[i][0];
    snapshot->last_solved_feature_parameters[i][0] =
        lastSolvedFeatureArray[i][0];
  }
  snapshot->time_delay_parameter[0] = para_Td[0][0];
  snapshot->range_parameter[0] = para_Tr[0][0];
  snapshot->last_solved_time_delay_parameter[0] =
      lastSolvedTimeDelayArray[0];
  snapshot->last_solved_feature_count = lastSolvedFeatureCount;
  snapshot->last_solved_parameters_valid =
      lastSolvedParametersValid;

  if (tmp_pre_integration) {
    snapshot->temporary_preintegration =
        tmp_pre_integration->clone();
  }
  for (const auto &entry : all_image_frame) {
    ImageFrame frame = entry.second;
    if (entry.second.pre_integration) {
      frame.pre_integration =
          entry.second.pre_integration->clone();
    }
    snapshot->image_frames.emplace(entry.first, std::move(frame));
  }

  const auto map_live_block = [&](const double *block) -> double * {
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      if (block == poseArray[i]) return snapshot->poseBlock(i);
      if (block == speedBiasArray[i])
        return snapshot->speedBiasBlock(i);
    }
    for (int i = 0; i < NUM_OF_F; ++i) {
      if (block == para_Feature[i])
        return snapshot->featureBlock(i);
    }
    for (int i = 0; i < 2; ++i) {
      if (block == para_Ex_Pose[i])
        return snapshot->extrinsicBlock(i);
    }
    if (block == para_Td[0]) return snapshot->timeDelayBlock();
    if (block == para_Tr[0]) return snapshot->rangeBlock();
    return nullptr;
  };
  snapshot->prior_parameter_blocks.reserve(
      last_marginalization_parameter_blocks.size());
  for (double *block : last_marginalization_parameter_blocks) {
    double *mapped = map_live_block(block);
    if (!mapped) {
      return fail("live prior references an unknown parameter block");
    }
    snapshot->prior_parameter_blocks.push_back(mapped);
  }
  if (last_marginalization_info) {
    snapshot->marginalization_prior =
        last_marginalization_info->cloneRebound(
            last_marginalization_parameter_blocks,
            snapshot->prior_parameter_blocks, error);
    if (!snapshot->marginalization_prior) return nullptr;
  } else if (!last_marginalization_parameter_blocks.empty()) {
    return fail("live prior blocks exist without marginalization info");
  }
  if (!snapshot->priorAddressesAreOwned()) {
    return fail("captured prior failed BackendState ownership check");
  }
  if (error) error->clear();
  return snapshot;
}

bool Estimator::applyBackendState(const vins::BackendState &state,
                                  std::string *error) {
  const auto fail = [&](const char *message) {
    if (error) *error = message;
    return false;
  };
  auto staging = state.clone(error);
  if (!staging) return false;
  if (!staging->options) {
    return fail("backend state has no options");
  }
  if (!options) options = std::make_shared<VINSOptions>();
  *options = *staging->options;

  backendGeneration = staging->generation;
  previousTimestamp = staging->previous_timestamp;
  currentTimestamp = staging->current_timestamp;
  initialTimestamp = staging->initial_timestamp;
  openExEstimation = staging->open_ex_estimation;
  solver_flag = staging->solver_flag;
  marginalization_flag = staging->marginalization_flag;
  gravity = staging->gravity;
  frameCount = staging->frame_count;
  backCount = staging->back_count;
  frontCount = staging->front_count;
  nonLinearUpdateCount = staging->nonlinear_update_count;
  stationaryVisualUpdateCount =
      staging->stationary_visual_update_count;
  consecutivePositionInnovationRejects =
      staging->consecutive_position_innovation_rejects;
  isFirstIMUReceived = staging->is_first_imu_received;
  failure_occur = staging->failure_occurred;
  rangeGroundInitialized = staging->range_ground_initialized;
  rangeGroundZ = staging->range_ground_z;
  lastPublishedAlignmentScale =
      staging->last_published_alignment_scale;
  currentAlignmentScale = staging->current_alignment_scale;
  currentFeatureFlowMedianPxS =
      staging->current_feature_flow_median_px_s;
  currentFeatureFlowCount = staging->current_feature_flow_count;
  lastOptimizationInitialCost =
      staging->last_optimization_initial_cost;
  lastOptimizationFinalCost =
      staging->last_optimization_final_cost;
  lastOptimizationTimeSeconds =
      staging->last_optimization_time_seconds;
  lastOptimizationSuccessfulSteps =
      staging->last_optimization_successful_steps;
  lastOptimizationUnsuccessfulSteps =
      staging->last_optimization_unsuccessful_steps;
  lastOptimizationResidualBlocks =
      staging->last_optimization_residual_blocks;
  lastOptimizationResiduals = staging->last_optimization_residuals;
  lastOptimizationTermination =
      staging->last_optimization_termination;
  lastMarginalizationSourceTimestamp =
      staging->last_marginalization_source_timestamp;
  lastMarginalizationSourceType =
      staging->last_marginalization_source_type;
  latest_Q = staging->latest_orientation;
  isFirstPoseInitialized = staging->is_first_pose_initialized;
  back_state = staging->back_state;
  last_state = staging->last_state;
  last_state0 = staging->last_state0;
  latest_state = staging->latest_state;
  previousImuData = staging->previous_imu_data;
  latestImuData = staging->latest_imu_data;
  featureManager =
      staging->feature_manager.cloneWithOptions(options);

  for (int i = 0; i <= WINDOW_SIZE; ++i) {
    estimator_state[i] = staging->states[i];
    rangeMeasurements[i] = staging->range_measurements[i];
    metricVelocityMeasurements[i] =
        staging->metric_velocity_measurements[i];
    rotorDynamicsMeasurements[i] =
        staging->rotor_dynamics_measurements[i];
    deltaTimeImuBuffer[i] = staging->delta_time_imu_buffer[i];
    pre_integrations[i] = staging->preintegrations[i];
    std::copy(staging->pose_parameters[i].begin(),
              staging->pose_parameters[i].end(), poseArray[i]);
    std::copy(staging->speed_bias_parameters[i].begin(),
              staging->speed_bias_parameters[i].end(),
              speedBiasArray[i]);
    std::copy(staging->last_solved_pose_parameters[i].begin(),
              staging->last_solved_pose_parameters[i].end(),
              lastSolvedPoseArray[i]);
    std::copy(
        staging->last_solved_speed_bias_parameters[i].begin(),
        staging->last_solved_speed_bias_parameters[i].end(),
        lastSolvedSpeedBiasArray[i]);
  }
  metricDisplacementAnchorInitialized =
      staging->metric_displacement_anchor_initialized;
  metricDisplacementAnchorSegment =
      staging->metric_displacement_anchor_segment;
  metricDisplacementAnchorMeasurement =
      staging->metric_displacement_anchor_measurement;
  metricDisplacementAnchorPosition =
      staging->metric_displacement_anchor_position;
  gpsDropoutGyroBiasAnchorInitialized =
      staging->gps_dropout_gyro_bias_anchor_initialized;
  gpsDropoutGyroBiasAnchorZ =
      staging->gps_dropout_gyro_bias_anchor_z;
  for (int i = 0; i < 2; ++i) {
    cameraRotation[i] = staging->camera_rotation[i];
    cameraTranslation[i] = staging->camera_translation[i];
    std::copy(staging->extrinsic_parameters[i].begin(),
              staging->extrinsic_parameters[i].end(),
              para_Ex_Pose[i]);
    std::copy(staging->last_solved_extrinsic_parameters[i].begin(),
              staging->last_solved_extrinsic_parameters[i].end(),
              lastSolvedExtrinsicArray[i]);
  }
  for (int i = 0; i < NUM_OF_F; ++i) {
    para_Feature[i][0] = staging->feature_parameters[i][0];
    lastSolvedFeatureArray[i][0] =
        staging->last_solved_feature_parameters[i][0];
  }
  para_Td[0][0] = staging->time_delay_parameter[0];
  para_Tr[0][0] = staging->range_parameter[0];
  lastSolvedTimeDelayArray[0] =
      staging->last_solved_time_delay_parameter[0];
  lastSolvedFeatureCount = staging->last_solved_feature_count;
  lastSolvedParametersValid =
      staging->last_solved_parameters_valid;
  tmp_pre_integration = staging->temporary_preintegration;
  all_image_frame = std::move(staging->image_frames);

  const auto map_staging_block = [&](const double *block) -> double * {
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      if (block == staging->poseBlock(i)) return poseArray[i];
      if (block == staging->speedBiasBlock(i))
        return speedBiasArray[i];
    }
    for (int i = 0; i < NUM_OF_F; ++i) {
      if (block == staging->featureBlock(i)) return para_Feature[i];
    }
    for (int i = 0; i < 2; ++i) {
      if (block == staging->extrinsicBlock(i))
        return para_Ex_Pose[i];
    }
    if (block == staging->timeDelayBlock()) return para_Td[0];
    if (block == staging->rangeBlock()) return para_Tr[0];
    return nullptr;
  };
  std::vector<double *> live_prior_blocks;
  live_prior_blocks.reserve(staging->prior_parameter_blocks.size());
  for (double *block : staging->prior_parameter_blocks) {
    double *mapped = map_staging_block(block);
    if (!mapped) return fail("backend prior contains an unknown block");
    live_prior_blocks.push_back(mapped);
  }
  if (staging->marginalization_prior) {
    last_marginalization_info =
        staging->marginalization_prior->cloneRebound(
            staging->prior_parameter_blocks, live_prior_blocks, error);
    if (!last_marginalization_info) return false;
  } else {
    last_marginalization_info.reset();
  }
  last_marginalization_parameter_blocks =
      std::move(live_prior_blocks);
  if (error) error->clear();
  return true;
}

bool Estimator::evaluateRecoveryCandidate(
    const vins::BackendState &checkpoint_backend,
    const FeatureTracker &checkpoint_tracker,
    const vins::BackendRecoverySensorSlice &sensor_slice,
    int downsample_phase,
    std::uint64_t anomaly_corridor_end_raw_sequence,
    const StateData &published_state,
    vins::BackendRecoveryCandidateEvidence *evidence,
    vins::BackendState::Ptr *recovered_backend,
    std::unique_ptr<FeatureTracker> *recovered_tracker,
    std::string *error) {
  const auto fail = [&](const char *message) {
    if (error) *error = message;
    if (evidence) evidence->fatal_processing_error = true;
    return false;
  };
  if (!evidence || !recovered_backend || !recovered_tracker ||
      (downsample_phase != 0 && downsample_phase != 1) ||
      anomaly_corridor_end_raw_sequence == 0 ||
      !checkpoint_backend.options) {
    return fail("invalid recovery candidate input");
  }

  *evidence = vins::BackendRecoveryCandidateEvidence();
  recovered_backend->reset();
  recovered_tracker->reset();
  evidence->downsample_phase = downsample_phase;
  Estimator recovery;
  recovery.backendCandidateWorkspaceActive = true;
  recovery.options =
      std::make_shared<VINSOptions>(*checkpoint_backend.options);
  if (!recovery.applyBackendState(checkpoint_backend, error)) {
    return fail("recovery backend apply failed");
  }
  auto tracker =
      checkpoint_tracker.cloneWithOptions(recovery.options);
  if (!tracker) return fail("recovery tracker clone failed");
  recovery.featureTracker = std::move(*tracker);
  for (const auto &imu : sensor_slice.imu) {
    recovery.imuBuffer.push(imu);
  }
  for (const auto &range : sensor_slice.ranges) {
    recovery.rangeBuffer.push(range);
  }

  bool metric_seen = false;
  std::size_t consecutive_healthy_after_corridor = 0;
  for (const auto &image : sensor_slice.images) {
    FeatureFrame features;
    if (image.image1.empty()) {
      features = recovery.featureTracker.trackImage(
          image.timestamp, image.image0);
    } else {
      features = recovery.featureTracker.trackImage(
          image.timestamp, image.image0, image.image1);
    }
    if (static_cast<int>(image.sequence % 2) != downsample_phase) {
      continue;
    }

    const double target_timestamp =
        image.timestamp + recovery.options->time_delay;
    std::vector<IMUData> imu_data;
    if (recovery.options->hasImu() &&
        !recovery.getIMUInterval(
            recovery.previousTimestamp, target_timestamp, imu_data)) {
      return fail("recovery IMU interval unavailable");
    }
    if (recovery.options->hasImu()) {
      for (std::size_t i = 0; i < imu_data.size(); ++i) {
        double dt = 0.0;
        if (i == 0) {
          dt = imu_data[i].timestamp - recovery.previousTimestamp;
        } else if (i == imu_data.size() - 1) {
          dt = target_timestamp - imu_data[i - 1].timestamp;
        } else {
          dt = imu_data[i].timestamp - imu_data[i - 1].timestamp;
        }
        recovery.processIMU(imu_data[i], dt);
      }
    }

    const int previous_rejects =
        recovery.consecutivePositionInnovationRejects;
    recovery.currentTimestamp = target_timestamp;
    recovery.backendCandidateProcessingFailed = false;
    recovery.lastBackendHealthAvailable = false;
    recovery.lastBackendSolverUsable = false;
    recovery.lastBackendTransactionRejected = false;
    recovery.processImage(features, image.timestamp);
    if (recovery.backendCandidateProcessingFailed) {
      return fail("recovery camera update requested reset");
    }
    recovery.previousTimestamp = target_timestamp;
    evidence->replayed_through_sequence = image.sequence;

    bool healthy = false;
    if (recovery.lastBackendHealthAvailable) {
      const auto approval = recovery.backendApprovalGate.evaluate(
          recovery.lastBackendHealthMetrics,
          recovery.lastBackendSolverUsable, previous_rejects,
          recovery.consecutivePositionInnovationRejects);
      healthy = approval.healthy;
      const double scale =
          recovery.lastBackendHealthMetrics.maximum_scale_log_change;
      const double kinematic =
          recovery.lastBackendHealthMetrics.window_kinematic_rms_mps;
      if (std::isfinite(scale) && std::isfinite(kinematic)) {
        if (!metric_seen) {
          evidence->maximum_scale_log_change = scale;
          evidence->maximum_kinematic_rms = kinematic;
          metric_seen = true;
        } else {
          evidence->maximum_scale_log_change =
              std::max(evidence->maximum_scale_log_change, scale);
          evidence->maximum_kinematic_rms =
              std::max(evidence->maximum_kinematic_rms, kinematic);
        }
      }
    }
    if (image.sequence > anomaly_corridor_end_raw_sequence) {
      consecutive_healthy_after_corridor =
          healthy ? consecutive_healthy_after_corridor + 1 : 0;
    }
  }

  evidence->healthy_updates_after_corridor =
      consecutive_healthy_after_corridor;
  evidence->published_output_correction_m =
      (recovery.estimator_state[recovery.frameCount].position -
       published_state.position)
          .norm();
  if (!metric_seen ||
      !std::isfinite(evidence->published_output_correction_m)) {
    return fail("recovery evidence is nonfinite or incomplete");
  }
  *recovered_backend = recovery.captureBackendState(error);
  if (!*recovered_backend) {
    return fail("recovered backend capture failed");
  }
  *recovered_tracker =
      recovery.featureTracker.cloneWithOptions(
          (*recovered_backend)->options);
  if (!*recovered_tracker) {
    recovered_backend->reset();
    return fail("recovered tracker capture failed");
  }
  if (error) error->clear();
  return true;
}

void Estimator::inputIMU(const IMUData &imu) {
  if (replayMultiCheckpointRecovery) {
    std::lock_guard<std::mutex> recovery_lock(backendRecoveryMutex);
    if (!backendRecoverySensorRing.appendImu(imu)) {
      VINS_ERROR << "Multi-checkpoint IMU ring protocol failed at "
                 << imu.timestamp;
      replayMultiCheckpointRecovery = false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    // A MultiThreadedExecutor may enter two IMU callbacks in the opposite
    // order even though DDS delivered monotonically stamped Gazebo samples.
    // Keeping that order in imuBuffer makes updateLatestStates() rewind the
    // propagation epoch and was visible as multi-second output gaps and
    // alternating old/new odometry during the D435i Stage-B cross.
    if (!imuBuffer.empty() &&
        imu.timestamp <= imuBuffer.back().timestamp) {
      VINS_WARN << "Dropping non-monotonic IMU sample: timestamp="
                << imu.timestamp
                << ", buffered_tail=" << imuBuffer.back().timestamp;
      return;
    }
    imuBuffer.push(imu);
  }
  if (replayMotionOnsetWarmReinitialization) {
    std::lock_guard<std::mutex> history_lock(
        motionOnsetImuHistoryMutex);
    if (motionOnsetImuHistory.empty() ||
        imu.timestamp > motionOnsetImuHistory.back().timestamp) {
      motionOnsetImuHistory.push_back(imu);
      constexpr double kHistorySpanS = 6.0;
      while (motionOnsetImuHistory.size() > 2 &&
             motionOnsetImuHistory[1].timestamp <
                 imu.timestamp - kHistorySpanS) {
        motionOnsetImuHistory.pop_front();
      }
    }
  }
  imuCondition.notify_all();

  if (solver_flag == SolverState::NON_LINEAR) {
    fastPredictIMU(imu);
  } else if (warmReinitializationBridge.active()) {
    predictWarmReinitialization(imu);
  }
}

void Estimator::inputRange(const RangeData &range) {
  if (!options->hasRange() || !range.valid || !std::isfinite(range.range)) return;
  if (replayMultiCheckpointRecovery) {
    std::lock_guard<std::mutex> recovery_lock(backendRecoveryMutex);
    if (!backendRecoverySensorRing.appendRange(range)) {
      VINS_ERROR << "Multi-checkpoint range ring protocol failed at "
                 << range.timestamp;
      replayMultiCheckpointRecovery = false;
    }
  }
  std::lock_guard<std::mutex> lock(rangeMutex);
  if (!rangeBuffer.empty() && range.timestamp <= rangeBuffer.back().timestamp)
    return;
  rangeBuffer.push(range);
  while (rangeBuffer.size() > 1000) rangeBuffer.pop();
}

void Estimator::inputMetricVelocity(const MetricVelocityData &velocity) {
  if (!options->hasGpsVelocity() || !velocity.valid ||
      !velocity.horizontal_velocity.allFinite() ||
      !velocity.horizontal_position.allFinite() ||
      !std::isfinite(velocity.sigma)) {
    return;
  }
  std::lock_guard<std::mutex> lock(metricVelocityMutex);
  if (!metricVelocityBuffer.empty() &&
      velocity.timestamp <= metricVelocityBuffer.back().timestamp) {
    return;
  }
  metricVelocityBuffer.push(velocity);
  while (metricVelocityBuffer.size() > 1000) metricVelocityBuffer.pop();
}

void Estimator::inputRotorDynamics(const RotorDynamicsData &dynamics) {
  if (!options->hasRotorDragDynamics() || !dynamics.valid ||
      !std::isfinite(dynamics.total_thrust_newton) ||
      !std::isfinite(dynamics.summed_rotor_speed_rad_s) ||
      !std::isfinite(dynamics.factor_weight_scale) ||
      dynamics.total_thrust_newton <= 0.0 ||
      dynamics.summed_rotor_speed_rad_s <= 0.0 ||
      dynamics.source_hash.size() != 16 ||
      (dynamics.quality_state != "NORMAL" &&
       dynamics.quality_state != "DEGRADED") ||
      (dynamics.quality_state == "NORMAL" &&
       dynamics.factor_weight_scale != 1.0) ||
      (dynamics.quality_state == "DEGRADED" &&
       dynamics.factor_weight_scale != 0.25)) {
    return;
  }
  std::lock_guard<std::mutex> lock(rotorDynamicsMutex);
  if (!rotorDynamicsBuffer.empty() &&
      dynamics.timestamp <= rotorDynamicsBuffer.back().timestamp) {
    return;
  }
  rotorDynamicsBuffer.push(dynamics);
  while (rotorDynamicsBuffer.size() > 1000) rotorDynamicsBuffer.pop();
}

void Estimator::inputStationaryHint(bool stationary) {
  const bool previous = stationaryHint.exchange(stationary);
  if (previous != stationary) {
    VINS_INFO << "Mission stationary hint changed: active="
              << stationary;
  }
}

void Estimator::inputReplayGroundTruth(const OdomData &ground_truth) {
  std::lock_guard<std::mutex> lock(replayGroundTruthMutex);
  if (!replayGroundTruthHistory.empty() &&
      ground_truth.timestamp <= replayGroundTruthHistory.back().timestamp) {
    return;
  }
  replayGroundTruthHistory.push_back(ground_truth);
  while (!replayGroundTruthHistory.empty() &&
         replayGroundTruthHistory.front().timestamp <
             ground_truth.timestamp - 10.0) {
    replayGroundTruthHistory.pop_front();
  }
}

void Estimator::inputFeature(double timestamp,
                             const FeatureFrame &featureFrame) {
  {
    std::lock_guard<std::mutex> lock(featureBufferMutex);
    const std::uint64_t sequence =
        featureInputSequence.fetch_add(1) + 1;
    featureBuffer.push(
        TimestampedFeatureFrame{sequence, 0, timestamp, featureFrame});
  }
  featureCondition.notify_one();
}

bool Estimator::getIMUInterval(double startTime, double endTime,
                               vector<IMUData> &data) {
  std::lock_guard<std::mutex> lock(imu_mutex);
  if (imuBuffer.empty()) {
    VINS_ERROR << "No IMU data received";
    return false;
  }

  if (endTime <= imuBuffer.back().timestamp) {
    while (imuBuffer.front().timestamp <= startTime) {
      imuBuffer.pop();
    }

    while (imuBuffer.front().timestamp < endTime) {
      data.push_back(imuBuffer.front());
      imuBuffer.pop();
    }
    if (!imuBuffer.empty()) {
      data.push_back(imuBuffer.front());
    }
    return true;
  }

  VINS_WARN << "Waiting for IMU data";
  return false;
}

bool Estimator::IMUAvailable(double t) {
  return !imuBuffer.empty() && t <= imuBuffer.back().timestamp;
}

void Estimator::processMeasurements() {
  while (isRunning.load()) {
    TimestampedFeatureFrame feature;
    std::shared_ptr<const FeatureTracker> feature_tracker_snapshot;
    std::uint64_t feature_raw_image_sequence = 0;
    size_t queue_depth_after_pop = 0;
    double queue_span_s = 0.0;
    {
      std::unique_lock<std::mutex> lock(featureBufferMutex);
      featureCondition.wait(
          lock, [this] { return !featureBuffer.empty() || !isRunning.load(); });
      if (!isRunning.load() || featureBuffer.empty()) break;
      feature = featureBuffer.front();
      featureBuffer.pop();
      const auto tracker_snapshot =
          backendFeatureTrackerSnapshots.find(feature.sequence);
      if (tracker_snapshot != backendFeatureTrackerSnapshots.end()) {
        feature_tracker_snapshot = tracker_snapshot->second;
        backendFeatureTrackerSnapshots.erase(tracker_snapshot);
      }
      const auto raw_sequence =
          backendFeatureRawImageSequences.find(feature.sequence);
      if (raw_sequence != backendFeatureRawImageSequences.end()) {
        feature_raw_image_sequence = raw_sequence->second;
        backendFeatureRawImageSequences.erase(raw_sequence);
      }
      queue_depth_after_pop = featureBuffer.size();
      if (!featureBuffer.empty()) {
        queue_span_s =
            featureBuffer.back().timestamp - feature.timestamp;
      }
      lock.unlock();
    }
    VINS_INFO << "Feature queue trace: sequence=" << feature.sequence
              << ", raw_image_sequence=" << feature.raw_image_sequence
              << ", timestamp=" << std::fixed
              << std::setprecision(9) << feature.timestamp
              << ", queue_depth_after_pop=" << queue_depth_after_pop
              << ", queue_span_s=" << queue_span_s;
    const double target_timestamp =
        feature.timestamp + options->time_delay;
    if (options->hasImu()) {
      std::unique_lock<std::mutex> lock(imu_mutex);

      imuCondition.wait(lock, [this, target_timestamp] {
        return IMUAvailable(target_timestamp) || !isRunning.load();
      });
      if (!isRunning.load()) {
        break;
      }
    }

    // Reset must not interleave with pre-integration or image processing.
    // Previously only processImage held this mutex, so a reset service could
    // clear pre_integrations while processIMU was still using them.
    std::lock_guard<std::mutex> processing_lock(processingMutex);
    // Do not arm speculative generations during nonlinear initialization
    // probation.  The approval thresholds were fixed from stable windows and
    // require full kinematic/depth support; early phase-0 initialization can
    // otherwise fail the first gate and accumulate an IMU-only backlog.
    constexpr int kDualStateStableEpochUpdates = 20;
    if (replayDualStateBackend && isNonLinearSolver() &&
        nonLinearUpdateCount >= kDualStateStableEpochUpdates) {
      std::string error;
      if (!backendCandidateStore) {
        auto committed = captureBackendState(&error);
        auto store = std::unique_ptr<vins::BackendCandidateStore>(
            new vins::BackendCandidateStore(
                static_cast<std::size_t>(
                    replayDualStateRequiredHealthyUpdates)));
        if (!committed ||
            !store->initialize(std::move(committed), &error)) {
          VINS_ERROR << "Dual-state backend initialization failed: "
                     << error;
          replayDualStateBackend = false;
          backendCandidateInputBuffer.reset();
        } else {
          backendCandidateStore = std::move(store);
          VINS_INFO << "Dual-state backend armed after stable epoch: "
                    << "updates=" << nonLinearUpdateCount
                    << ", sequence=" << feature.sequence
                    << ", timestamp=" << feature.timestamp;
        }
      }

      if (backendCandidateStore) {
        const bool beginning_generation =
            backendCandidateStore->candidate() == nullptr;
        if (beginning_generation &&
            (!backendCandidateStore->begin(
                 feature.sequence, feature.timestamp, &error) ||
             !backendCandidateInputBuffer.begin(
                 feature.sequence, feature.timestamp))) {
          VINS_ERROR << "Dual-state candidate begin failed: " << error;
          replayDualStateBackend = false;
          backendCandidateStore.reset();
          backendCandidateInputBuffer.reset();
        } else {
          // Published IMU propagation is an output-only view.  It must not be
          // rewound when the live optimization workspace is switched between
          // candidate and committed snapshots.
          const StateData published_latest_state = latest_state;
          const Quaterniond published_latest_orientation = latest_Q;
          const IMUData published_latest_imu_data = latestImuData;
          const OdomData published_imu_odometry = imu_odom;

          backendCandidateWorkspaceActive = true;
          backendCandidateProcessingFailed = false;
          lastBackendHealthAvailable = false;
          lastBackendSolverUsable = false;
          lastBackendTransactionRejected = false;

          if (!applyBackendState(
                  *backendCandidateStore->candidate(), &error)) {
            backendCandidateProcessingFailed = true;
            VINS_ERROR << "Dual-state candidate apply failed: " << error;
          }

          if (!backendCandidateProcessingFailed &&
              beginning_generation) {
            const std::size_t replay_count =
                backendCandidateInputBuffer.pendingStepCount();
            const auto &replay_steps =
                backendCandidateInputBuffer.stepsForActiveGeneration();
            for (std::size_t i = 0; i < replay_count; ++i) {
              processIMU(replay_steps[i].data,
                         replay_steps[i].delta_time);
            }
            if (replay_count > 0 &&
                std::isfinite(
                    backendCandidateInputBuffer.pendingEndTimestamp())) {
              previousTimestamp =
                  backendCandidateInputBuffer.pendingEndTimestamp();
            }
          }

          currentTimestamp =
              feature.timestamp + options->time_delay;
          if (!backendCandidateProcessingFailed &&
              options->hasImu()) {
            std::unique_lock<std::mutex> imu_lock(imu_mutex);
            imuCondition.wait(imu_lock, [this] {
              return IMUAvailable(currentTimestamp) ||
                     !isRunning.load();
            });
            if (!isRunning.load()) {
              backendCandidateProcessingFailed = true;
            }
          }
          vector<IMUData> imu_datas;
          std::vector<vins::BackendImuStep> current_steps;
          if (!backendCandidateProcessingFailed &&
              options->hasImu() &&
              !getIMUInterval(previousTimestamp, currentTimestamp,
                              imu_datas)) {
            backendCandidateProcessingFailed = true;
          }
          if (!backendCandidateProcessingFailed &&
              options->hasImu()) {
            if (!isFirstPoseInitialized) initFirstIMUPose(imu_datas);
            current_steps.reserve(imu_datas.size());
            for (size_t i = 0; i < imu_datas.size(); ++i) {
              double dt;
              if (i == 0)
                dt = imu_datas[i].timestamp - previousTimestamp;
              else if (i == imu_datas.size() - 1)
                dt = currentTimestamp - imu_datas[i - 1].timestamp;
              else
                dt = imu_datas[i].timestamp -
                     imu_datas[i - 1].timestamp;
              processIMU(imu_datas[i], dt);
              current_steps.push_back(
                  vins::BackendImuStep{imu_datas[i], dt});
            }
          }
          if (!backendCandidateInputBuffer.append(
                  feature.sequence, currentTimestamp, current_steps)) {
            backendCandidateProcessingFailed = true;
            VINS_ERROR << "Dual-state candidate input protocol failed.";
          }

          const int previous_rejects =
              consecutivePositionInnovationRejects;
          if (!backendCandidateProcessingFailed) {
            processImage(feature.features, feature.timestamp);
          }
          if (!backendCandidateProcessingFailed) {
            previousTimestamp = currentTimestamp;
          }

          bool healthy = false;
          bool safe_advance = false;
          bool candidate_captured = false;
          if (!backendCandidateProcessingFailed) {
            auto updated = captureBackendState(&error);
            if (!updated ||
                !backendCandidateStore->replaceCandidate(
                    std::move(updated), &error)) {
              backendCandidateProcessingFailed = true;
              VINS_ERROR << "Dual-state candidate capture failed: "
                         << error;
            } else {
              candidate_captured = true;
            }
            if (candidate_captured &&
                lastBackendTransactionRejected) {
              safe_advance = true;
              VINS_INFO << "Dual-state candidate safe advance: sequence="
                        << feature.sequence
                        << ", timestamp=" << feature.timestamp
                        << ", raw solve transaction was rolled back";
            } else if (candidate_captured &&
                       lastBackendHealthAvailable) {
              const auto approval = backendApprovalGate.evaluate(
                  lastBackendHealthMetrics,
                  lastBackendSolverUsable, previous_rejects,
                  consecutivePositionInnovationRejects);
              const auto *candidate =
                  backendCandidateStore->candidate();
              constexpr double kMaximumCommitOutputStepM = 0.50;
              const double output_step =
                  (candidate->states[candidate->frame_count].position -
                   published_latest_state.position)
                      .norm();
              healthy =
                  approval.healthy &&
                  std::isfinite(output_step) &&
                  output_step <= kMaximumCommitOutputStepM;
              VINS_INFO << "Dual-state candidate observation: sequence="
                        << feature.sequence
                        << ", timestamp=" << feature.timestamp
                        << ", generation="
                        << candidate->generation
                        << ", healthy=" << healthy
                        << ", metric_healthy="
                        << approval.healthy
                        << ", finite=" << approval.finite
                        << ", solver_usable="
                        << approval.solver_usable
                        << ", position_bounded="
                        << approval.position_bounded
                        << ", velocity_bounded="
                        << approval.velocity_bounded
                        << ", kinematics_bounded="
                        << approval.kinematics_bounded
                        << ", scale_change_bounded="
                        << approval.scale_change_bounded
                        << ", scale_spread_bounded="
                        << approval.scale_spread_bounded
                        << ", support_sufficient="
                        << approval.support_sufficient
                        << ", reject_chain_not_worsening="
                        << approval.reject_chain_not_worsening
                        << ", output_step_m=" << output_step
                        << ", healthy_prefix="
                        << backendCandidateStore->lifecycle()
                               .healthyUpdates();
            }
          }

          const bool candidate_was_suspect =
              backendCandidateStore->lifecycle().suspect();
          if (replayMultiCheckpointRecovery &&
              !candidate_was_suspect && candidate_captured &&
              !healthy && !safe_advance &&
              backendRecoveryFirstSuspectSequence == 0) {
            backendRecoveryFirstSuspectSequence = feature.sequence;
            backendRecoveryFirstSuspectRawImageSequence =
                feature_raw_image_sequence;
            backendRecoverySelectionLogged = false;
            backendRecoveryEvaluationComplete = false;
          }
          if ((healthy || safe_advance) &&
              !candidate_was_suspect &&
              (!backendCandidateStore->checkpointCandidate(&error) ||
               !backendCandidateInputBuffer.checkpoint())) {
            VINS_ERROR << "Dual-state candidate checkpoint failed: "
                       << error;
            healthy = false;
            safe_advance = false;
          }

          bool recovery_handover = false;
          int recovery_handover_phase = -1;
          double recovery_handover_timestamp = feature.timestamp;
          vins::BackendCandidateLifecycle::Decision decision;
          if (safe_advance) {
            decision =
                backendCandidateStore->observeSafeAdvance(
                    feature.sequence, feature.timestamp, &error);
          } else if (healthy) {
            decision = backendCandidateStore->observe(
                feature.sequence, feature.timestamp, true, &error);
          } else if (candidate_captured) {
            decision = backendCandidateStore->observeSuspect(
                feature.sequence, feature.timestamp, &error);
            if (decision ==
                vins::BackendCandidateLifecycle::Decision::kContinue) {
              VINS_WARN << "Dual-state suspect candidate retained for "
                           "delayed approval: sequence="
                        << feature.sequence << ", timestamp="
                        << feature.timestamp;
            }
          } else {
            decision = backendCandidateStore->observe(
                feature.sequence, feature.timestamp, false, &error);
          }
          if (decision ==
              vins::BackendCandidateLifecycle::Decision::kCommit) {
            backendCandidateInputBuffer.commit();
            backendRecoveryFirstSuspectSequence = 0;
            backendRecoveryFirstSuspectRawImageSequence = 0;
            backendRecoverySelectionLogged = false;
            backendRecoveryEvaluationComplete = false;
          } else if (
              decision ==
              vins::BackendCandidateLifecycle::Decision::kDiscard) {
            backendCandidateInputBuffer.discard();
            if (replayMultiCheckpointRecovery &&
                backendRecoveryFirstSuspectSequence > 0 &&
                !backendRecoverySelectionLogged) {
              constexpr std::uint64_t kMinimumRecoveryRewindFrames =
                  WINDOW_SIZE;
              const vins::BackendRecoveryCheckpoint *selected = nullptr;
              {
                std::lock_guard<std::mutex> recovery_lock(
                    backendRecoveryMutex);
                selected =
                    backendRecoveryCheckpoints.selectForRewind(
                        backendRecoveryFirstSuspectSequence,
                        kMinimumRecoveryRewindFrames);
                if (selected) {
                  VINS_WARN
                      << "Multi-checkpoint rewind candidate selected: "
                      << "first_suspect_sequence="
                      << backendRecoveryFirstSuspectSequence
                      << ", checkpoint_feature_sequence="
                      << selected->feature_sequence
                      << ", checkpoint_raw_image_sequence="
                      << selected->raw_image_sequence
                      << ", checkpoint_timestamp="
                      << selected->timestamp
                      << "; execution remains disabled";
                }
              }
              if (!selected) {
                VINS_ERROR
                    << "Multi-checkpoint has no eligible rewind "
                       "candidate for first_suspect_sequence="
                    << backendRecoveryFirstSuspectSequence;
              }
              backendRecoverySelectionLogged = true;
            }
            // The first phase-1 candidate appeared healthy for only three
            // selected updates and became suspect again two raw frames after
            // handover.  Observe a full eight selected updates so this
            // short-lived prefix cannot be approved.
            constexpr std::uint64_t kRecoveryApprovalRawFrames = 16;
            if (replayMultiCheckpointRecovery &&
                !backendRecoveryEvaluationComplete &&
                backendRecoveryFirstSuspectRawImageSequence > 0 &&
                feature_raw_image_sequence >=
                    backendRecoveryFirstSuspectRawImageSequence +
                        kRecoveryApprovalRawFrames) {
              // Freeze the live tracker at a raw-image boundary while both
              // isolated candidates catch up.  The callback lock order is
              // tracker -> recovery ring -> feature queue, which is preserved
              // here for an atomic backend/tracker handover.
              std::unique_lock<std::mutex> live_tracker_lock(
                  featureTrackerMutex);
              vins::BackendState::Ptr recovery_backend;
              std::unique_ptr<FeatureTracker> recovery_tracker;
              vins::BackendRecoverySensorSlice recovery_slice;
              std::uint64_t checkpoint_sequence = 0;
              {
                std::lock_guard<std::mutex> recovery_lock(
                    backendRecoveryMutex);
                const auto *checkpoint =
                    backendRecoveryCheckpoints.selectForRewind(
                        backendRecoveryFirstSuspectSequence,
                        WINDOW_SIZE);
                if (checkpoint) {
                  recovery_backend =
                      checkpoint->backend->clone(&error);
                  recovery_tracker =
                      checkpoint->tracker->cloneWithOptions(
                          recovery_backend
                              ? recovery_backend->options
                              : std::shared_ptr<VINSOptions>{});
                  const double replay_end =
                      backendRecoverySensorRing.newestImageTimestamp();
                  recovery_slice = backendRecoverySensorRing.slice(
                      checkpoint->timestamp,
                      std::isfinite(replay_end)
                          ? replay_end + options->time_delay + 0.02
                          : feature.timestamp +
                                options->time_delay + 0.02);
                  checkpoint_sequence =
                      checkpoint->feature_sequence;
                  backendRecoveryCheckpoints.markAttempted(
                      checkpoint_sequence);
                }
              }
              if (recovery_backend && recovery_tracker) {
                vins::BackendRecoveryCandidateEvidence phase0;
                vins::BackendRecoveryCandidateEvidence phase1;
                vins::BackendState::Ptr phase0_backend;
                vins::BackendState::Ptr phase1_backend;
                std::unique_ptr<FeatureTracker> phase0_tracker;
                std::unique_ptr<FeatureTracker> phase1_tracker;
                std::string phase0_error;
                std::string phase1_error;
                evaluateRecoveryCandidate(
                    *recovery_backend, *recovery_tracker,
                    recovery_slice, 0,
                    backendRecoveryFirstSuspectRawImageSequence,
                    published_latest_state, &phase0,
                    &phase0_backend, &phase0_tracker, &phase0_error);
                evaluateRecoveryCandidate(
                    *recovery_backend, *recovery_tracker,
                    recovery_slice, 1,
                    backendRecoveryFirstSuspectRawImageSequence,
                    published_latest_state, &phase1,
                    &phase1_backend, &phase1_tracker, &phase1_error);
                vins::BackendRecoveryCandidateSelector selector(
                    static_cast<std::size_t>(
                        replayDualStateRequiredHealthyUpdates));
                const auto selection = selector.select(
                    phase0, phase1,
                    backendRecoveryFirstSuspectRawImageSequence);
                VINS_WARN
                    << "Multi-checkpoint isolated replay result: "
                    << "checkpoint_sequence=" << checkpoint_sequence
                    << ", phase0_through="
                    << phase0.replayed_through_sequence
                    << ", phase0_healthy_followups="
                    << phase0.healthy_updates_after_corridor
                    << ", phase0_fatal="
                    << phase0.fatal_processing_error
                    << ", phase0_correction="
                    << phase0.published_output_correction_m
                    << ", phase0_error=" << phase0_error
                    << ", phase1_through="
                    << phase1.replayed_through_sequence
                    << ", phase1_healthy_followups="
                    << phase1.healthy_updates_after_corridor
                    << ", phase1_fatal="
                    << phase1.fatal_processing_error
                    << ", phase1_correction="
                    << phase1.published_output_correction_m
                    << ", phase1_error=" << phase1_error
                    << ", selected_phase="
                    << selection.selected_phase;
                vins::BackendState::Ptr selected_backend;
                std::unique_ptr<FeatureTracker> selected_tracker;
                if (selection.selected_phase == 0) {
                  selected_backend = std::move(phase0_backend);
                  selected_tracker = std::move(phase0_tracker);
                } else if (selection.selected_phase == 1) {
                  selected_backend = std::move(phase1_backend);
                  selected_tracker = std::move(phase1_tracker);
                }
                if (selected_backend && selected_tracker) {
                  recovery_handover_timestamp =
                      selected_backend->current_timestamp -
                      selected_backend->options->time_delay;
                  if (!backendCandidateStore->adoptRecoveredCommitted(
                          std::move(selected_backend), &error)) {
                    VINS_ERROR
                        << "Multi-checkpoint recovered commit adoption "
                           "failed: "
                        << error;
                  } else {
                    featureTracker = std::move(*selected_tracker);
                    inputDownsamplePhase = selection.selected_phase;
                    {
                      std::lock_guard<std::mutex> feature_lock(
                          featureBufferMutex);
                      clearBuffer(featureBuffer);
                      backendFeatureTrackerSnapshots.clear();
                      backendFeatureRawImageSequences.clear();
                    }
                    backendCandidateInputBuffer.reset();
                    // Keep the older, unattempted checkpoints and their raw
                    // sensor suffixes.  A successful handover can expose a
                    // second suspect corridor almost immediately, before a
                    // new checkpoint is old enough to satisfy the rewind
                    // distance.  The attempted set prevents reusing the
                    // checkpoint that produced this handover.
                    recovery_handover = true;
                    recovery_handover_phase =
                        selection.selected_phase;
                    backendRecoveryFirstSuspectSequence = 0;
                    backendRecoveryFirstSuspectRawImageSequence = 0;
                    backendRecoverySelectionLogged = false;
                    VINS_WARN
                        << "Multi-checkpoint recovery handover accepted: "
                        << "phase=" << recovery_handover_phase
                        << ", timestamp="
                        << recovery_handover_timestamp;
                  }
                }
              } else if (checkpoint_sequence == 0) {
                VINS_ERROR
                    << "Multi-checkpoint isolated replay has no eligible "
                       "checkpoint for first_suspect_sequence="
                    << backendRecoveryFirstSuspectSequence;
              } else {
                VINS_ERROR
                    << "Multi-checkpoint isolated replay could not "
                       "clone selected checkpoint: "
                    << error;
              }
              // One paired replay is the maximum synchronous work allowed
              // for a live discard.  Exhaustively trying older checkpoints
              // blocked camera callbacks long enough to corrupt IMU timing.
              backendRecoveryEvaluationComplete = true;
            }
          } else if (
              decision ==
              vins::BackendCandidateLifecycle::Decision::kProtocolError) {
            VINS_ERROR << "Dual-state lifecycle protocol failed: "
                       << error;
            replayDualStateBackend = false;
            backendCandidateInputBuffer.reset();
          }

          if (!applyBackendState(
                  *backendCandidateStore->committed(), &error)) {
            VINS_ERROR << "Dual-state committed restore failed: "
                       << error;
            replayDualStateBackend = false;
          }
          if (decision !=
                  vins::BackendCandidateLifecycle::Decision::kCommit &&
              !recovery_handover) {
            latest_state = published_latest_state;
            latest_Q = published_latest_orientation;
            latestImuData = published_latest_imu_data;
            imu_odom = published_imu_odometry;
          }
          backendCandidateWorkspaceActive = false;
          backendCandidateProcessingFailed = false;

          if (recovery_handover) {
            publishCurrentBackendOutputs(
                recovery_handover_timestamp);
            VINS_WARN
                << "Multi-checkpoint recovery published committed "
                   "handover: phase="
                << recovery_handover_phase << ", timestamp="
                << recovery_handover_timestamp
                << ", generation="
                << backendCandidateStore->committed()->generation;
          } else if (decision ==
              vins::BackendCandidateLifecycle::Decision::kCommit) {
            publishCurrentBackendOutputs(feature.timestamp);
            constexpr std::uint64_t kRecoveryCheckpointSpacing = 5;
            if (replayMultiCheckpointRecovery &&
                feature_tracker_snapshot &&
                feature_raw_image_sequence > 0 &&
                (lastBackendRecoveryCheckpointSequence == 0 ||
                 feature.sequence >=
                     lastBackendRecoveryCheckpointSequence +
                         kRecoveryCheckpointSpacing)) {
              auto checkpoint_backend =
                  backendCandidateStore->committed()->clone(&error);
              auto checkpoint_tracker =
                  checkpoint_backend
                      ? feature_tracker_snapshot->cloneWithOptions(
                            checkpoint_backend->options)
                      : std::unique_ptr<FeatureTracker>{};
              bool checkpoint_saved = false;
              {
                std::lock_guard<std::mutex> recovery_lock(
                    backendRecoveryMutex);
                checkpoint_saved =
                    backendRecoveryCheckpoints.push(
                        feature.sequence, feature_raw_image_sequence,
                        feature.timestamp, std::move(checkpoint_backend),
                        std::move(checkpoint_tracker), &error);
                if (checkpoint_saved) {
                  lastBackendRecoveryCheckpointSequence =
                      feature.sequence;
                  backendRecoverySensorRing.trimBefore(
                      backendRecoveryCheckpoints.oldestTimestamp());
                }
              }
              if (!checkpoint_saved) {
                VINS_ERROR
                    << "Multi-checkpoint capture failed: " << error;
                replayMultiCheckpointRecovery = false;
              } else {
                VINS_INFO
                    << "Multi-checkpoint saved: feature_sequence="
                    << feature.sequence << ", raw_image_sequence="
                    << feature_raw_image_sequence << ", timestamp="
                    << feature.timestamp;
              }
            }
            VINS_INFO << "Dual-state candidate committed: sequence="
                      << feature.sequence << ", timestamp="
                      << feature.timestamp << ", generation="
                      << backendCandidateStore->committed()->generation;
          } else if (
              decision ==
              vins::BackendCandidateLifecycle::Decision::kDiscard) {
            VINS_WARN << "Dual-state candidate discarded: sequence="
                      << feature.sequence << ", timestamp="
                      << feature.timestamp << ", retained_imu_steps="
                      << backendCandidateInputBuffer.pendingStepCount();
          }
          continue;
        }
      }
    }

    currentTimestamp = target_timestamp;
    vector<IMUData> imu_datas;
    if (options->hasImu()) {
      getIMUInterval(previousTimestamp, currentTimestamp, imu_datas);
      VINS_INFO << "IMU interval trace: feature_sequence="
                << feature.sequence << ", raw_image_sequence="
                << feature.raw_image_sequence << ", image_timestamp="
                << std::fixed << std::setprecision(9) << feature.timestamp
                << ", previous_timestamp=" << previousTimestamp
                << ", current_timestamp=" << currentTimestamp
                << ", sample_count=" << imu_datas.size()
                << ", first_sample_sequence="
                << (imu_datas.empty() ? 0 :
                    imu_datas.front().source_sequence)
                << ", last_sample_sequence="
                << (imu_datas.empty() ? 0 :
                    imu_datas.back().source_sequence)
                << ", first_sample_timestamp="
                << (imu_datas.empty() ? 0.0 : imu_datas.front().timestamp)
                << ", last_sample_timestamp="
                << (imu_datas.empty() ? 0.0 : imu_datas.back().timestamp)
                << ", interval_fingerprint=0x" << std::hex
                << imuIntervalFingerprint(imu_datas) << std::dec;
      if (!isFirstPoseInitialized) initFirstIMUPose(imu_datas);
      for (size_t i = 0; i < imu_datas.size(); i++) {
        double dt;
        if (i == 0)
          dt = imu_datas[i].timestamp - previousTimestamp;
        else if (i == imu_datas.size() - 1)
          dt = currentTimestamp - imu_datas[i - 1].timestamp;
        else
          dt = imu_datas[i].timestamp - imu_datas[i - 1].timestamp;
        processIMU(imu_datas[i], dt);
      }
    }
    processImage(feature.features, feature.timestamp);
    VINS_INFO << "Estimator commit trace: feature_sequence="
              << feature.sequence << ", raw_image_sequence="
              << feature.raw_image_sequence << ", timestamp=" << std::fixed
              << std::setprecision(9) << feature.timestamp
              << ", nonlinear_update=" << nonLinearUpdateCount
              << ", solver_state="
              << (solver_flag == SolverState::NON_LINEAR ? "nonlinear" :
                                                            "initial")
              << ", marginalization="
              << (marginalization_flag == MarginalizationType::MARGIN_OLD
                      ? "old" : "second_new")
              << ", state_window_hash=0x" << std::hex
              << stateWindowFingerprint(estimator_state, frameCount)
              << ", outgoing_prior_hash=0x"
              << priorFingerprint(last_marginalization_info) << std::dec
              << ", prior_blocks="
              << last_marginalization_parameter_blocks.size()
              << ", solver_termination="
              << ceres::TerminationTypeToString(lastOptimizationTermination)
              << ", reset_requested="
              << (previousTimestamp < 0.0 && currentTimestamp == 0.0);
    emitDeterministicCommitOutput();
    // An internal fail-safe reset can happen inside processImage().  In that
    // case resetStateLocked() deliberately clears the estimator epoch.  Do
    // not overwrite it with the pre-reset timestamp; the next frame must be
    // treated as the first frame of a new IMU integration epoch.
    if (previousTimestamp < 0.0 && currentTimestamp == 0.0) {
      continue;
    }
    if (replayMotionOnsetWarmReinitialization &&
        isNonLinearSolver()) {
      recordMotionOnsetFrameHistory(feature);
    }
    printStatistics(currentTimestamp);
    collectPointCloudAll(feature.timestamp);
    previousTimestamp = currentTimestamp;
  }
}
void Estimator::updateCameraPose(int index) {
  PoseData pose;
  pose.position = cameraTranslation[index];
  pose.orientation = Quaterniond(cameraRotation[index]);
  if (!backendCandidateWorkspaceActive) {
    safe_camera_pose[index].set(pose);
  }
}

void Estimator::collectPointCloudAll(Timestamp timestamp) {
  if (backendCandidateWorkspaceActive) return;
  PointCloudData main_cloud;
  PointCloudData point_cloud;
  PointCloudData margin_cloud;
  main_cloud.timestamp = timestamp;
  point_cloud.timestamp = timestamp;
  margin_cloud.timestamp = timestamp;
  point_cloud.channels.resize(5);
  point_cloud.channels[0].name = "normalized_x";
  point_cloud.channels[1].name = "normalized_y";
  point_cloud.channels[2].name = "pixel_u";
  point_cloud.channels[3].name = "pixel_v";
  point_cloud.channels[4].name = "feature_id";

  for (const auto &it_per_id : featureManager.feature) {
    int used_num = it_per_id.feature_per_frame.size();
    int start_frame = it_per_id.start_frame;

    // Only use well-tracked and solved features
    if (used_num < 2 || !it_per_id.isSolved()) continue;

    int imu_i = start_frame;
    const auto &first_obs = it_per_id.feature_per_frame[0];
    Eigen::Vector3d pts_i = first_obs.point * it_per_id.estimated_depth;
    Eigen::Vector3d w_pts_i =
        estimator_state[imu_i].rotation *
            (cameraRotation[0] * pts_i + cameraTranslation[0]) +
        estimator_state[imu_i].position;

    // 1. Main cloud
    if (start_frame < WINDOW_SIZE - 2 && start_frame <= WINDOW_SIZE * 3 / 4) {
      main_cloud.points.emplace_back(w_pts_i);
    }

    // 2. Margin cloud
    if (start_frame == 0 && used_num <= 2) {
      margin_cloud.points.emplace_back(w_pts_i);
    }

    // 3. Current frame cloud
    if (solver_flag == SolverState::NON_LINEAR &&
        marginalization_flag == MarginalizationType::MARGIN_OLD) {
      if (start_frame < WINDOW_SIZE - 2 &&
          start_frame + used_num - 1 >= WINDOW_SIZE - 2) {
        int imu_j = WINDOW_SIZE - 2 - start_frame;
        const auto &cur_obs = it_per_id.feature_per_frame[imu_j];
        point_cloud.points.emplace_back(w_pts_i);

        point_cloud.channels[0].values.push_back(cur_obs.point.x());
        point_cloud.channels[1].values.push_back(cur_obs.point.y());
        point_cloud.channels[2].values.push_back(cur_obs.uv.x());
        point_cloud.channels[3].values.push_back(cur_obs.uv.y());
        point_cloud.channels[4].values.push_back(it_per_id.feature_id);
      }
    }
  }
  if (!main_cloud.points.empty()) {
    safe_main_cloud.set(main_cloud);
  }
  if (!margin_cloud.points.empty()) {
    safe_margin_cloud.set(margin_cloud);
  }
  if (solver_flag == SolverState::NON_LINEAR &&
      marginalization_flag == MarginalizationType::MARGIN_OLD) {
    // The keyframe observations above come from this window slot, not from
    // the newest camera frame passed to collectPointCloudAll().  Set this
    // before copying into the thread-safe publisher buffer.
    point_cloud.timestamp = estimator_state[WINDOW_SIZE - 2].timestamp;
  }
  if (!point_cloud.points.empty()) {
    safe_point_cloud.set(point_cloud);
  }
  if (solver_flag == SolverState::NON_LINEAR &&
      marginalization_flag == MarginalizationType::MARGIN_OLD) {
    int i = WINDOW_SIZE - 2;
    PoseData pose;
    pose.timestamp = estimator_state[i].timestamp;
    pose.position = estimator_state[i].position;
    pose.orientation = Quaterniond(estimator_state[i].rotation);
    safe_keyframe_pose.set(pose);
  }
}

void Estimator::printStatistics(Timestamp timestamp) {
  if (solver_flag != SolverState::NON_LINEAR) return;
  VINS_DEBUG << "position: (" << estimator_state[WINDOW_SIZE].position.x()
             << "," << estimator_state[WINDOW_SIZE].position.y() << ","
             << estimator_state[WINDOW_SIZE].position.z() << ")";
}

void Estimator::initFirstIMUPose(const vector<IMUData> &data) {
  isFirstPoseInitialized = true;
  Vector3d averageAcceleration = Vector3d::Zero();

  for (const auto &imu : data) {
    averageAcceleration += imu.linear_acceleration;
  }

  averageAcceleration /= data.size();
  VINS_INFO << "Average acceleration: " << averageAcceleration.transpose();

  Matrix3d initialRotation = Utility::g2R(averageAcceleration);
  double yaw = Utility::R2ypr(initialRotation).x();
  initialRotation = Utility::ypr2R(Vector3d{-yaw, 0, 0}) * initialRotation;
  estimator_state[0].rotation = initialRotation;
  VINS_INFO << "init R0: \n" << estimator_state[0].rotation;
}

void Estimator::setFirstPose(const Eigen::Vector3d &position,
                             const Eigen::Matrix3d &rotation) {
  estimator_state[0].position = position;
  estimator_state[0].rotation = rotation;
}

void Estimator::processIMU(const IMUData &data, double deltaTime) {
  if (!isFirstIMUReceived) {
    isFirstIMUReceived = true;
    previousImuData = data;
  }

  if (!pre_integrations[frameCount]) {
    pre_integrations[frameCount] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[frameCount].accel_bias,
        estimator_state[frameCount].gyro_bias, options->imu);
  }
  if (frameCount != 0) {
    IMUData delta_imu = data;
    delta_imu.timestamp = deltaTime;

    pre_integrations[frameCount]->push_back(delta_imu);
    tmp_pre_integration->push_back(delta_imu);
    deltaTimeImuBuffer[frameCount].push_back(delta_imu);

    updateStateWithIMU(data, deltaTime);
  }
  previousImuData = data;
}

void Estimator::updateStateWithIMU(const IMUData &data, double deltaTime) {
  int j = frameCount;
  Vector3d un_acc_0 =
      estimator_state[j].rotation * (previousImuData.linear_acceleration -
                                     estimator_state[j].accel_bias) -
      gravity;
  Vector3d un_gyr =
      0.5 * (previousImuData.angular_velocity + data.angular_velocity) -
      estimator_state[j].gyro_bias;
  estimator_state[j].rotation *=
      Utility::deltaQ(un_gyr * deltaTime).toRotationMatrix();
  Vector3d un_acc_1 =
      estimator_state[j].rotation *
          (data.linear_acceleration - estimator_state[j].accel_bias) -
      gravity;
  Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  estimator_state[j].position += deltaTime * estimator_state[j].velocity +
                                 0.5 * deltaTime * deltaTime * un_acc;
  estimator_state[j].velocity += deltaTime * un_acc;
}

void Estimator::processImage(const FeatureFrame &features,
                             Timestamp timestamp) {
  motionOnsetWarmReinitializationTriggeredThisSolve = false;
  motionOnsetWarmReinitializationAppliedThisSolve = false;
  motionOnsetReleaseThisSolve = motionOnsetReleasePending;
  motionOnsetBiasAnchorThisSolve = motionOnsetBiasAnchorActive;
  constexpr double kCameraFocalLengthPx = 277.191356;
  vector<double> feature_flow_speeds;
  feature_flow_speeds.reserve(features.size());
  for (const auto &feature : features) {
    for (const auto &observation : feature.second) {
      if (observation.first != 0 || observation.second.size() < 7) continue;
      const double normalized_speed =
          observation.second.segment<2>(5).norm();
      if (std::isfinite(normalized_speed) && normalized_speed > 1e-9) {
        feature_flow_speeds.push_back(
            normalized_speed * kCameraFocalLengthPx);
      }
      break;
    }
  }
  currentFeatureFlowCount = static_cast<int>(feature_flow_speeds.size());
  if (feature_flow_speeds.empty()) {
    currentFeatureFlowMedianPxS = std::numeric_limits<double>::infinity();
  } else {
    const size_t middle = feature_flow_speeds.size() / 2;
    std::nth_element(feature_flow_speeds.begin(),
                     feature_flow_speeds.begin() + middle,
                     feature_flow_speeds.end());
    currentFeatureFlowMedianPxS = feature_flow_speeds[middle];
  }
  setMarginalizationFlag(features);
  if (shouldForceReplayOldMarginalization(timestamp)) {
    marginalization_flag = MarginalizationType::MARGIN_OLD;
    VINS_WARN << "Replay-only topology-matched MARGIN_OLD override: "
              << "timestamp=" << timestamp;
  }
  if (motionOnsetReleaseThisSolve) {
    // The release solve must form the first usable prior even if the regular
    // parallax policy would otherwise discard the second-newest frame.
    marginalization_flag = MarginalizationType::MARGIN_OLD;
  }
  insertImageFrame(features, timestamp);
  maybeRebuildContinuousWindowAtMotionOnset(features, timestamp);
  if (motionOnsetBiasAnchorThisSolve) {
    // A prior formed while the replacement window is still vertically
    // excited but horizontally weak simply stores the same BA ambiguity that
    // the stale prior carried. Keep the warm transition as a sequence of
    // continuous-window batch solves; the solve that satisfies the release
    // evidence below creates the first prior handed to normal operation.
    last_marginalization_info.reset();
    last_marginalization_parameter_blocks.clear();
    lastMarginalizationSourceTimestamp =
        std::numeric_limits<double>::quiet_NaN();
    lastMarginalizationSourceType =
        "motion_onset_warm_transition";
  }
  handleExtrinsicInitialization();
  if (!isNonLinearSolver()) {
    processInitialization(timestamp);
  } else {
    processNonLinearSolver(timestamp);
  }
  if (motionOnsetReleaseThisSolve && isNonLinearSolver()) {
    motionOnsetBiasAnchorActive = false;
    motionOnsetReleasePending = false;
    VINS_WARN
        << "Motion-onset warm transition released: timestamp="
        << timestamp << ", anchored_solves="
        << motionOnsetBiasAnchorSolveCount
        << ", accel_bias="
        << estimator_state[WINDOW_SIZE].accel_bias.transpose()
        << ", accel_bias_norm="
        << estimator_state[WINDOW_SIZE].accel_bias.norm();
  } else if (motionOnsetBiasAnchorThisSolve &&
      motionOnsetBiasAnchorActive && isNonLinearSolver()) {
    ++motionOnsetBiasAnchorSolveCount;
    constexpr int kExcitingMotionFrames = 3;
    const StateData &latest = estimator_state[WINDOW_SIZE];
    vins::MotionOnsetWarmReleaseEvidence release_evidence;
    release_evidence.prior_free_solve_count =
        motionOnsetBiasAnchorSolveCount;
    release_evidence.horizontal_speed_mps =
        latest.velocity.head<2>().norm();
    release_evidence.vertical_speed_mps = latest.velocity.z();
    release_evidence.median_flow_px_s = currentFeatureFlowMedianPxS;
    release_evidence.visual_tracks = currentFeatureFlowCount;
    const bool exciting_motion =
        vins::MotionOnsetWarmReleaseGate().evaluate(release_evidence);
    motionOnsetExcitingMotionCount =
        exciting_motion ? motionOnsetExcitingMotionCount + 1 : 0;
    if (motionOnsetExcitingMotionCount >= kExcitingMotionFrames &&
        marginalization_flag == MarginalizationType::MARGIN_OLD) {
      motionOnsetReleasePending = true;
      VINS_WARN
          << "Motion-onset warm release solve scheduled: timestamp="
          << timestamp << ", anchored_solves="
          << motionOnsetBiasAnchorSolveCount
          << ", horizontal_speed_mps="
          << latest.velocity.head<2>().norm()
          << ", median_flow_px_s=" << currentFeatureFlowMedianPxS;
    }
  }
}

void Estimator::recordMotionOnsetFrameHistory(
    const TimestampedFeatureFrame &frame) {
  int state_index = -1;
  for (int i = frameCount; i >= 0; --i) {
    if (std::abs(estimator_state[i].timestamp - frame.timestamp) < 1e-6) {
      state_index = i;
      break;
    }
  }
  if (state_index < 0) return;

  MotionOnsetFrameHistory entry;
  entry.frame = frame;
  entry.state = estimator_state[state_index];
  entry.range = rangeMeasurements[state_index];
  entry.metric_velocity = metricVelocityMeasurements[state_index];
  entry.rotor_dynamics = rotorDynamicsMeasurements[state_index];
  entry.median_flow_px_s = currentFeatureFlowMedianPxS;
  entry.visual_tracks = currentFeatureFlowCount;
  motionOnsetFrameHistory.push_back(std::move(entry));
  constexpr std::size_t kHistoryFrames = 24;
  while (motionOnsetFrameHistory.size() > kHistoryFrames) {
    motionOnsetFrameHistory.pop_front();
  }
}

bool Estimator::maybeRebuildContinuousWindowAtMotionOnset(
    const FeatureFrame &features, Timestamp timestamp) {
  if (!replayMotionOnsetWarmReinitialization ||
      replayMotionOnsetWarmReinitializationConsumed ||
      !isNonLinearSolver() || frameCount != WINDOW_SIZE ||
      motionOnsetFrameHistory.size() < WINDOW_SIZE) {
    return false;
  }

  double maximum_interval = 0.0;
  int gap_index = -1;
  for (int i = 1; i <= WINDOW_SIZE; ++i) {
    if (!pre_integrations[i]) continue;
    const double interval = pre_integrations[i]->sum_dt;
    if (interval > maximum_interval) {
      maximum_interval = interval;
      gap_index = i;
    }
  }
  if (gap_index <= 0) return false;

  std::vector<TimestampedFeatureFrame> recent_frames;
  std::vector<StateData> recent_states;
  recent_frames.reserve(WINDOW_SIZE + 1);
  recent_states.reserve(WINDOW_SIZE + 1);
  const auto history_begin =
      motionOnsetFrameHistory.end() - WINDOW_SIZE;
  for (auto it = history_begin; it != motionOnsetFrameHistory.end(); ++it) {
    recent_frames.push_back(it->frame);
    recent_states.push_back(it->state);
  }
  recent_frames.push_back(
      TimestampedFeatureFrame{0, 0, timestamp, features});
  recent_states.push_back(estimator_state[WINDOW_SIZE]);

  vins::MotionOnsetWindowEvidence gate_evidence;
  gate_evidence.maximum_imu_interval_s = maximum_interval;
  gate_evidence.maximum_interval_would_be_cut_off =
      maximum_interval > 10.0 && !replayIncludeLongImuFactors;
  gate_evidence.old_slots_before_gap = gap_index;
  gate_evidence.recent_slots_after_gap =
      WINDOW_SIZE + 1 - gap_index;
  gate_evidence.recent_frames.reserve(recent_frames.size());
  for (std::size_t i = 0; i < recent_frames.size(); ++i) {
    vins::MotionOnsetFrameEvidence frame_evidence;
    frame_evidence.timestamp = recent_frames[i].timestamp;
    if (i > 0) {
      const double dt = recent_states[i].timestamp -
                        recent_states[i - 1].timestamp;
      if (dt > 0.0) {
        frame_evidence.position_step_speed_mps =
            (recent_states[i].position -
             recent_states[i - 1].position)
                .norm() /
            dt;
      }
    }
    if (i < recent_frames.size() - 1) {
      const auto &history =
          *(history_begin + static_cast<std::ptrdiff_t>(i));
      frame_evidence.median_flow_px_s = history.median_flow_px_s;
      frame_evidence.visual_tracks = history.visual_tracks;
    } else {
      frame_evidence.median_flow_px_s = currentFeatureFlowMedianPxS;
      frame_evidence.visual_tracks = currentFeatureFlowCount;
    }
    gate_evidence.recent_frames.push_back(frame_evidence);
  }

  const vins::MotionOnsetWarmReinitializationGate gate;
  const bool gate_passed = gate.evaluate(gate_evidence);
  if (!gate_passed && maximum_interval > 10.0 &&
      gate_evidence.recent_slots_after_gap >= 3) {
    std::ostringstream trace;
    for (const auto &frame : gate_evidence.recent_frames) {
      trace << "[" << frame.timestamp << ":"
            << frame.position_step_speed_mps << ":"
            << frame.median_flow_px_s << ":"
            << frame.visual_tracks << "]";
    }
    VINS_INFO
        << "Motion-onset warm gate evidence: timestamp=" << timestamp
        << ", maximum_imu_interval_s=" << maximum_interval
        << ", old_slots=" << gate_evidence.old_slots_before_gap
        << ", recent_slots=" << gate_evidence.recent_slots_after_gap
        << ", frames_timestamp_speed_flow_tracks=" << trace.str();
  }
  if (!gate_passed) return false;
  if (!rebuildContinuousWindow(recent_frames, recent_states)) {
    VINS_ERROR
        << "Motion-onset warm reinitialization could not reconstruct "
           "the continuous camera/IMU window; stale backend retained.";
    return false;
  }

  replayMotionOnsetWarmReinitializationConsumed = true;
  motionOnsetWarmReinitializationAppliedThisSolve = true;
  motionOnsetWarmReinitializationTriggeredThisSolve = true;
  // The replacement solve uses the established one-window gauge below. Keep
  // its latest bias through the weakly observable vertical-onset phase, then
  // release it after one full window and sustained horizontal excitation.
  motionOnsetBiasAnchorActive = true;
  motionOnsetReleasePending = false;
  motionOnsetReleaseThisSolve = false;
  motionOnsetBiasAnchorSolveCount = 0;
  motionOnsetExcitingMotionCount = 0;
  VINS_WARN
      << "Motion-onset continuous-window warm reinitialization applied: "
      << "timestamp=" << timestamp
      << ", discarded_maximum_imu_interval_s=" << maximum_interval
      << ", discarded_old_slots=" << gap_index
      << ", discarded_recent_slots="
      << WINDOW_SIZE + 1 - gap_index
      << ", rebuilt_start_timestamp="
      << estimator_state[0].timestamp
      << ", rebuilt_end_timestamp="
      << estimator_state[WINDOW_SIZE].timestamp
      << ", retained_position="
      << estimator_state[WINDOW_SIZE].position.transpose()
      << ", retained_velocity="
      << estimator_state[WINDOW_SIZE].velocity.transpose();
  return true;
}

bool Estimator::rebuildContinuousWindow(
    const std::vector<TimestampedFeatureFrame> &frames,
    const std::vector<StateData> &states) {
  if (frames.size() != WINDOW_SIZE + 1 ||
      states.size() != WINDOW_SIZE + 1) {
    return false;
  }

  std::vector<IMUData> imu_history;
  {
    std::lock_guard<std::mutex> history_lock(
        motionOnsetImuHistoryMutex);
    imu_history.assign(motionOnsetImuHistory.begin(),
                       motionOnsetImuHistory.end());
  }
  if (imu_history.size() < 2 ||
      imu_history.front().timestamp > frames.front().timestamp ||
      imu_history.back().timestamp < frames.back().timestamp) {
    return false;
  }

  auto integration_for_interval =
      [&](double start, double end, const Vector3d &accel_bias,
          const Vector3d &gyro_bias,
          std::vector<IMUData> *steps) -> IntegrationBase::Ptr {
    auto first_after =
        std::upper_bound(
            imu_history.begin(), imu_history.end(), start,
            [](double value, const IMUData &sample) {
              return value < sample.timestamp;
            });
    if (first_after == imu_history.begin() ||
        first_after == imu_history.end()) {
      return nullptr;
    }
    // Match getIMUInterval()/processIMU exactly. The first sample strictly
    // after the camera boundary was already the previous endpoint sample in
    // the online path, so production holds that sample over the truncated
    // boundary interval instead of interpolating from the last sample before
    // the boundary.
    const IMUData &initial_sample = *first_after;
    auto integration = std::make_shared<IntegrationBase>(
        initial_sample, accel_bias, gyro_bias, options->imu);
    double previous_time = start;
    bool reached_end = false;
    for (auto it = first_after; it != imu_history.end(); ++it) {
      const double sample_time = std::min(it->timestamp, end);
      const double dt = sample_time - previous_time;
      if (dt > 0.0) {
        IMUData step = *it;
        step.timestamp = dt;
        integration->push_back(step);
        if (steps) steps->push_back(step);
        previous_time = sample_time;
      }
      if (it->timestamp >= end) {
        reached_end = true;
        break;
      }
    }
    return reached_end ? integration : nullptr;
  };

  RangeData current_range = rangeMeasurements[WINDOW_SIZE];
  MetricVelocityData current_metric_velocity =
      metricVelocityMeasurements[WINDOW_SIZE];
  RotorDynamicsData current_rotor_dynamics =
      rotorDynamicsMeasurements[WINDOW_SIZE];

  FeatureManager rebuilt_feature_manager;
  rebuilt_feature_manager.setOptions(options);
  std::map<double, ImageFrame> rebuilt_image_frames;
  for (int i = 0; i <= WINDOW_SIZE; ++i) {
    if (!std::isfinite(frames[i].timestamp) ||
        std::abs(states[i].timestamp - frames[i].timestamp) > 1e-6 ||
        (i > 0 &&
         frames[i].timestamp <= frames[i - 1].timestamp)) {
      return false;
    }
    rebuilt_feature_manager.addFeatureCheckParallax(
        i, frames[i].features, options->time_delay);
  }

  std::array<IntegrationBase::Ptr, WINDOW_SIZE + 1>
      rebuilt_preintegrations;
  std::array<std::vector<IMUData>, WINDOW_SIZE + 1>
      rebuilt_imu_steps;
  const IMUData &first_imu = *std::upper_bound(
      imu_history.begin(), imu_history.end(), frames.front().timestamp,
      [](double value, const IMUData &sample) {
        return value < sample.timestamp;
      });
  rebuilt_preintegrations[0] =
      std::make_shared<IntegrationBase>(
          first_imu, states[0].accel_bias, states[0].gyro_bias,
          options->imu);
  for (int i = 1; i <= WINDOW_SIZE; ++i) {
    rebuilt_preintegrations[i] = integration_for_interval(
        frames[i - 1].timestamp, frames[i].timestamp,
        states[i - 1].accel_bias, states[i - 1].gyro_bias,
        &rebuilt_imu_steps[i]);
    if (!rebuilt_preintegrations[i] ||
        rebuilt_preintegrations[i]->sum_dt <= 0.0 ||
        rebuilt_preintegrations[i]->sum_dt >= 10.0) {
      return false;
    }
  }

  for (int i = 0; i <= WINDOW_SIZE; ++i) {
    estimator_state[i] = states[i];
    pre_integrations[i] = rebuilt_preintegrations[i];
    deltaTimeImuBuffer[i] = std::move(rebuilt_imu_steps[i]);
    if (i < WINDOW_SIZE) {
      const auto &history =
          *(motionOnsetFrameHistory.end() - WINDOW_SIZE + i);
      rangeMeasurements[i] = history.range;
      metricVelocityMeasurements[i] = history.metric_velocity;
      rotorDynamicsMeasurements[i] = history.rotor_dynamics;
    } else {
      rangeMeasurements[i] = current_range;
      metricVelocityMeasurements[i] = current_metric_velocity;
      rotorDynamicsMeasurements[i] = current_rotor_dynamics;
    }
    ImageFrame image_frame(frames[i].features, frames[i].timestamp);
    image_frame.R = states[i].rotation;
    image_frame.T = states[i].position;
    image_frame.pre_integration = pre_integrations[i];
    rebuilt_image_frames.emplace(frames[i].timestamp,
                                 std::move(image_frame));
  }

  featureManager = std::move(rebuilt_feature_manager);
  all_image_frame = std::move(rebuilt_image_frames);
  last_marginalization_info = nullptr;
  last_marginalization_parameter_blocks.clear();
  lastMarginalizationSourceTimestamp =
      std::numeric_limits<double>::quiet_NaN();
  lastMarginalizationSourceType =
      "motion_onset_continuous_window";
  marginalization_flag = MarginalizationType::MARGIN_OLD;
  return true;
}

void Estimator::setMarginalizationFlag(const FeatureFrame &features) {
  if (featureManager.addFeatureCheckParallax(frameCount, features,
                                             options->time_delay)) {
    marginalization_flag = MarginalizationType::MARGIN_OLD;
  } else {
    marginalization_flag = MarginalizationType::MARGIN_SECOND_NEW;
  }
  if (marginalization_flag == MarginalizationType::MARGIN_SECOND_NEW &&
      replayMaximumImuWindowInterval > 0.0) {
    double maximum_interval = 0.0;
    for (int i = 1; i <= frameCount; ++i) {
      if (pre_integrations[i]) {
        maximum_interval =
            std::max(maximum_interval, pre_integrations[i]->sum_dt);
      }
    }
    if (maximum_interval > replayMaximumImuWindowInterval) {
      marginalization_flag = MarginalizationType::MARGIN_OLD;
      VINS_WARN << "Replay-only stale-keyframe flush selected: "
                << "maximum_imu_interval_s=" << maximum_interval
                << ", limit_s=" << replayMaximumImuWindowInterval;
    }
  }
}
void Estimator::insertImageFrame(const FeatureFrame &features,
                                 Timestamp timestamp) {
  estimator_state[frameCount].timestamp = timestamp;
  ImageFrame imageframe(features, timestamp);
  imageframe.pre_integration = std::move(tmp_pre_integration);
  all_image_frame.insert(make_pair(timestamp, imageframe));
  rangeMeasurements[frameCount] = RangeData();
  if (options->hasRange()) {
    std::lock_guard<std::mutex> lock(rangeMutex);
    while (rangeBuffer.size() > 1 &&
           rangeBuffer.front().timestamp < timestamp - options->range_max_age)
      rangeBuffer.pop();
    RangeData closest;
    double closest_dt = options->range_max_age;
    std::queue<RangeData> copy = rangeBuffer;
    while (!copy.empty()) {
      const double dt = std::abs(copy.front().timestamp - timestamp);
      if (dt <= closest_dt) {
        closest = copy.front();
        closest_dt = dt;
      }
      copy.pop();
    }
    if (closest.valid) rangeMeasurements[frameCount] = closest;
  }
  metricVelocityMeasurements[frameCount] = MetricVelocityData();
  if (options->hasGpsVelocity()) {
    std::lock_guard<std::mutex> lock(metricVelocityMutex);
    while (metricVelocityBuffer.size() > 1 &&
           metricVelocityBuffer.front().timestamp <
               timestamp - options->gps_velocity_max_age) {
      metricVelocityBuffer.pop();
    }
    MetricVelocityData closest;
    double closest_dt = options->gps_velocity_max_age;
    std::queue<MetricVelocityData> copy = metricVelocityBuffer;
    while (!copy.empty()) {
      const double dt = std::abs(copy.front().timestamp - timestamp);
      if (dt <= closest_dt) {
        closest = copy.front();
        closest_dt = dt;
      }
      copy.pop();
    }
    if (closest.valid) metricVelocityMeasurements[frameCount] = closest;
  }
  rotorDynamicsMeasurements[frameCount] = RotorDynamicsData();
  if (options->hasRotorDragDynamics()) {
    std::lock_guard<std::mutex> lock(rotorDynamicsMutex);
    while (rotorDynamicsBuffer.size() > 1 &&
           rotorDynamicsBuffer.front().timestamp <
               timestamp - options->dynamics_max_age) {
      rotorDynamicsBuffer.pop();
    }
    RotorDynamicsData closest;
    double closest_dt = options->dynamics_max_age;
    std::queue<RotorDynamicsData> copy = rotorDynamicsBuffer;
    while (!copy.empty()) {
      const double dt = std::abs(copy.front().timestamp - timestamp);
      if (dt <= closest_dt) {
        closest = copy.front();
        closest_dt = dt;
      }
      copy.pop();
    }
    if (closest.valid) rotorDynamicsMeasurements[frameCount] = closest;
  }
  tmp_pre_integration = std::make_shared<IntegrationBase>(
      previousImuData, estimator_state[frameCount].accel_bias,
      estimator_state[frameCount].gyro_bias, options->imu);
}

void Estimator::handleExtrinsicInitialization() {
  if (!options->isInitializingExtrinsic() || frameCount == 0) {
    return;
  }
  vector<pair<Vector3d, Vector3d>> corres =
      featureManager.getCorresponding(frameCount - 1, frameCount);
  Matrix3d calib_ric;
  if (initial_ex_rotation.CalibrationExRotation(
          corres, pre_integrations[frameCount]->delta_q, calib_ric)) {
    cameraRotation[0] = calib_ric;
    options->RIC[0] = calib_ric;
    options->extrinsic_estimation_mode = ExtrinsicEstimationMode::APPROXIMATE;
  }
}

void Estimator::processInitialization(Timestamp timestamp) {
  if (options->isMonoWithImu()) {
    processMonoWithImuInitialization(timestamp);
  }
  if (options->isStereoWithImu()) {
    processStereoWithImuInitialization();
  }
  if (options->isStereoWithoutImu()) {
    processStereoWithoutImuInitialization();
  }

  if (frameCount < WINDOW_SIZE) {
    frameCount++;
    estimator_state[frameCount] = estimator_state[frameCount - 1];
  }
}
void Estimator::processNonLinearSolver(Timestamp timestamp) {
  lastBackendHealthAvailable = false;
  lastBackendSolverUsable = false;
  lastBackendTransactionRejected = false;
  const bool replay_velocity_trace_enabled =
      replayVelocityTraceStartTimestamp >= 0.0 &&
      replayVelocityTraceEndTimestamp >= replayVelocityTraceStartTimestamp &&
      timestamp >= replayVelocityTraceStartTimestamp &&
      timestamp <= replayVelocityTraceEndTimestamp;
  StateData states_before_optimization[WINDOW_SIZE + 1];
  double pose_arrays_before_optimization[WINDOW_SIZE + 1][SIZE_POSE];
  double speed_bias_arrays_before_optimization[WINDOW_SIZE + 1]
                                              [SIZE_SPEEDBIAS];
  double extrinsic_arrays_before_optimization[2][SIZE_POSE];
  double time_delay_array_before_optimization[1];
  vector<double> inverse_depths_before_optimization;
  VectorXd depths_before_optimization;
  Matrix3d camera_rotation_before_optimization[2];
  Vector3d camera_translation_before_optimization[2];
  double time_delay_before_optimization = options->time_delay;
  MarginalizationInfo::Ptr marginalization_before_optimization;
  vector<double *> marginalization_blocks_before_optimization;
  double marginalization_source_timestamp_before_optimization =
      lastMarginalizationSourceTimestamp;
  const std::string marginalization_source_type_before_optimization =
      lastMarginalizationSourceType;
  int nonlinear_update_count_before_optimization = nonLinearUpdateCount;
  // Every nonlinear solve is a transaction.  Keeping this bounded window
  // snapshot lets the estimator reject a pose innovation without destroying
  // the healthy IMU-propagated state or starting a new VIO epoch.
  const bool snapshot_replay_solve = true;
  if (snapshot_replay_solve) {
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      states_before_optimization[i] = estimator_state[i];
    }
    depths_before_optimization = featureManager.getDepthVector();
    for (int i = 0; i < options->getNumCameras(); ++i) {
      camera_rotation_before_optimization[i] = cameraRotation[i];
      camera_translation_before_optimization[i] = cameraTranslation[i];
    }
    marginalization_before_optimization = last_marginalization_info;
    marginalization_blocks_before_optimization =
        last_marginalization_parameter_blocks;
  }
  const auto restore_replay_solve_snapshot = [&]() {
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      estimator_state[i] = states_before_optimization[i];
    }
    featureManager.setDepth(depths_before_optimization);
    for (int i = 0; i < options->getNumCameras(); ++i) {
      cameraRotation[i] = camera_rotation_before_optimization[i];
      cameraTranslation[i] = camera_translation_before_optimization[i];
      updateCameraPose(i);
    }
    options->time_delay = time_delay_before_optimization;
    last_marginalization_info = marginalization_before_optimization;
    last_marginalization_parameter_blocks =
        marginalization_blocks_before_optimization;
    lastMarginalizationSourceTimestamp =
        marginalization_source_timestamp_before_optimization;
    lastMarginalizationSourceType =
        marginalization_source_type_before_optimization;
    nonLinearUpdateCount = nonlinear_update_count_before_optimization;
  };

  if (!options->hasImu()) {
    featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                      cameraTranslation, cameraRotation);
  }
  std::unique_ptr<vins::BackendState> precommit_audit_solve_entry;
  const bool precommit_audit_this_solve =
      marginalization_flag == MarginalizationType::MARGIN_OLD &&
      shouldRunPreCommitConsistencyAudit(timestamp);
  if (precommit_audit_this_solve) {
    std::string audit_error;
    precommit_audit_solve_entry = captureBackendState(&audit_error);
    if (!precommit_audit_solve_entry) {
      throw std::runtime_error(
          "cannot capture pre-commit audit solve entry: " + audit_error);
    }
  }
  std::unordered_map<int, double> feature_depths_before_triangulation;
  if (replay_velocity_trace_enabled) {
    for (const auto &feature : featureManager.feature) {
      feature_depths_before_triangulation.emplace(
          feature.feature_id, feature.estimated_depth);
    }
  }
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  const char *recent_depth_refresh_value =
      std::getenv("VINS_REPLAY_RECENT_STEREO_DEPTH_REFRESH");
  const bool replay_recent_depth_refresh =
      recent_depth_refresh_value &&
      std::string(recent_depth_refresh_value) == "1";
  if (replay_recent_depth_refresh &&
      marginalization_flag == MarginalizationType::MARGIN_OLD) {
    const auto refresh =
        featureManager.refreshLongTrackDepthsFromLatestStereo(
            frameCount, estimator_state, cameraTranslation, cameraRotation,
            8);
    VINS_INFO << "Recent-stereo long-track depth refresh: timestamp="
              << timestamp << ", eligible=" << refresh.eligible_tracks
              << ", refreshed=" << refresh.refreshed_tracks
              << ", mean_relative_inverse_depth_change="
              << refresh.mean_relative_inverse_depth_change
              << ", maximum_relative_inverse_depth_change="
              << refresh.maximum_relative_inverse_depth_change;
  }
  if (replay_velocity_trace_enabled) {
    constexpr double kTraceFocalLengthPx = 277.191356;
    for (const auto &feature : featureManager.feature) {
      const auto depth_before_it =
          feature_depths_before_triangulation.find(feature.feature_id);
      const double depth_before =
          depth_before_it != feature_depths_before_triangulation.end()
              ? depth_before_it->second
              : std::numeric_limits<double>::quiet_NaN();
      const int start_slot = feature.start_frame;
      const int observation_count =
          static_cast<int>(feature.feature_per_frame.size());
      double first_pair_baseline = std::numeric_limits<double>::quiet_NaN();
      double first_pair_parallax_px =
          std::numeric_limits<double>::quiet_NaN();
      double first_pair_ray_angle_deg =
          std::numeric_limits<double>::quiet_NaN();
      double first_pair_raw_depth =
          std::numeric_limits<double>::quiet_NaN();
      double first_pair_singular_ratio =
          std::numeric_limits<double>::quiet_NaN();
      if (observation_count >= 2 && start_slot >= 0 &&
          start_slot + 1 <= frameCount) {
        const int second_slot = start_slot + 1;
        const Vector3d first_camera_position =
            estimator_state[start_slot].position +
            estimator_state[start_slot].rotation * cameraTranslation[0];
        const Vector3d second_camera_position =
            estimator_state[second_slot].position +
            estimator_state[second_slot].rotation * cameraTranslation[0];
        first_pair_baseline =
            (second_camera_position - first_camera_position).norm();
        const Vector3d first_bearing =
            feature.feature_per_frame[0].point.normalized();
        const Vector3d second_bearing =
            feature.feature_per_frame[1].point.normalized();
        first_pair_parallax_px =
            (first_bearing.head<2>() - second_bearing.head<2>()).norm() *
            kTraceFocalLengthPx;
        const Vector3d first_world_ray =
            (estimator_state[start_slot].rotation * cameraRotation[0] *
             first_bearing)
                .normalized();
        const Vector3d second_world_ray =
            (estimator_state[second_slot].rotation * cameraRotation[0] *
             second_bearing)
                .normalized();
        const double ray_cosine =
            std::max(-1.0, std::min(
                               1.0, first_world_ray.dot(second_world_ray)));
        first_pair_ray_angle_deg =
            std::acos(ray_cosine) * 180.0 / M_PI;

        Eigen::Matrix<double, 3, 4> first_pose;
        const Matrix3d first_camera_rotation =
            estimator_state[start_slot].rotation * cameraRotation[0];
        first_pose.leftCols<3>() = first_camera_rotation.transpose();
        first_pose.rightCols<1>() =
            -first_camera_rotation.transpose() * first_camera_position;
        Eigen::Matrix<double, 3, 4> second_pose;
        const Matrix3d second_camera_rotation =
            estimator_state[second_slot].rotation * cameraRotation[0];
        second_pose.leftCols<3>() = second_camera_rotation.transpose();
        second_pose.rightCols<1>() =
            -second_camera_rotation.transpose() * second_camera_position;
        Eigen::Matrix4d design = Eigen::Matrix4d::Zero();
        const Vector2d first_point =
            feature.feature_per_frame[0].point.head<2>();
        const Vector2d second_point =
            feature.feature_per_frame[1].point.head<2>();
        design.row(0) =
            first_point.x() * first_pose.row(2) - first_pose.row(0);
        design.row(1) =
            first_point.y() * first_pose.row(2) - first_pose.row(1);
        design.row(2) =
            second_point.x() * second_pose.row(2) - second_pose.row(0);
        design.row(3) =
            second_point.y() * second_pose.row(2) - second_pose.row(1);
        const Eigen::JacobiSVD<Eigen::Matrix4d> svd(
            design, Eigen::ComputeFullV);
        const Vector4d homogeneous =
            svd.matrixV().rightCols<1>();
        if (std::abs(homogeneous.w()) > 1e-12) {
          const Vector3d world_point =
              homogeneous.head<3>() / homogeneous.w();
          first_pair_raw_depth =
              (first_pose.leftCols<3>() * world_point +
               first_pose.rightCols<1>())
                  .z();
        }
        const auto singular_values = svd.singularValues();
        if (singular_values[2] > 1e-12) {
          first_pair_singular_ratio =
              singular_values[3] / singular_values[2];
        }
      }
      VINS_INFO << "Feature geometry trace: timestamp=" << timestamp
                << ", feature_id=" << feature.feature_id
                << ", start_slot=" << start_slot
                << ", start_timestamp="
                << (start_slot >= 0 && start_slot <= frameCount
                        ? estimator_state[start_slot].timestamp
                        : std::numeric_limits<double>::quiet_NaN())
                << ", observations=" << observation_count
                << ", depth_before=" << depth_before
                << ", depth_entry=" << feature.estimated_depth
                << ", inverse_depth_entry="
                << (feature.estimated_depth > 0.0
                        ? 1.0 / feature.estimated_depth
                        : std::numeric_limits<double>::quiet_NaN())
                << ", newly_triangulated="
                << (!(depth_before > 0.0) &&
                    feature.estimated_depth > 0.0)
                << ", first_pair_baseline_m=" << first_pair_baseline
                << ", first_pair_parallax_px=" << first_pair_parallax_px
                << ", first_pair_ray_angle_deg="
                << first_pair_ray_angle_deg
                << ", first_pair_raw_depth_m=" << first_pair_raw_depth
                << ", first_pair_singular_ratio="
                << first_pair_singular_ratio;
      for (int observation_index = 0;
           observation_index < observation_count; ++observation_index) {
        const int slot = start_slot + observation_index;
        const auto &observation =
            feature.feature_per_frame[observation_index];
        const Vector3d anchor_bearing =
            feature.feature_per_frame[0].point.normalized();
        const Vector3d bearing = observation.point.normalized();
        VINS_INFO << "Feature observation geometry trace: timestamp="
                  << timestamp
                  << ", feature_id=" << feature.feature_id
                  << ", observation_index=" << observation_index
                  << ", slot=" << slot
                  << ", observation_timestamp="
                  << (slot >= 0 && slot <= frameCount
                          ? estimator_state[slot].timestamp
                          : std::numeric_limits<double>::quiet_NaN())
                  << ", normalized_x=" << observation.point.x()
                  << ", normalized_y=" << observation.point.y()
                  << ", pixel_u=" << observation.uv.x()
                  << ", pixel_v=" << observation.uv.y()
                  << ", flow_x_px_s="
                  << observation.velocity.x() * kTraceFocalLengthPx
                  << ", flow_y_px_s="
                  << observation.velocity.y() * kTraceFocalLengthPx
                  << ", anchor_parallax_px="
                  << (bearing.head<2>() - anchor_bearing.head<2>()).norm() *
                         kTraceFocalLengthPx;
      }
    }
  }

  // Preserve the exact Ceres parameterization at solve entry.  StateData is
  // expressed in the post-solve yaw-normalized frame, so reconstructing these
  // arrays later would not evaluate the marginalization prior at the same
  // linearization coordinates.
  prepareParameters();
  std::memcpy(pose_arrays_before_optimization, poseArray,
              sizeof(pose_arrays_before_optimization));
  std::memcpy(speed_bias_arrays_before_optimization, speedBiasArray,
              sizeof(speed_bias_arrays_before_optimization));
  std::memcpy(extrinsic_arrays_before_optimization, para_Ex_Pose,
              sizeof(extrinsic_arrays_before_optimization));
  time_delay_array_before_optimization[0] = para_Td[0][0];
  inverse_depths_before_optimization.resize(
      featureManager.getFeatureCount());
  for (size_t i = 0; i < inverse_depths_before_optimization.size(); ++i) {
    inverse_depths_before_optimization[i] = para_Feature[i][0];
  }
  // optimization
  TicToc t_solve;
  const StateData state_before_optimization = estimator_state[WINDOW_SIZE];
  if (!optimize()) {
    VINS_ERROR << "Nonlinear optimization failed; resetting estimator.";
    failure_occur = 1;
    resetStateLocked(true);
    initializeCamerasFromOptions();
    return;
  }
  if (precommit_audit_this_solve) {
    std::string audit_error;
    auto production_candidate = captureBackendState(&audit_error);
    if (!production_candidate) {
      throw std::runtime_error(
          "cannot capture production pre-commit candidate: " + audit_error);
    }
    if (!runPreCommitConsistencyAudit(*precommit_audit_solve_entry,
                                      *production_candidate,
                                      &audit_error)) {
      VINS_ERROR << "Pre-commit consistency audit skipped after restoring "
                    "production candidate: "
                 << audit_error;
    }
  }
  if (motionOnsetWarmReinitializationTriggeredThisSolve) {
    const StateData &warm_state = estimator_state[WINDOW_SIZE];
    VINS_WARN
        << "Motion-onset warm reinitialization solve result: timestamp="
        << timestamp
        << ", accel_bias=" << warm_state.accel_bias.transpose()
        << ", accel_bias_norm=" << warm_state.accel_bias.norm()
        << ", velocity=" << warm_state.velocity.transpose()
        << ", position=" << warm_state.position.transpose()
        << ", solver_initial_cost=" << lastOptimizationInitialCost
        << ", solver_final_cost=" << lastOptimizationFinalCost;
  }
  lastBackendHealthMetrics =
      vins::BackendHealthMetricCalculator::compute(
          states_before_optimization, estimator_state, frameCount,
          pre_integrations, inverse_depths_before_optimization,
          lastSolvedFeatureArray, lastSolvedFeatureCount);
  lastBackendHealthAvailable = true;
  lastBackendSolverUsable = true;
  const int active_feature_count = featureManager.getFeatureCount();
  const bool replay_backend_health_trace_enabled =
      replayBackendHealthTraceStartTimestamp >= 0.0 &&
      replayBackendHealthTraceEndTimestamp >=
          replayBackendHealthTraceStartTimestamp &&
      timestamp >= replayBackendHealthTraceStartTimestamp &&
      timestamp <= replayBackendHealthTraceEndTimestamp;
  if (replay_backend_health_trace_enabled) {
    const auto &health = lastBackendHealthMetrics;
    VINS_INFO << "Backend health metric trace: timestamp=" << timestamp
              << ", finite=" << health.finite
              << ", position_innovation_m="
              << health.position_innovation_m
              << ", velocity_innovation_mps="
              << health.velocity_innovation_mps
              << ", window_kinematic_rms_mps="
              << health.window_kinematic_rms_mps
              << ", kinematic_intervals="
              << health.kinematic_intervals
              << ", pose_scale_ratio=" << health.pose_scale_ratio
              << ", velocity_scale_ratio="
              << health.velocity_scale_ratio
              << ", landmark_depth_scale_ratio="
              << health.landmark_depth_scale_ratio
              << ", landmark_depths=" << health.landmark_depths
              << ", maximum_scale_log_change="
              << health.maximum_scale_log_change
              << ", scale_log_spread=" << health.scale_log_spread
              << ", usable_scale_components="
              << health.usable_scale_components
              << ", solver_termination="
              << ceres::TerminationTypeToString(
                     lastOptimizationTermination)
              << ", solver_successful_steps="
              << lastOptimizationSuccessfulSteps
              << ", persistent_innovation_rejects="
              << consecutivePositionInnovationRejects;
  }
  constexpr int kMinimumUpdatesBeforeStationaryDetection = 20;
  constexpr int kStationaryConfirmationUpdates = 3;
  vins::StationaryEvidence stationary_evidence;
  stationary_evidence.visual_tracks = currentFeatureFlowCount;
  stationary_evidence.median_flow_px_s =
      currentFeatureFlowMedianPxS;
  stationary_evidence.angular_rate_rad_s =
      latestImuData.angular_velocity.norm();
  stationary_evidence.acceleration_norm_error_mps2 =
      std::abs(latestImuData.linear_acceleration.norm() -
               gravity.norm());
  stationary_evidence.range_required = options->hasRange();
  stationary_evidence.range_pair_valid =
      rangeMeasurements[WINDOW_SIZE - 1].valid &&
      rangeMeasurements[WINDOW_SIZE].valid;
  if (stationary_evidence.range_pair_valid) {
    stationary_evidence.range_delta_m =
        std::abs(rangeMeasurements[WINDOW_SIZE].range -
                 rangeMeasurements[WINDOW_SIZE - 1].range);
  }
  const vins::StationaryEvidenceGate stationary_gate;
  const bool stationary_evidence_pass =
      stationary_gate.evaluate(stationary_evidence);
  if (stationaryHint.load() && nonLinearUpdateCount % 10 == 0) {
    VINS_INFO << "Mission stationary evidence: pass="
              << stationary_evidence_pass
              << ", median_flow_px_s="
              << stationary_evidence.median_flow_px_s
              << ", flow_tracks="
              << stationary_evidence.visual_tracks
              << ", angular_rate_rad_s="
              << stationary_evidence.angular_rate_rad_s
              << ", acceleration_norm_error_mps2="
              << stationary_evidence.acceleration_norm_error_mps2
              << ", range_pair_valid="
              << stationary_evidence.range_pair_valid
              << ", range_delta_m="
              << stationary_evidence.range_delta_m;
  }
  if (stationaryHint.load() &&
      nonLinearUpdateCount >= kMinimumUpdatesBeforeStationaryDetection &&
      stationary_evidence_pass) {
    ++stationaryVisualUpdateCount;
  } else {
    stationaryVisualUpdateCount = 0;
  }
  bool stationary_update_applied = false;
  if (stationaryVisualUpdateCount >= kStationaryConfirmationUpdates) {
    const double rejected_drift =
        (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
    estimator_state[WINDOW_SIZE].position = last_state.position;
    estimator_state[WINDOW_SIZE].velocity.setZero();
    last_state.velocity.setZero();
    stationary_update_applied = true;
    if (stationaryVisualUpdateCount == kStationaryConfirmationUpdates ||
        stationaryVisualUpdateCount % 20 == 0) {
      VINS_WARN << "Optical-flow stationary update suppressed inertial drift: "
                << "timestamp=" << timestamp
                << ", confirmations=" << stationaryVisualUpdateCount
                << ", median_flow_px_s=" << currentFeatureFlowMedianPxS
                << ", flow_tracks=" << currentFeatureFlowCount
                << ", angular_rate_rad_s="
                << stationary_evidence.angular_rate_rad_s
                << ", acceleration_norm_error_mps2="
                << stationary_evidence.acceleration_norm_error_mps2
                << ", range_delta_m="
                << stationary_evidence.range_delta_m
                << ", features=" << active_feature_count
                << ", rejected_position_step=" << rejected_drift;
    }
  }
  const StateData &state_after_optimization = estimator_state[WINDOW_SIZE];
  const Vector3d raw_optimized_velocity =
      state_after_optimization.velocity;
  double optimization_position_delta =
      (state_after_optimization.position - state_before_optimization.position)
          .norm();
  constexpr double kMaxQuarantineCorrection = 0.15;
  constexpr double kMaxQuarantineOutputStep = 0.15;
  constexpr int kReplayQuarantineObservationUpdates = 3;
  double output_position_step =
      (state_after_optimization.position - last_state.position).norm();
  const double output_dt =
      state_after_optimization.timestamp - last_state.timestamp;
  Vector3d predicted_output_position = last_state.position;
  if (output_dt > 0.0 && output_dt <= 0.5) {
    predicted_output_position += output_dt * last_state.velocity;
  }
  double output_position_innovation =
      (state_after_optimization.position - predicted_output_position).norm();
  constexpr double kMaxPositionInnovation = 0.10;
  if (nonLinearUpdateCount % 10 == 0 || active_feature_count < 100 ||
      optimization_position_delta > 0.2) {
    VINS_INFO << "Nonlinear state trace: update=" << nonLinearUpdateCount
              << ", timestamp=" << timestamp
              << ", features=" << active_feature_count
              << ", last_tracks=" << featureManager.last_track_num
              << ", long_tracks=" << featureManager.long_track_num
              << ", new_features=" << featureManager.new_feature_num
              << ", parallax_px=" << featureManager.last_average_parallax
              << ", marginalization="
              << (isNewMarginalization() ? "second_new" : "old")
              << ", position_delta=" << optimization_position_delta
              << ", position=" << state_after_optimization.position.transpose()
              << ", accel_bias="
              << state_after_optimization.accel_bias.transpose()
              << ", gyro_bias="
              << state_after_optimization.gyro_bias.transpose();
  }
  if (replayJumpFallbackIterations > 0 &&
      output_position_step > kMaxQuarantineOutputStep) {
    const double primary_correction = optimization_position_delta;
    const double primary_output_step = output_position_step;
    const double primary_initial_cost = lastOptimizationInitialCost;
    const double primary_final_cost = lastOptimizationFinalCost;
    const auto primary_termination = lastOptimizationTermination;
    const Vector3d primary_velocity =
        estimator_state[WINDOW_SIZE].velocity;
    const double primary_velocity_delta =
        (primary_velocity - state_before_optimization.velocity).norm();
    restore_replay_solve_snapshot();
    if (!optimize(replayJumpFallbackIterations)) {
      VINS_ERROR << "Replay bounded LM fallback was unusable: timestamp="
                 << timestamp << ", max_iterations="
                 << replayJumpFallbackIterations;
      failure_occur = 1;
      resetStateLocked(true);
      initializeCamerasFromOptions();
      return;
    }
    optimization_position_delta =
        (estimator_state[WINDOW_SIZE].position -
         state_before_optimization.position).norm();
    output_position_step =
        (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
    VINS_WARN << "Replay bounded LM fallback: timestamp=" << timestamp
              << ", max_iterations=" << replayJumpFallbackIterations
              << ", primary_correction=" << primary_correction
              << ", primary_output_step=" << primary_output_step
              << ", primary_velocity=" << primary_velocity.transpose()
              << ", primary_velocity_delta=" << primary_velocity_delta
              << ", primary_cost=" << primary_initial_cost << "->"
              << primary_final_cost << ", primary_termination="
              << ceres::TerminationTypeToString(primary_termination)
              << ", fallback_correction=" << optimization_position_delta
              << ", fallback_output_step=" << output_position_step
              << ", fallback_velocity="
              << estimator_state[WINDOW_SIZE].velocity.transpose()
              << ", fallback_velocity_delta="
              << (estimator_state[WINDOW_SIZE].velocity -
                  state_before_optimization.velocity).norm()
              << ", fallback_cost=" << lastOptimizationInitialCost << "->"
              << lastOptimizationFinalCost << ", fallback_steps="
              << lastOptimizationSuccessfulSteps
              << ", fallback_termination="
              << ceres::TerminationTypeToString(lastOptimizationTermination);
  }
  if (replayJumpPriorReset &&
      output_position_step > kMaxQuarantineOutputStep) {
    const double rejected_correction = optimization_position_delta;
    const double rejected_output_step = output_position_step;
    const double rejected_initial_cost = lastOptimizationInitialCost;
    const double rejected_final_cost = lastOptimizationFinalCost;
    const auto rejected_termination = lastOptimizationTermination;
    restore_replay_solve_snapshot();
    last_marginalization_info.reset();
    last_marginalization_parameter_blocks.clear();
    if (!optimize()) {
      VINS_ERROR << "Replay jump prior reset solve was unusable: timestamp="
                 << timestamp;
      failure_occur = 1;
      resetStateLocked(true);
      initializeCamerasFromOptions();
      return;
    }
    optimization_position_delta =
        (estimator_state[WINDOW_SIZE].position -
         state_before_optimization.position).norm();
    output_position_step =
        (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
    VINS_WARN << "Replay jump prior reset: timestamp=" << timestamp
              << ", rejected_correction=" << rejected_correction
              << ", rejected_output_step=" << rejected_output_step
              << ", rejected_cost=" << rejected_initial_cost << "->"
              << rejected_final_cost << ", rejected_termination="
              << ceres::TerminationTypeToString(rejected_termination)
              << ", retry_correction=" << optimization_position_delta
              << ", retry_output_step=" << output_position_step
              << ", retry_velocity="
              << estimator_state[WINDOW_SIZE].velocity.transpose()
              << ", retry_cost=" << lastOptimizationInitialCost << "->"
              << lastOptimizationFinalCost << ", retry_termination="
              << ceres::TerminationTypeToString(lastOptimizationTermination);
  }
  constexpr double kReplayTimestampMatchToleranceS = 0.01;
  const bool replay_reject_selected_accepted_update =
      !replayRejectAcceptedUpdateConsumed &&
      replayRejectAcceptedUpdateTimestamp >= 0.0 &&
      std::abs(timestamp - replayRejectAcceptedUpdateTimestamp) <=
          kReplayTimestampMatchToleranceS;
  if (replay_reject_selected_accepted_update) {
    replayRejectAcceptedUpdateConsumed = true;
    const double rejected_correction = optimization_position_delta;
    const double rejected_output_step = output_position_step;
    const Vector3d rejected_velocity =
        estimator_state[WINDOW_SIZE].velocity;
    restore_replay_solve_snapshot();
    lastBackendTransactionRejected = true;
    if (isNewMarginalization()) {
      processNewMarginalization();
    } else {
      processOldMarginalization();
    }
    optimization_position_delta =
        (estimator_state[WINDOW_SIZE].position -
         state_before_optimization.position).norm();
    output_position_step =
        (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
    output_position_innovation =
        (estimator_state[WINDOW_SIZE].position -
         predicted_output_position).norm();
    VINS_WARN << "Replay accepted update transaction rejected: timestamp="
              << timestamp
              << ", rejected_correction=" << rejected_correction
              << ", rejected_output_step=" << rejected_output_step
              << ", rejected_velocity=" << rejected_velocity.transpose()
              << ", propagated_output_step=" << output_position_step
              << ", propagated_velocity="
              << estimator_state[WINDOW_SIZE].velocity.transpose();
  }
  TrustedMetricCorrectionEvidence metric_correction;
  if (options->hasGpsVelocity() && metricDisplacementAnchorInitialized &&
      rangeGroundInitialized) {
    const auto &gps = metricVelocityMeasurements[WINDOW_SIZE];
    const auto &range = rangeMeasurements[WINDOW_SIZE];
    const double gps_dt =
        estimator_state[WINDOW_SIZE].timestamp - gps.timestamp;
    metric_correction.gps_fresh =
        gps.valid &&
        gps.integration_segment == metricDisplacementAnchorSegment &&
        std::isfinite(gps_dt) &&
        std::abs(gps_dt) <= options->gps_velocity_max_age;
    metric_correction.range_fresh = range.valid;
    if (metric_correction.gps_fresh) {
      const Eigen::Vector2d gps_position =
          gps.horizontal_position + gps.horizontal_velocity * gps_dt;
      const Eigen::Vector2d gps_target =
          metricDisplacementAnchorPosition + gps_position -
          metricDisplacementAnchorMeasurement;
      metric_correction.gps_position_error_m =
          (estimator_state[WINDOW_SIZE].position.head<2>() - gps_target)
              .norm();
      metric_correction.gps_velocity_error_mps =
          (estimator_state[WINDOW_SIZE].velocity.head<2>() -
           gps.horizontal_velocity)
              .norm();
    }
    if (metric_correction.range_fresh) {
      const Quaterniond q(estimator_state[WINDOW_SIZE].rotation);
      const Vector3d down(0.0, 0.0, -1.0);
      const double vertical_offset =
          (q * options->range_translation).z() +
          range.range * (q * down).z();
      const double range_target_z = rangeGroundZ - vertical_offset;
      metric_correction.range_position_error_m =
          std::abs(estimator_state[WINDOW_SIZE].position.z() -
                   range_target_z);
    }
    metric_correction.output_step_m = output_position_step;
  }
  const bool trusted_metric_correction =
      IsTrustedMetricCorrection(metric_correction);
  TrustedGpsDropoutCorrectionEvidence dropout_correction;
  dropout_correction.anchored_epoch =
      options->hasGpsVelocity() &&
      metricDisplacementAnchorInitialized;
  dropout_correction.gps_position_factor_active =
      metricPositionFactorActiveThisSolve;
  dropout_correction.range_factor_active =
      options->hasRange() && solver_flag == SolverState::NON_LINEAR &&
      nonLinearUpdateCount >= WINDOW_SIZE && rangeGroundInitialized;
  int newest_valid_range_index = -1;
  if (dropout_correction.range_factor_active) {
    dropout_correction.range_factor_active = false;
    for (int i = frameCount; i >= 0; --i) {
      if (rangeMeasurements[i].valid) {
        dropout_correction.range_factor_active = true;
        newest_valid_range_index = i;
        break;
      }
    }
  }
  dropout_correction.horizontal_speed_mps =
      estimator_state[WINDOW_SIZE].velocity.head<2>().norm();
  dropout_correction.maximum_speed_mps =
      options->gps_dropout_speed_max;
  dropout_correction.optimization_position_correction_m =
      optimization_position_delta;
  const Vector3d dropout_output_delta =
      estimator_state[WINDOW_SIZE].position - last_state.position;
  dropout_correction.horizontal_output_step_m =
      dropout_output_delta.head<2>().norm();
  dropout_correction.vertical_output_step_m =
      std::abs(dropout_output_delta.z());
  dropout_correction.vertical_output_innovation_m =
      std::abs(estimator_state[WINDOW_SIZE].position.z() -
               predicted_output_position.z());
  if (newest_valid_range_index >= 0) {
    const Quaterniond q(estimator_state[WINDOW_SIZE].rotation);
    const Vector3d down(0.0, 0.0, -1.0);
    const double vertical_offset =
        (q * options->range_translation).z() +
        rangeMeasurements[newest_valid_range_index].range *
            (q * down).z();
    const double range_target_z = rangeGroundZ - vertical_offset;
    dropout_correction.range_position_error_m =
        std::abs(estimator_state[WINDOW_SIZE].position.z() -
                 range_target_z);
  } else {
    dropout_correction.range_position_error_m =
        std::numeric_limits<double>::infinity();
  }
  const bool trusted_dropout_correction =
      IsTrustedGpsDropoutCorrection(dropout_correction);
  if (trusted_metric_correction &&
      output_position_innovation > kMaxPositionInnovation) {
    VINS_WARN << "Trusted GPS/range correction accepted: timestamp="
              << timestamp
              << ", output_step=" << output_position_step
              << ", innovation=" << output_position_innovation
              << ", gps_position_error="
              << metric_correction.gps_position_error_m
              << ", gps_velocity_error="
              << metric_correction.gps_velocity_error_mps
              << ", range_error="
              << metric_correction.range_position_error_m;
  }
  if (trusted_dropout_correction &&
      output_position_innovation > kMaxPositionInnovation) {
    VINS_WARN << "Trusted GPS-off speed/range correction accepted: timestamp="
              << timestamp
              << ", output_step=" << output_position_step
              << ", innovation=" << output_position_innovation
              << ", horizontal_speed="
              << dropout_correction.horizontal_speed_mps
              << ", optimizer_correction="
              << dropout_correction.optimization_position_correction_m
              << ", horizontal_step="
              << dropout_correction.horizontal_output_step_m
              << ", vertical_step="
              << dropout_correction.vertical_output_step_m
              << ", vertical_innovation="
              << dropout_correction.vertical_output_innovation_m
              << ", window_range_error="
              << dropout_correction.range_position_error_m;
  }
  const bool suspect_position_innovation =
      !stationary_update_applied && nonLinearUpdateCount > 3 &&
      !trusted_metric_correction && !trusted_dropout_correction &&
      output_position_innovation > kMaxPositionInnovation;
  if (suspect_position_innovation) {
    ++consecutivePositionInnovationRejects;
    const double rejected_correction = optimization_position_delta;
    const double rejected_output_step = output_position_step;
    const double rejected_innovation = output_position_innovation;
    const Vector3d rejected_velocity =
        estimator_state[WINDOW_SIZE].velocity;
    const double rejected_initial_cost = lastOptimizationInitialCost;
    const double rejected_final_cost = lastOptimizationFinalCost;
    const auto rejected_termination = lastOptimizationTermination;
    constexpr int kPersistentInnovationConfirmationUpdates = 3;
    constexpr double kMaximumPersistentCorrectionM = 0.20;
    constexpr double kMaximumPersistentVelocityChangeMps = 0.35;
    const double rejected_velocity_change =
        (rejected_velocity - state_before_optimization.velocity).norm();
    const bool gps_dropout_persistent_solve_is_plausible =
        dropout_correction.anchored_epoch &&
        !dropout_correction.gps_position_factor_active &&
        dropout_correction.range_factor_active &&
        std::isfinite(dropout_correction.horizontal_speed_mps) &&
        dropout_correction.horizontal_speed_mps <=
            dropout_correction.maximum_speed_mps + 0.05 &&
        std::isfinite(dropout_correction.range_position_error_m) &&
        dropout_correction.range_position_error_m <= 0.15 &&
        rejected_velocity_change <=
            kMaximumPersistentVelocityChangeMps &&
        std::isfinite(rejected_initial_cost) &&
        std::isfinite(rejected_final_cost) &&
        rejected_final_cost < rejected_initial_cost &&
        rejected_termination != ceres::FAILURE;
    const bool persistent_solve_is_plausible =
        (rejected_correction <= kMaximumPersistentCorrectionM &&
         rejected_velocity_change <=
             kMaximumPersistentVelocityChangeMps) ||
        gps_dropout_persistent_solve_is_plausible;
    const bool replay_reject_selected_persistent_approval =
        !replayRejectPersistentInnovationConsumed &&
        replayRejectPersistentInnovationTimestamp >= 0.0 &&
        std::abs(timestamp - replayRejectPersistentInnovationTimestamp) <=
            kReplayTimestampMatchToleranceS;
    if (consecutivePositionInnovationRejects <= 10) {
      struct ResidualDiagnostics {
        int imu_factor_count = 0;
        double imu_rms = std::numeric_limits<double>::quiet_NaN();
        std::array<double, 5> latest_imu_component_norms{
            {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::quiet_NaN()}};
        int prior_residual_count = 0;
        double prior_norm = std::numeric_limits<double>::quiet_NaN();
        double prior_rms = std::numeric_limits<double>::quiet_NaN();
      };
      const auto evaluate_residuals = [&](bool solve_entry) {
        ResidualDiagnostics diagnostics;
        double imu_squared_norm = 0.0;
        int imu_residual_count = 0;
        for (int j = 1; j <= frameCount; ++j) {
          if (!pre_integrations[j] ||
              (pre_integrations[j]->sum_dt >= 10.0 &&
               !replayIncludeLongImuFactors)) {
            continue;
          }
          const double *pose_i =
              solve_entry ? pose_arrays_before_optimization[j - 1]
                          : poseArray[j - 1];
          const double *speed_bias_i =
              solve_entry ? speed_bias_arrays_before_optimization[j - 1]
                          : speedBiasArray[j - 1];
          const double *pose_j =
              solve_entry ? pose_arrays_before_optimization[j] : poseArray[j];
          const double *speed_bias_j =
              solve_entry ? speed_bias_arrays_before_optimization[j]
                          : speedBiasArray[j];
          const double *parameters[] = {pose_i, speed_bias_i, pose_j,
                                        speed_bias_j};
          std::array<double, 15> residual{};
          IMUFactor factor(pre_integrations[j]);
          if (!factor.Evaluate(parameters, residual.data(), nullptr)) continue;
          const Eigen::Map<const Eigen::Matrix<double, 15, 1>> vector(
              residual.data());
          imu_squared_norm += vector.squaredNorm();
          imu_residual_count += vector.size();
          ++diagnostics.imu_factor_count;
          if (j == frameCount) {
            diagnostics.latest_imu_component_norms = {
                vector.segment<3>(O_P).norm(), vector.segment<3>(O_R).norm(),
                vector.segment<3>(O_V).norm(), vector.segment<3>(O_BA).norm(),
                vector.segment<3>(O_BG).norm()};
          }
        }
        if (imu_residual_count > 0) {
          diagnostics.imu_rms =
              std::sqrt(imu_squared_norm / imu_residual_count);
        }

        const auto &prior = marginalization_before_optimization;
        if (!prior || !prior->valid || prior->n <= 0) return diagnostics;
        std::vector<double *> prior_parameters;
        prior_parameters.reserve(
            marginalization_blocks_before_optimization.size());
        for (double *block : marginalization_blocks_before_optimization) {
          double *mapped = nullptr;
          for (int i = 0; i <= WINDOW_SIZE && mapped == nullptr; ++i) {
            if (block == poseArray[i]) {
              mapped = solve_entry ? pose_arrays_before_optimization[i]
                                   : poseArray[i];
            } else if (block == speedBiasArray[i]) {
              mapped = solve_entry
                           ? speed_bias_arrays_before_optimization[i]
                           : speedBiasArray[i];
            }
          }
          for (int i = 0;
               i < options->getNumCameras() && mapped == nullptr; ++i) {
            if (block == para_Ex_Pose[i]) {
              mapped = solve_entry ? extrinsic_arrays_before_optimization[i]
                                   : para_Ex_Pose[i];
            }
          }
          if (block == para_Td[0]) {
            mapped = solve_entry ? time_delay_array_before_optimization
                                 : para_Td[0];
          }
          // A valid prior contains only pose, speed/bias, extrinsic and time
          // delay blocks.  Fail closed in diagnostics rather than evaluating
          // a mismatched block list.
          if (mapped == nullptr) return diagnostics;
          prior_parameters.push_back(mapped);
        }
        MarginalizationFactor prior_factor(prior);
        Eigen::VectorXd residual(prior->n);
        if (prior_factor.Evaluate(prior_parameters.data(), residual.data(),
                                  nullptr)) {
          diagnostics.prior_residual_count = residual.size();
          diagnostics.prior_norm = residual.norm();
          diagnostics.prior_rms =
              residual.norm() / std::sqrt(static_cast<double>(residual.size()));
        }
        return diagnostics;
      };

      const ResidualDiagnostics entry = evaluate_residuals(true);
      const ResidualDiagnostics optimized = evaluate_residuals(false);
      double prior_jacobian_condition =
          std::numeric_limits<double>::quiet_NaN();
      double prior_jacobian_sigma_min =
          std::numeric_limits<double>::quiet_NaN();
      double prior_jacobian_sigma_max =
          std::numeric_limits<double>::quiet_NaN();
      int prior_jacobian_rank = 0;
      if (marginalization_before_optimization &&
          marginalization_before_optimization->valid &&
          marginalization_before_optimization->linearized_jacobians.size() >
              0) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            marginalization_before_optimization->linearized_jacobians,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::VectorXd singular_values = svd.singularValues();
        if (singular_values.size() > 0) {
          prior_jacobian_sigma_max = singular_values.maxCoeff();
          const double rank_threshold =
              std::max(1e-12, prior_jacobian_sigma_max * 1e-10);
          prior_jacobian_sigma_min =
              std::numeric_limits<double>::infinity();
          for (int i = 0; i < singular_values.size(); ++i) {
            if (singular_values(i) > rank_threshold) {
              ++prior_jacobian_rank;
              prior_jacobian_sigma_min =
                  std::min(prior_jacobian_sigma_min, singular_values(i));
            }
          }
          if (prior_jacobian_rank > 0) {
            prior_jacobian_condition =
                prior_jacobian_sigma_max / prior_jacobian_sigma_min;
          } else {
            prior_jacobian_sigma_min =
                std::numeric_limits<double>::quiet_NaN();
          }
        }
      }
      double window_accel_bias_delta_max = 0.0;
      double window_gyro_bias_delta_max = 0.0;
      for (int i = 0; i <= WINDOW_SIZE; ++i) {
        window_accel_bias_delta_max =
            std::max(window_accel_bias_delta_max,
                     (estimator_state[i].accel_bias -
                      states_before_optimization[i].accel_bias)
                         .norm());
        window_gyro_bias_delta_max =
            std::max(window_gyro_bias_delta_max,
                     (estimator_state[i].gyro_bias -
                      states_before_optimization[i].gyro_bias)
                         .norm());
      }
      const auto &entry_components = entry.latest_imu_component_norms;
      const auto &optimized_components =
          optimized.latest_imu_component_norms;
      VINS_WARN
          << "Reject chain residual diagnostic: timestamp=" << timestamp
          << ", consecutive=" << consecutivePositionInnovationRejects
          << ", velocity_entry="
          << state_before_optimization.velocity.transpose()
          << ", velocity_optimized=" << rejected_velocity.transpose()
          << ", velocity_delta=" << rejected_velocity_change
          << ", accel_bias_entry="
          << state_before_optimization.accel_bias.transpose()
          << ", accel_bias_optimized="
          << estimator_state[WINDOW_SIZE].accel_bias.transpose()
          << ", accel_bias_delta="
          << (estimator_state[WINDOW_SIZE].accel_bias -
              state_before_optimization.accel_bias)
                 .norm()
          << ", accel_bias_window_delta_max="
          << window_accel_bias_delta_max << ", gyro_bias_entry="
          << state_before_optimization.gyro_bias.transpose()
          << ", gyro_bias_optimized="
          << estimator_state[WINDOW_SIZE].gyro_bias.transpose()
          << ", gyro_bias_delta="
          << (estimator_state[WINDOW_SIZE].gyro_bias -
              state_before_optimization.gyro_bias)
                 .norm()
          << ", gyro_bias_window_delta_max=" << window_gyro_bias_delta_max
          << ", imu_factors=" << entry.imu_factor_count
          << ", imu_rms_entry=" << entry.imu_rms
          << ", imu_rms_optimized=" << optimized.imu_rms
          << ", imu_latest_prvbag_entry=" << entry_components[0] << "/"
          << entry_components[1] << "/" << entry_components[2] << "/"
          << entry_components[3] << "/" << entry_components[4]
          << ", imu_latest_prvbag_optimized=" << optimized_components[0]
          << "/" << optimized_components[1] << "/"
          << optimized_components[2] << "/" << optimized_components[3]
          << "/" << optimized_components[4]
          << ", prior_residuals=" << entry.prior_residual_count
          << ", prior_norm_entry=" << entry.prior_norm
          << ", prior_norm_optimized=" << optimized.prior_norm
          << ", prior_rms_entry=" << entry.prior_rms
          << ", prior_rms_optimized=" << optimized.prior_rms
          << ", prior_jacobian_rank=" << prior_jacobian_rank
          << ", prior_jacobian_sigma_min=" << prior_jacobian_sigma_min
          << ", prior_jacobian_sigma_max=" << prior_jacobian_sigma_max
          << ", prior_jacobian_condition=" << prior_jacobian_condition;
    }
    if (consecutivePositionInnovationRejects <
            kPersistentInnovationConfirmationUpdates ||
        !persistent_solve_is_plausible ||
        replay_reject_selected_persistent_approval) {
      if (replay_reject_selected_persistent_approval) {
        replayRejectPersistentInnovationConsumed = true;
        VINS_WARN << "Replay persistent innovation approval rejected: "
                  << "timestamp=" << timestamp
                  << ", consecutive=" << consecutivePositionInnovationRejects
                  << ", velocity_entry="
                  << state_before_optimization.velocity.transpose()
                  << ", velocity_optimized="
                  << rejected_velocity.transpose();
      }
      restore_replay_solve_snapshot();
      lastBackendTransactionRejected = true;

      // During a GPS outage, the same physical speed ceiling used by Ceres
      // must also bound the IMU-propagated solve-entry state. Otherwise a
      // rejected position correction leaves an ever-faster propagation
      // behind, forcing every subsequent solve to request a larger position
      // correction. Project only velocity (never position) and let the next
      // visual/IMU solve refine it.
      if (options->hasGpsVelocity() &&
          metricDisplacementAnchorInitialized &&
          !metricPositionFactorActiveThisSolve &&
          std::isfinite(options->gps_dropout_speed_max) &&
          options->gps_dropout_speed_max > 0.0) {
        auto horizontal_velocity =
            estimator_state[WINDOW_SIZE].velocity.head<2>();
        const double propagated_horizontal_speed =
            horizontal_velocity.norm();
        if (std::isfinite(propagated_horizontal_speed) &&
            propagated_horizontal_speed >
                options->gps_dropout_speed_max) {
          horizontal_velocity *=
              options->gps_dropout_speed_max /
              propagated_horizontal_speed;
          VINS_WARN << "GPS-off rejected transaction projected propagated "
                       "horizontal speed: timestamp="
                    << timestamp << ", before="
                    << propagated_horizontal_speed << ", after="
                    << horizontal_velocity.norm();
        }

        // The downward range is the independent vertical metric reference.
        // Keep a rejected IMU propagation inside both its physical vertical
        // speed envelope and the public 0.15 m output-step contract. Without
        // this projection, rollback can preserve a bad vertical velocity and
        // make every later range-corrected solve look like a larger jump.
        constexpr double kGpsDropoutVerticalSpeedLimitMps = 0.50;
        double &vertical_velocity =
            estimator_state[WINDOW_SIZE].velocity.z();
        const double propagated_vertical_velocity = vertical_velocity;
        vertical_velocity =
            std::max(-kGpsDropoutVerticalSpeedLimitMps,
                     std::min(kGpsDropoutVerticalSpeedLimitMps,
                              vertical_velocity));
        if (newest_valid_range_index >= 0) {
          // This endurance profile flies over a validated fixed ground
          // plane. Once range directly fixes the current z state, retaining
          // a nonzero propagated vertical velocity would publish mutually
          // inconsistent position/velocity and make PX4 fight a fictitious
          // climb or descent.
          vertical_velocity = 0.0;
          const Quaterniond q(
              estimator_state[WINDOW_SIZE].rotation);
          const Vector3d down(0.0, 0.0, -1.0);
          const double vertical_offset =
              (q * options->range_translation).z() +
              rangeMeasurements[newest_valid_range_index].range *
                  (q * down).z();
          const double range_target_z =
              rangeGroundZ - vertical_offset;
          const double horizontal_output_step =
              (estimator_state[WINDOW_SIZE].position.head<2>() -
               last_state.position.head<2>())
                  .norm();
          // Keep 1 cm numerical and downstream-filter margin below the
          // public/failure 0.15 m step limit.
          constexpr double kMaximumOutputStepM = 0.14;
          const double maximum_vertical_step =
              horizontal_output_step < kMaximumOutputStepM
                  ? std::sqrt(kMaximumOutputStepM *
                                  kMaximumOutputStepM -
                              horizontal_output_step *
                                  horizontal_output_step)
                  : 0.0;
          if (std::isfinite(range_target_z)) {
            estimator_state[WINDOW_SIZE].position.z() =
                std::max(last_state.position.z() -
                             maximum_vertical_step,
                         std::min(last_state.position.z() +
                                      maximum_vertical_step,
                                  range_target_z));
          }
        }
        if (vertical_velocity != propagated_vertical_velocity ||
            newest_valid_range_index >= 0) {
          VINS_WARN << "GPS-off rejected transaction projected range axis: "
                    << "timestamp=" << timestamp
                    << ", vertical_velocity_before="
                    << propagated_vertical_velocity
                    << ", vertical_velocity_after="
                    << vertical_velocity << ", position_z="
                    << estimator_state[WINDOW_SIZE].position.z();
        }
      }

      // optimize() already built a marginalization prior from the rejected
      // state. Rebuild it from the solve-entry IMU-propagated state so no
      // rejected pose, velocity, bias, depth, or prior mutation survives.
      if (solver_flag == SolverState::NON_LINEAR) {
        ++nonLinearUpdateCount;
      }
      if (isNewMarginalization()) {
        processNewMarginalization();
      } else {
        processOldMarginalization();
      }
      optimization_position_delta = 0.0;
      output_position_step =
          (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
      output_position_innovation =
          (estimator_state[WINDOW_SIZE].position -
           predicted_output_position).norm();
      VINS_WARN << "Position innovation transaction rejected: timestamp="
                << timestamp
                << ", consecutive=" << consecutivePositionInnovationRejects
                << ", rejected_correction=" << rejected_correction
                << ", rejected_output_step=" << rejected_output_step
                << ", rejected_innovation=" << rejected_innovation
                << ", rejected_velocity=" << rejected_velocity.transpose()
                << ", rejected_velocity_change="
                << rejected_velocity_change
                << ", persistent_plausible="
                << persistent_solve_is_plausible
                << ", gps_dropout_persistent_plausible="
                << gps_dropout_persistent_solve_is_plausible
                << ", rejected_cost=" << rejected_initial_cost << "->"
                << rejected_final_cost << ", rejected_termination="
                << ceres::TerminationTypeToString(rejected_termination)
                << ", propagated_output_step=" << output_position_step
                << ", propagated_innovation=" << output_position_innovation
                << ", propagated_velocity="
                << estimator_state[WINDOW_SIZE].velocity.transpose();
    } else {
      // A one-off correction is a corrupt solve and is rolled back above.
      // A correction that recurs for three consecutive camera updates is
      // instead evidence that visual tracking is consistently pulling the
      // IMU prediction back.  Accept the optimized state, but saturate the
      // externally visible position innovation and rebuild the prior from
      // that bounded state.  This prevents indefinite IMU-only drift without
      // relaxing the existing output/failure guards.
      const auto window_kinematic_rms =
          [&](const StateData *states) {
            double squared_norm_sum = 0.0;
            int interval_count = 0;
            for (int j = 1; j <= frameCount; ++j) {
              if (!pre_integrations[j]) continue;
              const double dt = pre_integrations[j]->sum_dt;
              if (!std::isfinite(dt) || dt <= 1e-6 || dt >= 10.0) continue;
              const Vector3d finite_difference_velocity =
                  (states[j].position - states[j - 1].position) / dt;
              const Vector3d mean_state_velocity =
                  0.5 * (states[j].velocity + states[j - 1].velocity);
              squared_norm_sum +=
                  (finite_difference_velocity - mean_state_velocity)
                      .squaredNorm();
              ++interval_count;
            }
            return interval_count > 0
                       ? std::sqrt(squared_norm_sum / interval_count)
                       : std::numeric_limits<double>::quiet_NaN();
          };
      const double entry_window_kinematic_rms =
          window_kinematic_rms(states_before_optimization);
      const double optimized_window_kinematic_rms =
          window_kinematic_rms(estimator_state);
      constexpr double kPersistentInnovationLimitM = 0.08;
      const Vector3d innovation =
          estimator_state[WINDOW_SIZE].position - predicted_output_position;
      Vector3d bounded_newest_position =
          predicted_output_position +
          innovation.normalized() * kPersistentInnovationLimitM;
      constexpr double kPersistentOutputStepLimitM = 0.12;
      const Vector3d bounded_output_delta =
          bounded_newest_position - last_state.position;
      if (bounded_output_delta.norm() > kPersistentOutputStepLimitM) {
        bounded_newest_position =
            last_state.position +
            bounded_output_delta.normalized() *
                kPersistentOutputStepLimitM;
      }
      estimator_state[WINDOW_SIZE].position = bounded_newest_position;
      constexpr double kPersistentVelocityChangeLimitMps = 0.20;
      const Vector3d optimized_velocity_delta =
          estimator_state[WINDOW_SIZE].velocity -
          state_before_optimization.velocity;
      if (optimized_velocity_delta.norm() >
          kPersistentVelocityChangeLimitMps) {
        estimator_state[WINDOW_SIZE].velocity =
            state_before_optimization.velocity +
                optimized_velocity_delta.normalized() *
                kPersistentVelocityChangeLimitMps;
      }
      const double bounded_window_kinematic_rms =
          window_kinematic_rms(estimator_state);
      last_marginalization_info = marginalization_before_optimization;
      last_marginalization_parameter_blocks =
          marginalization_blocks_before_optimization;
      if (isNewMarginalization()) {
        processNewMarginalization();
      } else {
        processOldMarginalization();
      }
      optimization_position_delta =
          (estimator_state[WINDOW_SIZE].position -
           state_before_optimization.position).norm();
      output_position_step =
          (estimator_state[WINDOW_SIZE].position - last_state.position).norm();
      output_position_innovation =
          (estimator_state[WINDOW_SIZE].position -
           predicted_output_position).norm();
      VINS_WARN << "Persistent position innovation saturated: timestamp="
                << timestamp
                << ", consecutive=" << consecutivePositionInnovationRejects
                << ", optimized_correction=" << rejected_correction
                << ", optimized_output_step=" << rejected_output_step
                << ", optimized_innovation=" << rejected_innovation
                << ", bounded_output_step=" << output_position_step
                << ", bounded_innovation=" << output_position_innovation
                << ", optimized_velocity=" << rejected_velocity.transpose()
                << ", window_kinematic_rms_entry="
                << entry_window_kinematic_rms
                << ", window_kinematic_rms_optimized="
                << optimized_window_kinematic_rms
                << ", window_kinematic_rms_bounded="
                << bounded_window_kinematic_rms;
    }
  } else {
    consecutivePositionInnovationRejects = 0;
  }
  if (replay_velocity_trace_enabled) {
    std::ostringstream window_timestamps;
    std::ostringstream preintegration_lengths;
    int connected_imu_factor_count = 0;
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      if (i > 0) window_timestamps << "/";
      window_timestamps << std::fixed << std::setprecision(3)
                        << states_before_optimization[i].timestamp;
      if (i == 0) continue;
      if (i > 1) preintegration_lengths << "/";
      const double length = pre_integrations[i]
                                ? pre_integrations[i]->sum_dt
                                : std::numeric_limits<double>::quiet_NaN();
      preintegration_lengths << std::fixed << std::setprecision(3) << length;
      if (pre_integrations[i] &&
          (length < 10.0 || replayIncludeLongImuFactors)) {
        ++connected_imu_factor_count;
      }
    }
    const Vector3d entry_rpy = Utility::R2ypr(
        states_before_optimization[WINDOW_SIZE].rotation);
    const Vector3d optimized_rpy = Utility::R2ypr(
        estimator_state[WINDOW_SIZE].rotation);
    const Vector3d velocity_step =
        estimator_state[WINDOW_SIZE].velocity -
        states_before_optimization[WINDOW_SIZE].velocity;
    VINS_INFO << "Exact window direction trace: timestamp=" << timestamp
              << ", state_timestamps_s=" << window_timestamps.str()
              << ", preintegration_lengths_s="
              << preintegration_lengths.str()
              << ", imu_factors=" << connected_imu_factor_count
              << ", marginalization_source_timestamp="
              << marginalization_source_timestamp_before_optimization
              << ", marginalization_source_type="
              << marginalization_source_type_before_optimization
              << ", roll_step_deg=" << optimized_rpy.z() - entry_rpy.z()
              << ", pitch_step_deg=" << optimized_rpy.y() - entry_rpy.y()
              << ", horizontal_velocity_step="
              << velocity_step.head<2>().norm()
              << ", vertical_velocity_step=" << velocity_step.z()
              << ", accel_bias_step="
              << (estimator_state[WINDOW_SIZE].accel_bias -
                  states_before_optimization[WINDOW_SIZE].accel_bias)
                     .transpose()
              << ", gyro_bias_step="
              << (estimator_state[WINDOW_SIZE].gyro_bias -
                  states_before_optimization[WINDOW_SIZE].gyro_bias)
                     .transpose();
    VINS_INFO << "Velocity prior trace: timestamp=" << timestamp
              << ", update=" << nonLinearUpdateCount
              << ", accepted_non_suspect=" << !suspect_position_innovation
              << ", suspect_innovation=" << suspect_position_innovation
              << ", entry_velocity="
              << state_before_optimization.velocity.transpose()
              << ", optimized_velocity="
              << raw_optimized_velocity.transpose()
              << ", committed_velocity="
              << estimator_state[WINDOW_SIZE].velocity.transpose()
              << ", optimized_velocity_delta="
              << (raw_optimized_velocity -
                  state_before_optimization.velocity).norm()
              << ", features=" << active_feature_count
              << ", last_tracks=" << featureManager.last_track_num
              << ", long_tracks=" << featureManager.long_track_num
              << ", parallax_px=" << featureManager.last_average_parallax
              << ", flow_tracks=" << currentFeatureFlowCount
              << ", median_flow_px_s=" << currentFeatureFlowMedianPxS
              << ", marginalization="
              << (isNewMarginalization() ? "second_new" : "old")
              << ", prior_source_timestamp="
              << marginalization_source_timestamp_before_optimization
              << ", prior_source_type="
              << marginalization_source_type_before_optimization
              << ", correction=" << optimization_position_delta
              << ", output_innovation=" << output_position_innovation
              << ", solver_cost=" << lastOptimizationInitialCost << "->"
              << lastOptimizationFinalCost;
    // Decompose the exact solve step in the speed/bias coordinates.  The
    // directional derivative g^T dx is evaluated at solve entry: a negative
    // value means that factor family directly rewards the Ceres update.  Cost
    // deltas provide a nonlinear cross-check over the complete solve step.
    struct SpeedBiasDirectionTrace {
      Eigen::Matrix<double, WINDOW_SIZE + 1, SIZE_SPEEDBIAS> imu_gradient =
          Eigen::Matrix<double, WINDOW_SIZE + 1,
                        SIZE_SPEEDBIAS>::Zero();
      Eigen::Matrix<double, WINDOW_SIZE + 1, SIZE_SPEEDBIAS> prior_gradient =
          Eigen::Matrix<double, WINDOW_SIZE + 1,
                        SIZE_SPEEDBIAS>::Zero();
      double imu_entry_cost = 0.0;
      double imu_optimized_cost = 0.0;
      double prior_entry_cost = 0.0;
      double prior_optimized_cost = 0.0;
      bool prior_mapping_complete = false;
    } speed_bias_trace;
    if (options->hasImu()) {
      for (int j = 1; j <= frameCount; ++j) {
        if (!pre_integrations[j] ||
            (pre_integrations[j]->sum_dt >= 10.0 &&
             !replayIncludeLongImuFactors)) {
          continue;
        }
        const double *entry_parameters[] = {
            pose_arrays_before_optimization[j - 1],
            speed_bias_arrays_before_optimization[j - 1],
            pose_arrays_before_optimization[j],
            speed_bias_arrays_before_optimization[j]};
        const double *optimized_parameters[] = {
            lastSolvedPoseArray[j - 1], lastSolvedSpeedBiasArray[j - 1],
            lastSolvedPoseArray[j], lastSolvedSpeedBiasArray[j]};
        std::array<double, 15> entry_residual{};
        std::array<double, 15> optimized_residual{};
        std::array<double, 15 * SIZE_SPEEDBIAS> jacobian_i{};
        std::array<double, 15 * SIZE_SPEEDBIAS> jacobian_j{};
        double *entry_jacobians[] = {nullptr, jacobian_i.data(), nullptr,
                                     jacobian_j.data()};
        IMUFactor factor(pre_integrations[j]);
        if (!factor.Evaluate(entry_parameters, entry_residual.data(),
                             entry_jacobians) ||
            !factor.Evaluate(optimized_parameters, optimized_residual.data(),
                             nullptr)) {
          continue;
        }
        const Eigen::Map<const Eigen::Matrix<double, 15, 1>> entry_vector(
            entry_residual.data());
        const Eigen::Map<const Eigen::Matrix<double, 15, 1>>
            optimized_vector(optimized_residual.data());
        const Eigen::Map<
            const Eigen::Matrix<double, 15, SIZE_SPEEDBIAS, Eigen::RowMajor>>
            jacobian_i_matrix(jacobian_i.data());
        const Eigen::Map<
            const Eigen::Matrix<double, 15, SIZE_SPEEDBIAS, Eigen::RowMajor>>
            jacobian_j_matrix(jacobian_j.data());
        speed_bias_trace.imu_gradient.row(j - 1) +=
            (jacobian_i_matrix.transpose() * entry_vector).transpose();
        speed_bias_trace.imu_gradient.row(j) +=
            (jacobian_j_matrix.transpose() * entry_vector).transpose();
        speed_bias_trace.imu_entry_cost +=
            0.5 * entry_vector.squaredNorm();
        speed_bias_trace.imu_optimized_cost +=
            0.5 * optimized_vector.squaredNorm();
      }
    }
    const auto &speed_bias_prior = marginalization_before_optimization;
    if (speed_bias_prior && speed_bias_prior->valid &&
        speed_bias_prior->n > 0 &&
        speed_bias_prior->keep_block_size.size() ==
            marginalization_blocks_before_optimization.size()) {
      std::vector<double *> entry_parameters;
      std::vector<double *> optimized_parameters;
      entry_parameters.reserve(
          marginalization_blocks_before_optimization.size());
      optimized_parameters.reserve(
          marginalization_blocks_before_optimization.size());
      bool complete_mapping = true;
      for (double *block : marginalization_blocks_before_optimization) {
        double *entry = nullptr;
        double *optimized = nullptr;
        for (int i = 0; i <= WINDOW_SIZE && entry == nullptr; ++i) {
          if (block == poseArray[i]) {
            entry = pose_arrays_before_optimization[i];
            optimized = lastSolvedPoseArray[i];
          } else if (block == speedBiasArray[i]) {
            entry = speed_bias_arrays_before_optimization[i];
            optimized = lastSolvedSpeedBiasArray[i];
          }
        }
        for (int i = 0;
             i < options->getNumCameras() && entry == nullptr; ++i) {
          if (block == para_Ex_Pose[i]) {
            entry = extrinsic_arrays_before_optimization[i];
            optimized = lastSolvedExtrinsicArray[i];
          }
        }
        if (block == para_Td[0]) {
          entry = time_delay_array_before_optimization;
          optimized = lastSolvedTimeDelayArray;
        }
        if (entry == nullptr || optimized == nullptr) {
          complete_mapping = false;
          break;
        }
        entry_parameters.push_back(entry);
        optimized_parameters.push_back(optimized);
      }
      if (complete_mapping) {
        MarginalizationFactor prior_factor(speed_bias_prior);
        Eigen::VectorXd entry_residual(speed_bias_prior->n);
        Eigen::VectorXd optimized_residual(speed_bias_prior->n);
        std::vector<std::vector<double>> jacobian_storage;
        std::vector<double *> jacobian_pointers;
        jacobian_storage.reserve(speed_bias_prior->keep_block_size.size());
        jacobian_pointers.reserve(speed_bias_prior->keep_block_size.size());
        for (const int block_size : speed_bias_prior->keep_block_size) {
          jacobian_storage.emplace_back(
              static_cast<size_t>(speed_bias_prior->n * block_size), 0.0);
          jacobian_pointers.push_back(jacobian_storage.back().data());
        }
        if (prior_factor.Evaluate(entry_parameters.data(),
                                  entry_residual.data(),
                                  jacobian_pointers.data()) &&
            prior_factor.Evaluate(optimized_parameters.data(),
                                  optimized_residual.data(), nullptr)) {
          speed_bias_trace.prior_mapping_complete = true;
          speed_bias_trace.prior_entry_cost =
              0.5 * entry_residual.squaredNorm();
          speed_bias_trace.prior_optimized_cost =
              0.5 * optimized_residual.squaredNorm();
          for (size_t block_index = 0;
               block_index <
               marginalization_blocks_before_optimization.size();
               ++block_index) {
            double *block =
                marginalization_blocks_before_optimization[block_index];
            for (int state_index = 0; state_index <= WINDOW_SIZE;
                 ++state_index) {
              if (block != speedBiasArray[state_index]) continue;
              const int block_size =
                  speed_bias_prior->keep_block_size[block_index];
              if (block_size != SIZE_SPEEDBIAS) continue;
              const Eigen::Map<
                  const Eigen::Matrix<double, Eigen::Dynamic,
                                      SIZE_SPEEDBIAS, Eigen::RowMajor>>
                  jacobian(jacobian_storage[block_index].data(),
                           speed_bias_prior->n, SIZE_SPEEDBIAS);
              speed_bias_trace.prior_gradient.row(state_index) +=
                  (jacobian.transpose() * entry_residual).transpose();
            }
          }
        }
      }
    }
    Eigen::Matrix<double, WINDOW_SIZE + 1, SIZE_SPEEDBIAS> solve_step;
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      for (int component = 0; component < SIZE_SPEEDBIAS; ++component) {
        solve_step(i, component) =
            lastSolvedSpeedBiasArray[i][component] -
            speed_bias_arrays_before_optimization[i][component];
      }
    }
    const auto component_dot =
        [&](const Eigen::Matrix<double, WINDOW_SIZE + 1,
                                SIZE_SPEEDBIAS> &gradient,
            int offset) {
          return (gradient.template block<WINDOW_SIZE + 1, 3>(0, offset)
                      .array() *
                  solve_step.template block<WINDOW_SIZE + 1, 3>(0, offset)
                      .array())
              .sum();
        };
    const auto component_norm =
        [&](const Eigen::Matrix<double, WINDOW_SIZE + 1,
                                SIZE_SPEEDBIAS> &values,
            int offset) {
          return values.template block<WINDOW_SIZE + 1, 3>(0, offset).norm();
        };
    const double imu_v_dot =
        component_dot(speed_bias_trace.imu_gradient, 0);
    const double imu_ba_dot =
        component_dot(speed_bias_trace.imu_gradient, 3);
    const double imu_bg_dot =
        component_dot(speed_bias_trace.imu_gradient, 6);
    const double prior_v_dot =
        component_dot(speed_bias_trace.prior_gradient, 0);
    const double prior_ba_dot =
        component_dot(speed_bias_trace.prior_gradient, 3);
    const double prior_bg_dot =
        component_dot(speed_bias_trace.prior_gradient, 6);
    VINS_INFO << "Speed-bias direction factor trace: timestamp=" << timestamp
              << ", update=" << nonLinearUpdateCount
              << ", prior_mapping_complete="
              << speed_bias_trace.prior_mapping_complete
              << ", v_step_norm=" << component_norm(solve_step, 0)
              << ", ba_step_norm=" << component_norm(solve_step, 3)
              << ", bg_step_norm=" << component_norm(solve_step, 6)
              << ", imu_entry_cost=" << speed_bias_trace.imu_entry_cost
              << ", imu_optimized_cost="
              << speed_bias_trace.imu_optimized_cost
              << ", imu_cost_delta="
              << speed_bias_trace.imu_optimized_cost -
                     speed_bias_trace.imu_entry_cost
              << ", prior_entry_cost="
              << speed_bias_trace.prior_entry_cost
              << ", prior_optimized_cost="
              << speed_bias_trace.prior_optimized_cost
              << ", prior_cost_delta="
              << speed_bias_trace.prior_optimized_cost -
                     speed_bias_trace.prior_entry_cost
              << ", visual_other_entry_cost="
              << lastOptimizationInitialCost -
                     speed_bias_trace.imu_entry_cost -
                     speed_bias_trace.prior_entry_cost
              << ", visual_other_optimized_cost="
              << lastOptimizationFinalCost -
                     speed_bias_trace.imu_optimized_cost -
                     speed_bias_trace.prior_optimized_cost
              << ", imu_v_gradient_dot_step=" << imu_v_dot
              << ", imu_ba_gradient_dot_step=" << imu_ba_dot
              << ", imu_bg_gradient_dot_step=" << imu_bg_dot
              << ", prior_v_gradient_dot_step=" << prior_v_dot
              << ", prior_ba_gradient_dot_step=" << prior_ba_dot
              << ", prior_bg_gradient_dot_step=" << prior_bg_dot
              << ", total_v_gradient_dot_step=" << imu_v_dot + prior_v_dot
              << ", total_ba_gradient_dot_step=" << imu_ba_dot + prior_ba_dot
              << ", total_bg_gradient_dot_step=" << imu_bg_dot + prior_bg_dot
              << ", newest_entry_velocity="
              << Eigen::Map<const Vector3d>(
                     speed_bias_arrays_before_optimization[WINDOW_SIZE])
                     .transpose()
              << ", newest_optimized_velocity="
              << Eigen::Map<const Vector3d>(
                     lastSolvedSpeedBiasArray[WINDOW_SIZE])
                     .transpose()
              << ", newest_entry_ba="
              << Eigen::Map<const Vector3d>(
                     speed_bias_arrays_before_optimization[WINDOW_SIZE] + 3)
                     .transpose()
              << ", newest_optimized_ba="
              << Eigen::Map<const Vector3d>(
                     lastSolvedSpeedBiasArray[WINDOW_SIZE] + 3)
                     .transpose();
    // Probe the actual metric-scale direction without changing the solve.
    // Positions are scaled about the fixed oldest pose, velocities and
    // landmark depths use the same scale, while rotation, gravity, biases,
    // camera extrinsics, and time delay remain metric constants.  Central
    // finite differences then expose which factor family rewards contraction
    // (positive gradient means lowering scale lowers that factor's cost).
    struct ScaleDirectionCost {
      double visual = 0.0;
      double imu = 0.0;
      double prior = 0.0;
      double range = 0.0;

      double total() const { return visual + imu + prior + range; }
    };
    const auto evaluate_scale_direction_cost =
        [&](bool solve_entry, double scale) {
          ScaleDirectionCost cost;
          double scaled_pose[WINDOW_SIZE + 1][SIZE_POSE];
          double scaled_speed_bias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
          double scaled_feature[NUM_OF_F][SIZE_FEATURE];
          const double(*base_pose)[SIZE_POSE] =
              solve_entry ? pose_arrays_before_optimization
                          : lastSolvedPoseArray;
          const double(*base_speed_bias)[SIZE_SPEEDBIAS] =
              solve_entry ? speed_bias_arrays_before_optimization
                          : lastSolvedSpeedBiasArray;
          std::memcpy(scaled_pose, base_pose, sizeof(scaled_pose));
          std::memcpy(scaled_speed_bias, base_speed_bias,
                      sizeof(scaled_speed_bias));
          const Vector3d anchor(base_pose[0][0], base_pose[0][1],
                                base_pose[0][2]);
          for (int i = 0; i <= frameCount; ++i) {
            const Vector3d position(base_pose[i][0], base_pose[i][1],
                                    base_pose[i][2]);
            const Vector3d scaled_position =
                anchor + scale * (position - anchor);
            for (int axis = 0; axis < 3; ++axis) {
              scaled_pose[i][axis] = scaled_position(axis);
              scaled_speed_bias[i][axis] =
                  scale * base_speed_bias[i][axis];
            }
          }
          const size_t feature_count =
              solve_entry
                  ? inverse_depths_before_optimization.size()
                  : static_cast<size_t>(lastSolvedFeatureCount);
          for (size_t i = 0; i < feature_count; ++i) {
            const double inverse_depth =
                solve_entry ? inverse_depths_before_optimization[i]
                            : lastSolvedFeatureArray[i][0];
            scaled_feature[i][0] = inverse_depth / scale;
          }
          const auto huber_cost = [](double squared_norm, double delta) {
            const double delta_squared = delta * delta;
            return squared_norm <= delta_squared
                       ? 0.5 * squared_norm
                       : delta * std::sqrt(squared_norm) -
                             0.5 * delta_squared;
          };

          if (options->hasImu()) {
            for (int j = 1; j <= frameCount; ++j) {
              if (!pre_integrations[j] ||
                  (pre_integrations[j]->sum_dt >= 10.0 &&
                   !replayIncludeLongImuFactors)) {
                continue;
              }
              const double *parameters[] = {
                  scaled_pose[j - 1], scaled_speed_bias[j - 1],
                  scaled_pose[j], scaled_speed_bias[j]};
              std::array<double, 15> residual{};
              IMUFactor factor(pre_integrations[j]);
              if (factor.Evaluate(parameters, residual.data(), nullptr)) {
                const Eigen::Map<const Eigen::Matrix<double, 15, 1>> vector(
                    residual.data());
                cost.imu += 0.5 * vector.squaredNorm();
              }
            }
          }

          int feature_index = -1;
          for (const auto &feature : featureManager.feature) {
            if (feature.feature_per_frame.size() <
                kMinFeatureTrackFrames) {
              continue;
            }
            ++feature_index;
            int imu_i = feature.start_frame;
            int imu_j = imu_i - 1;
            const Vector3d pts_i = feature.feature_per_frame[0].point;
            for (const auto &observation : feature.feature_per_frame) {
              ++imu_j;
              if (imu_i == imu_j) continue;
              ProjectionTwoFrameOneCamFactor factor(
                  pts_i, observation.point,
                  feature.feature_per_frame[0].velocity,
                  observation.velocity,
                  feature.feature_per_frame[0].cur_td,
                  observation.cur_td);
              const double *parameters[] = {
                  scaled_pose[imu_i], scaled_pose[imu_j],
                  solve_entry ? extrinsic_arrays_before_optimization[0]
                              : lastSolvedExtrinsicArray[0],
                  scaled_feature[feature_index],
                  solve_entry ? time_delay_array_before_optimization
                              : lastSolvedTimeDelayArray};
              std::array<double, 2> residual{};
              if (factor.Evaluate(parameters, residual.data(), nullptr)) {
                cost.visual += huber_cost(
                    residual[0] * residual[0] +
                        residual[1] * residual[1],
                    1.0);
              }
            }
          }

          const auto &prior = marginalization_before_optimization;
          if (prior && prior->valid && prior->n > 0) {
            std::vector<double *> prior_parameters;
            prior_parameters.reserve(
                marginalization_blocks_before_optimization.size());
            bool complete_mapping = true;
            for (double *block :
                 marginalization_blocks_before_optimization) {
              double *mapped = nullptr;
              for (int i = 0; i <= WINDOW_SIZE && mapped == nullptr; ++i) {
                if (block == poseArray[i]) {
                  mapped = scaled_pose[i];
                } else if (block == speedBiasArray[i]) {
                  mapped = scaled_speed_bias[i];
                }
              }
              for (int i = 0;
                   i < options->getNumCameras() && mapped == nullptr; ++i) {
                if (block == para_Ex_Pose[i]) {
                  mapped =
                      solve_entry ? extrinsic_arrays_before_optimization[i]
                                  : lastSolvedExtrinsicArray[i];
                }
              }
              if (block == para_Td[0]) {
                mapped = solve_entry ? time_delay_array_before_optimization
                                     : lastSolvedTimeDelayArray;
              }
              if (mapped == nullptr) {
                complete_mapping = false;
                break;
              }
              prior_parameters.push_back(mapped);
            }
            if (complete_mapping) {
              MarginalizationFactor prior_factor(prior);
              Eigen::VectorXd residual(prior->n);
              if (prior_factor.Evaluate(prior_parameters.data(),
                                        residual.data(), nullptr)) {
                cost.prior = 0.5 * residual.squaredNorm();
              }
            }
          }

          if (options->hasRange() &&
              solver_flag == SolverState::NON_LINEAR &&
              nonLinearUpdateCount >= WINDOW_SIZE && frameCount >= 1 &&
              rangeGroundInitialized) {
            const double sigma = std::max(0.005, options->range_noise);
            for (int j = 0; j <= frameCount; ++j) {
              if (!rangeMeasurements[j].valid) continue;
              const Quaterniond q(
                  scaled_pose[j][6], scaled_pose[j][3],
                  scaled_pose[j][4], scaled_pose[j][5]);
              const Vector3d down(0.0, 0.0, -1.0);
              const double vertical_offset =
                  (q * options->range_translation).z() +
                  rangeMeasurements[j].range * (q * down).z();
              const double target_z = rangeGroundZ - vertical_offset;
              const double residual =
                  (scaled_pose[j][2] - target_z) / sigma;
              cost.range += huber_cost(residual * residual, 2.0);
            }
          }
          return cost;
        };
    constexpr double kScaleDirectionStep = 0.005;
    const auto log_scale_direction = [&](bool solve_entry,
                                         const char *state_name) {
      const ScaleDirectionCost minus = evaluate_scale_direction_cost(
          solve_entry, 1.0 - kScaleDirectionStep);
      const ScaleDirectionCost center =
          evaluate_scale_direction_cost(solve_entry, 1.0);
      const ScaleDirectionCost plus = evaluate_scale_direction_cost(
          solve_entry, 1.0 + kScaleDirectionStep);
      const auto gradient = [&](double minus_cost, double plus_cost) {
        return (plus_cost - minus_cost) /
               (2.0 * kScaleDirectionStep);
      };
      const auto curvature = [&](double minus_cost, double center_cost,
                                 double plus_cost) {
        return (plus_cost - 2.0 * center_cost + minus_cost) /
               (kScaleDirectionStep * kScaleDirectionStep);
      };
      VINS_INFO << "Scale direction factor trace: timestamp=" << timestamp
                << ", state=" << state_name
                << ", step=" << kScaleDirectionStep
                << ", visual_cost=" << center.visual
                << ", visual_gradient="
                << gradient(minus.visual, plus.visual)
                << ", visual_curvature="
                << curvature(minus.visual, center.visual, plus.visual)
                << ", imu_cost=" << center.imu
                << ", imu_gradient=" << gradient(minus.imu, plus.imu)
                << ", imu_curvature="
                << curvature(minus.imu, center.imu, plus.imu)
                << ", prior_cost=" << center.prior
                << ", prior_gradient="
                << gradient(minus.prior, plus.prior)
                << ", prior_curvature="
                << curvature(minus.prior, center.prior, plus.prior)
                << ", range_cost=" << center.range
                << ", range_gradient="
                << gradient(minus.range, plus.range)
                << ", range_curvature="
                << curvature(minus.range, center.range, plus.range)
                << ", total_cost=" << center.total()
                << ", total_gradient="
                << gradient(minus.total(), plus.total())
                << ", total_curvature="
                << curvature(minus.total(), center.total(), plus.total());
    };
    log_scale_direction(true, "entry");
    log_scale_direction(false, "optimized");
    if (optimization_position_delta > 0.04) {
      const auto evaluate_visual_residuals = [&](bool solve_entry) {
        double squared_norm = 0.0;
        double newest_squared_norm = 0.0;
        int residual_count = 0;
        int newest_residual_count = 0;
        int feature_index = -1;
        for (const auto &feature : featureManager.feature) {
          if (feature.feature_per_frame.size() <
              kMinFeatureTrackFrames) {
            continue;
          }
          ++feature_index;
          int imu_i = feature.start_frame;
          int imu_j = imu_i - 1;
          const Vector3d pts_i = feature.feature_per_frame[0].point;
          for (const auto &observation : feature.feature_per_frame) {
            ++imu_j;
            if (imu_i == imu_j) continue;
            ProjectionTwoFrameOneCamFactor factor(
                pts_i, observation.point,
                feature.feature_per_frame[0].velocity,
                observation.velocity,
                feature.feature_per_frame[0].cur_td,
                observation.cur_td);
            double entry_inverse_depth[1]{
                inverse_depths_before_optimization[feature_index]};
            const double *parameters[] = {
                solve_entry ? pose_arrays_before_optimization[imu_i]
                            : lastSolvedPoseArray[imu_i],
                solve_entry ? pose_arrays_before_optimization[imu_j]
                            : lastSolvedPoseArray[imu_j],
                solve_entry ? extrinsic_arrays_before_optimization[0]
                            : lastSolvedExtrinsicArray[0],
                solve_entry ? entry_inverse_depth
                            : lastSolvedFeatureArray[feature_index],
                solve_entry ? time_delay_array_before_optimization
                            : lastSolvedTimeDelayArray};
            std::array<double, 2> residual{};
            if (!factor.Evaluate(parameters, residual.data(), nullptr)) {
              continue;
            }
            const double norm_squared =
                residual[0] * residual[0] +
                residual[1] * residual[1];
            squared_norm += norm_squared;
            residual_count += 2;
            if (imu_j == frameCount) {
              newest_squared_norm += norm_squared;
              newest_residual_count += 2;
            }
          }
        }
        return std::array<double, 4>{
            residual_count > 0
                ? std::sqrt(squared_norm / residual_count)
                : std::numeric_limits<double>::quiet_NaN(),
            static_cast<double>(residual_count),
            newest_residual_count > 0
                ? std::sqrt(newest_squared_norm / newest_residual_count)
                : std::numeric_limits<double>::quiet_NaN(),
            static_cast<double>(newest_residual_count)};
      };
      const auto visual_entry = evaluate_visual_residuals(true);
      const auto visual_optimized = evaluate_visual_residuals(false);
      vector<double> entry_depths;
      vector<double> optimized_depths;
      vector<double> optimized_to_entry_depth_ratios;
      for (size_t i = 0; i < inverse_depths_before_optimization.size(); ++i) {
        const double entry_inverse_depth =
            inverse_depths_before_optimization[i];
        const double optimized_inverse_depth =
            lastSolvedFeatureArray[i][0];
        if (!std::isfinite(entry_inverse_depth) ||
            !std::isfinite(optimized_inverse_depth) ||
            entry_inverse_depth <= 1e-9 ||
            optimized_inverse_depth <= 1e-9) {
          continue;
        }
        const double entry_depth = 1.0 / entry_inverse_depth;
        const double optimized_depth = 1.0 / optimized_inverse_depth;
        entry_depths.push_back(entry_depth);
        optimized_depths.push_back(optimized_depth);
        optimized_to_entry_depth_ratios.push_back(
            optimized_depth / entry_depth);
      }
      const auto median = [](vector<double> values) {
        if (values.empty()) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        const size_t middle = values.size() / 2;
        std::nth_element(
            values.begin(), values.begin() + middle, values.end());
        return values[middle];
      };
      VINS_INFO << "Velocity visual residual trace: timestamp=" << timestamp
                << ", all_rms_entry=" << visual_entry[0]
                << ", all_rms_optimized=" << visual_optimized[0]
                << ", all_residuals=" << visual_entry[1]
                << ", newest_rms_entry=" << visual_entry[2]
                << ", newest_rms_optimized=" << visual_optimized[2]
                << ", newest_residuals=" << visual_entry[3]
                << ", valid_depths=" << entry_depths.size()
                << ", median_depth_entry=" << median(entry_depths)
                << ", median_depth_optimized=" << median(optimized_depths)
                << ", median_depth_ratio="
                << median(optimized_to_entry_depth_ratios);
      struct VisualTrackCost {
        int feature_id = -1;
        int observations = 0;
        int saturated_entry = 0;
        int saturated_optimized = 0;
        double entry = 0.0;
        double optimized = 0.0;
        double newest_entry = 0.0;
        double newest_optimized = 0.0;
      };
      const auto huber_cost = [](double squared_norm) {
        return squared_norm <= 1.0
                   ? 0.5 * squared_norm
                   : std::sqrt(squared_norm) - 0.5;
      };
      vector<VisualTrackCost> visual_track_costs;
      double visual_robust_entry = 0.0;
      double visual_robust_optimized = 0.0;
      double visual_newest_robust_entry = 0.0;
      double visual_newest_robust_optimized = 0.0;
      int visual_saturated_entry = 0;
      int visual_saturated_optimized = 0;
      int feature_index = -1;
      for (const auto &feature : featureManager.feature) {
        if (feature.feature_per_frame.size() < kMinFeatureTrackFrames) {
          continue;
        }
        ++feature_index;
        VisualTrackCost track;
        track.feature_id = feature.feature_id;
        int imu_i = feature.start_frame;
        int imu_j = imu_i - 1;
        const Vector3d pts_i = feature.feature_per_frame[0].point;
        int observation_index = 0;
        for (const auto &observation : feature.feature_per_frame) {
          ++imu_j;
          if (imu_i == imu_j) {
            ++observation_index;
            continue;
          }
          ProjectionTwoFrameOneCamFactor factor(
              pts_i, observation.point,
              feature.feature_per_frame[0].velocity,
              observation.velocity,
              feature.feature_per_frame[0].cur_td,
              observation.cur_td);
          const double *entry_parameters[] = {
              pose_arrays_before_optimization[imu_i],
              pose_arrays_before_optimization[imu_j],
              extrinsic_arrays_before_optimization[0],
              &inverse_depths_before_optimization[feature_index],
              time_delay_array_before_optimization};
          const double *optimized_parameters[] = {
              lastSolvedPoseArray[imu_i], lastSolvedPoseArray[imu_j],
              lastSolvedExtrinsicArray[0],
              lastSolvedFeatureArray[feature_index],
              lastSolvedTimeDelayArray};
          std::array<double, 2> entry_residual{};
          std::array<double, 2> optimized_residual{};
          if (!factor.Evaluate(entry_parameters, entry_residual.data(),
                               nullptr) ||
              !factor.Evaluate(optimized_parameters,
                               optimized_residual.data(), nullptr)) {
            continue;
          }
          const double entry_squared =
              entry_residual[0] * entry_residual[0] +
              entry_residual[1] * entry_residual[1];
          const double optimized_squared =
              optimized_residual[0] * optimized_residual[0] +
              optimized_residual[1] * optimized_residual[1];
          const double entry_cost = huber_cost(entry_squared);
          const double optimized_cost = huber_cost(optimized_squared);
          const auto observation_cost_at_scale =
              [&](bool solve_entry, double scale) {
                const double(*base_pose)[SIZE_POSE] =
                    solve_entry ? pose_arrays_before_optimization
                                : lastSolvedPoseArray;
                double scaled_anchor_pose[SIZE_POSE];
                double scaled_observation_pose[SIZE_POSE];
                std::memcpy(scaled_anchor_pose, base_pose[imu_i],
                            sizeof(scaled_anchor_pose));
                std::memcpy(scaled_observation_pose, base_pose[imu_j],
                            sizeof(scaled_observation_pose));
                const Vector3d scale_anchor(
                    base_pose[0][0], base_pose[0][1], base_pose[0][2]);
                for (int axis = 0; axis < 3; ++axis) {
                  scaled_anchor_pose[axis] =
                      scale_anchor(axis) +
                      scale * (base_pose[imu_i][axis] -
                               scale_anchor(axis));
                  scaled_observation_pose[axis] =
                      scale_anchor(axis) +
                      scale * (base_pose[imu_j][axis] -
                               scale_anchor(axis));
                }
                double scaled_inverse_depth[1]{
                    (solve_entry
                         ? inverse_depths_before_optimization[feature_index]
                         : lastSolvedFeatureArray[feature_index][0]) /
                    scale};
                const double *parameters[] = {
                    scaled_anchor_pose, scaled_observation_pose,
                    solve_entry
                        ? extrinsic_arrays_before_optimization[0]
                        : lastSolvedExtrinsicArray[0],
                    scaled_inverse_depth,
                    solve_entry ? time_delay_array_before_optimization
                                : lastSolvedTimeDelayArray};
                std::array<double, 2> residual{};
                if (!factor.Evaluate(parameters, residual.data(), nullptr)) {
                  return std::numeric_limits<double>::quiet_NaN();
                }
                return huber_cost(residual[0] * residual[0] +
                                  residual[1] * residual[1]);
              };
          const double entry_minus = observation_cost_at_scale(
              true, 1.0 - kScaleDirectionStep);
          const double entry_plus = observation_cost_at_scale(
              true, 1.0 + kScaleDirectionStep);
          const double optimized_minus = observation_cost_at_scale(
              false, 1.0 - kScaleDirectionStep);
          const double optimized_plus = observation_cost_at_scale(
              false, 1.0 + kScaleDirectionStep);
          VINS_INFO
              << "Visual observation scale trace: timestamp=" << timestamp
              << ", feature_id=" << feature.feature_id
              << ", observation_index=" << observation_index
              << ", observation_slot=" << imu_j
              << ", observation_timestamp="
              << estimator_state[imu_j].timestamp
              << ", newest=" << (imu_j == frameCount)
              << ", cost_entry=" << entry_cost
              << ", cost_optimized=" << optimized_cost
              << ", improvement=" << entry_cost - optimized_cost
              << ", saturated_entry=" << (entry_squared > 1.0)
              << ", saturated_optimized=" << (optimized_squared > 1.0)
              << ", scale_gradient_entry="
              << (entry_plus - entry_minus) /
                     (2.0 * kScaleDirectionStep)
              << ", scale_gradient_optimized="
              << (optimized_plus - optimized_minus) /
                     (2.0 * kScaleDirectionStep);
          track.entry += entry_cost;
          track.optimized += optimized_cost;
          ++track.observations;
          if (entry_squared > 1.0) {
            ++track.saturated_entry;
            ++visual_saturated_entry;
          }
          if (optimized_squared > 1.0) {
            ++track.saturated_optimized;
            ++visual_saturated_optimized;
          }
          visual_robust_entry += entry_cost;
          visual_robust_optimized += optimized_cost;
          if (imu_j == frameCount) {
            track.newest_entry += entry_cost;
            track.newest_optimized += optimized_cost;
            visual_newest_robust_entry += entry_cost;
            visual_newest_robust_optimized += optimized_cost;
          }
          ++observation_index;
        }
        visual_track_costs.push_back(track);
      }
      std::sort(visual_track_costs.begin(), visual_track_costs.end(),
                [](const VisualTrackCost &a, const VisualTrackCost &b) {
                  return (a.entry - a.optimized) >
                         (b.entry - b.optimized);
                });
      double top_five_improvement = 0.0;
      for (size_t rank = 0;
           rank < std::min<size_t>(5, visual_track_costs.size()); ++rank) {
        const auto &track = visual_track_costs[rank];
        top_five_improvement += track.entry - track.optimized;
        VINS_INFO << "Visual track cost trace: timestamp=" << timestamp
                  << ", rank=" << rank + 1
                  << ", feature_id=" << track.feature_id
                  << ", observations=" << track.observations
                  << ", cost_entry=" << track.entry
                  << ", cost_optimized=" << track.optimized
                  << ", improvement=" << track.entry - track.optimized
                  << ", newest_entry=" << track.newest_entry
                  << ", newest_optimized=" << track.newest_optimized
                  << ", saturated_entry=" << track.saturated_entry
                  << ", saturated_optimized="
                  << track.saturated_optimized;
      }
      const double visual_improvement =
          visual_robust_entry - visual_robust_optimized;
      VINS_INFO << "Visual robust cost trace: timestamp=" << timestamp
                << ", cost_entry=" << visual_robust_entry
                << ", cost_optimized=" << visual_robust_optimized
                << ", improvement=" << visual_improvement
                << ", newest_entry=" << visual_newest_robust_entry
                << ", newest_optimized="
                << visual_newest_robust_optimized
                << ", newest_improvement="
                << visual_newest_robust_entry -
                       visual_newest_robust_optimized
                << ", historical_improvement="
                << visual_improvement -
                       (visual_newest_robust_entry -
                        visual_newest_robust_optimized)
                << ", saturated_entry=" << visual_saturated_entry
                << ", saturated_optimized="
                << visual_saturated_optimized
                << ", top_five_improvement=" << top_five_improvement
                << ", top_five_fraction="
                << (visual_improvement > 0.0
                        ? top_five_improvement / visual_improvement
                        : std::numeric_limits<double>::quiet_NaN());
      for (int i = 0; i <= frameCount; ++i) {
        VINS_INFO << "Velocity window slot trace: timestamp=" << timestamp
                  << ", slot=" << i
                  << ", slot_timestamp=" << estimator_state[i].timestamp
                  << ", position_delta="
                  << (estimator_state[i].position -
                      states_before_optimization[i].position).transpose()
                  << ", velocity_entry="
                  << states_before_optimization[i].velocity.transpose()
                  << ", velocity_optimized="
                  << estimator_state[i].velocity.transpose();
      }
    }
  }
  if (replayPositionJumpQuarantine &&
      (replayQuarantineUpdatesRemaining > 0 ||
       output_position_step > kMaxQuarantineOutputStep)) {
    const bool entering_quarantine =
        replayQuarantineUpdatesRemaining == 0;
    if (entering_quarantine) {
      replayQuarantineUpdatesRemaining =
          kReplayQuarantineObservationUpdates;
      replayQuarantineConsecutiveHealthy = 0;
      VINS_WARN << "Replay quarantine entered: timestamp=" << timestamp
                << ", features=" << active_feature_count
                << ", correction=" << optimization_position_delta
                << ", output_step=" << output_position_step
                << ", marginalization="
                << (isNewMarginalization() ? "second_new" : "old")
                << ", solver_termination="
                << ceres::TerminationTypeToString(
                       lastOptimizationTermination);
    } else {
      const bool healthy =
          optimization_position_delta <= kMaxQuarantineCorrection &&
          output_position_step <= kMaxQuarantineOutputStep &&
          validateEstimatorState("replay quarantine");
      if (healthy) {
        ++replayQuarantineConsecutiveHealthy;
      } else {
        replayQuarantineConsecutiveHealthy = 0;
      }
      --replayQuarantineUpdatesRemaining;
      VINS_WARN << "Replay quarantine observation: timestamp=" << timestamp
                << ", healthy=" << healthy
                << ", remaining=" << replayQuarantineUpdatesRemaining
                << ", consecutive_healthy="
                << replayQuarantineConsecutiveHealthy
                << ", features=" << active_feature_count
                << ", correction=" << optimization_position_delta
                << ", output_step=" << output_position_step
                << ", marginalization="
                << (isNewMarginalization() ? "second_new" : "old")
                << ", solver_termination="
                << ceres::TerminationTypeToString(
                       lastOptimizationTermination);
      if (replayQuarantineUpdatesRemaining == 0) {
        if (replayQuarantineConsecutiveHealthy ==
            kReplayQuarantineObservationUpdates) {
          VINS_WARN << "Replay quarantine recovered after three bounded "
                       "updates; resuming output.";
          replayQuarantineConsecutiveHealthy = 0;
        } else {
          VINS_ERROR << "Replay quarantine failed; restoring fail-closed "
                        "reset after three observations.";
          replayQuarantineConsecutiveHealthy = 0;
          failure_occur = 1;
          resetStateLocked(true);
          initializeCamerasFromOptions();
          return;
        }
      }
    }

    if (entering_quarantine || replayQuarantineUpdatesRemaining > 0) {
      restore_replay_solve_snapshot();
      lastBackendTransactionRejected = true;
      VINS_WARN << "Replay quarantine restored suspect solve mutation and "
                   "withheld odometry output.";
      slideWindow();
      featureManager.removeFailures();
      updateLatestStates();
      return;
    }
  }
  // A low feature count alone is normal during yaw, and a moderate initial
  // correction alone is normal during probation.  Persistent corruption runs
  // instead showed both weak active geometry and a large optimizer mutation:
  // healthy clean runs stayed <=0.026 m while the failed run reached 0.285 m.
  // Reject the combined condition before safe_vio_odom is updated.
  constexpr int kMinActiveFeaturesForLargeCorrection = 100;
  // Three-frame tracks occasionally need a 0.10--0.11 m correction while
  // turning, and resetting at that boundary caused otherwise recoverable
  // runs to cascade into repeated reinitialization. Keep this aligned with
  // the independent 0.15 m estimator/output guards; the observed corrupt
  // low-geometry mutations were 0.222--0.285 m and remain rejected here.
  constexpr double kMaxLowGeometryOptimizationCorrection = 0.15;
  if (!stationary_update_applied &&
      active_feature_count < kMinActiveFeaturesForLargeCorrection &&
      optimization_position_delta >
          kMaxLowGeometryOptimizationCorrection) {
    VINS_ERROR << "Nonlinear geometry health rejected update: timestamp="
               << timestamp << ", features=" << active_feature_count
               << ", position_delta=" << optimization_position_delta
               << ", accel_bias="
               << state_after_optimization.accel_bias.transpose();
    failure_occur = 1;
    resetStateLocked(true);
    initializeCamerasFromOptions();
    return;
  }
  if (nonLinearUpdateCount <= 3) {
    VINS_INFO << "Early nonlinear update: index=" << nonLinearUpdateCount
              << ", timestamp=" << timestamp
              << ", features=" << featureManager.getFeatureCount()
              << ", position_delta="
              << optimization_position_delta
              << ", accel_bias_before="
              << state_before_optimization.accel_bias.transpose()
              << ", accel_bias_after="
              << state_after_optimization.accel_bias.transpose()
              << ", gyro_bias_before="
              << state_before_optimization.gyro_bias.transpose()
              << ", gyro_bias_after="
              << state_after_optimization.gyro_bias.transpose();
  }
  set<int> removeIndex;
  if (!options->replacesFeatureVisual()) {
    outliersRejection(removeIndex);
    featureManager.removeOutlier(removeIndex);
  }
  // The first three nonlinear updates are already withheld from every
  // odometry publisher.  Let their internal position origin settle without
  // treating that unpublished correction as an in-flight output jump.
  // Finiteness, bias and attitude checks in failureDetection remain active.
  const bool absorb_probation_position_correction =
      nonLinearUpdateCount <= 3 &&
      output_position_step > kMaxQuarantineOutputStep;
  const Vector3d failure_reference_position = last_state.position;
  if (absorb_probation_position_correction) {
    last_state.position = estimator_state[WINDOW_SIZE].position;
    VINS_WARN << "Initialization probation absorbed unpublished position "
                 "correction: timestamp="
              << timestamp << ", update=" << nonLinearUpdateCount
              << ", correction=" << optimization_position_delta
              << ", output_step=" << output_position_step;
  }
  const bool failure_detected =
      failureDetection(trusted_metric_correction ||
                       trusted_dropout_correction);
  if (absorb_probation_position_correction) {
    last_state.position = failure_reference_position;
  }
  if (failure_detected) {
    failure_occur = 1;
    resetStateLocked(true);
    initializeCamerasFromOptions();
    return;
  }

  slideWindow();
  featureManager.removeFailures();
  last_state = estimator_state[WINDOW_SIZE];
  last_state0 = estimator_state[0];
  updateLatestStates();

  constexpr int kInitializationProbationUpdates = 3;
  if (nonLinearUpdateCount < kInitializationProbationUpdates) {
    VINS_INFO << "Initialization probation: update=" << nonLinearUpdateCount
              << "/" << kInitializationProbationUpdates
              << ", odometry publication withheld.";
    return;
  }
  if (nonLinearUpdateCount == kInitializationProbationUpdates) {
    VINS_INFO << "Initialization probation passed; odometry publication "
                 "enabled; candidate_alignment_scale="
              << currentAlignmentScale;
  }
  constexpr int kStableEpochUpdates = 20;
  if (nonLinearUpdateCount == kStableEpochUpdates &&
      currentAlignmentScale > 0.0) {
    lastPublishedAlignmentScale = currentAlignmentScale;
    VINS_INFO << "Stable VIO epoch accepted alignment scale: updates="
              << nonLinearUpdateCount
              << ", scale=" << lastPublishedAlignmentScale;
  }

  // prepare output of VINS
  {
    key_poses.timestamp = timestamp;
    key_poses.poses.clear();
    for (int i = 0; i <= WINDOW_SIZE; i++)
      key_poses.poses.push_back(estimator_state[i].position);

    if (!backendCandidateWorkspaceActive) {
      safe_key_poses.set(key_poses);
    }
  }

  // Publish the stereo+IMU state at the freshest integrated IMU timestamp.
  // A camera keyframe can be 100--160 ms old under renderer jitter; exposing
  // that old header on the primary odometry stream violates the Stage-B
  // freshness contract even though updateLatestStates() has already
  // propagated the optimized camera state to the current IMU sample.
  vio_odom.timestamp = latestImuData.timestamp;
  vio_odom.position = latest_state.position;
  vio_odom.orientation = latest_Q;
  vio_odom.velocity = latest_state.velocity;
  if (!backendCandidateWorkspaceActive) {
    safe_vio_odom.set(vio_odom);
  }
}

void Estimator::processMonoWithImuInitialization(Timestamp timestamp) {
  if (frameCount != WINDOW_SIZE) return;

  bool result = false;
  if (!options->isInitializingExtrinsic() &&
      (timestamp - initialTimestamp) > 0.5) {
    result = initialStructure();
    initialTimestamp = timestamp;
  }
  if (result) {
    StateData states_before_initial_optimization[WINDOW_SIZE + 1];
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      states_before_initial_optimization[i] = estimator_state[i];
    }
    const VectorXd depths_before_initial_optimization =
        featureManager.getDepthVector();
    Matrix3d camera_rotation_before_initial_optimization[2];
    Vector3d camera_translation_before_initial_optimization[2];
    for (int i = 0; i < options->getNumCameras(); ++i) {
      camera_rotation_before_initial_optimization[i] = cameraRotation[i];
      camera_translation_before_initial_optimization[i] = cameraTranslation[i];
    }
    const double time_delay_before_initial_optimization = options->time_delay;

    // The initialization window contains non-keyframe slots that are filled
    // by the first nonlinear solve.  Validating every slot before that solve
    // rejects otherwise valid visual/IMU alignments.  optimize() validates
    // the fully populated state immediately after Ceres updates it.
    if (!optimize()) {
      VINS_ERROR << "Initialization produced an unusable state; resetting.";
      resetStateLocked();
      initializeCamerasFromOptions();
      return;
    }
    // Do not expose a candidate that already violates the same bias limits
    // used by the nonlinear failure detector.  Waiting for the next image
    // would publish a corrupt pose briefly and then reset.  Restore the
    // pre-solve window and slide it instead of resetting the live input epoch;
    // this keeps the next healthy initialization window available.
    bool warm_handover_valid = true;
    double warm_position_correction =
        std::numeric_limits<double>::quiet_NaN();
    double warm_velocity_correction =
        std::numeric_limits<double>::quiet_NaN();
    double warm_attitude_correction_deg =
        std::numeric_limits<double>::quiet_NaN();
    if (replayWarmReinitialization &&
        (warmReinitializationBridge.active() ||
         warmReinitializationBridge.expired())) {
      OdomData propagated;
      const StateData &candidate = estimator_state[WINDOW_SIZE];
      warm_handover_valid =
          warmReinitializationBridge.active() &&
          warmReinitializationBridge.sample(candidate.timestamp,
                                             &propagated);
      if (warm_handover_valid) {
        warm_position_correction =
            (candidate.position - propagated.position).norm();
        warm_velocity_correction =
            (candidate.velocity - propagated.velocity).norm();
        const Eigen::Quaterniond candidate_q(candidate.rotation);
        const Eigen::Quaterniond delta_q =
            propagated.orientation.conjugate() * candidate_q;
        warm_attitude_correction_deg =
            2.0 * std::acos(std::max(
                      -1.0, std::min(1.0, std::abs(delta_q.w())))) *
            180.0 / M_PI;
        warm_handover_valid =
            std::isfinite(warm_position_correction) &&
            std::isfinite(warm_velocity_correction) &&
            std::isfinite(warm_attitude_correction_deg) &&
            warm_position_correction <= 0.5 &&
            warm_velocity_correction <= 0.5 &&
            warm_attitude_correction_deg <= 15.0;
      }
      VINS_WARN << "Warm reinitialization handover evidence: valid="
                << warm_handover_valid
                << ", position_correction_m="
                << warm_position_correction
                << ", velocity_correction_m="
                << warm_velocity_correction
                << ", attitude_correction_deg="
                << warm_attitude_correction_deg
                << ", bridge_expired="
                << warmReinitializationBridge.expired();
    }
    if (!validateInitializationHealth() || !warm_handover_valid) {
      for (int i = 0; i <= WINDOW_SIZE; ++i) {
        estimator_state[i] = states_before_initial_optimization[i];
      }
      featureManager.setDepth(depths_before_initial_optimization);
      for (int i = 0; i < options->getNumCameras(); ++i) {
        cameraRotation[i] = camera_rotation_before_initial_optimization[i];
        cameraTranslation[i] =
            camera_translation_before_initial_optimization[i];
        updateCameraPose(i);
      }
      options->time_delay = time_delay_before_initial_optimization;
      last_marginalization_info = nullptr;
      last_marginalization_parameter_blocks.clear();
      VINS_INFO << "Initialization candidate rollback complete; retrying on "
                   "the next window.";
      slideWindow();
      return;
    }
    updateLatestStates();
    // failureDetection compares against last_state on the very next frame.
    // Seed it from the initialized solution instead of the default zero pose.
    last_state = estimator_state[WINDOW_SIZE];
    last_state0 = estimator_state[0];
    solver_flag = SolverState::NON_LINEAR;
    if (warmReinitializationBridge.active()) {
      VINS_WARN << "Warm reinitialization handover accepted: "
                << "position_correction_m=" << warm_position_correction
                << ", velocity_correction_m="
                << warm_velocity_correction
                << ", attitude_correction_deg="
                << warm_attitude_correction_deg;
      warmReinitializationBridge.clear();
    }
    slideWindow();
    const StateData &initialized = estimator_state[WINDOW_SIZE];
    VINS_INFO << "Initialization state: features="
              << featureManager.getFeatureCount()
              << ", solver_initial_cost=" << lastOptimizationInitialCost
              << ", solver_final_cost=" << lastOptimizationFinalCost
              << ", solver_residual_blocks="
              << lastOptimizationResidualBlocks
              << ", solver_residuals=" << lastOptimizationResiduals
              << ", solver_cost_per_block="
              << (lastOptimizationResidualBlocks > 0
                      ? lastOptimizationFinalCost /
                            lastOptimizationResidualBlocks
                      : 0.0)
              << ", velocity=" << initialized.velocity.transpose()
              << ", accel_bias=" << initialized.accel_bias.transpose()
              << ", gyro_bias=" << initialized.gyro_bias.transpose();
    VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
  } else {
    slideWindow();
  }
}

void Estimator::processStereoWithImuInitialization() {
  featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                    cameraTranslation, cameraRotation);
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);

  if (frameCount != WINDOW_SIZE) return;

  map<double, ImageFrame>::iterator frame_it;
  int i = 0;
  for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end();
       frame_it++) {
    frame_it->second.R = estimator_state[i].rotation;
    frame_it->second.T = estimator_state[i].position;
    i++;
  }
  solveGyroscopeBias(all_image_frame, estimator_state);
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Vector3d::Zero(),
                                     estimator_state[i].gyro_bias);
  }
  optimize();
  updateLatestStates();
  last_state = estimator_state[WINDOW_SIZE];
  last_state0 = estimator_state[0];
  solver_flag = SolverState::NON_LINEAR;
  slideWindow();
  VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
}

void Estimator::processStereoWithoutImuInitialization() {
  featureManager.initFramePoseByPnP(frameCount, estimator_state,
                                    cameraTranslation, cameraRotation);
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  optimize();

  if (frameCount != WINDOW_SIZE) return;

  optimize();
  updateLatestStates();
  last_state = estimator_state[WINDOW_SIZE];
  last_state0 = estimator_state[0];
  solver_flag = SolverState::NON_LINEAR;
  slideWindow();
  VINS_INFO << "Initialization complete. Switching to NON_LINEAR mode.";
}

bool Estimator::isNonLinearSolver() const {
  return solver_flag == SolverState::NON_LINEAR;
}

bool Estimator::isNewMarginalization() const {
  return marginalization_flag == MarginalizationType::MARGIN_SECOND_NEW;
}

bool Estimator::checkIMUExcitation() {
  Vector3d sumG = Vector3d::Zero();
  auto it = std::next(all_image_frame.begin());
  for (; it != all_image_frame.end(); ++it) {
    double dt = it->second.pre_integration->sum_dt;
    sumG += it->second.pre_integration->delta_v / dt;
  }

  Vector3d avgG = sumG / (all_image_frame.size() - 1);
  double variance = 0;
  for (it = std::next(all_image_frame.begin()); it != all_image_frame.end();
       ++it) {
    double dt = it->second.pre_integration->sum_dt;
    Vector3d tmpG = it->second.pre_integration->delta_v / dt;
    variance += (tmpG - avgG).squaredNorm();
  }

  variance = std::sqrt(variance / (all_image_frame.size() - 1));
  // The upstream 0.25 m/s^2 threshold assumes hand-held, jerky motion.  A
  // multicopter excitation is lower, but accepting the ~0.01 m/s^2 hover
  // vibration produced severely under-observed scale estimates.  Require a
  // deliberate post-hover initialization manoeuvre well above the ~0.002
  // m/s^2 Gazebo noise floor.
  // Reject the low-dynamic start of a position-controlled hover.  It has
  // enough pixel parallax to pass the visual test but not enough inertial
  // information to determine a trustworthy mono metric scale.
  constexpr double kMinExcitation = 0.15;
  VINS_INFO << "IMU excitation=" << variance
            << " m/s^2 (required " << kMinExcitation << ")";
  return std::isfinite(variance) && variance >= kMinExcitation;
}

void Estimator::logInitializationDiagnostics(Timestamp timestamp) const {
  size_t track_observations = 0;
  size_t tracks_at_least_four_frames = 0;
  size_t max_track_frames = 0;
  for (const auto &feature : featureManager.feature) {
    const size_t track_frames = feature.feature_per_frame.size();
    track_observations += track_frames;
    max_track_frames = std::max(max_track_frames, track_frames);
    if (track_frames >= 4) ++tracks_at_least_four_frames;
  }

  size_t imu_samples = 0;
  double imu_duration = 0.0;
  double min_imu_dt = std::numeric_limits<double>::infinity();
  double max_imu_dt = 0.0;
  Vector3d mean_accel = Vector3d::Zero();
  Vector3d mean_gyro = Vector3d::Zero();
  for (int frame = 1; frame <= WINDOW_SIZE; ++frame) {
    for (const auto &imu : deltaTimeImuBuffer[frame]) {
      const double dt = imu.timestamp;
      if (!std::isfinite(dt) || dt <= 0.0) continue;
      ++imu_samples;
      imu_duration += dt;
      min_imu_dt = std::min(min_imu_dt, dt);
      max_imu_dt = std::max(max_imu_dt, dt);
      mean_accel += dt * imu.linear_acceleration;
      mean_gyro += dt * imu.angular_velocity;
    }
  }
  if (imu_duration > 0.0) {
    mean_accel /= imu_duration;
    mean_gyro /= imu_duration;
  }

  double accel_variance = 0.0;
  double gyro_variance = 0.0;
  for (int frame = 1; frame <= WINDOW_SIZE; ++frame) {
    for (const auto &imu : deltaTimeImuBuffer[frame]) {
      const double dt = imu.timestamp;
      if (!std::isfinite(dt) || dt <= 0.0) continue;
      accel_variance +=
          dt * (imu.linear_acceleration - mean_accel).squaredNorm();
      gyro_variance +=
          dt * (imu.angular_velocity - mean_gyro).squaredNorm();
    }
  }
  const double accel_rms = imu_duration > 0.0
                               ? std::sqrt(accel_variance / imu_duration)
                               : 0.0;
  const double gyro_rms = imu_duration > 0.0
                              ? std::sqrt(gyro_variance / imu_duration)
                              : 0.0;
  const double camera_duration =
      estimator_state[WINDOW_SIZE].timestamp - estimator_state[0].timestamp;
  const double mean_track_frames = featureManager.feature.empty()
                                       ? 0.0
                                       : static_cast<double>(track_observations) /
                                             featureManager.feature.size();

  VINS_INFO << "Initialization diagnostics: timestamp=" << timestamp
            << ", window_start=" << estimator_state[0].timestamp
            << ", window_end=" << estimator_state[WINDOW_SIZE].timestamp
            << ", camera_duration=" << camera_duration
            << ", imu_duration=" << imu_duration
            << ", integration_duration_error="
            << (imu_duration - camera_duration)
            << ", imu_samples=" << imu_samples
            << ", imu_dt_min="
            << (std::isfinite(min_imu_dt) ? min_imu_dt : 0.0)
            << ", imu_dt_max=" << max_imu_dt
            << ", accel_mean=" << mean_accel.transpose()
            << ", accel_rms=" << accel_rms
            << ", gyro_mean=" << mean_gyro.transpose()
            << ", gyro_rms=" << gyro_rms
            << ", features=" << featureManager.feature.size()
            << ", mean_track_frames=" << mean_track_frames
            << ", max_track_frames=" << max_track_frames
            << ", tracks_ge_4=" << tracks_at_least_four_frames
            << ", last_track_num=" << featureManager.last_track_num
            << ", long_track_num=" << featureManager.long_track_num
            << ", new_feature_num=" << featureManager.new_feature_num
            << ", last_parallax_px="
            << featureManager.last_average_parallax;
}

std::vector<SFMFeature> Estimator::buildSFMFeatures() {
  std::vector<SFMFeature> sfm_features;

  for (auto &feature : featureManager.feature) {
    SFMFeature sf;
    sf.state = false;
    sf.id = feature.feature_id;
    int frame_id = feature.start_frame - 1;
    for (auto &f : feature.feature_per_frame) {
      ++frame_id;
      sf.observation.emplace_back(frame_id, Vector2d(f.point.x(), f.point.y()));
    }
    sfm_features.emplace_back(std::move(sf));
  }

  return sfm_features;
}

bool Estimator::solvePoseWithPnP(const std::vector<Quaterniond> &rotations,
                                 const std::vector<Vector3d> &positions,
                                 const std::map<int, Vector3d> &sfmPoints) {
  auto frame_it = all_image_frame.begin();
  int i = 0;

  while (frame_it != all_image_frame.end()) {
    if (frame_it->first == estimator_state[i].timestamp) {
      frame_it->second.is_key_frame = true;
      frame_it->second.R =
          rotations[i].toRotationMatrix() * options->RIC[0].transpose();
      frame_it->second.T = positions[i];
      ++i;
      ++frame_it;
      continue;
    }

    if (frame_it->first > estimator_state[i].timestamp) {
      ++i;
    }
    cv::Mat tmp_r;
    cv::Mat r, rvec, tvec, D;
    Matrix3d R_inital = (rotations[i].inverse()).toRotationMatrix();
    Vector3d P_inital = -R_inital * positions[i];
    cv::eigen2cv(R_inital, tmp_r);
    cv::Rodrigues(tmp_r, rvec);
    cv::eigen2cv(P_inital, tvec);

    frame_it->second.is_key_frame = false;
    vector<cv::Point3f> pts3;
    vector<cv::Point2f> pts2;

    for (const auto &pt : frame_it->second.points) {
      auto it = sfmPoints.find(pt.first);
      if (it == sfmPoints.end()) continue;

      for (const auto &obs : pt.second) {
        Vector3d p3d = it->second;
        Vector2d p2d = obs.second.head<2>();
        pts3.emplace_back(p3d.x(), p3d.y(), p3d.z());
        pts2.emplace_back(p2d.x(), p2d.y());
      }
    }

    if (pts3.size() < 6) {
      VINS_WARN << "Not enough points for solvePnP: " << pts3.size();
      return false;
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    if (!cv::solvePnP(pts3, pts2, K, D, rvec, tvec, true)) {
      VINS_WARN << "Failed to solvePnP";
      return false;
    }

    cv::Rodrigues(rvec, r);
    MatrixXd R_pnp, tmp_R_pnp;
    cv::cv2eigen(r, tmp_R_pnp);
    R_pnp = tmp_R_pnp.transpose();
    MatrixXd T_pnp;
    cv::cv2eigen(tvec, T_pnp);
    T_pnp = R_pnp * (-T_pnp);
    frame_it->second.R = R_pnp * options->RIC[0].transpose();
    frame_it->second.T = T_pnp;
    ++frame_it;
  }

  return true;
}

bool Estimator::initialStructure() {
  if (!checkIMUExcitation()) {
    VINS_WARN << "IMU excitation insufficient";
    return false;
  }
  logInitializationDiagnostics(estimator_state[WINDOW_SIZE].timestamp);

  auto sfm_features = buildSFMFeatures();

  Matrix3d relativeRotation;
  Vector3d relativeTranslation;
  int referenceFrame;
  if (!computeRelativePose(relativeRotation, relativeTranslation,
                           referenceFrame)) {
    VINS_WARN << "Not enough features or parallax; Move device around!!!";
    return false;
  }

  std::vector<Quaterniond> rotations(WINDOW_SIZE + 1);
  std::vector<Vector3d> positions(WINDOW_SIZE + 1);
  std::map<int, Vector3d> sfmPoints;
  GlobalSFM sfm;
  if (!sfm.construct(frameCount + 1, rotations, positions, referenceFrame,
                     relativeRotation, relativeTranslation, sfm_features,
                     sfmPoints)) {
    marginalization_flag = MarginalizationType::MARGIN_OLD;
    VINS_WARN << "Failed to construect sfm!!!";
    return false;
  }

  if (!solvePoseWithPnP(rotations, positions, sfmPoints)) {
    return false;
  }

  return visualInitialAlign();
}

bool Estimator::visualInitialAlign() {
  TicToc t_g;
  VectorXd x;
  // solve scale
  bool result = VisualIMUAlignment(all_image_frame, estimator_state, gravity, x,
                                   *options);
  if (!result) {
    VINS_DEBUG << "misalign visual structure with IMU";
    return false;
  }

  // change state
  for (int i = 0; i <= frameCount; i++) {
    Matrix3d Ri = all_image_frame[estimator_state[i].timestamp].R;
    Vector3d Pi = all_image_frame[estimator_state[i].timestamp].T;
    estimator_state[i].position = Pi;
    estimator_state[i].rotation = Ri;
    all_image_frame[estimator_state[i].timestamp].is_key_frame = true;
  }

  double s = (x.tail<1>())(0);
  VINS_INFO << "Visual/IMU alignment candidate: scale=" << s
            << ", gravity=" << gravity.transpose()
            << ", gravity_norm=" << gravity.norm();
  // SfM translation has an arbitrary unit, so its positive scale can be much
  // smaller than one.  Metric correctness is checked with range constraints
  // and the preflight path-scale validator, not an arbitrary scale floor.
  if (!x.allFinite() || !std::isfinite(s) || s <= 1e-3 || s > 100.0 ||
      !gravity.allFinite() || gravity.norm() < 8.0 || gravity.norm() > 11.0) {
    VINS_WARN << "Rejected invalid visual/IMU alignment: scale=" << s
              << ", gravity_norm=" << gravity.norm();
    return false;
  }
  if (lastPublishedAlignmentScale > 0.0) {
    const double epoch_scale_ratio = s / lastPublishedAlignmentScale;
    constexpr double kMinEpochScaleRatio = 0.2;
    constexpr double kMaxEpochScaleRatio = 5.0;
    if (epoch_scale_ratio < kMinEpochScaleRatio ||
        epoch_scale_ratio > kMaxEpochScaleRatio) {
      VINS_WARN << "Rejected visual/IMU alignment scale discontinuity: "
                << "candidate=" << s
                << ", last_published=" << lastPublishedAlignmentScale
                << ", ratio=" << epoch_scale_ratio << ", accepted_range=["
                << kMinEpochScaleRatio << ", " << kMaxEpochScaleRatio << "]";
      return false;
    }
  }
  currentAlignmentScale = s;

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    pre_integrations[i]->repropagate(Vector3d::Zero(),
                                     estimator_state[i].gyro_bias);
  }
  for (int i = frameCount; i >= 0; i--)
    estimator_state[i].position =
        s * estimator_state[i].position -
        estimator_state[i].rotation * options->TIC[0] -
        (s * estimator_state[0].position -
         estimator_state[0].rotation * options->TIC[0]);
  int kv = -1;
  map<double, ImageFrame>::iterator frame_i;
  for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end();
       frame_i++) {
    if (frame_i->second.is_key_frame) {
      kv++;
      estimator_state[kv].velocity =
          frame_i->second.R * x.segment<3>(kv * 3);
    }
  }

  Matrix3d R0 = Utility::g2R(gravity);
  double yaw = Utility::R2ypr(R0 * estimator_state[0].rotation).x();
  R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
  gravity = R0 * gravity;
  Matrix3d rot_diff = R0;
  for (int i = 0; i <= frameCount; i++) {
    estimator_state[i].position = rot_diff * estimator_state[i].position;
    estimator_state[i].rotation = rot_diff * estimator_state[i].rotation;
    estimator_state[i].velocity = rot_diff * estimator_state[i].velocity;
  }
  featureManager.clearDepth();
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);

  return true;
}

bool Estimator::computeRelativePose(Matrix3d &relative_R, Vector3d &relative_T,
                                    int &referenceFrame) {
  constexpr double kSITLFocalLength = 277.191356;
  // Preserve VINS' normalized parallax threshold (30 / 460) while reporting
  // it in pixels for this camera.
  constexpr double kMinParallaxPx = 30.0 * kSITLFocalLength / 460.0;
  size_t max_correspondences = 0;
  double max_parallax_px = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    auto correspondences = featureManager.getCorresponding(i, WINDOW_SIZE);
    max_correspondences = std::max(max_correspondences, correspondences.size());
    if (correspondences.size() > 20) {
      double totalParallax = 0;
      for (const auto &corr : correspondences) {
        Vector2d pts0(corr.first(0), corr.first(1));
        Vector2d pts1(corr.second(0), corr.second(1));
        totalParallax += (pts0 - pts1).norm();
      }

      double avgParallax = totalParallax / correspondences.size();
      max_parallax_px =
          std::max(max_parallax_px, avgParallax * kSITLFocalLength);
      if (avgParallax * kSITLFocalLength > kMinParallaxPx &&
          m_estimator.solveRelativeRT(correspondences, relative_R,
                                      relative_T)) {
        VINS_INFO << "Relative pose accepted: reference=" << i
                  << ", correspondences=" << correspondences.size()
                  << ", parallax=" << avgParallax * kSITLFocalLength
                  << " px, translation=" << relative_T.transpose();
        referenceFrame = i;
        return true;
      }
    }
  }
  VINS_WARN << "Initialization candidates: max correspondences="
            << max_correspondences << ", max parallax=" << max_parallax_px
            << " px (requires >20 correspondences and >"
            << kMinParallaxPx << " px)";
  return false;
}

void Estimator::prepareParameters() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    estimator_state[i].toPoseArray(poseArray[i]);
    estimator_state[i].toSpeedBiasArray(speedBiasArray[i]);
  }

  for (int i = 0; i < options->getNumCameras(); i++) {
    para_Ex_Pose[i][0] = cameraTranslation[i].x();
    para_Ex_Pose[i][1] = cameraTranslation[i].y();
    para_Ex_Pose[i][2] = cameraTranslation[i].z();
    Quaterniond q{cameraRotation[i]};
    para_Ex_Pose[i][3] = q.x();
    para_Ex_Pose[i][4] = q.y();
    para_Ex_Pose[i][5] = q.z();
    para_Ex_Pose[i][6] = q.w();
  }

  VectorXd dep = featureManager.getDepthVector();
  for (int i = 0; i < featureManager.getFeatureCount(); i++)
    para_Feature[i][0] = dep(i);

  para_Td[0][0] = options->time_delay;
}

void Estimator::updateEstimates() {
  Vector3d origin_R0 = Utility::R2ypr(estimator_state[0].rotation);
  Vector3d origin_P0 = estimator_state[0].position;

  OdomData warm_origin;
  const bool warm_origin_available =
      warmReinitializationBridge.active() &&
      warmReinitializationBridge.sample(estimator_state[0].timestamp,
                                        &warm_origin);
  if (warm_origin_available) {
    origin_R0 =
        Utility::R2ypr(warm_origin.orientation.toRotationMatrix());
    origin_P0 = warm_origin.position;
  } else if (failure_occur) {
    origin_R0 = Utility::R2ypr(last_state0.rotation);
    origin_P0 = last_state0.position;
    failure_occur = 0;
  }

  if (ShouldUseAnchoredMetricGauge(
          options->hasImu(), options->hasGpsVelocity(),
          metricDisplacementAnchorInitialized,
          metricPositionFactorActiveThisSolve)) {
    // The GPS-relative position factors remove horizontal translation/yaw
    // gauge freedom in the Ceres parameterization. Reapplying the legacy
    // monocular first-pose gauge transform after the solve would rotate and
    // translate the state away from those factors. Preserve the anchored
    // optimizer gauge only for solves that actually contain a fresh position
    // factor. Once GPS is stale/off the horizontal gauge is unobservable
    // again, so the standard first-pose normalization below must resume.
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      estimator_state[i].rotation =
          Utility::toQuaternion(poseArray[i]).toRotationMatrix();
      estimator_state[i].position =
          Utility::toPositionVector(poseArray[i]);
      estimator_state[i].velocity =
          Utility::toPositionVector(speedBiasArray[i]);
      estimator_state[i].accel_bias =
          Utility::toPositionVector(speedBiasArray[i] + 3);
      estimator_state[i].gyro_bias =
          Utility::toPositionVector(speedBiasArray[i] + 6);
    }
  } else if (options->hasImu()) {
    const auto &pose0Vector = Utility::toPositionVector(poseArray[0]);
    const auto &pose0Matrix =
        Utility::toQuaternion(poseArray[0]).toRotationMatrix();

    Vector3d origin_R00 = Utility::R2ypr(pose0Matrix);
    double y_diff = origin_R0.x() - origin_R00.x();
    // TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 ||
        abs(abs(origin_R00.y()) - 90) < 1.0) {
      rot_diff = estimator_state[0].rotation * pose0Matrix.transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++) {
      estimator_state[i].rotation =
          rot_diff * Utility::toQuaternion(poseArray[i]).toRotationMatrix();

      estimator_state[i].position =
          rot_diff * (Utility::toPositionVector(poseArray[i]) - pose0Vector) +
          origin_P0;
      estimator_state[i].velocity =
          rot_diff * Utility::toPositionVector(speedBiasArray[i]);
      estimator_state[i].accel_bias =
          Utility::toPositionVector(speedBiasArray[i] + 3);
      estimator_state[i].gyro_bias =
          Utility::toPositionVector(speedBiasArray[i] + 6);
    }
  } else {
    for (int i = 0; i <= WINDOW_SIZE; i++) {
      estimator_state[i].rotation =
          Utility::toQuaternion(poseArray[i]).toRotationMatrix();
      estimator_state[i].position = Utility::toPositionVector(poseArray[i]);
    }
  }

  if (options->hasImu()) {
    for (int i = 0; i < options->getNumCameras(); i++) {
      cameraTranslation[i] =
          Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
      cameraRotation[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                                      para_Ex_Pose[i][4], para_Ex_Pose[i][5])
                              .normalized()
                              .toRotationMatrix();
      updateCameraPose(i);
    }
  }

  VectorXd dep = featureManager.getDepthVector();
  for (int i = 0; i < featureManager.getFeatureCount(); i++)
    dep(i) = para_Feature[i][0];
  featureManager.setDepth(dep);

  if (options->hasImu()) options->time_delay = para_Td[0][0];
}

bool Estimator::failureDetection(
    bool trusted_horizontal_metric_correction) {
  if (!validateEstimatorState("failure detection")) return true;
  // A single low-feature frame can occur during a yaw turn.  State finiteness,
  // bias, jump and rotation checks below are the actual reset criteria.
  if (estimator_state[WINDOW_SIZE].accel_bias.norm() > kMaxAccelBiasNorm) {
    VINS_ERROR << "Accelerometer bias limit exceeded: "
               << estimator_state[WINDOW_SIZE].accel_bias.norm();
    return true;
  }
  if (estimator_state[WINDOW_SIZE].gyro_bias.norm() > kMaxGyroBiasNorm) {
    VINS_ERROR << "Gyroscope bias limit exceeded: "
               << estimator_state[WINDOW_SIZE].gyro_bias.norm();
    return true;
  }
  Vector3d tmp_P = estimator_state[WINDOW_SIZE].position;
  const double state_dt =
      estimator_state[WINDOW_SIZE].timestamp - last_state.timestamp;
  Vector3d predicted_position = last_state.position;
  if (state_dt > 0.0 && state_dt <= 0.5) {
    predicted_position += state_dt * last_state.velocity;
  }
  const Vector3d position_innovation = tmp_P - predicted_position;
  constexpr double kMaxPositionInnovation = 0.10;
  constexpr double kMaxSafeOutputStep = 0.15;
  const double actual_output_step =
      (tmp_P - last_state.position).norm();
  const bool bounded_output_state =
      actual_output_step <= kMaxSafeOutputStep;
  if (position_innovation.norm() > kMaxPositionInnovation &&
      !trusted_horizontal_metric_correction &&
      !bounded_output_state) {
    VINS_ERROR << "Position innovation detected: "
               << position_innovation.norm()
               << ", timestamp=" << estimator_state[WINDOW_SIZE].timestamp
               << ", dt=" << state_dt
               << ", previous_position=" << last_state.position.transpose()
               << ", predicted_position=" << predicted_position.transpose()
               << ", current_position=" << tmp_P.transpose()
               << ", previous_velocity=" << last_state.velocity.transpose()
               << ", current_velocity="
               << estimator_state[WINDOW_SIZE].velocity.transpose()
               << ", accel_bias="
               << estimator_state[WINDOW_SIZE].accel_bias.transpose()
               << ", gyro_bias="
               << estimator_state[WINDOW_SIZE].gyro_bias.transpose()
               << ", features=" << featureManager.getFeatureCount()
               << ", solver_initial_cost=" << lastOptimizationInitialCost
               << ", solver_final_cost=" << lastOptimizationFinalCost
               << ", solver_time_s=" << lastOptimizationTimeSeconds
               << ", solver_successful_steps="
               << lastOptimizationSuccessfulSteps
               << ", solver_unsuccessful_steps="
               << lastOptimizationUnsuccessfulSteps
               << ", solver_termination="
               << ceres::TerminationTypeToString(lastOptimizationTermination);
    return true;
  }
  if (position_innovation.norm() > kMaxPositionInnovation) {
    VINS_WARN << "Bounded metric/rollback state bypassed duplicate "
                 "prediction-only failure detection: innovation="
              << position_innovation.norm()
              << ", output_step=" << actual_output_step
              << ", rejected_transaction="
              << lastBackendTransactionRejected
              << ", horizontal_innovation="
              << position_innovation.head<2>().norm()
              << ", vertical_innovation="
              << std::abs(position_innovation.z());
  }
  if (abs(position_innovation.z()) > kMaxPositionInnovation &&
      !trusted_horizontal_metric_correction &&
      !bounded_output_state) {
    VINS_ERROR << "Vertical position innovation detected: "
               << abs(position_innovation.z());
    return true;
  }
  Matrix3d tmp_R = estimator_state[WINDOW_SIZE].rotation;
  Matrix3d delta_R = tmp_R.transpose() * last_state.rotation;
  Quaterniond delta_Q(delta_R);
  double delta_angle =
      acos(std::max(-1.0, std::min(1.0, std::abs(delta_Q.w())))) * 2.0 /
      M_PI * 180.0;
  if (delta_angle > 50) {
    VINS_ERROR << "Attitude jump detected: " << delta_angle << " deg";
    return true;
  }
  return false;
}

bool Estimator::validateInitializationHealth() const {
  const StateData &candidate = estimator_state[WINDOW_SIZE];
  const double accel_bias_norm = candidate.accel_bias.norm();
  const double gyro_bias_norm = candidate.gyro_bias.norm();
  if (accel_bias_norm > kMaxAccelBiasNorm) {
    VINS_WARN << "Rejected unhealthy initialization candidate: reason="
                 "accel_bias, accel_bias_norm="
              << accel_bias_norm << ", limit=" << kMaxAccelBiasNorm
              << ", timestamp=" << candidate.timestamp;
    return false;
  }
  if (gyro_bias_norm > kMaxGyroBiasNorm) {
    VINS_WARN << "Rejected unhealthy initialization candidate: reason="
                 "gyro_bias, gyro_bias_norm="
              << gyro_bias_norm << ", limit=" << kMaxGyroBiasNorm
              << ", timestamp=" << candidate.timestamp;
    return false;
  }
  return true;
}

bool Estimator::validateEstimatorState(const char *context) const {
  // The endurance world is a 500 x 500 m ground.  The former 100 m norm
  // bound deterministically reset an otherwise healthy estimator as soon as
  // the first open-route leg crossed E=100 m.  Keep an absolute corruption
  // bound beyond every valid point on this ground; incremental mutation and
  // publication continuity remain guarded by the stricter solve/output gates.
  constexpr double kMaxLocalPositionNormM = 500.0;
  if (!gravity.allFinite() || gravity.norm() < 8.0 || gravity.norm() > 11.0) {
    VINS_ERROR << context << ": invalid gravity " << gravity.transpose();
    return false;
  }
  for (int i = 0; i <= frameCount; ++i) {
    const StateData &s = estimator_state[i];
    const Quaterniond q(s.rotation);
    if (!s.position.allFinite() || !s.velocity.allFinite() ||
        !s.accel_bias.allFinite() || !s.gyro_bias.allFinite() ||
        !s.rotation.allFinite() || !std::isfinite(q.norm()) ||
        std::abs(q.norm() - 1.0) > 1e-3 ||
        s.position.norm() > kMaxLocalPositionNormM ||
        s.velocity.norm() > 20.0) {
      VINS_ERROR << context << ": invalid state at window index " << i
                 << ", p=" << s.position.transpose()
                 << ", v=" << s.velocity.transpose()
                 << ", ba=" << s.accel_bias.transpose()
                 << ", bg=" << s.gyro_bias.transpose()
                 << ", qnorm=" << q.norm();
      return false;
    }
  }
  return true;
}

void Estimator::AddPoseParameterBlocks(ceres::Problem &problem) {
  for (int i = 0; i < frameCount + 1; i++) {
    auto *local_param = new PoseLocalParameterization();
    problem.AddParameterBlock(poseArray[i], SIZE_POSE, local_param);
    if ((motionOnsetWarmReinitializationAppliedThisSolve ||
         motionOnsetBiasAnchorThisSolve) &&
        i == WINDOW_SIZE) {
      // The discarded marginalization prior used to carry the window gauge.
      // Anchor the currently published metric pose for this one replacement
      // solve. This is not a stale prior: no residual or covariance from the
      // old window survives.
      problem.SetParameterBlockConstant(poseArray[i]);
    }
    if (options->hasImu()) {
      std::vector<int> fixed_speed_bias_coordinates;
      if (replayGtAccelBiasZeroOracle) {
        speedBiasArray[i][3] = 0.0;
        speedBiasArray[i][4] = 0.0;
        speedBiasArray[i][5] = 0.0;
        fixed_speed_bias_coordinates.insert(
            fixed_speed_bias_coordinates.end(), {3, 4, 5});
      }
      if (replayGtInitialVelocityZeroOracle && !isNonLinearSolver()) {
        speedBiasArray[i][0] = 0.0;
        speedBiasArray[i][1] = 0.0;
        speedBiasArray[i][2] = 0.0;
        fixed_speed_bias_coordinates.insert(
            fixed_speed_bias_coordinates.end(), {0, 1, 2});
      }
      if (motionOnsetReleaseThisSolve && i == 0) {
        fixed_speed_bias_coordinates.insert(
            fixed_speed_bias_coordinates.end(), {0, 1, 2});
      }
      problem.AddParameterBlock(speedBiasArray[i], SIZE_SPEEDBIAS);
      if ((motionOnsetWarmReinitializationAppliedThisSolve ||
           (motionOnsetBiasAnchorThisSolve &&
            !motionOnsetReleaseThisSolve)) &&
          i == 0) {
        // The replacement solve retains the established oldest velocity/bias
        // gauge. Warm-transition solves remain prior-free and use the same
        // gauge until horizontal motion makes a fresh prior observable.
        problem.SetParameterBlockConstant(speedBiasArray[i]);
      }
      if (!fixed_speed_bias_coordinates.empty()) {
        problem.SetParameterization(
            speedBiasArray[i],
            new ceres::SubsetParameterization(
                SIZE_SPEEDBIAS, fixed_speed_bias_coordinates));
      }
    }
  }
  if (!options->hasImu()) {
    problem.SetParameterBlockConstant(poseArray[0]);
  }
}
void Estimator::AddExtrinsicParameterBlocks(ceres::Problem &problem) {
  for (int i = 0; i < options->getNumCameras(); i++) {
    auto *local_param = new PoseLocalParameterization();
    problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_param);
    if (((options->isExtrinsicEstimationApproximate() ||
          options->isInitializingExtrinsic()) &&
         frameCount == WINDOW_SIZE &&
         estimator_state[0].velocity.norm() > 0.2) ||
        openExEstimation) {
      openExEstimation = 1;
    } else {
      problem.SetParameterBlockConstant(para_Ex_Pose[i]);
    }
  }
}
void Estimator::AddTimeDelayParameterBlock(ceres::Problem &problem) {
  problem.AddParameterBlock(para_Td[0], 1);
  if (!options->shouldEstimateTD() || estimator_state[0].velocity.norm() < 0.2)
    problem.SetParameterBlockConstant(para_Td[0]);
}
void Estimator::AddMarginalizationFactor(ceres::Problem &problem) {
  constexpr double kReplayTimestampMatchToleranceS = 0.01;
  if (!replaySkipMarginalizationPriorConsumed &&
      replaySkipMarginalizationPriorTimestamp >= 0.0 &&
      std::abs(estimator_state[WINDOW_SIZE].timestamp -
               replaySkipMarginalizationPriorTimestamp) <=
          kReplayTimestampMatchToleranceS) {
    replaySkipMarginalizationPriorConsumed = true;
    VINS_WARN << "Replay-only marginalization prior skipped for one solve: "
              << "timestamp=" << estimator_state[WINDOW_SIZE].timestamp;
    return;
  }
  if (last_marginalization_info && last_marginalization_info->valid) {
    auto *marginalization_factor =
        new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(marginalization_factor, nullptr,
                             last_marginalization_parameter_blocks);
  }
}
void Estimator::AddIMUFactors(ceres::Problem &problem) {
  if (!options->hasImu()) return;

  for (int i = 0; i < frameCount; i++) {
    int j = i + 1;
    if (pre_integrations[j]->sum_dt > 10.0 &&
        !replayIncludeLongImuFactors) {
      continue;
    }
    auto *imu_factor = new IMUFactor(pre_integrations[j]);
    problem.AddResidualBlock(imu_factor, nullptr, poseArray[i],
                             speedBiasArray[i], poseArray[j],
                             speedBiasArray[j]);
  }
}
void Estimator::AddFeatureFactors(ceres::Problem &problem) {
  if (options->replacesFeatureVisual() && !options->isUsingStereo()) {
    AddPoseOnlyVisualFactors(problem);
    return;
  }
  std::set<int> solve_excluded_history_ids;
  const char *solve_exclusion_value =
      std::getenv("VINS_REPLAY_EXCLUDE_INCONSISTENT_HISTORY_FROM_SOLVE");
  if (solve_exclusion_value && std::string(solve_exclusion_value) == "1") {
    solve_excluded_history_ids = featureManager.inconsistentLongTrackIds(
        frameCount, estimator_state, cameraTranslation, cameraRotation, 8,
        0.005);
  }
  int feature_index = -1;
  for (auto &it : featureManager.feature) {
    if (it.feature_per_frame.size() < kMinFeatureTrackFrames) continue;
    ++feature_index;
    if (solve_excluded_history_ids.count(it.feature_id)) continue;

    int imu_i = it.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it.feature_per_frame[0].point;

    for (auto &f : it.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = f.point;
        auto *factor = new ProjectionTwoFrameOneCamFactor(
            pts_i, pts_j, it.feature_per_frame[0].velocity, f.velocity,
            it.feature_per_frame[0].cur_td, f.cur_td);
        problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                 poseArray[imu_i], poseArray[imu_j],
                                 para_Ex_Pose[0], para_Feature[feature_index],
                                 para_Td[0]);
      }

      if (options->isUsingStereo() && f.is_stereo) {
        Vector3d pts_j_right = f.pointRight;
        if (imu_i != imu_j) {
          auto *factor = new ProjectionTwoFrameTwoCamFactor(
              pts_i, pts_j_right, it.feature_per_frame[0].velocity,
              f.velocityRight, it.feature_per_frame[0].cur_td, f.cur_td);
          problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                   poseArray[imu_i], poseArray[imu_j],
                                   para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index], para_Td[0]);
        } else {
          auto *factor = new ProjectionOneFrameTwoCamFactor(
              pts_i, pts_j_right, it.feature_per_frame[0].velocity,
              f.velocityRight, it.feature_per_frame[0].cur_td, f.cur_td);
          problem.AddResidualBlock(factor, new ceres::HuberLoss(1.0),
                                   para_Ex_Pose[0], para_Ex_Pose[1],
                                   para_Feature[feature_index], para_Td[0]);
        }
      }
    }
  }
  if (options->augmentsFeatureVisual() && !options->isUsingStereo()) {
    AddPoseOnlyVisualFactors(problem);
  }
}

void Estimator::AddPoseOnlyVisualFactors(ceres::Problem &problem) {
  const Eigen::Quaterniond q_imu_camera(cameraRotation[0]);
  const Eigen::Vector3d t_imu_camera = cameraTranslation[0];
  const double sqrt_information =
      (FOCAL_LENGTH / 1.5) * options->pose_only_visual_weight_scale;
  int track_count = 0;
  int residual_block_count = 0;
  for (const auto &track : featureManager.feature) {
    const int observation_count =
        static_cast<int>(track.feature_per_frame.size());
    if (observation_count < kMinFeatureTrackFrames) continue;
    const int slot_j = track.start_frame;
    const int slot_k = slot_j + observation_count - 1;
    if (slot_j < 0 || slot_k > frameCount || slot_j >= slot_k) continue;
    const Eigen::Vector3d &bearing_j =
        track.feature_per_frame.front().point;
    const Eigen::Vector3d &bearing_k =
        track.feature_per_frame.back().point;
    problem.AddResidualBlock(
        pose_only_visual::EpipolarFunctor::Create(
            bearing_j, bearing_k, q_imu_camera, t_imu_camera,
            sqrt_information),
        new ceres::HuberLoss(1.0),
        poseArray[slot_j], poseArray[slot_k]);
    ++residual_block_count;
    for (int observation_index = 1;
         observation_index < observation_count - 1;
         ++observation_index) {
      const int slot_i = slot_j + observation_index;
      problem.AddResidualBlock(
          pose_only_visual::ThreeViewFunctor::Create(
              bearing_j, bearing_k,
              track.feature_per_frame[observation_index].point,
              q_imu_camera, t_imu_camera, sqrt_information),
          new ceres::HuberLoss(1.0),
          poseArray[slot_j], poseArray[slot_k], poseArray[slot_i]);
      ++residual_block_count;
    }
    ++track_count;
  }
  if (residual_block_count > 0 &&
      !poseOnlyVisualFactorActivationLogged) {
    VINS_INFO << "Pose-only visual factors active: tracks="
              << track_count << ", residual_blocks="
              << residual_block_count
              << ", sqrt_information=" << sqrt_information;
    poseOnlyVisualFactorActivationLogged = true;
  }
}
void Estimator::AddRangeFactors(ceres::Problem &problem) {
  // Visual/IMU alignment must establish gravity, velocity and scale without
  // auxiliary residuals.  Applying range residuals in that first Ceres solve
  // couples an unobservable SfM pose to the ground plane and can drive the
  // accelerometer bias away before the state is fully initialized.
  if (!options->hasRange() || solver_flag != SolverState::NON_LINEAR ||
      nonLinearUpdateCount < WINDOW_SIZE ||
      frameCount < 1) return;
  int reference = -1;
  for (int i = 0; i <= frameCount; ++i) {
    if (rangeMeasurements[i].valid) {
      reference = i;
      break;
    }
  }
  if (reference < 0) return;

  if (!rangeGroundInitialized) {
    const double *pose = poseArray[reference];
    const Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    const Vector3d down(0.0, 0.0, -1.0);
    rangeGroundZ =
        pose[2] + (q * options->range_translation).z() +
        rangeMeasurements[reference].range * (q * down).z();
    if (!std::isfinite(rangeGroundZ)) return;
    rangeGroundInitialized = true;
    VINS_INFO << "Downward range ground initialized: z=" << rangeGroundZ
              << ", range=" << rangeMeasurements[reference].range;
  }

  const double sigma = std::max(0.005, options->range_noise);
  int factor_count = 0;
  for (int j = 0; j <= frameCount; ++j) {
    if (!rangeMeasurements[j].valid) continue;
    const double *pose = poseArray[j];
    const Quaterniond q(pose[6], pose[3], pose[4], pose[5]);
    const Vector3d down(0.0, 0.0, -1.0);
    const double vertical_offset =
        (q * options->range_translation).z() +
        rangeMeasurements[j].range * (q * down).z();
    const double target_z = rangeGroundZ - vertical_offset;
    if (!std::isfinite(target_z)) continue;
    problem.AddResidualBlock(
        RangeFactor::Create(target_z, sigma),
        // The validity gate excludes pad/self returns and the endurance world
        // has a fixed ground plane. Keep the only persistent vertical metric
        // reference quadratic so visual scale pressure cannot down-weight it
        // during a long GPS outage.
        nullptr, poseArray[j]);
    ++factor_count;
  }
  if (factor_count > 0 && !rangeFactorActivationLogged) {
    VINS_INFO << "Downward range factors active: " << factor_count
              << ", sigma=" << sigma
              << ", ground_z=" << rangeGroundZ;
    rangeFactorActivationLogged = true;
  }
}

void Estimator::AddMetricVelocityFactors(ceres::Problem &problem) {
  if (!options->hasGpsVelocity() ||
      solver_flag != SolverState::NON_LINEAR ||
      nonLinearUpdateCount < WINDOW_SIZE || frameCount < 1) {
    return;
  }
  int factor_count = 0;
  uint64_t newest_segment = 0;
  for (int j = 0; j <= frameCount; ++j) {
    if (metricVelocityMeasurements[j].valid) {
      newest_segment = std::max(
          newest_segment,
          metricVelocityMeasurements[j].integration_segment);
    }
  }
  if (newest_segment == 0) return;
  if (!metricDisplacementAnchorInitialized ||
      metricDisplacementAnchorSegment != newest_segment) {
    for (int j = 0; j <= frameCount; ++j) {
      const auto &measurement = metricVelocityMeasurements[j];
      if (!measurement.valid ||
          measurement.integration_segment != newest_segment) {
        continue;
      }
      metricDisplacementAnchorInitialized = true;
      metricDisplacementAnchorSegment = newest_segment;
      const double anchor_delta_time =
          estimator_state[j].timestamp - measurement.timestamp;
      metricDisplacementAnchorMeasurement =
          measurement.horizontal_position +
          measurement.horizontal_velocity * anchor_delta_time;
      metricDisplacementAnchorPosition =
          Eigen::Vector2d(poseArray[j][0], poseArray[j][1]);
      VINS_INFO << "GPS relative-position anchor active: segment="
                << newest_segment << ", position="
                << metricDisplacementAnchorPosition.transpose()
                << ", gps_relative_position="
                << metricDisplacementAnchorMeasurement.transpose();
      break;
    }
  }
  for (int j = 0; j <= frameCount; ++j) {
    const auto &measurement = metricVelocityMeasurements[j];
    if (!measurement.valid ||
        measurement.integration_segment !=
            metricDisplacementAnchorSegment) {
      continue;
    }
    // SensorGps::s_variance_m_s is a fixed conservative 0.25 m/s in the
    // PX4 SITL model, while recorded east/north velocity has 0.025 m/s vector
    // RMSE against simulator truth. Use the explicitly configured observation
    // noise; the reported value remains part of the validity contract.
    const double sigma = options->gps_velocity_noise;
    if (!std::isfinite(sigma) || sigma <= 0.0) continue;
    problem.AddResidualBlock(
        HorizontalVelocityFactor::Create(
            measurement.horizontal_velocity.x(),
            measurement.horizontal_velocity.y(), sigma),
        new ceres::HuberLoss(2.0), speedBiasArray[j]);
    const double measurement_delta_time =
        estimator_state[j].timestamp - measurement.timestamp;
    if (!std::isfinite(measurement_delta_time) ||
        std::abs(measurement_delta_time) >
            options->gps_velocity_max_age) {
      continue;
    }
    // SensorGps arrives at about 2.5 Hz while image states are optimized at
    // 10 Hz. Extrapolate its relative EN position to each image timestamp so
    // a repeated nearest sample cannot create a staircase target.
    const Eigen::Vector2d frame_position =
        measurement.horizontal_position +
        measurement.horizontal_velocity * measurement_delta_time;
    const Eigen::Vector2d target_position =
        metricDisplacementAnchorPosition +
        frame_position -
        metricDisplacementAnchorMeasurement;
    problem.AddResidualBlock(
        HorizontalDisplacementFactor::Create(
            target_position.x(), target_position.y(),
            options->gps_position_noise),
        nullptr, poseArray[j]);
    metricPositionFactorActiveThisSolve = true;
    ++factor_count;
  }
  if (factor_count > 0 && !metricVelocityFactorActivationLogged) {
    VINS_INFO << "GPS horizontal velocity factors active: "
              << factor_count
              << ", configured_sigma=" << options->gps_velocity_noise;
    metricVelocityFactorActivationLogged = true;
  }
}

void Estimator::AddGpsDropoutSpeedFactors(ceres::Problem &problem) {
  if (!options->hasGpsVelocity() ||
      !metricDisplacementAnchorInitialized ||
      metricPositionFactorActiveThisSolve ||
      solver_flag != SolverState::NON_LINEAR ||
      nonLinearUpdateCount < WINDOW_SIZE || frameCount < 1) {
    return;
  }
  const double maximum_speed = options->gps_dropout_speed_max;
  const double maximum_vertical_speed =
      options->gps_dropout_vertical_speed_max;
  const double sigma = options->gps_dropout_speed_noise;
  if (!std::isfinite(maximum_speed) || maximum_speed <= 0.0 ||
      !std::isfinite(maximum_vertical_speed) ||
      maximum_vertical_speed <= 0.0 ||
      !std::isfinite(sigma) || sigma <= 0.0) {
    return;
  }
  for (int j = 0; j <= frameCount; ++j) {
    problem.AddResidualBlock(
        HorizontalSpeedUpperBoundFactor::Create(maximum_speed, sigma),
        nullptr, speedBiasArray[j]);
    problem.AddResidualBlock(
        VerticalSpeedUpperBoundFactor::Create(
            maximum_vertical_speed, sigma),
        nullptr, speedBiasArray[j]);
  }
  if (!gpsDropoutSpeedFactorActivationLogged) {
    VINS_INFO << "GPS-off axis speed upper-bound factors active: "
              << frameCount + 1 << ", maximum_speed=" << maximum_speed
              << ", maximum_vertical_speed="
              << maximum_vertical_speed
              << ", sigma=" << sigma;
    gpsDropoutSpeedFactorActivationLogged = true;
  }
}

void Estimator::AddGpsDropoutGyroBiasPriorFactors(
    ceres::Problem &problem) {
  if (!options->hasGpsDropoutGyroBiasPrior() ||
      !gpsDropoutGyroBiasAnchorInitialized ||
      metricPositionFactorActiveThisSolve ||
      solver_flag != SolverState::NON_LINEAR ||
      nonLinearUpdateCount < WINDOW_SIZE || frameCount < 1) {
    return;
  }
  const double sigma = options->gps_dropout_gyro_bias_sigma;
  if (!std::isfinite(gpsDropoutGyroBiasAnchorZ) ||
      !std::isfinite(sigma) || sigma <= 0.0) {
    return;
  }
  for (int j = 0; j <= frameCount; ++j) {
    problem.AddResidualBlock(
        new GyroBiasZPriorFactor(
            gpsDropoutGyroBiasAnchorZ, sigma),
        nullptr, speedBiasArray[j]);
  }
  if (!gpsDropoutGyroBiasPriorActivationLogged) {
    VINS_INFO << "GPS-dropout gyro z-bias prior active: "
              << frameCount + 1
              << ", target_rad_s=" << gpsDropoutGyroBiasAnchorZ
              << ", sigma_rad_s=" << sigma;
    gpsDropoutGyroBiasPriorActivationLogged = true;
  }
}

void Estimator::AddRotorDragDynamicsFactors(ceres::Problem &problem) {
  const std::uint64_t factor_solve_sequence = ++motorFactorSolveSequence;
  const auto record_factor_count = [this](int factor_count) {
    if (!replayRotorFactorLedger.is_open()) return;
    std::lock_guard<std::mutex> lock(replayRotorFactorLedgerMutex);
    replayRotorFactorLedger
        << ++replayRotorFactorSolveSequence << ','
        << static_cast<std::int64_t>(
               std::llround(currentTimestamp * 1000000000.0))
        << ',' << frameCount << ',' << factor_count << '\n';
    replayRotorFactorLedger.flush();
  };
  if (!options->hasRotorDragDynamics() ||
      solver_flag != SolverState::NON_LINEAR ||
      nonLinearUpdateCount < WINDOW_SIZE || frameCount < 1) {
    record_factor_count(0);
    return;
  }
  const double mass = options->dynamics_mass_kg;
  const double drag = options->dynamics_rotor_drag_coefficient;
  const double sigma = options->dynamics_accel_noise;
  if (!std::isfinite(mass) || mass <= 0.0 ||
      !std::isfinite(drag) || drag <= 0.0 ||
      !std::isfinite(sigma) || sigma <= 0.0) {
    record_factor_count(0);
    return;
  }

  int factor_count = 0;
  for (int i = 0; i < frameCount; ++i) {
    const int j = i + 1;
    const auto &dynamics_i = rotorDynamicsMeasurements[i];
    const auto &dynamics_j = rotorDynamicsMeasurements[j];
    if (!dynamics_i.valid || !dynamics_j.valid) continue;
    const auto quality_state = [](const std::string &value) {
      return value == "NORMAL"
          ? vins_fusion_ros2::MotorQualityState::kNormal
          : (value == "DEGRADED"
                 ? vins_fusion_ros2::MotorQualityState::kDegraded
                 : vins_fusion_ros2::MotorQualityState::kRejected);
    };
    vins_fusion_ros2::MotorFactorEndpoint endpoint_i;
    endpoint_i.sample_timestamp_ns = static_cast<int64_t>(
        std::llround(dynamics_i.timestamp * 1000000000.0));
    endpoint_i.source_sequence = dynamics_i.source_sequence;
    endpoint_i.source_hash = dynamics_i.source_hash;
    endpoint_i.quality_state = quality_state(dynamics_i.quality_state);
    endpoint_i.factor_weight_scale = dynamics_i.factor_weight_scale;
    vins_fusion_ros2::MotorFactorEndpoint endpoint_j;
    endpoint_j.sample_timestamp_ns = static_cast<int64_t>(
        std::llround(dynamics_j.timestamp * 1000000000.0));
    endpoint_j.source_sequence = dynamics_j.source_sequence;
    endpoint_j.source_hash = dynamics_j.source_hash;
    endpoint_j.quality_state = quality_state(dynamics_j.quality_state);
    endpoint_j.factor_weight_scale = dynamics_j.factor_weight_scale;
    const auto pair = vins_fusion_ros2::EvaluateMotorFactorPair(
        sigma, endpoint_i, endpoint_j);
    if (!pair.admitted) continue;
    const double dt =
        estimator_state[j].timestamp - estimator_state[i].timestamp;
    if (!std::isfinite(dt) || dt < 0.02 || dt > 0.25) continue;
    problem.AddResidualBlock(
        RotorDragDynamicsFactor::Create(
            dt, dynamics_i.total_thrust_newton,
            dynamics_i.summed_rotor_speed_rad_s,
            dynamics_j.total_thrust_newton,
            dynamics_j.summed_rotor_speed_rad_s, mass, drag,
            pair.sigma_effective),
        new ceres::HuberLoss(2.0), poseArray[i], speedBiasArray[i],
        poseArray[j], speedBiasArray[j]);
    ++factor_count;
    if (motorFactorTimeline.is_open()) {
      std::lock_guard<std::mutex> lock(motorFactorTimelineMutex);
      motorFactorTimeline << vins_fusion_ros2::MotorFactorTimelineJson(
          factor_solve_sequence, i, j, endpoint_i, endpoint_j, pair) << '\n';
      motorFactorTimeline.flush();
    }
  }
  record_factor_count(factor_count);
  if (factor_count > 0 && !rotorDragDynamicsFactorActivationLogged) {
    VINS_INFO << "Rotor-drag horizontal dynamics factors active: "
              << factor_count << ", mass_kg=" << mass
              << ", drag_coefficient=" << drag
              << ", accel_sigma=" << sigma;
    rotorDragDynamicsFactorActivationLogged = true;
  }
}

bool Estimator::solveOptimization(int max_iterations_override) {
  ceres::Problem problem;
  metricPositionFactorActiveThisSolve = false;
  bool gt_intervention_this_solve = false;
  std::string gt_intervention_directory;
  bool gt_intervention_force_commit = false;
  bool gt_intervention_prior_recentered = false;
  double gt_intervention_prior_residual_norm_before =
      std::numeric_limits<double>::quiet_NaN();
  std::vector<std::array<double, SIZE_POSE>> gt_commit_pose;
  std::vector<std::array<double, SIZE_SPEEDBIAS>> gt_commit_speed_bias;
  std::vector<double> gt_commit_inverse_depth;

  const auto residualBlocks = [&problem]() {
    std::vector<ceres::ResidualBlockId> blocks;
    problem.GetResidualBlocks(&blocks);
    return blocks;
  };
  const auto appendedResidualBlocks =
      [&residualBlocks](std::size_t previous_size) {
        auto blocks = residualBlocks();
        if (blocks.size() < previous_size) {
          throw std::runtime_error("Ceres residual block list shrank");
        }
        return std::vector<ceres::ResidualBlockId>(
            blocks.begin() + static_cast<std::ptrdiff_t>(previous_size),
            blocks.end());
      };

  AddPoseParameterBlocks(problem);
  AddExtrinsicParameterBlocks(problem);
  AddTimeDelayParameterBlock(problem);
  std::size_t residual_count_before = residualBlocks().size();
  AddMarginalizationFactor(problem);
  const auto prior_residual_blocks =
      appendedResidualBlocks(residual_count_before);
  residual_count_before = residualBlocks().size();
  AddIMUFactors(problem);
  const auto imu_residual_blocks =
      appendedResidualBlocks(residual_count_before);
  residual_count_before = residualBlocks().size();
  AddFeatureFactors(problem);
  const auto visual_residual_blocks =
      appendedResidualBlocks(residual_count_before);
  AddRangeFactors(problem);
  AddMetricVelocityFactors(problem);
  AddGpsDropoutSpeedFactors(problem);
  AddGpsDropoutGyroBiasPriorFactors(problem);
  AddRotorDragDynamicsFactors(problem);

  // Replay-only causal intervention. Parameter values are changed at the
  // exact solve entry, so Ceres evaluates the real prior, IMU and visual cost
  // functions and their Jacobians at the hybrid state. The resulting optimum
  // then follows the normal marginalization/commit path and subsequent input
  // replay. No matrix or already-linearized source term is substituted.
  const double solve_timestamp = estimator_state[WINDOW_SIZE].timestamp;
  const auto &gt_config = replayGtInterventionConfig();
  if (std::isfinite(gt_config.target) && !replayGtInterventionApplied &&
      std::abs(solve_timestamp - gt_config.target) <= 0.005) {
    const char *directory_value =
        std::getenv("VINS_REPLAY_GT_INTERVENTION_DIRECTORY");
    if (!directory_value || !*directory_value) {
      throw std::runtime_error(
          "replay GT intervention target set without output directory");
    }
    gt_intervention_directory = directory_value;
    if (::mkdir(gt_intervention_directory.c_str(), 0775) != 0 &&
        errno != EEXIST) {
      throw std::runtime_error("cannot create GT intervention directory");
    }
    const std::deque<OdomData> ground_truth_history =
        replayGtInterventionGroundTruth();
    if (ground_truth_history.size() < 2) {
      throw std::runtime_error("missing replay ground truth for intervention");
    }
    const auto interpolate_ground_truth = [&](double timestamp) {
      auto upper = std::lower_bound(
          ground_truth_history.begin(), ground_truth_history.end(), timestamp,
          [](const OdomData &sample, double value) {
            return sample.timestamp < value;
          });
      if (upper == ground_truth_history.begin() ||
          upper == ground_truth_history.end()) {
        throw std::runtime_error(
            "ground truth does not bracket intervention graph state");
      }
      const auto lower = std::prev(upper);
      const double duration = upper->timestamp - lower->timestamp;
      if (!std::isfinite(duration) || duration <= 0.0 || duration > 0.1) {
        throw std::runtime_error(
            "invalid intervention ground-truth interpolation interval");
      }
      const double ratio = (timestamp - lower->timestamp) / duration;
      OdomData result;
      result.timestamp = timestamp;
      result.position =
          (1.0 - ratio) * lower->position + ratio * upper->position;
      result.orientation =
          lower->orientation.slerp(ratio, upper->orientation).normalized();
      return result;
    };
    std::vector<OdomData> aligned_ground_truth;
    aligned_ground_truth.reserve(frameCount + 1);
    for (int i = 0; i <= frameCount; ++i) {
      aligned_ground_truth.push_back(
          interpolate_ground_truth(estimator_state[i].timestamp));
    }
    const auto yaw = [](const Eigen::Matrix3d &rotation) {
      return std::atan2(rotation(1, 0), rotation(0, 0));
    };
    const double yaw_offset =
        yaw(estimator_state[0].rotation) -
        yaw(aligned_ground_truth[0].orientation.toRotationMatrix());
    const Eigen::Matrix3d yaw_rotation =
        Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3d translation =
        estimator_state[0].position -
        yaw_rotation * aligned_ground_truth[0].position;
    for (auto &state : aligned_ground_truth) {
      state.position = yaw_rotation * state.position + translation;
      state.orientation = Eigen::Quaterniond(
          yaw_rotation * state.orientation.toRotationMatrix());
    }
    for (int i = 0; i <= frameCount; ++i) {
      const int before = std::max(0, i - 1);
      const int after = std::min(frameCount, i + 1);
      const double dt = aligned_ground_truth[after].timestamp -
                        aligned_ground_truth[before].timestamp;
      if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::runtime_error(
            "invalid intervention GT velocity differentiation interval");
      }
      aligned_ground_truth[i].velocity =
          (aligned_ground_truth[after].position -
           aligned_ground_truth[before].position) /
          dt;
    }
    const bool full = gt_config.families.count("full") != 0;
    const auto selected = [&](const char *family) {
      return full || gt_config.families.count(family) != 0;
    };
    const auto write_state = [&](const std::string &phase) {
      std::ofstream state_file(gt_intervention_directory + "/" + phase +
                               "_state.csv");
      if (!state_file) {
        throw std::runtime_error("cannot write GT intervention state audit");
      }
      state_file << "slot,timestamp_s,px,py,pz,qx,qy,qz,qw,vx,vy,vz,"
                    "bax,bay,baz,bgx,bgy,bgz\n";
      state_file << std::setprecision(17);
      for (int i = 0; i <= frameCount; ++i) {
        state_file << i << ',' << estimator_state[i].timestamp;
        for (int k = 0; k < SIZE_POSE; ++k)
          state_file << ',' << poseArray[i][k];
        for (int k = 0; k < SIZE_SPEEDBIAS; ++k)
          state_file << ',' << speedBiasArray[i][k];
        state_file << '\n';
      }
      std::ofstream depth_file(gt_intervention_directory + "/" + phase +
                               "_landmark_depth.csv");
      if (!depth_file) {
        throw std::runtime_error("cannot write GT intervention depth audit");
      }
      depth_file << "feature_id,inverse_depth\n" << std::setprecision(17);
      int feature_index = -1;
      for (const auto &feature : featureManager.feature) {
        if (feature.feature_per_frame.size() < kMinFeatureTrackFrames) continue;
        ++feature_index;
        depth_file << feature.feature_id << ','
                   << para_Feature[feature_index][0] << '\n';
      }
    };
    write_state("production_entry");
    for (int i = 0; i <= frameCount; ++i) {
      const auto &gt = aligned_ground_truth[i];
      if (selected("position")) {
        for (int axis = 0; axis < 3; ++axis)
          poseArray[i][axis] = gt.position[axis];
      }
      if (selected("orientation")) {
        poseArray[i][3] = gt.orientation.x();
        poseArray[i][4] = gt.orientation.y();
        poseArray[i][5] = gt.orientation.z();
        poseArray[i][6] = gt.orientation.w();
      } else if (selected("roll_pitch")) {
        const Eigen::Quaterniond current(
            poseArray[i][6], poseArray[i][3], poseArray[i][4],
            poseArray[i][5]);
        const Eigen::Matrix3d gt_rotation =
            gt.orientation.toRotationMatrix();
        const double roll = std::atan2(gt_rotation(2, 1), gt_rotation(2, 2));
        const double pitch = std::asin(std::max(
            -1.0, std::min(1.0, -gt_rotation(2, 0))));
        const Eigen::Quaterniond hybrid(
            Eigen::AngleAxisd(yaw(current.toRotationMatrix()),
                              Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        poseArray[i][3] = hybrid.x();
        poseArray[i][4] = hybrid.y();
        poseArray[i][5] = hybrid.z();
        poseArray[i][6] = hybrid.w();
      }
      if (selected("velocity")) {
        for (int axis = 0; axis < 3; ++axis)
          speedBiasArray[i][axis] = gt.velocity[axis];
      }
      if (selected("accel_bias")) {
        for (int axis = 3; axis < 6; ++axis) speedBiasArray[i][axis] = 0.0;
      }
      if (selected("gyro_bias")) {
        for (int axis = 6; axis < 9; ++axis) speedBiasArray[i][axis] = 0.0;
      }
    }
    if (selected("imu_preintegration")) {
      for (int j = 1; j <= frameCount; ++j) {
        pre_integrations[j]->repropagate(
            Eigen::Map<const Eigen::Vector3d>(&speedBiasArray[j - 1][3]),
            Eigen::Map<const Eigen::Vector3d>(&speedBiasArray[j - 1][6]));
      }
    }
    if (selected("landmark_depth")) {
      std::vector<std::array<double, SIZE_POSE>> intended_pose(frameCount + 1);
      for (int i = 0; i <= frameCount; ++i) {
        std::copy(poseArray[i], poseArray[i] + SIZE_POSE,
                  intended_pose[i].begin());
        const auto &gt = aligned_ground_truth[i];
        poseArray[i][0] = gt.position.x();
        poseArray[i][1] = gt.position.y();
        poseArray[i][2] = gt.position.z();
        poseArray[i][3] = gt.orientation.x();
        poseArray[i][4] = gt.orientation.y();
        poseArray[i][5] = gt.orientation.z();
        poseArray[i][6] = gt.orientation.w();
      }
      std::set<double *> feature_blocks;
      for (int i = 0; i < featureManager.getFeatureCount(); ++i)
        feature_blocks.insert(para_Feature[i]);
      std::vector<double *> parameter_blocks;
      problem.GetParameterBlocks(&parameter_blocks);
      std::map<double *, bool> was_constant;
      for (double *block : parameter_blocks) {
        was_constant[block] = problem.IsParameterBlockConstant(block);
        problem.SetParameterBlockConstant(block);
      }
      for (double *block : feature_blocks) {
        if (problem.HasParameterBlock(block))
          problem.SetParameterBlockVariable(block);
      }
      ceres::Solver::Options depth_options;
      depth_options.linear_solver_type = ceres::DENSE_SCHUR;
      depth_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
      depth_options.max_num_iterations = 30;
      depth_options.max_solver_time_in_seconds = 5.0;
      ceres::Solver::Summary depth_summary;
      ceres::Solve(depth_options, &problem, &depth_summary);
      if (!depth_summary.IsSolutionUsable() ||
          !std::isfinite(depth_summary.final_cost)) {
        throw std::runtime_error("GT intervention depth solve failed");
      }
      for (const auto &entry : was_constant) {
        if (!entry.second) problem.SetParameterBlockVariable(entry.first);
      }
      for (int i = 0; i <= frameCount; ++i) {
        std::copy(intended_pose[i].begin(), intended_pose[i].end(),
                  poseArray[i]);
      }
    }
    write_state("intervention_entry");
    const char *force_commit_value =
        std::getenv("VINS_REPLAY_GT_INTERVENTION_FORCE_COMMIT");
    gt_intervention_force_commit =
        force_commit_value && std::string(force_commit_value) == "1";
    if (gt_intervention_force_commit && !full) {
      throw std::runtime_error(
          "forced GT checkpoint commit is valid only for full intervention");
    }
    if (gt_intervention_force_commit) {
      if (!last_marginalization_info ||
          last_marginalization_parameter_blocks.size() !=
              last_marginalization_info->keep_block_data.size() ||
          last_marginalization_info->linearized_residuals.size() == 0) {
        throw std::runtime_error(
            "full GT checkpoint cannot recenter missing online prior");
      }
      // A checkpoint is not GT-consistent if its state is replaced while the
      // incoming prior still has the Production linearization point and
      // residual.  Preserve the prior information/Jacobian, move every kept
      // block's linearization point to the injected state, and zero the
      // residual so the checkpoint itself is the prior mean.  This is a
      // replay-only oracle operation, gated by force-commit.
      gt_intervention_prior_residual_norm_before =
          last_marginalization_info->linearized_residuals.norm();
      for (std::size_t block_index = 0;
           block_index < last_marginalization_parameter_blocks.size();
           ++block_index) {
        const int block_size =
            last_marginalization_info->keep_block_size[block_index];
        if (block_size <= 0 ||
            !last_marginalization_parameter_blocks[block_index] ||
            !last_marginalization_info->keep_block_data[block_index]) {
          throw std::runtime_error(
              "invalid online prior block during GT recenter");
        }
        std::copy(last_marginalization_parameter_blocks[block_index],
                  last_marginalization_parameter_blocks[block_index] +
                      block_size,
                  last_marginalization_info->keep_block_data[block_index]);
      }
      last_marginalization_info->linearized_residuals.setZero();
      gt_intervention_prior_recentered = true;
      gt_commit_pose.resize(frameCount + 1);
      gt_commit_speed_bias.resize(frameCount + 1);
      for (int i = 0; i <= frameCount; ++i) {
        std::copy(poseArray[i], poseArray[i] + SIZE_POSE,
                  gt_commit_pose[i].begin());
        std::copy(speedBiasArray[i], speedBiasArray[i] + SIZE_SPEEDBIAS,
                  gt_commit_speed_bias[i].begin());
      }
      gt_commit_inverse_depth.resize(featureManager.getFeatureCount());
      for (int i = 0; i < featureManager.getFeatureCount(); ++i)
        gt_commit_inverse_depth[i] = para_Feature[i][0];
    }
    std::ofstream metadata(gt_intervention_directory + "/metadata.json");
    if (!metadata) {
      throw std::runtime_error("cannot write GT intervention metadata");
    }
    metadata << std::setprecision(17)
             << "{\n  \"schema_version\": 1,\n"
             << "  \"target_timestamp_s\": " << gt_config.target << ",\n"
             << "  \"actual_timestamp_s\": " << solve_timestamp << ",\n"
             << "  \"families\": \"" << gt_config.family_text << "\",\n"
             << "  \"factor_re_evaluation\": true,\n"
             << "  \"incoming_prior_recentered\": "
             << (gt_intervention_prior_recentered ? "true" : "false")
             << ",\n  \"incoming_prior_residual_norm_before\": ";
    if (std::isfinite(gt_intervention_prior_residual_norm_before)) {
      metadata << gt_intervention_prior_residual_norm_before;
    } else {
      metadata << "null";
    }
    metadata << ",\n"
             << "  \"checkpoint_commit\": true\n}\n";
    replayGtInterventionApplied = true;
    gt_intervention_this_solve = true;
    VINS_WARN << "Replay GT factor intervention applied: timestamp="
              << solve_timestamp << ", families=" << gt_config.family_text;
  }

  // Replay-only, read-only export of the exact solve-entry linear graph.  It
  // is disabled unless both environment variables are explicitly supplied;
  // no estimator state, factor, ordering, or solver option is changed.
  if (shouldExportObservability(solve_timestamp)) {
    const char *directory_value =
        std::getenv("VINS_OBSERVABILITY_EXPORT_DIRECTORY");
    if (!directory_value || !*directory_value) {
      throw std::runtime_error(
          "observability timestamps set without export directory");
    }
    const std::string directory(directory_value);
    if (::mkdir(directory.c_str(), 0775) != 0 && errno != EEXIST) {
      throw std::runtime_error("cannot create observability export directory");
    }
    std::ostringstream label_stream;
    label_stream << std::fixed << std::setprecision(3) << solve_timestamp;
    std::string label = label_stream.str();
    std::replace(label.begin(), label.end(), '.', 'p');
    const std::string prefix = directory + "/graph_" + label;

    std::vector<double *> ordered_blocks;
    std::vector<std::string> block_names;
    std::vector<std::string> column_names;
    std::vector<int> block_global_sizes;
    std::vector<int> block_local_sizes;
    const auto add_block = [&](double *block, const std::string &name,
                               const std::vector<std::string> &locals) {
      if (!problem.HasParameterBlock(block) ||
          problem.IsParameterBlockConstant(block)) {
        return;
      }
      const int global_size = problem.ParameterBlockSize(block);
      const int local_size = problem.ParameterBlockLocalSize(block);
      if (local_size != static_cast<int>(locals.size())) {
        throw std::runtime_error("observability block local-size mismatch");
      }
      ordered_blocks.push_back(block);
      block_names.push_back(name);
      block_global_sizes.push_back(global_size);
      block_local_sizes.push_back(local_size);
      for (const auto &local : locals) column_names.push_back(name + "." + local);
    };
    for (int i = 0; i <= frameCount; ++i) {
      const std::string state = "state[" + std::to_string(i) + "]";
      add_block(poseArray[i], state + ".pose",
                {"px", "py", "pz", "roll", "pitch", "yaw"});
      if (options->hasImu()) {
        add_block(speedBiasArray[i], state + ".speed_bias",
                  {"vx", "vy", "vz", "bax", "bay", "baz", "bgx",
                   "bgy", "bgz"});
      }
    }
    int feature_index = -1;
    std::vector<int> parameterized_feature_ids;
    for (const auto &feature : featureManager.feature) {
      if (feature.feature_per_frame.size() < kMinFeatureTrackFrames) continue;
      ++feature_index;
      parameterized_feature_ids.push_back(feature.feature_id);
      add_block(para_Feature[feature_index],
                "landmark[" + std::to_string(feature.feature_id) + "]",
                {"inverse_depth"});
    }

    struct FamilyExport {
      double cost = 0.0;
      int residual_count = 0;
      Eigen::MatrixXd hessian;
      Eigen::VectorXd gradient;
    };
    const auto evaluate_family =
        [&](const std::vector<ceres::ResidualBlockId> &family) {
          FamilyExport output;
          const int columns = static_cast<int>(column_names.size());
          output.hessian = Eigen::MatrixXd::Zero(columns, columns);
          output.gradient = Eigen::VectorXd::Zero(columns);
          if (family.empty()) return output;
          ceres::Problem::EvaluateOptions evaluate_options;
          evaluate_options.parameter_blocks = ordered_blocks;
          evaluate_options.residual_blocks = family;
          evaluate_options.apply_loss_function = true;
          std::vector<double> residuals;
          std::vector<double> gradient;
          ceres::CRSMatrix jacobian;
          if (!problem.Evaluate(evaluate_options, &output.cost, &residuals,
                                &gradient, &jacobian)) {
            throw std::runtime_error("observability family evaluation failed");
          }
          if (jacobian.num_cols != columns ||
              static_cast<int>(gradient.size()) != columns) {
            throw std::runtime_error(
                "observability family column ordering mismatch");
          }
          output.residual_count = jacobian.num_rows;
          output.gradient = Eigen::Map<const Eigen::VectorXd>(
              gradient.data(), static_cast<int>(gradient.size()));
          for (int row = 0; row < jacobian.num_rows; ++row) {
            for (int left = jacobian.rows[row];
                 left < jacobian.rows[row + 1]; ++left) {
              const int left_column = jacobian.cols[left];
              const double left_value = jacobian.values[left];
              for (int right = jacobian.rows[row];
                   right < jacobian.rows[row + 1]; ++right) {
                output.hessian(left_column, jacobian.cols[right]) +=
                    left_value * jacobian.values[right];
              }
            }
          }
          output.hessian =
              0.5 * (output.hessian + output.hessian.transpose());
          if (!output.hessian.allFinite() || !output.gradient.allFinite() ||
              !std::isfinite(output.cost)) {
            throw std::runtime_error("nonfinite observability family export");
          }
          return output;
        };
    const FamilyExport visual = evaluate_family(visual_residual_blocks);
    const FamilyExport imu = evaluate_family(imu_residual_blocks);
    const FamilyExport prior = evaluate_family(prior_residual_blocks);
    if (prior_residual_blocks.size() != 1 ||
        !last_marginalization_info || !last_marginalization_info->valid) {
      throw std::runtime_error("exact solve has no valid unique online prior");
    }
    std::deque<OdomData> ground_truth_history;
    {
      std::lock_guard<std::mutex> lock(replayGroundTruthMutex);
      ground_truth_history = replayGroundTruthHistory;
    }
    if (ground_truth_history.size() < 2) {
      throw std::runtime_error("missing replay ground truth for exact export");
    }
    const auto interpolate_ground_truth = [&](double timestamp) {
      auto upper = std::lower_bound(
          ground_truth_history.begin(), ground_truth_history.end(), timestamp,
          [](const OdomData &sample, double value) {
            return sample.timestamp < value;
          });
      if (upper == ground_truth_history.begin() ||
          upper == ground_truth_history.end()) {
        throw std::runtime_error("ground truth does not bracket graph state");
      }
      const auto lower = std::prev(upper);
      const double duration = upper->timestamp - lower->timestamp;
      if (!std::isfinite(duration) || duration <= 0.0 || duration > 0.1) {
        throw std::runtime_error("invalid ground-truth interpolation interval");
      }
      const double ratio = (timestamp - lower->timestamp) / duration;
      OdomData result;
      result.timestamp = timestamp;
      result.position =
          (1.0 - ratio) * lower->position + ratio * upper->position;
      result.orientation =
          lower->orientation.slerp(ratio, upper->orientation).normalized();
      return result;
    };
    std::vector<OdomData> aligned_ground_truth;
    aligned_ground_truth.reserve(frameCount + 1);
    for (int i = 0; i <= frameCount; ++i) {
      aligned_ground_truth.push_back(
          interpolate_ground_truth(estimator_state[i].timestamp));
    }
    const auto yaw = [](const Eigen::Matrix3d &rotation) {
      return std::atan2(rotation(1, 0), rotation(0, 0));
    };
    const double yaw_offset =
        yaw(estimator_state[0].rotation) -
        yaw(aligned_ground_truth[0].orientation.toRotationMatrix());
    const Eigen::Matrix3d yaw_rotation =
        Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3d translation =
        estimator_state[0].position -
        yaw_rotation * aligned_ground_truth[0].position;
    for (auto &state : aligned_ground_truth) {
      state.position = yaw_rotation * state.position + translation;
      state.orientation = Eigen::Quaterniond(
          yaw_rotation * state.orientation.toRotationMatrix());
    }
    for (int i = 0; i <= frameCount; ++i) {
      int before = std::max(0, i - 1);
      int after = std::min(frameCount, i + 1);
      const double dt = aligned_ground_truth[after].timestamp -
                        aligned_ground_truth[before].timestamp;
      if (!std::isfinite(dt) || dt <= 0.0) {
        throw std::runtime_error("invalid GT velocity differentiation interval");
      }
      aligned_ground_truth[i].velocity =
          (aligned_ground_truth[after].position -
           aligned_ground_truth[before].position) /
          dt;
    }
    std::vector<std::vector<double>> saved_blocks;
    saved_blocks.reserve(ordered_blocks.size());
    for (std::size_t i = 0; i < ordered_blocks.size(); ++i) {
      saved_blocks.emplace_back(ordered_blocks[i],
                                ordered_blocks[i] + block_global_sizes[i]);
    }
    for (int i = 0; i <= frameCount; ++i) {
      const auto &state = aligned_ground_truth[i];
      for (int axis = 0; axis < 3; ++axis) poseArray[i][axis] = state.position[axis];
      poseArray[i][3] = state.orientation.x();
      poseArray[i][4] = state.orientation.y();
      poseArray[i][5] = state.orientation.z();
      poseArray[i][6] = state.orientation.w();
      for (int axis = 0; axis < 3; ++axis)
        speedBiasArray[i][axis] = state.velocity[axis];
      for (int axis = 3; axis < SIZE_SPEEDBIAS; ++axis)
        speedBiasArray[i][axis] = 0.0;
    }
    // Conditional GT residuals must not be evaluated with stale online
    // landmark depths. Hold the complete GT state fixed, optimize only the
    // landmark nuisance blocks, then restore the original graph exactly.
    for (int i = 0; i <= frameCount; ++i) {
      problem.SetParameterBlockConstant(poseArray[i]);
      problem.SetParameterBlockConstant(speedBiasArray[i]);
    }
    ceres::Solver::Options gt_nuisance_options;
    gt_nuisance_options.linear_solver_type = ceres::DENSE_SCHUR;
    gt_nuisance_options.trust_region_strategy_type =
        ceres::LEVENBERG_MARQUARDT;
    gt_nuisance_options.max_num_iterations = 30;
    gt_nuisance_options.max_solver_time_in_seconds = 5.0;
    ceres::Solver::Summary gt_nuisance_summary;
    ceres::Solve(gt_nuisance_options, &problem, &gt_nuisance_summary);
    if (!gt_nuisance_summary.IsSolutionUsable() ||
        !std::isfinite(gt_nuisance_summary.final_cost)) {
      throw std::runtime_error("GT landmark nuisance optimization failed");
    }
    for (int i = 0; i <= frameCount; ++i) {
      problem.SetParameterBlockVariable(poseArray[i]);
      problem.SetParameterBlockVariable(speedBiasArray[i]);
    }
    const FamilyExport gt_visual = evaluate_family(visual_residual_blocks);
    const FamilyExport gt_imu = evaluate_family(imu_residual_blocks);
    const FamilyExport gt_prior = evaluate_family(prior_residual_blocks);
    for (std::size_t i = 0; i < ordered_blocks.size(); ++i) {
      std::copy(saved_blocks[i].begin(), saved_blocks[i].end(),
                ordered_blocks[i]);
    }
    for (const auto &family : {std::make_pair("visual", &visual),
                               std::make_pair("imu", &imu),
                               std::make_pair("prior", &prior)}) {
      writeDenseCsv(prefix + "_" + family.first + "_hessian.csv",
                    family.second->hessian);
      writeVectorCsv(prefix + "_" + family.first + "_gradient.csv",
                     family.second->gradient);
    }
    for (const auto &family : {std::make_pair("visual", &gt_visual),
                               std::make_pair("imu", &gt_imu),
                               std::make_pair("prior", &gt_prior)}) {
      writeDenseCsv(prefix + "_gt_exact_" + family.first + "_hessian.csv",
                    family.second->hessian);
      writeVectorCsv(prefix + "_gt_exact_" + family.first + "_gradient.csv",
                     family.second->gradient);
    }

    std::ofstream columns(prefix + "_columns.csv");
    if (!columns) throw std::runtime_error("cannot write observability columns");
    columns << "column,name\n";
    for (std::size_t i = 0; i < column_names.size(); ++i)
      columns << i << ',' << column_names[i] << '\n';

    std::ofstream parameters(prefix + "_parameters.csv");
    if (!parameters)
      throw std::runtime_error("cannot write observability parameters");
    parameters << "block,name,global_size,local_size,values\n";
    for (std::size_t i = 0; i < ordered_blocks.size(); ++i) {
      parameters << i << ',' << block_names[i] << ','
                 << block_global_sizes[i] << ',' << block_local_sizes[i]
                 << ',';
      for (int component = 0; component < block_global_sizes[i]; ++component) {
        if (component) parameters << ';';
        parameters << std::setprecision(17) << ordered_blocks[i][component];
      }
      parameters << '\n';
    }

    std::ofstream prior_parameters(prefix + "_prior_linearization.csv");
    if (!prior_parameters)
      throw std::runtime_error("cannot write prior linearization data");
    prior_parameters << "block,name,global_size,values\n";

    std::ofstream topology(prefix + "_topology.csv");
    if (!topology) throw std::runtime_error("cannot write topology export");
    topology << "feature_id,start_slot,lifetime,mono_observations,"
                "stereo_observations,estimated_depth_m,parameterized\n";
    for (const auto &feature : featureManager.feature) {
      int stereo = 0;
      for (const auto &observation : feature.feature_per_frame)
        stereo += observation.is_stereo ? 1 : 0;
      const int lifetime = static_cast<int>(feature.feature_per_frame.size());
      topology << feature.feature_id << ',' << feature.start_frame << ','
               << lifetime << ',' << lifetime << ',' << stereo << ','
               << std::setprecision(17) << feature.estimated_depth << ','
               << (lifetime >= kMinFeatureTrackFrames ? 1 : 0) << '\n';
    }
    std::ofstream observations(prefix + "_observations.csv");
    if (!observations)
      throw std::runtime_error("cannot write observation topology export");
    observations << "feature_id,state_slot,stereo,u_px,v_px,depth_m\n";
    for (const auto &feature : featureManager.feature) {
      for (std::size_t observation_index = 0;
           observation_index < feature.feature_per_frame.size();
           ++observation_index) {
        const auto &observation = feature.feature_per_frame[observation_index];
        const double z = observation.point.z();
        if (!std::isfinite(z) || std::abs(z) < 1e-12) continue;
        observations << feature.feature_id << ','
                     << feature.start_frame + observation_index << ','
                     << (observation.is_stereo ? 1 : 0) << ','
                     << std::setprecision(17)
                     << FOCAL_LENGTH * observation.point.x() / z + 160.0
                     << ','
                     << FOCAL_LENGTH * observation.point.y() / z + 120.0
                     << ',' << feature.estimated_depth << '\n';
      }
    }

    std::ofstream metadata(prefix + "_metadata.json");
    if (!metadata) throw std::runtime_error("cannot write observability metadata");
    metadata << std::setprecision(17) << "{\n"
             << "  \"schema_version\": 1,\n"
             << "  \"timestamp_s\": " << solve_timestamp << ",\n"
             << "  \"marginalization_source_timestamp_s\": "
             << lastMarginalizationSourceTimestamp << ",\n"
             << "  \"marginalization_source_type\": \""
             << lastMarginalizationSourceType << "\",\n"
             << "  \"next_marginalization_type\": \""
             << (isNewMarginalization() ? "second_new" : "old") << "\",\n"
             << "  \"column_count\": " << column_names.size() << ",\n"
             << "  \"parameterized_landmark_count\": "
             << parameterized_feature_ids.size() << ",\n"
             << "  \"factor_families\": {\n"
             << "    \"visual\": {\"residual_blocks\": "
             << visual_residual_blocks.size() << ", \"residual_count\": "
             << visual.residual_count << ", \"cost\": " << visual.cost
             << "},\n"
             << "    \"imu\": {\"residual_blocks\": "
             << imu_residual_blocks.size() << ", \"residual_count\": "
             << imu.residual_count << ", \"cost\": " << imu.cost << "},\n"
             << "    \"prior\": {\"residual_blocks\": "
             << prior_residual_blocks.size() << ", \"residual_count\": "
             << prior.residual_count << ", \"cost\": " << prior.cost
             << "}\n  },\n"
             << "  \"gt_exact_factor_families\": {\n"
             << "    \"visual\": {\"residual_count\": "
             << gt_visual.residual_count << ", \"cost\": "
             << gt_visual.cost << "},\n"
             << "    \"imu\": {\"residual_count\": "
             << gt_imu.residual_count << ", \"cost\": "
             << gt_imu.cost << "},\n"
             << "    \"prior\": {\"residual_count\": "
             << gt_prior.residual_count << ", \"cost\": "
             << gt_prior.cost << "}\n  },\n"
             << "  \"gt_alignment_yaw_rad\": " << yaw_offset << ",\n"
             << "  \"gt_alignment_translation_m\": ["
             << translation.x() << ',' << translation.y() << ','
             << translation.z() << "],\n"
             << "  \"state_timestamps_s\": [";
    for (int i = 0; i <= frameCount; ++i) {
      if (i) metadata << ',';
      metadata << estimator_state[i].timestamp;
    }
    metadata << "],\n  \"prior_blocks\": [\n";
    if (last_marginalization_parameter_blocks.size() !=
        last_marginalization_info->keep_block_size.size()) {
      throw std::runtime_error("online prior block metadata size mismatch");
    }
    for (std::size_t block_index = 0;
         block_index < last_marginalization_parameter_blocks.size();
         ++block_index) {
      double *block = last_marginalization_parameter_blocks[block_index];
      std::string name;
      for (int i = 0; i <= frameCount && name.empty(); ++i) {
        if (block == poseArray[i]) name = "state[" + std::to_string(i) + "].pose";
        if (block == speedBiasArray[i])
          name = "state[" + std::to_string(i) + "].speed_bias";
      }
      for (int i = 0; i < options->getNumCameras() && name.empty(); ++i)
        if (block == para_Ex_Pose[i]) name = "extrinsic[" + std::to_string(i) + "]";
      if (block == para_Td[0]) name = "time_delay";
      if (name.empty()) throw std::runtime_error("unknown online prior block");
      prior_parameters << block_index << ',' << name << ','
                       << last_marginalization_info->keep_block_size[block_index]
                       << ',';
      for (int component = 0;
           component < last_marginalization_info->keep_block_size[block_index];
           ++component) {
        if (component) prior_parameters << ';';
        prior_parameters << std::setprecision(17)
                         << last_marginalization_info
                                ->keep_block_data[block_index][component];
      }
      prior_parameters << '\n';
      metadata << "    {\"index\": " << block_index << ", \"name\": \""
               << name << "\", \"global_size\": "
               << last_marginalization_info->keep_block_size[block_index]
               << ", \"local_index\": "
               << last_marginalization_info->keep_block_idx[block_index]
               << "}";
      if (block_index + 1 != last_marginalization_parameter_blocks.size())
        metadata << ',';
      metadata << '\n';
    }
    metadata << "  ],\n  \"block_order\": [\n";
    int local_offset = 0;
    for (std::size_t i = 0; i < block_names.size(); ++i) {
      metadata << "    {\"name\": \"" << block_names[i]
               << "\", \"global_size\": " << block_global_sizes[i]
               << ", \"local_size\": " << block_local_sizes[i]
               << ", \"local_offset\": " << local_offset << "}";
      local_offset += block_local_sizes[i];
      if (i + 1 != block_names.size()) metadata << ',';
      metadata << '\n';
    }
    metadata << "  ]\n}\n";
    VINS_INFO << "Exact online observability graph exported: timestamp="
              << solve_timestamp << ", prefix=" << prefix;
  }

  ceres::Solver::Options ceres_options;
  ceres_options.linear_solver_type =
      options->USE_GPU_CERES ? ceres::DENSE_QR : ceres::DENSE_SCHUR;
  // In the repeated monocular SITL trajectory, Dogleg occasionally accepted
  // a single 0.17--0.43 m state mutation after tens of healthy seconds. LM's
  // damped steps are slower but substantially safer near weak geometry and
  // remain within the existing 80 ms solver budget.
  ceres_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  ceres_options.max_num_iterations =
      max_iterations_override > 0 ? max_iterations_override
                                  : options->max_num_iterations();
  ceres_options.max_solver_time_in_seconds =
      (!isNewMarginalization()) ? options->max_solver_time() * 0.8
                                : options->max_solver_time();

  ceres::Solver::Summary summary;
  ceres::Solve(ceres_options, &problem, &summary);
  if (gt_intervention_force_commit) {
    for (int i = 0; i <= frameCount; ++i) {
      std::copy(gt_commit_pose[i].begin(), gt_commit_pose[i].end(),
                poseArray[i]);
      std::copy(gt_commit_speed_bias[i].begin(),
                gt_commit_speed_bias[i].end(), speedBiasArray[i]);
    }
    for (int i = 0; i < featureManager.getFeatureCount(); ++i)
      para_Feature[i][0] = gt_commit_inverse_depth[i];
    VINS_WARN << "Replay full-GT checkpoint state force-committed after raw "
                 "factor evaluation: timestamp=" << solve_timestamp;
  }
  if (gt_intervention_this_solve) {
    std::ofstream state_file(gt_intervention_directory +
                             "/optimized_state.csv");
    if (!state_file) {
      throw std::runtime_error("cannot write optimized intervention state");
    }
    state_file << "slot,timestamp_s,px,py,pz,qx,qy,qz,qw,vx,vy,vz,"
                  "bax,bay,baz,bgx,bgy,bgz\n" << std::setprecision(17);
    for (int i = 0; i <= frameCount; ++i) {
      state_file << i << ',' << estimator_state[i].timestamp;
      for (int k = 0; k < SIZE_POSE; ++k)
        state_file << ',' << poseArray[i][k];
      for (int k = 0; k < SIZE_SPEEDBIAS; ++k)
        state_file << ',' << speedBiasArray[i][k];
      state_file << '\n';
    }
  }
  lastOptimizationInitialCost = summary.initial_cost;
  lastOptimizationFinalCost = summary.final_cost;
  lastOptimizationTimeSeconds = summary.total_time_in_seconds;
  lastOptimizationSuccessfulSteps = summary.num_successful_steps;
  lastOptimizationUnsuccessfulSteps = summary.num_unsuccessful_steps;
  lastOptimizationResidualBlocks = problem.NumResidualBlocks();
  lastOptimizationResiduals = problem.NumResiduals();
  lastOptimizationTermination = summary.termination_type;
  if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) ||
      summary.num_successful_steps == 0) {
    VINS_ERROR << "Ceres rejected solution: " << summary.BriefReport();
    return false;
  }
  return true;
}

void Estimator::exportMarginalizationTransport(
    const std::shared_ptr<MarginalizationInfo> &marg_info,
    std::size_t prior_factor_count, std::size_t imu_factor_count,
    const char *marginalization_type) {
  if (preCommitConsistencyAuditShadowActive) return;
  const double timestamp = estimator_state[WINDOW_SIZE].timestamp;
  if (!shouldExportMarginalizationTransport(timestamp)) return;
  const char *directory_value =
      std::getenv("VINS_MARGINALIZATION_TRANSPORT_DIRECTORY");
  if (!directory_value || !*directory_value) {
    throw std::runtime_error(
        "marginalization transport range set without export directory");
  }
  if (!marg_info || !marg_info->valid || marg_info->m <= 0 ||
      marg_info->n <= 0 ||
      prior_factor_count + imu_factor_count > marg_info->factors.size()) {
    throw std::runtime_error("invalid marginalization transport graph");
  }

  const std::string directory(directory_value);
  if (::mkdir(directory.c_str(), 0775) != 0 && errno != EEXIST) {
    throw std::runtime_error("cannot create marginalization transport directory");
  }
  std::ostringstream label_stream;
  label_stream << std::fixed << std::setprecision(3) << timestamp;
  std::string label = label_stream.str();
  std::replace(label.begin(), label.end(), '.', 'p');
  const std::string prefix = directory + "/event_" + label;

  std::unordered_map<long, std::string> names;
  std::unordered_map<long, double> state_timestamps;
  const auto add_name = [&](double *address, const std::string &name) {
    if (!address || !names.emplace(reinterpret_cast<long>(address), name).second) {
      throw std::runtime_error("duplicate marginalization parameter address");
    }
  };
  for (int i = 0; i <= frameCount; ++i) {
    add_name(poseArray[i], "state[" + std::to_string(i) + "].pose");
    state_timestamps[reinterpret_cast<long>(poseArray[i])] =
        estimator_state[i].timestamp;
    if (options->hasImu()) {
      add_name(speedBiasArray[i],
               "state[" + std::to_string(i) + "].speed_bias");
      state_timestamps[reinterpret_cast<long>(speedBiasArray[i])] =
          estimator_state[i].timestamp;
    }
  }
  for (int i = 0; i < options->getNumCameras(); ++i) {
    add_name(para_Ex_Pose[i], "extrinsic[" + std::to_string(i) + "]");
  }
  add_name(para_Td[0], "time_delay");
  int feature_index = -1;
  for (const auto &feature : featureManager.feature) {
    if (feature.feature_per_frame.size() < kMinFeatureTrackFrames) continue;
    ++feature_index;
    add_name(para_Feature[feature_index],
             "landmark[" + std::to_string(feature.feature_id) + "]");
  }

  const int dimension = marg_info->m + marg_info->n;
  struct SourceNormal {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;
  };
  const auto zero_source = [&]() {
    return SourceNormal{Eigen::MatrixXd::Zero(dimension, dimension),
                        Eigen::VectorXd::Zero(dimension)};
  };
  SourceNormal incoming = zero_source();
  SourceNormal imu = zero_source();
  SourceNormal visual = zero_source();
  SourceNormal other = zero_source();
  std::vector<int> occupancy(dimension, 0);
  for (const auto &entry : marg_info->parameter_block_idx) {
    const auto size_it = marg_info->parameter_block_size.find(entry.first);
    if (size_it == marg_info->parameter_block_size.end() ||
        names.find(entry.first) == names.end()) {
      throw std::runtime_error("unknown marginalization parameter block");
    }
    const int local_size = marg_info->localSize(size_it->second);
    if (entry.second < 0 || entry.second + local_size > dimension) {
      throw std::runtime_error("invalid marginalization block ordering");
    }
    for (int i = 0; i < local_size; ++i) ++occupancy[entry.second + i];
  }
  if (std::any_of(occupancy.begin(), occupancy.end(),
                  [](int count) { return count != 1; })) {
    throw std::runtime_error("duplicate or missing marginalization columns");
  }

  for (std::size_t factor_index = 0;
       factor_index < marg_info->factors.size(); ++factor_index) {
    const auto &factor = marg_info->factors[factor_index];
    if (!factor || factor->jacobians.size() != factor->parameter_blocks.size() ||
        !factor->residuals.allFinite()) {
      throw std::runtime_error("invalid marginalization residual block");
    }
    SourceNormal *source = &visual;
    if (factor_index < prior_factor_count) {
      source = &incoming;
    } else if (factor_index < prior_factor_count + imu_factor_count) {
      source = &imu;
    }
    for (std::size_t i = 0; i < factor->parameter_blocks.size(); ++i) {
      const long key_i = reinterpret_cast<long>(factor->parameter_blocks[i]);
      const int index_i = marg_info->parameter_block_idx.at(key_i);
      const int size_i = marg_info->localSize(
          marg_info->parameter_block_size.at(key_i));
      const Eigen::MatrixXd jacobian_i =
          factor->jacobians[i].leftCols(size_i);
      source->gradient.segment(index_i, size_i) +=
          jacobian_i.transpose() * factor->residuals;
      for (std::size_t j = i; j < factor->parameter_blocks.size(); ++j) {
        const long key_j = reinterpret_cast<long>(factor->parameter_blocks[j]);
        const int index_j = marg_info->parameter_block_idx.at(key_j);
        const int size_j = marg_info->localSize(
            marg_info->parameter_block_size.at(key_j));
        const Eigen::MatrixXd product =
            jacobian_i.transpose() * factor->jacobians[j].leftCols(size_j);
        source->hessian.block(index_i, index_j, size_i, size_j) += product;
        if (i != j) {
          source->hessian.block(index_j, index_i, size_j, size_i) +=
              product.transpose();
        }
      }
    }
  }
  const Eigen::MatrixXd total_hessian = incoming.hessian + imu.hessian +
                                         visual.hessian + other.hessian;
  const Eigen::VectorXd total_gradient = incoming.gradient + imu.gradient +
                                          visual.gradient + other.gradient;
  if (!total_hessian.allFinite() || !total_gradient.allFinite()) {
    throw std::runtime_error("nonfinite marginalization source normal equation");
  }
  const Eigen::MatrixXd Amm = 0.5 *
      (total_hessian.topLeftCorner(marg_info->m, marg_info->m) +
       total_hessian.topLeftCorner(marg_info->m, marg_info->m).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> decomposition(Amm);
  if (decomposition.info() != Eigen::Success) {
    throw std::runtime_error("marginalization nuisance eigensolve failed");
  }
  const Eigen::VectorXd inverse_values =
      Eigen::VectorXd((decomposition.eigenvalues().array() > marg_info->eps)
                          .select(decomposition.eigenvalues().array().inverse(),
                                  0.0));
  const Eigen::MatrixXd Amm_inverse =
      decomposition.eigenvectors() * inverse_values.asDiagonal() *
      decomposition.eigenvectors().transpose();
  const Eigen::MatrixXd expected_outgoing =
      total_hessian.bottomRightCorner(marg_info->n, marg_info->n) -
      total_hessian.bottomLeftCorner(marg_info->n, marg_info->m) *
          Amm_inverse *
          total_hessian.topRightCorner(marg_info->m, marg_info->n);
  const Eigen::VectorXd expected_gradient =
      total_gradient.tail(marg_info->n) -
      total_hessian.bottomLeftCorner(marg_info->n, marg_info->m) *
          Amm_inverse * total_gradient.head(marg_info->m);
  const Eigen::MatrixXd actual_outgoing =
      marg_info->linearized_jacobians.transpose() *
      marg_info->linearized_jacobians;
  const Eigen::VectorXd actual_gradient =
      marg_info->linearized_jacobians.transpose() *
      marg_info->linearized_residuals;
  const double hessian_error =
      (actual_outgoing - expected_outgoing).norm() /
      std::max(1.0, expected_outgoing.norm());
  const double gradient_error =
      (actual_gradient - expected_gradient).norm() /
      std::max(1.0, expected_gradient.norm());
  if (!actual_outgoing.allFinite() || !actual_gradient.allFinite() ||
      hessian_error > 1e-8 || gradient_error > 1e-8) {
    throw std::runtime_error("outgoing prior does not match Schur result");
  }

  for (const auto &source :
       {std::make_pair("incoming_prior", &incoming),
        std::make_pair("imu", &imu), std::make_pair("visual", &visual),
        std::make_pair("other", &other)}) {
    writeDenseCsv(prefix + "_" + source.first + "_hessian.csv",
                  source.second->hessian);
    writeVectorCsv(prefix + "_" + source.first + "_gradient.csv",
                   source.second->gradient);
  }
  writeDenseCsv(prefix + "_total_pre_schur_hessian.csv", total_hessian);
  writeVectorCsv(prefix + "_total_pre_schur_gradient.csv", total_gradient);
  writeDenseCsv(prefix + "_computed_outgoing_hessian.csv", expected_outgoing);
  writeVectorCsv(prefix + "_computed_outgoing_gradient.csv", expected_gradient);
  writeDenseCsv(prefix + "_actual_outgoing_hessian.csv", actual_outgoing);
  writeVectorCsv(prefix + "_actual_outgoing_gradient.csv", actual_gradient);

  std::vector<std::pair<int, long>> ordered_addresses;
  ordered_addresses.reserve(marg_info->parameter_block_idx.size());
  for (const auto &entry : marg_info->parameter_block_idx) {
    ordered_addresses.emplace_back(entry.second, entry.first);
  }
  std::sort(ordered_addresses.begin(), ordered_addresses.end());
  std::ofstream blocks(prefix + "_blocks.csv");
  if (!blocks) throw std::runtime_error("cannot write transport blocks");
  blocks << "name,state_timestamp_s,local_offset,global_size,local_size,"
            "dropped,values\n";
  for (const auto &entry : ordered_addresses) {
    const int global_size = marg_info->parameter_block_size.at(entry.second);
    const int local_size = marg_info->localSize(global_size);
    const auto data = marg_info->parameter_block_data.find(entry.second);
    if (data == marg_info->parameter_block_data.end() || !data->second) {
      throw std::runtime_error("missing marginalization linearization data");
    }
    blocks << names.at(entry.second) << ',';
    const auto timestamp_it = state_timestamps.find(entry.second);
    if (timestamp_it != state_timestamps.end()) {
      blocks << std::setprecision(17) << timestamp_it->second;
    }
    blocks << ',' << entry.first << ','
           << global_size << ',' << local_size << ','
           << (entry.first < marg_info->m ? 1 : 0) << ',';
    for (int i = 0; i < global_size; ++i) {
      if (i) blocks << ';';
      blocks << std::setprecision(17) << data->second[i];
    }
    blocks << '\n';
  }
  std::ofstream metadata(prefix + "_metadata.json");
  if (!metadata) throw std::runtime_error("cannot write transport metadata");
  metadata << std::setprecision(17)
           << "{\n  \"schema_version\": 1,\n"
           << "  \"timestamp_s\": " << timestamp << ",\n"
           << "  \"marginalization_type\": \"" << marginalization_type
           << "\",\n  \"factor_counts\": {\"incoming_prior\": "
           << prior_factor_count << ", \"imu\": " << imu_factor_count
           << ", \"visual\": "
           << marg_info->factors.size() - prior_factor_count - imu_factor_count
           << ", \"other\": 0},\n"
           << "  \"dropped_dimension\": " << marg_info->m << ",\n"
           << "  \"retained_dimension\": " << marg_info->n << ",\n"
           << "  \"source_sum_relative_error\": 0.0,\n"
           << "  \"outgoing_hessian_relative_error\": " << hessian_error
           << ",\n  \"outgoing_gradient_relative_error\": "
           << gradient_error << "\n}\n";
}

void Estimator::processOldMarginalization() {
  auto marginalization_info = std::make_shared<MarginalizationInfo>();
  prepareParameters();

  std::size_t prior_factor_count = 0;
  std::size_t imu_factor_count = 0;

  if (last_marginalization_info && last_marginalization_info->valid) {
    vector<int> drop_set;
    for (int i = 0;
         i < static_cast<int>(last_marginalization_parameter_blocks.size());
         i++) {
      if (last_marginalization_parameter_blocks[i] == poseArray[0] ||
          last_marginalization_parameter_blocks[i] == speedBiasArray[0]) {
        drop_set.push_back(i);
      }
    }

    auto marginalization_factor =
        std::make_shared<MarginalizationFactor>(last_marginalization_info);
    auto residual_block_info = std::make_shared<ResidualBlockInfo>(
        marginalization_factor, nullptr, last_marginalization_parameter_blocks,
        drop_set);
    marginalization_info->addResidualBlockInfo(residual_block_info);
    prior_factor_count = 1;
  }

  // IMU
  if (options->hasImu() &&
      (pre_integrations[1]->sum_dt < 10.0 ||
       replayIncludeLongImuFactors)) {
    marginalization_info->addResidualBlockInfo(createIMUResidualBlock());
    imu_factor_count = 1;
  }

  // Features
  if (options->shouldTraceFeatureFrame(
          estimator_state[WINDOW_SIZE].timestamp)) {
    std::ostringstream marginalization_ids;
    bool first = true;
    for (const auto &track : featureManager.feature) {
      if (static_cast<int>(track.feature_per_frame.size()) <
              kMinFeatureTrackFrames ||
          track.start_frame != 0) {
        continue;
      }
      if (!first) marginalization_ids << ",";
      marginalization_ids << track.feature_id;
      first = false;
    }
    VINS_INFO << "Marginalization feature trace: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", source=old, ids=" << marginalization_ids.str();
  }
  const double visual_gt_target = replayVisualGtTargetTimestamp();
  const bool replace_visual_with_gt =
      std::isfinite(visual_gt_target) &&
      std::abs(estimator_state[WINDOW_SIZE].timestamp - visual_gt_target) <=
          0.005;
  if (!replace_visual_with_gt) {
    addFeatureResidualBlocks(marginalization_info);
  } else {
    struct Replacement {
      FeaturePerFrame *frame;
      ReplayVisualMeasurement left;
      ReplayVisualMeasurement right;
      bool has_right;
    };
    const auto &measurements = replayVisualGtMeasurements();
    std::vector<Replacement> replacements;
    int qualifying_tracks = 0;
    for (auto &track : featureManager.feature) {
      if (static_cast<int>(track.feature_per_frame.size()) <
              kMinFeatureTrackFrames ||
          track.start_frame != 0) {
        continue;
      }
      ++qualifying_tracks;
      for (int observation_index = 0;
           observation_index <
               static_cast<int>(track.feature_per_frame.size());
           ++observation_index) {
        const int state_slot = track.start_frame + observation_index;
        if (state_slot < 0 || state_slot > WINDOW_SIZE) {
          throw std::runtime_error(
              "replay visual-GT observation has invalid state slot");
        }
        auto &frame = track.feature_per_frame[observation_index];
        const int64_t timestamp_key = replayVisualTimestampKey(
            estimator_state[state_slot].timestamp);
        const auto left = measurements.find(
            ReplayVisualMeasurementKey{timestamp_key, track.feature_id, 0});
        if (left == measurements.end()) {
          throw std::runtime_error(
              "missing replay visual-GT left measurement for feature " +
              std::to_string(track.feature_id) + " at " +
              std::to_string(estimator_state[state_slot].timestamp));
        }
        Replacement replacement{&frame, left->second, {}, frame.is_stereo};
        if (frame.is_stereo) {
          const auto right = measurements.find(ReplayVisualMeasurementKey{
              timestamp_key, track.feature_id, 1});
          if (right == measurements.end()) {
            throw std::runtime_error(
                "missing replay visual-GT right measurement for feature " +
                std::to_string(track.feature_id) + " at " +
                std::to_string(estimator_state[state_slot].timestamp));
          }
          replacement.right = right->second;
        }
        replacements.push_back(replacement);
      }
    }
    if (qualifying_tracks == 0 || replacements.empty()) {
      throw std::runtime_error(
          "replay visual-GT target has no qualifying visual observations");
    }
    std::vector<FeaturePerFrame> originals;
    originals.reserve(replacements.size());
    for (const auto &replacement : replacements) {
      originals.push_back(*replacement.frame);
      replacement.frame->point = replacement.left.point;
      replacement.frame->velocity = replacement.left.velocity;
      if (replacement.has_right) {
        replacement.frame->pointRight = replacement.right.point;
        replacement.frame->velocityRight = replacement.right.velocity;
      }
    }
    try {
      addFeatureResidualBlocks(marginalization_info);
    } catch (...) {
      for (std::size_t index = 0; index < replacements.size(); ++index) {
        *replacements[index].frame = originals[index];
      }
      throw;
    }
    for (std::size_t index = 0; index < replacements.size(); ++index) {
      *replacements[index].frame = originals[index];
    }
    VINS_WARN << "Replay visual measurements replaced with GT only for "
              << "marginalization factor construction: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", tracks=" << qualifying_tracks
              << ", observations=" << replacements.size();
  }

  marginalization_info->preMarginalize();
  marginalization_info->marginalize();
  exportMarginalizationTransport(marginalization_info, prior_factor_count,
                                 imu_factor_count, "old");

  auto addr_shift = createAddrShift(true);
  auto parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
  last_marginalization_info = std::move(marginalization_info);
  last_marginalization_parameter_blocks = parameter_blocks;
  lastMarginalizationSourceTimestamp =
      estimator_state[WINDOW_SIZE].timestamp;
  lastMarginalizationSourceType = "old";
  if (replayPriorScaleProjection &&
      !projectCurrentMarginalizationScaleDirection()) {
    VINS_ERROR << "Prior scale projection failed after old marginalization.";
  }
}
void Estimator::processNewMarginalization() {
  if (!last_marginalization_info) return;
  if (std::count(last_marginalization_parameter_blocks.begin(),
                 last_marginalization_parameter_blocks.end(),
                 poseArray[WINDOW_SIZE - 1])) {
    auto marginalization_info = std::make_shared<MarginalizationInfo>();
    prepareParameters();
    std::size_t prior_factor_count = 0;

    if (last_marginalization_info->valid) {
      vector<int> drop_set;
      for (int i = 0;
           i < static_cast<int>(last_marginalization_parameter_blocks.size());
           i++) {
        if (last_marginalization_parameter_blocks[i] ==
            poseArray[WINDOW_SIZE - 1]) {
          drop_set.push_back(i);
        }
      }

      auto marginalization_factor =
          std::make_shared<MarginalizationFactor>(last_marginalization_info);
      auto residual_block_info = std::make_shared<ResidualBlockInfo>(
          marginalization_factor, nullptr,
          last_marginalization_parameter_blocks, drop_set);
      marginalization_info->addResidualBlockInfo(residual_block_info);
      prior_factor_count = 1;
    }

    marginalization_info->preMarginalize();
    marginalization_info->marginalize();
    exportMarginalizationTransport(marginalization_info, prior_factor_count,
                                   0, "second_new");

    auto addr_shift = createAddrShift(false);
    auto parameter_blocks =
        marginalization_info->getParameterBlocks(addr_shift);
    last_marginalization_info = std::move(marginalization_info);
    last_marginalization_parameter_blocks = parameter_blocks;
    lastMarginalizationSourceTimestamp =
        estimator_state[WINDOW_SIZE].timestamp;
    lastMarginalizationSourceType = "second_new";
    if (replayPriorScaleProjection &&
        !projectCurrentMarginalizationScaleDirection()) {
      VINS_ERROR << "Prior scale projection failed after second-new "
                    "marginalization.";
    }
  }
}

bool Estimator::projectCurrentMarginalizationScaleDirection() {
  if (replayPriorScaleProjectionStartTimestamp >= 0.0 &&
      estimator_state[WINDOW_SIZE].timestamp <
          replayPriorScaleProjectionStartTimestamp) {
    return true;
  }
  if (replayPriorScaleProjectionMaximumCount > 0 &&
      replayPriorScaleProjectionCount >=
          replayPriorScaleProjectionMaximumCount) {
    return true;
  }
  if (!last_marginalization_info ||
      !last_marginalization_info->valid ||
      last_marginalization_info->n <= 0 ||
      last_marginalization_parameter_blocks.size() !=
          last_marginalization_info->keep_block_size.size() ||
      last_marginalization_info->keep_block_data.size() !=
          last_marginalization_info->keep_block_size.size()) {
    return false;
  }

  int anchor_block = -1;
  int anchor_pose_index = WINDOW_SIZE + 1;
  for (std::size_t block = 0;
       block < last_marginalization_parameter_blocks.size(); ++block) {
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      if (last_marginalization_parameter_blocks[block] == poseArray[i] &&
          i < anchor_pose_index) {
        anchor_pose_index = i;
        anchor_block = static_cast<int>(block);
      }
    }
  }
  if (anchor_block < 0) return false;
  const double *anchor_data =
      last_marginalization_info->keep_block_data[anchor_block];
  if (!anchor_data) return false;
  const Vector3d anchor(anchor_data[0], anchor_data[1], anchor_data[2]);

  Eigen::VectorXd direction =
      Eigen::VectorXd::Zero(last_marginalization_info->n);
  int pose_blocks = 0;
  int velocity_blocks = 0;
  for (std::size_t block = 0;
       block < last_marginalization_parameter_blocks.size(); ++block) {
    const int local_index =
        last_marginalization_info->keep_block_idx[block] -
        last_marginalization_info->m;
    const int global_size =
        last_marginalization_info->keep_block_size[block];
    const double *data =
        last_marginalization_info->keep_block_data[block];
    if (!data || local_index < 0 ||
        local_index +
                last_marginalization_info->localSize(global_size) >
            direction.size()) {
      return false;
    }
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
      if (last_marginalization_parameter_blocks[block] == poseArray[i]) {
        direction.segment<3>(local_index) =
            Vector3d(data[0], data[1], data[2]) - anchor;
        ++pose_blocks;
      } else if (last_marginalization_parameter_blocks[block] ==
                 speedBiasArray[i]) {
        direction.segment<3>(local_index) =
            Vector3d(data[0], data[1], data[2]);
        ++velocity_blocks;
      }
    }
  }
  if (pose_blocks < 2 || velocity_blocks < 1 ||
      !direction.allFinite() || direction.norm() <= 1e-6) {
    VINS_ERROR << "Prior scale projection direction invalid: pose_blocks="
               << pose_blocks << ", velocity_blocks=" << velocity_blocks
               << ", direction_norm=" << direction.norm()
               << ", prior_n=" << last_marginalization_info->n;
    return false;
  }

  const Eigen::VectorXd prior_direction =
      last_marginalization_info->linearized_jacobians * direction;
  const double prior_direction_norm = prior_direction.norm();
  const double prior_directional_gradient =
      prior_direction.dot(last_marginalization_info->linearized_residuals);
  if (!std::isfinite(prior_directional_gradient)) return false;
  if (prior_directional_gradient <
      replayPriorScaleProjectionMinimumDirectionalGradient) {
    return true;
  }
  double removed_jacobian_norm = 0.0;
  double removed_residual = 0.0;
  if (!last_marginalization_info->projectOutLocalDirection(
          direction, &removed_jacobian_norm, &removed_residual)) {
    return false;
  }
  if (removed_jacobian_norm > 1e-9) {
    ++replayPriorScaleProjectionCount;
  }
  VINS_WARN << "Marginalization prior scale direction projected: timestamp="
            << estimator_state[WINDOW_SIZE].timestamp
            << ", source=" << lastMarginalizationSourceType
            << ", pose_blocks=" << pose_blocks
            << ", velocity_blocks=" << velocity_blocks
            << ", direction_norm=" << direction.norm()
            << ", prior_direction_norm=" << prior_direction_norm
            << ", prior_directional_gradient="
            << prior_directional_gradient
            << ", removed_jacobian_norm=" << removed_jacobian_norm
            << ", removed_residual=" << removed_residual
            << ", projection_count=" << replayPriorScaleProjectionCount;
  return true;
}

std::shared_ptr<ResidualBlockInfo> Estimator::createIMUResidualBlock() {
  auto imu_factor = std::make_shared<IMUFactor>(pre_integrations[1]);
  return std::make_shared<ResidualBlockInfo>(
      imu_factor, nullptr,
      std::vector<double *>{poseArray[0], speedBiasArray[0], poseArray[1],
                            speedBiasArray[1]},
      std::vector<int>{0, 1});
}
void Estimator::addFeatureResidualBlocks(
    std::shared_ptr<MarginalizationInfo> &marg_info) {
  if (auditTriggeredDropAllVisualActive) {
    VINS_WARN << "Audit-triggered fallback prior omitted current visual "
                 "history: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp;
    return;
  }
  const auto add_pose_only_residual_blocks = [&]() {
    const Eigen::Quaterniond q_imu_camera(cameraRotation[0]);
    const Eigen::Vector3d t_imu_camera = cameraTranslation[0];
    const double sqrt_information =
        (FOCAL_LENGTH / 1.5) * options->pose_only_visual_weight_scale;
    for (const auto &track : featureManager.feature) {
      const int observation_count =
          static_cast<int>(track.feature_per_frame.size());
      if (observation_count < kMinFeatureTrackFrames ||
          track.start_frame != 0) {
        continue;
      }
      const int slot_k = observation_count - 1;
      if (slot_k > frameCount) continue;
      const Eigen::Vector3d &bearing_j =
          track.feature_per_frame.front().point;
      const Eigen::Vector3d &bearing_k =
          track.feature_per_frame.back().point;
      auto epipolar = std::shared_ptr<ceres::CostFunction>(
          pose_only_visual::EpipolarFunctor::Create(
              bearing_j, bearing_k, q_imu_camera, t_imu_camera,
              sqrt_information));
      auto epipolar_loss =
          std::make_shared<ceres::HuberLoss>(1.0);
      marg_info->addResidualBlockInfo(
          std::make_shared<ResidualBlockInfo>(
              epipolar, epipolar_loss,
              vector<double *>{poseArray[0], poseArray[slot_k]},
              vector<int>{0}));
      for (int observation_index = 1;
           observation_index < observation_count - 1;
           ++observation_index) {
        auto three_view = std::shared_ptr<ceres::CostFunction>(
            pose_only_visual::ThreeViewFunctor::Create(
                bearing_j, bearing_k,
                track.feature_per_frame[observation_index].point,
                q_imu_camera, t_imu_camera, sqrt_information));
        auto three_view_loss =
            std::make_shared<ceres::HuberLoss>(1.0);
        marg_info->addResidualBlockInfo(
            std::make_shared<ResidualBlockInfo>(
                three_view, three_view_loss,
                vector<double *>{poseArray[0], poseArray[slot_k],
                                 poseArray[observation_index]},
                vector<int>{0}));
      }
    }
  };
  if (options->replacesFeatureVisual() && !options->isUsingStereo()) {
    add_pose_only_residual_blocks();
    return;
  }
  int feature_index = -1;
  std::set<int> excluded_history_ids;
  const char *exclude_history_value =
      std::getenv("VINS_REPLAY_EXCLUDE_INCONSISTENT_HISTORY");
  const char *solve_exclude_history_value =
      std::getenv("VINS_REPLAY_EXCLUDE_INCONSISTENT_HISTORY_FROM_SOLVE");
  if ((exclude_history_value && std::string(exclude_history_value) == "1") ||
      (solve_exclude_history_value &&
       std::string(solve_exclude_history_value) == "1") ||
      auditTriggeredHistoryExclusionActive) {
    excluded_history_ids = featureManager.inconsistentLongTrackIds(
        frameCount, estimator_state, cameraTranslation, cameraRotation, 8,
        0.005);
    VINS_INFO << "Marginalization inconsistent-history exclusion: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", excluded_tracks=" << excluded_history_ids.size();
  }

  for (auto &it_per_id : featureManager.feature) {
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < kMinFeatureTrackFrames) continue;

    ++feature_index;

    int imu_i = it_per_id.start_frame;
    if (imu_i != 0) continue;
    if (excluded_history_ids.count(it_per_id.feature_id)) continue;

    int imu_j = imu_i - 1;
    const Vector3d &pts_i = it_per_id.feature_per_frame[0].point;

    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      const Vector3d &pts_j = it_per_frame.point;

      if (imu_i != imu_j) {
        auto f_td = std::make_shared<ProjectionTwoFrameOneCamFactor>(
            pts_i, pts_j, it_per_id.feature_per_frame[0].velocity,
            it_per_frame.velocity, it_per_id.feature_per_frame[0].cur_td,
            it_per_frame.cur_td);
        auto loss = std::make_shared<ceres::HuberLoss>(1.0);
        marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
            f_td, loss,
            vector<double *>{poseArray[imu_i], poseArray[imu_j],
                             para_Ex_Pose[0], para_Feature[feature_index],
                             para_Td[0]},
            vector<int>{0, 3}));
      }

      if (options->isUsingStereo() && it_per_frame.is_stereo) {
        const Vector3d &pts_j_right = it_per_frame.pointRight;
        auto loss = std::make_shared<ceres::HuberLoss>(1.0);

        if (imu_i != imu_j) {
          auto f = std::make_shared<ProjectionTwoFrameTwoCamFactor>(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
              f, loss,
              vector<double *>{poseArray[imu_i], poseArray[imu_j],
                               para_Ex_Pose[0], para_Ex_Pose[1],
                               para_Feature[feature_index], para_Td[0]},
              vector<int>{0, 4}));
        } else {
          auto f = std::make_shared<ProjectionOneFrameTwoCamFactor>(
              pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity,
              it_per_frame.velocityRight, it_per_id.feature_per_frame[0].cur_td,
              it_per_frame.cur_td);
          marg_info->addResidualBlockInfo(std::make_shared<ResidualBlockInfo>(
              f, loss,
              vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1],
                               para_Feature[feature_index], para_Td[0]},
              vector<int>{2}));
        }
      }
    }
  }
  if (options->augmentsFeatureVisual() && !options->isUsingStereo()) {
    add_pose_only_residual_blocks();
  }
}
std::unordered_map<long, double *> Estimator::createAddrShift(bool is_old) {
  std::unordered_map<long, double *> addr_shift;

  for (int i = 0; i <= WINDOW_SIZE; i++) {
    if (!is_old && i == WINDOW_SIZE - 1) continue;

    if (i == WINDOW_SIZE && !is_old) {
      addr_shift[reinterpret_cast<long>(poseArray[i])] = poseArray[i - 1];
      if (options->hasImu())
        addr_shift[reinterpret_cast<long>(speedBiasArray[i])] =
            speedBiasArray[i - 1];
    } else {
      addr_shift[reinterpret_cast<long>(poseArray[i])] =
          poseArray[i - (is_old ? 1 : 0)];
      if (options->hasImu())
        addr_shift[reinterpret_cast<long>(speedBiasArray[i])] =
            speedBiasArray[i - (is_old ? 1 : 0)];
    }
  }

  for (int i = 0; i < options->getNumCameras(); i++)
    addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

  addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

  return addr_shift;
}

bool Estimator::runPreCommitConsistencyAudit(
    const vins::BackendState &solve_entry,
    const vins::BackendState &production_candidate,
    std::string *error) {
  const auto restore_production = [&]() {
    preCommitConsistencyAuditShadowActive = false;
    std::string restore_error;
    const bool restored = applyBackendState(production_candidate,
                                            &restore_error);
    if (!restored && error) {
      *error = "cannot restore production candidate: " + restore_error;
    }
    return restored;
  };

  std::string apply_error;
  if (!applyBackendState(solve_entry, &apply_error)) {
    if (error) *error = "cannot restore audit solve entry: " + apply_error;
    return restore_production() && false;
  }

  // Force every currently eligible landmark to be reconstructed from the
  // observations and poses in this window.  The second optimize() therefore
  // relinearizes the same factor topology from a fresh inverse-depth history.
  featureManager.clearDepth();
  featureManager.triangulate(frameCount, estimator_state, cameraTranslation,
                             cameraRotation);
  preCommitConsistencyAuditShadowActive = true;
  if (!optimize()) {
    if (error) *error = "fresh-history shadow optimization was unusable";
    restore_production();
    return false;
  }

  // The accumulated incoming prior is precisely the history under audit.
  // Reusing it in both branches makes the comparison tautological.  Build
  // the fresh branch's outgoing prior from only the re-solved current-window
  // IMU and visual/depth factors while retaining identical state/feature
  // topology.  This remains diagnostic-only and is discarded below.
  last_marginalization_info.reset();
  last_marginalization_parameter_blocks.clear();
  processOldMarginalization();

  std::string capture_error;
  auto fresh_candidate = captureBackendState(&capture_error);
  if (!fresh_candidate) {
    if (error) {
      *error = "cannot capture fresh-history candidate: " + capture_error;
    }
    restore_production();
    return false;
  }

  try {
    exportPreCommitConsistencyAudit(solve_entry, production_candidate,
                                    *fresh_candidate);
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    restore_production();
    return false;
  }
  const char *margin_delay_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_MARGIN_OLD_DELAY");
  const bool delay_margin_old =
      margin_delay_value && std::string(margin_delay_value) == "1" &&
      lastPreCommitAuditSuspect && auditTriggeredMarginOldDelayCount < 5;
  if (delay_margin_old) {
    std::string delay_restore_error;
    if (!applyBackendState(solve_entry, &delay_restore_error)) {
      if (error) {
        *error = "cannot rollback for MARGIN_OLD delay: " +
                 delay_restore_error;
      }
      restore_production();
      return false;
    }
    marginalization_flag = MarginalizationType::MARGIN_SECOND_NEW;
    if (!optimize()) {
      if (error) *error = "MARGIN_OLD delay retry was unusable";
      restore_production();
      return false;
    }
    processNewMarginalization();
    ++auditTriggeredMarginOldDelayCount;
    VINS_WARN << "Audit-triggered MARGIN_OLD commit delayed: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", provisional=MARGIN_SECOND_NEW"
              << ", delay_count=" << auditTriggeredMarginOldDelayCount
              << ", state_depth_topology=current_window_preserved";
    if (error) error->clear();
    return true;
  }
  const char *relinearized_retry_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_RELINEARIZED_HISTORY_RETRY");
  const bool retry_with_relinearized_history =
      relinearized_retry_value &&
      std::string(relinearized_retry_value) == "1" &&
      lastPreCommitAuditSuspect &&
      !auditTriggeredRelinearizedHistoryRetryConsumed;
  if (retry_with_relinearized_history) {
    std::string retry_restore_error;
    if (!applyBackendState(solve_entry, &retry_restore_error)) {
      if (error) {
        *error = "cannot rollback for relinearized-history retry: " +
                 retry_restore_error;
      }
      restore_production();
      return false;
    }
    const std::set<int> inconsistent_ids =
        featureManager.inconsistentLongTrackIds(
            frameCount, estimator_state, cameraTranslation, cameraRotation,
            8, 0.005);
    const int relinearized =
        featureManager.relinearizeTrackHistoryFromLatestStereo(
            inconsistent_ids, frameCount, estimator_state,
            cameraTranslation, cameraRotation);
    if (relinearized <= 0 || !optimize()) {
      if (error) {
        *error = "relinearized-history retry produced no usable candidate";
      }
      restore_production();
      return false;
    }
    processOldMarginalization();
    auditTriggeredRelinearizedHistoryRetryConsumed = true;
    VINS_WARN << "Audit-triggered transaction retried with relinearized "
                 "history: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", inconsistent_tracks=" << inconsistent_ids.size()
              << ", relinearized_tracks=" << relinearized
              << ", topology=unchanged, source=latest_stereo";
    if (error) error->clear();
    return true;
  }
  const char *recent_retry_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_RECENT_HISTORY_RETRY");
  const bool retry_with_recent_history =
      recent_retry_value && std::string(recent_retry_value) == "1" &&
      lastPreCommitAuditSuspect && !auditTriggeredRecentHistoryRetryConsumed;
  if (retry_with_recent_history) {
    std::string retry_restore_error;
    if (!applyBackendState(solve_entry, &retry_restore_error)) {
      if (error) {
        *error = "cannot rollback for recent-history retry: " +
                 retry_restore_error;
      }
      restore_production();
      return false;
    }
    const std::set<int> inconsistent_ids =
        featureManager.inconsistentLongTrackIds(
            frameCount, estimator_state, cameraTranslation, cameraRotation,
            8, 0.005);
    const int truncated = featureManager.truncateTracksToRecentObservations(
        inconsistent_ids, 4);
    featureManager.clearDepth();
    featureManager.triangulate(frameCount, estimator_state,
                               cameraTranslation, cameraRotation);
    if (truncated <= 0 || !optimize()) {
      if (error) *error = "recent-history retry produced no usable candidate";
      restore_production();
      return false;
    }
    processOldMarginalization();
    auditTriggeredRecentHistoryRetryConsumed = true;
    VINS_WARN << "Audit-triggered transaction retried with recent history: "
              << "timestamp=" << estimator_state[WINDOW_SIZE].timestamp
              << ", inconsistent_tracks=" << inconsistent_ids.size()
              << ", truncated_tracks=" << truncated
              << ", observations_per_track=4";
    if (error) error->clear();
    return true;
  }
  const char *transaction_reject_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_TRANSACTION_REJECT");
  const char *quarantine_reject_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_QUARANTINE_REJECT");
  const bool quarantine_transaction =
      quarantine_reject_value &&
      std::string(quarantine_reject_value) == "1" &&
      lastPreCommitAuditSuspect &&
      !auditTriggeredQuarantineRejectConsumed;
  if (quarantine_transaction) {
    std::string reject_restore_error;
    if (!applyBackendState(solve_entry, &reject_restore_error)) {
      if (error) {
        *error = "cannot rollback quarantined transaction: " +
                 reject_restore_error;
      }
      restore_production();
      return false;
    }
    const int quarantined_tracks =
        static_cast<int>(featureManager.feature.size());
    auditTriggeredDropAllVisualActive = true;
    processOldMarginalization();
    auditTriggeredDropAllVisualActive = false;
    featureManager.clearState();
    auditTriggeredQuarantineRejectConsumed = true;
    lastBackendTransactionRejected = true;
    VINS_WARN << "Audit-triggered visual transaction rejected and history "
                 "quarantined: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", quarantined_tracks=" << quarantined_tracks
              << ", fallback=incoming_prior_plus_imu"
              << ", next_history=fresh_observations_only";
    if (error) error->clear();
    return true;
  }
  const bool reject_transaction =
      transaction_reject_value &&
      std::string(transaction_reject_value) == "1" &&
      lastPreCommitAuditSuspect &&
      !auditTriggeredTransactionRejectConsumed;
  if (reject_transaction) {
    std::string reject_restore_error;
    if (!applyBackendState(solve_entry, &reject_restore_error)) {
      if (error) {
        *error = "cannot rollback suspect transaction: " +
                 reject_restore_error;
      }
      restore_production();
      return false;
    }
    auditTriggeredDropAllVisualActive = true;
    processOldMarginalization();
    auditTriggeredDropAllVisualActive = false;
    auditTriggeredTransactionRejectConsumed = true;
    lastBackendTransactionRejected = true;
    VINS_WARN << "Audit-triggered visual solve/depth/prior transaction "
                 "rejected atomically: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", fallback=solve_entry_incoming_prior_plus_imu";
    if (error) error->clear();
    return true;
  }
  if (!restore_production()) return false;
  const char *prior_cap_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_PRIOR_INFORMATION_SCALE");
  if (prior_cap_value && *prior_cap_value && lastPreCommitAuditSuspect &&
      !auditTriggeredPriorInformationCapConsumed) {
    char *end = nullptr;
    const double information_scale = std::strtod(prior_cap_value, &end);
    if (end == prior_cap_value || *end != '\0' ||
        !std::isfinite(information_scale) || information_scale <= 0.0 ||
        information_scale >= 1.0) {
      if (error) *error = "invalid audit-triggered prior information scale";
      return false;
    }
    if (!last_marginalization_info || !last_marginalization_info->valid) {
      if (error) *error = "audit-triggered prior cap has no valid prior";
      return false;
    }
    // Preserve the accepted production state, depths, residual optimum, and
    // exact block topology.  Only make the suspect transaction's confidence
    // provisional: both H=J'J and g=J'r scale by information_scale.
    const double square_root_scale = std::sqrt(information_scale);
    last_marginalization_info->linearized_jacobians *= square_root_scale;
    last_marginalization_info->linearized_residuals *= square_root_scale;
    auditTriggeredPriorInformationCapConsumed = true;
    VINS_WARN << "Audit-triggered outgoing prior information capped: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp
              << ", information_scale=" << information_scale
              << ", state_depth_topology=production_unchanged";
  }
  const char *triggered_exclusion_value =
      std::getenv("VINS_REPLAY_AUDIT_TRIGGERED_HISTORY_EXCLUSION");
  const bool apply_triggered_exclusion =
      triggered_exclusion_value &&
      std::string(triggered_exclusion_value) == "1" &&
      lastPreCommitAuditSuspect &&
      !auditTriggeredHistoryExclusionConsumed;
  if (apply_triggered_exclusion) {
    if (!solve_entry.marginalization_prior) {
      if (error) *error = "audit-triggered rebuild has no incoming prior";
      return false;
    }
    std::vector<double *> rebound_blocks;
    rebound_blocks.reserve(solve_entry.prior_parameter_blocks.size());
    for (const double *source : solve_entry.prior_parameter_blocks) {
      double *target = nullptr;
      for (int state = 0; state <= WINDOW_SIZE && !target; ++state) {
        if (source == solve_entry.pose_parameters[state].data())
          target = poseArray[state];
        else if (source == solve_entry.speed_bias_parameters[state].data())
          target = speedBiasArray[state];
      }
      for (int camera = 0;
           camera < options->getNumCameras() && !target; ++camera) {
        if (source == solve_entry.extrinsic_parameters[camera].data())
          target = para_Ex_Pose[camera];
      }
      if (source == solve_entry.time_delay_parameter.data())
        target = para_Td[0];
      if (!target) {
        if (error) *error = "cannot rebind audit incoming prior block";
        return false;
      }
      rebound_blocks.push_back(target);
    }
    std::string clone_error;
    auto incoming_prior = solve_entry.marginalization_prior->cloneRebound(
        solve_entry.prior_parameter_blocks, rebound_blocks, &clone_error);
    if (!incoming_prior) {
      if (error) *error = "cannot clone audit incoming prior: " + clone_error;
      return false;
    }
    last_marginalization_info = std::move(incoming_prior);
    last_marginalization_parameter_blocks = std::move(rebound_blocks);
    lastMarginalizationSourceTimestamp =
        solve_entry.last_marginalization_source_timestamp;
    lastMarginalizationSourceType =
        solve_entry.last_marginalization_source_type;
    auditTriggeredHistoryExclusionActive = true;
    processOldMarginalization();
    auditTriggeredHistoryExclusionActive = false;
    auditTriggeredHistoryExclusionConsumed = true;
    VINS_WARN << "Audit-triggered inconsistent visual history excluded from "
                 "one outgoing prior: timestamp="
              << estimator_state[WINDOW_SIZE].timestamp;
  }
  if (error) error->clear();
  return true;
}

void Estimator::exportPreCommitConsistencyAudit(
    const vins::BackendState &solve_entry,
    const vins::BackendState &production_candidate,
    const vins::BackendState &fresh_candidate) {
  const char *directory_value =
      std::getenv("VINS_PRECOMMIT_AUDIT_DIRECTORY");
  if (!directory_value || !*directory_value) {
    throw std::runtime_error(
        "pre-commit audit range set without output directory");
  }
  if (!production_candidate.marginalization_prior ||
      !fresh_candidate.marginalization_prior ||
      !production_candidate.marginalization_prior->valid ||
      !fresh_candidate.marginalization_prior->valid) {
    throw std::runtime_error("pre-commit audit candidate has no valid prior");
  }
  const auto &production_prior =
      *production_candidate.marginalization_prior;
  const auto &fresh_prior = *fresh_candidate.marginalization_prior;
  struct WeakMetrics {
    double weakest = std::numeric_limits<double>::quiet_NaN();
    Eigen::VectorXd direction;
  };
  const auto weakest_mode = [](const Eigen::MatrixXd &input) {
    WeakMetrics result;
    if (input.rows() == 0 || input.rows() != input.cols()) return result;
    const Eigen::MatrixXd symmetric = 0.5 * (input + input.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(symmetric);
    if (eigen.info() != Eigen::Success) return result;
    const double maximum = eigen.eigenvalues().cwiseAbs().maxCoeff();
    const double threshold = std::max(1e-12, maximum * 1e-10);
    for (int i = 0; i < eigen.eigenvalues().size(); ++i) {
      if (eigen.eigenvalues()(i) > threshold) {
        result.weakest = eigen.eigenvalues()(i);
        result.direction = eigen.eigenvectors().col(i);
        break;
      }
    }
    return result;
  };
  const auto hessian = [](const MarginalizationInfo &prior) {
    return prior.linearized_jacobians.transpose() *
           prior.linearized_jacobians;
  };
  const Eigen::MatrixXd production_hessian = hessian(production_prior);
  const Eigen::MatrixXd fresh_hessian = hessian(fresh_prior);
  const WeakMetrics production_full = weakest_mode(production_hessian);
  const WeakMetrics fresh_full = weakest_mode(fresh_hessian);

  const auto selected_coordinates = [](const vins::BackendState &backend) {
    std::map<std::string, int> coordinates;
    const auto &prior = *backend.marginalization_prior;
    for (std::size_t block = 0;
         block < backend.prior_parameter_blocks.size(); ++block) {
      const double *address = backend.prior_parameter_blocks[block];
      const int local_offset = prior.keep_block_idx[block] - prior.m;
      int pose_state = -1;
      int speed_bias_state = -1;
      for (int state = 0; state <= WINDOW_SIZE; ++state) {
        if (address == backend.pose_parameters[state].data())
          pose_state = state;
        if (address == backend.speed_bias_parameters[state].data())
          speed_bias_state = state;
      }
      if (pose_state >= 0) {
        for (int component = 3; component < 6; ++component) {
          coordinates.emplace("state[" + std::to_string(pose_state) +
                                  "].attitude[" +
                                  std::to_string(component - 3) + "]",
                              local_offset + component);
        }
      } else if (speed_bias_state >= 0) {
        for (int component = 0; component < 6; ++component) {
          const char *family = component < 3 ? "velocity" : "accel_bias";
          const int family_component = component < 3 ? component
                                                      : component - 3;
          coordinates.emplace("state[" +
                                  std::to_string(speed_bias_state) + "]." +
                                  family + "[" +
                                  std::to_string(family_component) + "]",
                              local_offset + component);
        }
      }
    }
    return coordinates;
  };
  const auto production_coordinates =
      selected_coordinates(production_candidate);
  const auto fresh_coordinates = selected_coordinates(fresh_candidate);
  std::vector<int> production_common_coordinates;
  std::vector<int> fresh_common_coordinates;
  for (const auto &entry : production_coordinates) {
    const auto fresh = fresh_coordinates.find(entry.first);
    if (fresh == fresh_coordinates.end()) continue;
    production_common_coordinates.push_back(entry.second);
    fresh_common_coordinates.push_back(fresh->second);
  }
  if (production_common_coordinates.empty()) {
    throw std::runtime_error(
        "pre-commit audit has no common attitude/velocity/BA coordinates");
  }
  const auto select_hessian = [](const Eigen::MatrixXd &source,
                                 const std::vector<int> &coordinates) {
    Eigen::MatrixXd selected(coordinates.size(), coordinates.size());
    for (std::size_t row = 0; row < coordinates.size(); ++row) {
      for (std::size_t column = 0;
           column < coordinates.size(); ++column) {
        selected(row, column) = source(coordinates[row], coordinates[column]);
      }
    }
    return selected;
  };
  const WeakMetrics production_avba =
      weakest_mode(select_hessian(production_hessian,
                                   production_common_coordinates));
  const WeakMetrics fresh_avba =
      weakest_mode(select_hessian(fresh_hessian,
                                   fresh_common_coordinates));
  const auto alignment = [](const WeakMetrics &left,
                            const WeakMetrics &right) {
    if (left.direction.size() == 0 ||
        left.direction.size() != right.direction.size()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return std::abs(left.direction.dot(right.direction));
  };

  const StateData &entry = solve_entry.states[WINDOW_SIZE];
  const StateData &production = production_candidate.states[WINDOW_SIZE];
  const StateData &fresh = fresh_candidate.states[WINDOW_SIZE];
  const auto tilt_delta_deg = [](const StateData &before,
                                 const StateData &after) {
    const double cosine = std::max(
        -1.0, std::min(1.0, before.rotation.col(2).normalized().dot(
                                after.rotation.col(2).normalized())));
    return std::acos(cosine) * 180.0 / M_PI;
  };
  const auto body_velocity_delta = [](const StateData &before,
                                      const StateData &after) {
    return (after.rotation.transpose() * after.velocity -
            before.rotation.transpose() * before.velocity)
        .norm();
  };

  const int production_depth_count =
      production_candidate.last_solved_feature_count;
  const int fresh_depth_count = fresh_candidate.last_solved_feature_count;
  if (production_depth_count != fresh_depth_count ||
      production_depth_count < 0 || production_depth_count > NUM_OF_F) {
    throw std::runtime_error(
        "pre-commit audit inverse-depth topology differs");
  }
  double inverse_depth_abs_delta_mean = 0.0;
  double inverse_depth_delta_mean = 0.0;
  double inverse_depth_delta_square_mean = 0.0;
  for (int i = 0; i < production_depth_count; ++i) {
    const double delta =
        fresh_candidate.last_solved_feature_parameters[i][0] -
        production_candidate.last_solved_feature_parameters[i][0];
    if (!std::isfinite(delta)) {
      throw std::runtime_error("nonfinite pre-commit inverse-depth delta");
    }
    inverse_depth_abs_delta_mean += std::abs(delta);
    inverse_depth_delta_mean += delta;
    inverse_depth_delta_square_mean += delta * delta;
  }
  if (production_depth_count > 0) {
    inverse_depth_abs_delta_mean /= production_depth_count;
    inverse_depth_delta_mean /= production_depth_count;
    inverse_depth_delta_square_mean /= production_depth_count;
  }
  const double inverse_depth_delta_variance =
      std::max(0.0, inverse_depth_delta_square_mean -
                         inverse_depth_delta_mean * inverse_depth_delta_mean);

  const std::string directory(directory_value);
  if (::mkdir(directory.c_str(), 0775) != 0 && errno != EEXIST) {
    throw std::runtime_error("cannot create pre-commit audit directory");
  }
  const std::string csv_path = directory + "/precommit_consistency_audit.csv";
  std::ifstream existing(csv_path);
  const bool write_header = !existing.good() || existing.peek() == EOF;
  existing.close();
  std::ofstream output(csv_path, std::ios::app);
  if (!output) throw std::runtime_error("cannot write pre-commit audit CSV");
  if (write_header) {
    output << "timestamp_s,marginalization,production_retained_dimension,"
              "fresh_retained_dimension,production_prior_blocks,"
              "fresh_prior_blocks,common_avba_coordinates,"
              "feature_count,production_ba_norm,fresh_ba_norm,"
              "production_ba_delta,fresh_ba_delta,production_tilt_delta_deg,"
              "fresh_tilt_delta_deg,production_body_velocity_delta_mps,"
              "fresh_body_velocity_delta_mps,inverse_depth_abs_delta_mean,"
              "inverse_depth_delta_variance,production_avba_weakest,"
              "fresh_avba_weakest,avba_mode_alignment,production_full_weakest,"
              "fresh_full_weakest,full_mode_alignment,avba_information_ratio,"
              "full_information_ratio\n";
  }
  const auto ratio = [](double numerator, double denominator) {
    return std::isfinite(numerator) && std::isfinite(denominator) &&
                   denominator > 0.0
               ? numerator / denominator
               : std::numeric_limits<double>::quiet_NaN();
  };
  const double avba_information_ratio =
      ratio(fresh_avba.weakest, production_avba.weakest);
  const double avba_information_drop_log10 =
      std::isfinite(previousPreCommitAuditAvbaInformationRatio) &&
              previousPreCommitAuditAvbaInformationRatio > 0.0 &&
              avba_information_ratio > 0.0
          ? std::max(0.0, std::log10(
                              previousPreCommitAuditAvbaInformationRatio /
                              avba_information_ratio))
          : 0.0;
  previousPreCommitAuditAvbaInformationRatio = avba_information_ratio;
  lastPreCommitAuditSuspect = false;
  const char *drop_limit_value =
      std::getenv("VINS_PRECOMMIT_AUDIT_MAX_INFORMATION_DROP_LOG10");
  if (drop_limit_value && *drop_limit_value) {
    std::size_t consumed = 0;
    const double drop_limit = std::stod(drop_limit_value, &consumed);
    if (consumed != std::strlen(drop_limit_value) ||
        !std::isfinite(drop_limit) || drop_limit <= 0.0) {
      throw std::runtime_error("invalid pre-commit information-drop limit");
    }
    lastPreCommitAuditSuspect =
        avba_information_drop_log10 > drop_limit;
  }
  output << std::setprecision(17)
         << production.timestamp << ",old," << production_prior.n << ','
         << fresh_prior.n << ','
         << production_candidate.prior_parameter_blocks.size() << ','
         << fresh_candidate.prior_parameter_blocks.size() << ','
         << production_common_coordinates.size() << ','
         << production_depth_count << ',' << production.accel_bias.norm()
         << ',' << fresh.accel_bias.norm() << ','
         << (production.accel_bias - entry.accel_bias).norm() << ','
         << (fresh.accel_bias - entry.accel_bias).norm() << ','
         << tilt_delta_deg(entry, production) << ','
         << tilt_delta_deg(entry, fresh) << ','
         << body_velocity_delta(entry, production) << ','
         << body_velocity_delta(entry, fresh) << ','
         << inverse_depth_abs_delta_mean << ','
         << inverse_depth_delta_variance << ','
         << production_avba.weakest << ',' << fresh_avba.weakest << ','
         << alignment(production_avba, fresh_avba) << ','
         << production_full.weakest << ',' << fresh_full.weakest << ','
         << (production_hessian.rows() == fresh_hessian.rows()
                 ? alignment(production_full, fresh_full)
                 : std::numeric_limits<double>::quiet_NaN()) << ','
         << avba_information_ratio << ','
         << ratio(fresh_full.weakest, production_full.weakest) << '\n';
  VINS_INFO << "GT-free pre-commit consistency audit: timestamp="
            << production.timestamp
            << ", production_ba_norm=" << production.accel_bias.norm()
            << ", fresh_ba_norm=" << fresh.accel_bias.norm()
            << ", inverse_depth_abs_delta_mean="
            << inverse_depth_abs_delta_mean
            << ", avba_mode_alignment="
            << alignment(production_avba, fresh_avba)
            << ", avba_information_ratio="
            << avba_information_ratio
            << ", avba_information_drop_log10="
            << avba_information_drop_log10
            << ", suspect=" << lastPreCommitAuditSuspect;
}

bool Estimator::optimize(int max_iterations_override) {
  prepareParameters();
  if (!solveOptimization(max_iterations_override)) return false;
  std::memcpy(lastSolvedPoseArray, poseArray, sizeof(lastSolvedPoseArray));
  std::memcpy(lastSolvedSpeedBiasArray, speedBiasArray,
              sizeof(lastSolvedSpeedBiasArray));
  std::memcpy(lastSolvedFeatureArray, para_Feature,
              sizeof(lastSolvedFeatureArray));
  std::memcpy(lastSolvedExtrinsicArray, para_Ex_Pose,
              sizeof(lastSolvedExtrinsicArray));
  lastSolvedTimeDelayArray[0] = para_Td[0][0];
  lastSolvedFeatureCount = featureManager.getFeatureCount();
  lastSolvedParametersValid = true;
  updateEstimates();
  if (options->hasGpsDropoutGyroBiasPrior() &&
      metricPositionFactorActiveThisSolve &&
      std::isfinite(estimator_state[WINDOW_SIZE].gyro_bias.z())) {
    gpsDropoutGyroBiasAnchorInitialized = true;
    gpsDropoutGyroBiasAnchorZ =
        estimator_state[WINDOW_SIZE].gyro_bias.z();
  }
  if (!validateEstimatorState("post optimization")) return false;
  if (solver_flag == SolverState::NON_LINEAR) ++nonLinearUpdateCount;
  if (frameCount < WINDOW_SIZE) return true;
  if (isNewMarginalization()) {
    processNewMarginalization();
  } else {
    processOldMarginalization();
  }
  return true;
}

void Estimator::slideWindow() {
  if (!isNewMarginalization()) {
    double t_0 = estimator_state[0].timestamp;
    back_state = estimator_state[0];
    slideWindowOld();
  } else {
    slideWindowNew();
  }
}

void Estimator::slideWindowNew() {
  if (frameCount != WINDOW_SIZE) {
    return;
  }
  estimator_state[frameCount - 1] = estimator_state[frameCount];
  rangeMeasurements[frameCount - 1] = rangeMeasurements[frameCount];
  metricVelocityMeasurements[frameCount - 1] =
      metricVelocityMeasurements[frameCount];
  rotorDynamicsMeasurements[frameCount - 1] =
      rotorDynamicsMeasurements[frameCount];
  if (options->hasImu()) {
    for (unsigned int i = 0; i < deltaTimeImuBuffer[frameCount].size(); i++) {
      const auto &imu = deltaTimeImuBuffer[frameCount][i];
      pre_integrations[frameCount - 1]->push_back(imu);
      deltaTimeImuBuffer[frameCount - 1].push_back(imu);
    }
    pre_integrations[WINDOW_SIZE] = nullptr;
    pre_integrations[WINDOW_SIZE] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[WINDOW_SIZE].accel_bias,
        estimator_state[WINDOW_SIZE].gyro_bias, options->imu);

    deltaTimeImuBuffer[WINDOW_SIZE].clear();
  }
  frontCount++;
  featureManager.removeFront(frameCount);
}

void Estimator::slideWindowOld() {
  if (frameCount != WINDOW_SIZE) {
    return;
  }
  for (int i = 0; i < WINDOW_SIZE; i++) {
    estimator_state[i].timestamp = estimator_state[i + 1].timestamp;
    estimator_state[i].swap(estimator_state[i + 1]);
    rangeMeasurements[i] = rangeMeasurements[i + 1];
    metricVelocityMeasurements[i] = metricVelocityMeasurements[i + 1];
    rotorDynamicsMeasurements[i] = rotorDynamicsMeasurements[i + 1];
    if (options->hasImu()) {
      std::swap(pre_integrations[i], pre_integrations[i + 1]);
      deltaTimeImuBuffer[i].swap(deltaTimeImuBuffer[i + 1]);
    }
  }
  estimator_state[WINDOW_SIZE] = estimator_state[WINDOW_SIZE - 1];
  rangeMeasurements[WINDOW_SIZE] = rangeMeasurements[WINDOW_SIZE - 1];
  metricVelocityMeasurements[WINDOW_SIZE] =
      metricVelocityMeasurements[WINDOW_SIZE - 1];
  rotorDynamicsMeasurements[WINDOW_SIZE] =
      rotorDynamicsMeasurements[WINDOW_SIZE - 1];
  if (options->hasImu()) {
    pre_integrations[WINDOW_SIZE] = nullptr;
    pre_integrations[WINDOW_SIZE] = std::make_shared<IntegrationBase>(
        previousImuData, estimator_state[WINDOW_SIZE].accel_bias,
        estimator_state[WINDOW_SIZE].gyro_bias, options->imu);
    deltaTimeImuBuffer[WINDOW_SIZE].clear();
  }
  auto it_0 = all_image_frame.find(estimator_state[0].timestamp);
  all_image_frame.erase(all_image_frame.begin(), it_0);

  backCount++;
  if (isNonLinearSolver()) {
    Matrix3d R0, R1;
    Vector3d P0, P1;
    R0 = back_state.rotation * cameraRotation[0];
    R1 = estimator_state[0].rotation * cameraRotation[0];
    P0 = back_state.position + back_state.rotation * cameraTranslation[0];
    P1 = estimator_state[0].position +
         estimator_state[0].rotation * cameraTranslation[0];
    featureManager.removeBackShiftDepth(R0, P0, R1, P1);
  } else {
    featureManager.removeBack();
  }
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = estimator_state[frameCount].rotation;
  T.block<3, 1>(0, 3) = estimator_state[frameCount].position;
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T) {
  T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = estimator_state[index].rotation;
  T.block<3, 1>(0, 3) = estimator_state[index].position;
}

bool Estimator::getIntegratedImuOdom(OdomData &data) {
  if (!safe_imu_pre_odom.check()) {
    return false;
  }
  safe_imu_pre_odom.get(data);
  return true;
}
bool Estimator::getVisualInertialOdom(OdomData &data) {
  if (!safe_vio_odom.check()) {
    return false;
  }
  safe_vio_odom.get(data);
  return true;
}

bool Estimator::getKeyPoses(PoseSequenceData &poses) {
  if (!safe_key_poses.check()) {
    return false;
  }
  safe_key_poses.get(poses);
  if (poses.poses.size() < 1) {
    return false;
  }
  return true;
}

bool Estimator::getCameraPose(int index, PoseData &data) {
  safe_camera_pose[index].get(data, true);
  return true;
}

bool Estimator::getTrackImage(ImageData &image) {
  if (!safe_track_image.check()) {
    return false;
  }
  safe_track_image.get(image);
  return true;
}

bool Estimator::getMainCloud(PointCloudData &data) {
  if (!safe_main_cloud.check()) {
    return false;
  }
  safe_main_cloud.get(data);
  return true;
}
bool Estimator::getMarginCloud(PointCloudData &data) {
  if (!safe_margin_cloud.check()) {
    return false;
  }
  safe_margin_cloud.get(data);
  return true;
}
bool Estimator::getkeyframeCloud(PointCloudData &data) {
  if (!safe_point_cloud.check()) {
    return false;
  }
  safe_point_cloud.get(data);
  return true;
}

bool Estimator::getkeyframePose(PoseData &data) {
  if (!safe_keyframe_pose.check()) {
    return false;
  }
  safe_keyframe_pose.get(data);
  return true;
}

void Estimator::predictPtsInNextFrame() {
  if (frameCount < 2) return;
  Eigen::Matrix4d curT, prevT, nextT;
  getPoseInWorldFrame(curT);
  getPoseInWorldFrame(frameCount - 1, prevT);
  nextT = curT * (prevT.inverse() * curT);
  map<int, Eigen::Vector3d> predictPts;

  for (auto &it_per_id : featureManager.feature) {
    if (it_per_id.estimated_depth > 0) {
      int firstIndex = it_per_id.start_frame;
      int lastIndex =
          it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
      if ((int)it_per_id.feature_per_frame.size() >= 2 &&
          lastIndex == frameCount) {
        double depth = it_per_id.estimated_depth;
        Vector3d pts_j =
            cameraRotation[0] * (depth * it_per_id.feature_per_frame[0].point) +
            cameraTranslation[0];
        Vector3d pts_w = estimator_state[firstIndex].rotation * pts_j +
                         estimator_state[firstIndex].position;
        Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() *
                             (pts_w - nextT.block<3, 1>(0, 3));
        Vector3d pts_cam =
            cameraRotation[0].transpose() * (pts_local - cameraTranslation[0]);
        int ptsIndex = it_per_id.feature_id;
        predictPts[ptsIndex] = pts_cam;
      }
    }
  }
  featureTracker.setPrediction(predictPts);
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                                    Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                                    Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi,
                                    Vector3d &uvj) {
  Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
  Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
  Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
  double rx = residual.x();
  double ry = residual.y();
  return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex) {
  int feature_index = -1;
  for (auto &it_per_id : featureManager.feature) {
    double err = 0;
    int errCnt = 0;
    it_per_id.used_num = it_per_id.feature_per_frame.size();
    if (it_per_id.used_num < kMinFeatureTrackFrames) continue;
    feature_index++;
    int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
    double depth = it_per_id.estimated_depth;
    for (auto &it_per_frame : it_per_id.feature_per_frame) {
      imu_j++;
      if (imu_i != imu_j) {
        Vector3d pts_j = it_per_frame.point;
        double tmp_error = reprojectionError(
            estimator_state[imu_i].rotation, estimator_state[imu_i].position,
            cameraRotation[0], cameraTranslation[0],
            estimator_state[imu_j].rotation, estimator_state[imu_j].position,
            cameraRotation[0], cameraTranslation[0], depth, pts_i, pts_j);
        err += tmp_error;
        errCnt++;
      }
      // need to rewrite projecton factor.........
      if (options->isUsingStereo() && it_per_frame.is_stereo) {
        Vector3d pts_j_right = it_per_frame.pointRight;
        if (imu_i != imu_j) {
          double tmp_error = reprojectionError(
              estimator_state[imu_i].rotation, estimator_state[imu_i].position,
              cameraRotation[0], cameraTranslation[0],
              estimator_state[imu_j].rotation, estimator_state[imu_j].position,
              cameraRotation[1], cameraTranslation[1], depth, pts_i,
              pts_j_right);
          err += tmp_error;
          errCnt++;
        } else {
          double tmp_error = reprojectionError(
              estimator_state[imu_i].rotation, estimator_state[imu_i].position,
              cameraRotation[0], cameraTranslation[0],
              estimator_state[imu_j].rotation, estimator_state[imu_j].position,
              cameraRotation[1], cameraTranslation[1], depth, pts_i,
              pts_j_right);
          err += tmp_error;
          errCnt++;
        }
      }
    }
    double ave_err = err / errCnt;
    if (ave_err * FOCAL_LENGTH > options->outlier_rejection_threshold_px)
      removeIndex.insert(it_per_id.feature_id);
  }
}

void Estimator::predictWarmReinitialization(const IMUData &data) {
  if (!warmReinitializationBridge.propagate(data)) {
    if (warmReinitializationBridge.expired()) {
      VINS_ERROR << "Warm reinitialization IMU bridge expired before a "
                    "bounded visual handover.";
    } else {
      VINS_ERROR << "Warm reinitialization IMU bridge rejected propagation "
                    "at timestamp="
                 << data.timestamp;
    }
    return;
  }
  OdomData propagated;
  if (!warmReinitializationBridge.latest(&propagated)) return;
  safe_imu_pre_odom.set(propagated);
  // This is the same IMU-propagated metric epoch, not a new visual estimate.
  // Publishing it on the normal stream keeps receipt-time continuity while
  // the visual backend remains withheld.
  safe_vio_odom.set(propagated);
}

void Estimator::fastPredictIMU(const IMUData &data) {
  std::lock_guard<std::mutex> lock(propagateMutex);
  fastPredictIMULocked(data);
}

void Estimator::fastPredictIMULocked(const IMUData &data) {
  // inputIMU may have observed NON_LINEAR immediately before another thread
  // reset the estimator.  Re-check after acquiring the shared reset lock.
  if (solver_flag != SolverState::NON_LINEAR) return;
  double dt = data.timestamp - latestImuData.timestamp;
  // The same sample can already have been consumed from imuBuffer by
  // updateLatestStates() while its ROS callback was waiting for
  // processingMutex.  Never integrate it twice or rewind the published
  // propagation state.
  if (dt <= 0.0) return;
  if (dt > 1.0) {
    VINS_ERROR << "Abnormal IMU timestamp jump detected: " << dt;
    return;
  }
  Eigen::Vector3d un_acc_0 =
      latest_Q * (latestImuData.linear_acceleration - latest_state.accel_bias) -
      gravity;
  Eigen::Vector3d un_gyr =
      0.5 * (latestImuData.angular_velocity + data.angular_velocity) -
      latest_state.gyro_bias;
  latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
  Eigen::Vector3d un_acc_1 =
      latest_Q * (data.linear_acceleration - latest_state.accel_bias) - gravity;
  Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
  latest_state.position = latest_state.position + dt * latest_state.velocity +
                          0.5 * dt * dt * un_acc;
  latest_state.velocity = latest_state.velocity + dt * un_acc;

  latestImuData = data;
  imu_odom.timestamp = data.timestamp;
  imu_odom.position = latest_state.position;
  imu_odom.velocity = latest_state.velocity;
  imu_odom.orientation = latest_Q;
  if (!backendCandidateWorkspaceActive) {
    safe_imu_pre_odom.set(imu_odom);
    // The primary D435i output is the current stereo-optimized state with its
    // IMU suffix applied, not the lower-rate camera-keyframe snapshot.
    vio_odom = imu_odom;
    safe_vio_odom.set(vio_odom);
  }
}

void Estimator::updateLatestStates() {
  // Reset the propagation root and replay its buffered IMU suffix as one
  // atomic operation. Live IMU callbacks wait only for this short handoff,
  // not for the complete camera optimization.
  std::lock_guard<std::mutex> propagation_lock(propagateMutex);
  latest_state = estimator_state[frameCount];
  latest_Q = latest_state.rotation;
  latestImuData = previousImuData;
  latestImuData.timestamp =
      estimator_state[frameCount].timestamp + options->time_delay;

  queue<IMUData> tmp_imu;
  {
    std::lock_guard<std::mutex> imu_lock(imu_mutex);
    tmp_imu = imuBuffer;
  }
  while (!tmp_imu.empty()) {
    fastPredictIMULocked(tmp_imu.front());
    tmp_imu.pop();
  }
}

void Estimator::publishCurrentBackendOutputs(Timestamp timestamp) {
  if (backendCandidateWorkspaceActive ||
      solver_flag != SolverState::NON_LINEAR) {
    return;
  }
  for (int i = 0; i < options->getNumCameras(); ++i) {
    updateCameraPose(i);
  }
  constexpr int kInitializationProbationUpdates = 3;
  if (nonLinearUpdateCount < kInitializationProbationUpdates) return;

  key_poses.timestamp = timestamp;
  key_poses.poses.clear();
  for (int i = 0; i <= WINDOW_SIZE; ++i) {
    key_poses.poses.push_back(estimator_state[i].position);
  }
  safe_key_poses.set(key_poses);

  vio_odom.timestamp = latestImuData.timestamp;
  vio_odom.position = latest_state.position;
  vio_odom.orientation = latest_Q;
  vio_odom.velocity = latest_state.velocity;
  safe_vio_odom.set(vio_odom);

  imu_odom.timestamp = latestImuData.timestamp;
  imu_odom.position = latest_state.position;
  imu_odom.velocity = latest_state.velocity;
  imu_odom.orientation = latest_Q;
  safe_imu_pre_odom.set(imu_odom);
}
