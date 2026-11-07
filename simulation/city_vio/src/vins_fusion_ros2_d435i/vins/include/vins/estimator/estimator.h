/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ceres/ceres.h>
#include <vins/common/sensor_data_type.h>
#include <vins/estimator/backend_approval_gate.h>
#include <vins/estimator/backend_candidate_input_buffer.h>
#include <vins/estimator/backend_candidate_store.h>
#include <vins/estimator/backend_health_metrics.h>
#include <vins/estimator/backend_recovery_candidate_selector.h>
#include <vins/estimator/backend_recovery_checkpoint_store.h>
#include <vins/estimator/backend_state.h>
#include <vins/estimator/feature_manager.h>
#include <vins/estimator/motion_onset_warm_reinitialization_gate.h>
#include <vins/estimator/parameters.h>
#include <vins/estimator/warm_reinitialization_bridge.h>
#include <vins/factor/imu_factor.h>
#include <vins/factor/marginalization_factor.h>
#include <vins/factor/pose_local_parameterization.h>
#include <vins/factor/projectionOneFrameTwoCamFactor.h>
#include <vins/factor/projectionTwoFrameOneCamFactor.h>
#include <vins/factor/projectionTwoFrameTwoCamFactor.h>
#include <vins/factor/range_factor.h>
#include <vins/factor/horizontal_velocity_factor.h>
#include <vins/factor/horizontal_displacement_factor.h>
#include <vins/featureTracker/feature_tracker.h>
#include <vins/initial/initial_alignment.h>
#include <vins/initial/initial_ex_rotation.h>
#include <vins/initial/initial_sfm.h>
#include <vins/initial/solve_5pts.h>
#include <vins/logger/logger.h>
#include <vins/utility/tic_toc.h>
#include <vins/utility/utility.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <functional>
#include <opencv2/core/eigen.hpp>
#include <queue>
#include <thread>

using namespace std;

class Estimator {
 public:
  Estimator();
  ~Estimator();
  //
  void initialize(std::shared_ptr<VINSOptions> options_);
  // interface
  void inputIMU(const IMUData &imu);
  void inputRange(const RangeData &range);
  void inputMetricVelocity(const MetricVelocityData &velocity);
  void inputRotorDynamics(const RotorDynamicsData &dynamics);
  void inputStationaryHint(bool stationary);
  void inputReplayGroundTruth(const OdomData &ground_truth);
  void inputFeature(Timestamp timestamp, const FeatureFrame &featureFrame);
  void inputImage(const ImageData &image);
  //
  void processMeasurements();
  //
  void resetState();
  void restart();
  void setInputDownsamplePhase(int phase);
  void setReplayPositionJumpQuarantine(bool enabled);
  void setReplayJumpFallbackIterations(int iterations);
  void setReplayJumpTransactionReject(bool enabled);
  void setReplayJumpPriorReset(bool enabled);
  void setReplayRejectPersistentInnovationTimestamp(double timestamp);
  void setReplayRejectAcceptedUpdateTimestamp(double timestamp);
  void setReplayVelocityTraceWindow(double start_timestamp,
                                    double end_timestamp);
  void setReplayBackendHealthTraceWindow(double start_timestamp,
                                         double end_timestamp);
  void setReplayDualStateBackend(bool enabled,
                                 int required_healthy_updates);
  void setReplayMultiCheckpointRecovery(bool enabled);
  void setReplayWarmReinitialization(bool enabled);
  void setReplayMotionOnsetWarmReinitialization(bool enabled);
  void setReplayPriorScaleProjection(bool enabled, double start_timestamp,
                                     double minimum_directional_gradient,
                                     int maximum_projection_count);
  void setReplaySkipMarginalizationPriorTimestamp(double timestamp);
  void setReplayIncludeLongImuFactors(bool enabled);
  void setReplayMaximumImuWindowInterval(double maximum_seconds);
  void setReplayGtAccelBiasZeroOracle(bool enabled);
  void setReplayGtInitialVelocityZeroOracle(bool enabled);
  void setDeterministicCommitOutputCallback(
      std::function<void(const OdomData &)> callback);
  void setFirstPose(const Eigen::Vector3d &position,
                    const Eigen::Matrix3d &rotation);

  void getPoseInWorldFrame(Eigen::Matrix4d &T);
  void getPoseInWorldFrame(int index, Eigen::Matrix4d &T);

  bool getIntegratedImuOdom(OdomData &data);
  bool getVisualInertialOdom(OdomData &data);
  bool getKeyPoses(PoseSequenceData &poses);
  bool getTrackImage(ImageData &image);

  bool getCameraPose(int index, PoseData &data);

  bool getMainCloud(PointCloudData &data);
  bool getMarginCloud(PointCloudData &data);
  bool getkeyframeCloud(PointCloudData &data);
  bool getkeyframePose(PoseData &data);

 private:
  friend class EstimatorBackendStateTestAccess;
  void resetStateLocked(bool request_warm_reinitialization = false);
  void initializeCamerasFromOptions();
  void processIMU(const IMUData &data, double deltaTime);
  void updateStateWithIMU(const IMUData &data, double deltaTime);
  void processImage(const FeatureFrame &features, Timestamp timestamp);
  bool maybeRebuildContinuousWindowAtMotionOnset(
      const FeatureFrame &features, Timestamp timestamp);
  bool rebuildContinuousWindow(
      const std::vector<TimestampedFeatureFrame> &frames,
      const std::vector<StateData> &states);
  void recordMotionOnsetFrameHistory(
      const TimestampedFeatureFrame &frame);
  void fastPredictIMU(const IMUData &data);
  void fastPredictIMULocked(const IMUData &data);
  void predictWarmReinitialization(const IMUData &data);
  void publishCurrentBackendOutputs(Timestamp timestamp);
  bool projectCurrentMarginalizationScaleDirection();

  bool isNonLinearSolver() const;
  bool isNewMarginalization() const;
  void setMarginalizationFlag(const FeatureFrame &features);
  void insertImageFrame(const FeatureFrame &features, Timestamp timestamp);
  void handleExtrinsicInitialization();
  void processInitialization(Timestamp timestamp);
  void processNonLinearSolver(Timestamp timestamp);
  void processMonoWithImuInitialization(Timestamp timestamp);
  void processStereoWithImuInitialization();
  void processStereoWithoutImuInitialization();

  // internal
  bool initialStructure();
  bool checkIMUExcitation();
  void logInitializationDiagnostics(Timestamp timestamp) const;
  std::vector<SFMFeature> buildSFMFeatures();
  bool solvePoseWithPnP(const std::vector<Quaterniond> &rotations,
                        const std::vector<Vector3d> &positions,
                        const std::map<int, Vector3d> &sfmPoints);
  bool visualInitialAlign();
  bool computeRelativePose(Matrix3d &relative_R, Vector3d &relative_T,
                           int &referenceFrame);
  void slideWindow();
  void slideWindowNew();
  void slideWindowOld();
  bool optimize(int max_iterations_override = 0);
  bool runPreCommitConsistencyAudit(
      const vins::BackendState &solve_entry,
      const vins::BackendState &production_candidate,
      std::string *error = nullptr);
  void exportPreCommitConsistencyAudit(
      const vins::BackendState &solve_entry,
      const vins::BackendState &production_candidate,
      const vins::BackendState &fresh_candidate);
  bool validateEstimatorState(const char *context) const;
  bool validateInitializationHealth() const;
  void prepareParameters();
  void updateEstimates();
  bool failureDetection(bool trusted_horizontal_metric_correction);
  bool getIMUInterval(double t0, double t1, vector<IMUData> &data);
  void predictPtsInNextFrame();
  void outliersRejection(set<int> &removeIndex);
  double reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici,
                           Vector3d &tici, Matrix3d &Rj, Vector3d &Pj,
                           Matrix3d &ricj, Vector3d &ticj, double depth,
                           Vector3d &uvi, Vector3d &uvj);
  void updateLatestStates();
  bool IMUAvailable(double t);
  void initFirstIMUPose(const vector<IMUData> &data);
  std::unique_ptr<vins::BackendState> captureBackendState(
      std::string *error = nullptr) const;
  bool applyBackendState(const vins::BackendState &state,
                         std::string *error = nullptr);
  bool evaluateRecoveryCandidate(
      const vins::BackendState &checkpoint_backend,
      const FeatureTracker &checkpoint_tracker,
      const vins::BackendRecoverySensorSlice &sensor_slice,
      int downsample_phase,
      std::uint64_t anomaly_corridor_end_raw_sequence,
      const StateData &published_state,
      vins::BackendRecoveryCandidateEvidence *evidence,
      vins::BackendState::Ptr *recovered_backend,
      std::unique_ptr<FeatureTracker> *recovered_tracker,
      std::string *error = nullptr);

 private:
  void AddPoseParameterBlocks(ceres::Problem &problem);
  void AddExtrinsicParameterBlocks(ceres::Problem &problem);
  void AddTimeDelayParameterBlock(ceres::Problem &problem);
  void AddMarginalizationFactor(ceres::Problem &problem);
  void AddIMUFactors(ceres::Problem &problem);
  void AddFeatureFactors(ceres::Problem &problem);
  void AddPoseOnlyVisualFactors(ceres::Problem &problem);
  bool solveOptimization(int max_iterations_override = 0);
  void AddRangeFactors(ceres::Problem &problem);
  void AddMetricVelocityFactors(ceres::Problem &problem);
  void AddGpsDropoutSpeedFactors(ceres::Problem &problem);
  void AddGpsDropoutGyroBiasPriorFactors(ceres::Problem &problem);
  void AddRotorDragDynamicsFactors(ceres::Problem &problem);

  void processOldMarginalization();
  void processNewMarginalization();
  void exportMarginalizationTransport(
      const std::shared_ptr<MarginalizationInfo> &marg_info,
      std::size_t prior_factor_count, std::size_t imu_factor_count,
      const char *marginalization_type);
  std::shared_ptr<ResidualBlockInfo> createIMUResidualBlock();
  void addFeatureResidualBlocks(
      std::shared_ptr<MarginalizationInfo> &marg_info);
  std::unordered_map<long, double *> createAddrShift(bool is_old);

  void updateCameraPose(int index);
  void collectPointCloudAll(Timestamp timestamp);
  void printStatistics(Timestamp timestamp);
  void emitDeterministicCommitOutput();
  template <typename Container>
  void clearBuffer(Container &container) {
    container.clear();
  }
  template <typename T>
  void clearBuffer(std::queue<T> &queue) {
    std::queue<T> empty;
    std::swap(queue, empty);
  }

  std::thread processThread;
  std::mutex processingMutex;
  std::mutex featureBufferMutex;
  std::mutex featureTrackerMutex;
  std::condition_variable featureCondition;
  std::mutex propagateMutex;
  std::mutex deterministicCommitOutputMutex;
  std::function<void(const OdomData &)> deterministicCommitOutputCallback;
  std::mutex imu_mutex;
  std::condition_variable imuCondition;
  //
  std::queue<IMUData> imuBuffer;
  std::queue<RangeData> rangeBuffer;
  std::mutex rangeMutex;
  std::queue<MetricVelocityData> metricVelocityBuffer;
  std::mutex metricVelocityMutex;
  std::queue<RotorDynamicsData> rotorDynamicsBuffer;
  std::mutex rotorDynamicsMutex;
  queue<TimestampedFeatureFrame> featureBuffer;
  RangeData rangeMeasurements[WINDOW_SIZE + 1];
  MetricVelocityData metricVelocityMeasurements[WINDOW_SIZE + 1];
  RotorDynamicsData rotorDynamicsMeasurements[WINDOW_SIZE + 1];
  bool rotorDragDynamicsFactorActivationLogged = false;
  bool poseOnlyVisualFactorActivationLogged = false;
  bool metricVelocityFactorActivationLogged = false;
  bool gpsDropoutSpeedFactorActivationLogged = false;
  bool gpsDropoutGyroBiasPriorActivationLogged = false;
  bool metricPositionFactorActiveThisSolve = false;
  bool metricDisplacementAnchorInitialized = false;
  uint64_t metricDisplacementAnchorSegment = 0;
  Eigen::Vector2d metricDisplacementAnchorMeasurement =
      Eigen::Vector2d::Zero();
  Eigen::Vector2d metricDisplacementAnchorPosition =
      Eigen::Vector2d::Zero();
  bool gpsDropoutGyroBiasAnchorInitialized = false;
  double gpsDropoutGyroBiasAnchorZ = 0.0;
  int nonLinearUpdateCount = 0;
  bool replayPositionJumpQuarantine = false;
  int replayJumpFallbackIterations = 0;
  bool replayJumpTransactionReject = false;
  bool replayJumpTransactionRejectConsumed = false;
  bool replayJumpPriorReset = false;
  double replayRejectPersistentInnovationTimestamp = -1.0;
  bool replayRejectPersistentInnovationConsumed = false;
  double replayRejectAcceptedUpdateTimestamp = -1.0;
  bool replayRejectAcceptedUpdateConsumed = false;
  double replayVelocityTraceStartTimestamp = -1.0;
  double replayVelocityTraceEndTimestamp = -1.0;
  double replayBackendHealthTraceStartTimestamp = -1.0;
  double replayBackendHealthTraceEndTimestamp = -1.0;
  bool replayDualStateBackend = false;
  int replayDualStateRequiredHealthyUpdates = 3;
  bool replayMultiCheckpointRecovery = false;
  bool replayWarmReinitialization = false;
  bool replayMotionOnsetWarmReinitialization = false;
  bool replayMotionOnsetWarmReinitializationConsumed = false;
  bool motionOnsetWarmReinitializationAppliedThisSolve = false;
  bool motionOnsetWarmReinitializationTriggeredThisSolve = false;
  bool motionOnsetBiasAnchorThisSolve = false;
  bool motionOnsetBiasAnchorActive = false;
  bool motionOnsetReleasePending = false;
  bool motionOnsetReleaseThisSolve = false;
  int motionOnsetBiasAnchorSolveCount = 0;
  int motionOnsetExcitingMotionCount = 0;
  bool replayPriorScaleProjection = false;
  double replaySkipMarginalizationPriorTimestamp = -1.0;
  bool replaySkipMarginalizationPriorConsumed = false;
  bool replayIncludeLongImuFactors = false;
  double replayMaximumImuWindowInterval = -1.0;
  bool replayGtAccelBiasZeroOracle = false;
  bool replayGtInitialVelocityZeroOracle = false;
  bool preCommitConsistencyAuditShadowActive = false;
  bool auditTriggeredHistoryExclusionActive = false;
  bool auditTriggeredHistoryExclusionConsumed = false;
  bool auditTriggeredDropAllVisualActive = false;
  bool auditTriggeredTransactionRejectConsumed = false;
  bool auditTriggeredQuarantineRejectConsumed = false;
  bool auditTriggeredPriorInformationCapConsumed = false;
  bool auditTriggeredRecentHistoryRetryConsumed = false;
  bool auditTriggeredRelinearizedHistoryRetryConsumed = false;
  int auditTriggeredMarginOldDelayCount = 0;
  bool lastPreCommitAuditSuspect = false;
  double previousPreCommitAuditAvbaInformationRatio =
      std::numeric_limits<double>::quiet_NaN();
  double replayPriorScaleProjectionStartTimestamp = -1.0;
  double replayPriorScaleProjectionMinimumDirectionalGradient =
      -std::numeric_limits<double>::infinity();
  int replayPriorScaleProjectionMaximumCount = 0;
  int replayPriorScaleProjectionCount = 0;
  bool backendCandidateWorkspaceActive = false;
  bool backendCandidateProcessingFailed = false;
  bool lastBackendHealthAvailable = false;
  bool lastBackendSolverUsable = false;
  bool lastBackendTransactionRejected = false;
  vins::BackendHealthMetrics lastBackendHealthMetrics;
  std::unique_ptr<vins::BackendCandidateStore> backendCandidateStore;
  vins::BackendCandidateInputBuffer backendCandidateInputBuffer;
  vins::BackendApprovalGate backendApprovalGate;
  std::mutex backendRecoveryMutex;
  vins::BackendRecoveryCheckpointStore backendRecoveryCheckpoints{8};
  vins::BackendRecoverySensorRing backendRecoverySensorRing;
  vins::WarmReinitializationBridge warmReinitializationBridge;
  struct MotionOnsetFrameHistory {
    TimestampedFeatureFrame frame;
    StateData state;
    RangeData range;
    MetricVelocityData metric_velocity;
    RotorDynamicsData rotor_dynamics;
    double median_flow_px_s = std::numeric_limits<double>::infinity();
    int visual_tracks = 0;
  };
  std::deque<MotionOnsetFrameHistory> motionOnsetFrameHistory;
  std::deque<IMUData> motionOnsetImuHistory;
  std::mutex motionOnsetImuHistoryMutex;
  std::deque<OdomData> replayGroundTruthHistory;
  std::mutex replayGroundTruthMutex;
  bool replayGtInterventionApplied = false;
  std::map<std::uint64_t, std::shared_ptr<const FeatureTracker>>
      backendFeatureTrackerSnapshots;
  std::map<std::uint64_t, std::uint64_t>
      backendFeatureRawImageSequences;
  std::uint64_t lastBackendRecoveryCheckpointSequence = 0;
  std::uint64_t backendRecoveryFirstSuspectSequence = 0;
  std::uint64_t backendRecoveryFirstSuspectRawImageSequence = 0;
  bool backendRecoverySelectionLogged = false;
  bool backendRecoveryEvaluationComplete = false;
  double lastMarginalizationSourceTimestamp =
      std::numeric_limits<double>::quiet_NaN();
  std::string lastMarginalizationSourceType = "none";
  std::uint64_t backendGeneration = 0;
  int replayQuarantineUpdatesRemaining = 0;
  int replayQuarantineConsecutiveHealthy = 0;
  bool rangeFactorActivationLogged = false;
  bool rangeGroundInitialized = false;
  double rangeGroundZ = 0.0;
  // Retained across estimator resets.  A new mono-SfM reconstruction has an
  // arbitrary unit, but an extreme scale change between successful epochs of
  // the same in-flight camera is a strong failed-initialization signal.
  double lastPublishedAlignmentScale = -1.0;
  double currentAlignmentScale = -1.0;
  double currentFeatureFlowMedianPxS = std::numeric_limits<double>::infinity();
  int currentFeatureFlowCount = 0;
  int stationaryVisualUpdateCount = 0;
  int consecutivePositionInnovationRejects = 0;
  double lastOptimizationInitialCost = 0.0;
  double lastOptimizationFinalCost = 0.0;
  double lastOptimizationTimeSeconds = 0.0;
  int lastOptimizationSuccessfulSteps = 0;
  int lastOptimizationUnsuccessfulSteps = 0;
  int lastOptimizationResidualBlocks = 0;
  int lastOptimizationResiduals = 0;
  ceres::TerminationType lastOptimizationTermination = ceres::FAILURE;

  Timestamp previousTimestamp = -1;
  Timestamp currentTimestamp = 0;
  Timestamp initialTimestamp = 0;

  bool openExEstimation;
  SolverState solver_flag;
  MarginalizationType marginalization_flag;
  Vector3d gravity;
  Matrix3d cameraRotation[2];
  Vector3d cameraTranslation[2];
  StateData estimator_state[WINDOW_SIZE + 1];
  StateData back_state, last_state, last_state0;
  StateData latest_state;
  Eigen::Quaterniond latest_Q;

  IntegrationBase::Ptr pre_integrations[(WINDOW_SIZE + 1)];
  IMUData previousImuData;
  std::vector<IMUData> deltaTimeImuBuffer[(WINDOW_SIZE + 1)];

  // 计数变量
  std::atomic<int> inputImageCount{0};
  std::atomic<bool> stationaryHint{false};
  std::atomic<std::uint64_t> featureInputSequence{0};
  int inputDownsamplePhase = 0;
  double lastEnqueuedImageTimestamp =
      -std::numeric_limits<double>::infinity();
  int frameCount = 0;
  int backCount = 0;
  int frontCount = 0;

  FeatureManager featureManager;
  FeatureTracker featureTracker;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  bool isFirstIMUReceived;
  bool failure_occur;

  //
  SafeClass<PointCloudData> safe_main_cloud;
  SafeClass<PointCloudData> safe_point_cloud;
  SafeClass<PointCloudData> safe_margin_cloud;
  SafeClass<PoseData> safe_camera_pose[2];
  SafeClass<PoseData> safe_keyframe_pose;
  SafeClass<PoseSequenceData> safe_key_poses;
  PoseSequenceData key_poses;
  ImageData track_image;
  SafeClass<ImageData> safe_track_image;
  SafeClass<OdomData> safe_imu_pre_odom;
  OdomData imu_odom;
  SafeClass<OdomData> safe_vio_odom;
  OdomData vio_odom;

  double poseArray[WINDOW_SIZE + 1][SIZE_POSE];
  double speedBiasArray[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_FEATURE];
  double para_Ex_Pose[2][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];
  // Exact Ceres coordinates immediately after the latest solve, before
  // updateEstimates() applies yaw normalization and marginalization prepares
  // the next problem.  Replay diagnostics use these copies only for
  // side-effect-free factor-cost evaluation.
  double lastSolvedPoseArray[WINDOW_SIZE + 1][SIZE_POSE];
  double lastSolvedSpeedBiasArray[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double lastSolvedFeatureArray[NUM_OF_F][SIZE_FEATURE];
  double lastSolvedExtrinsicArray[2][SIZE_POSE];
  double lastSolvedTimeDelayArray[1];
  int lastSolvedFeatureCount = 0;
  bool lastSolvedParametersValid = false;

  MarginalizationInfo::Ptr last_marginalization_info;
  vector<double *> last_marginalization_parameter_blocks;

  map<double, ImageFrame> all_image_frame;
  IntegrationBase::Ptr tmp_pre_integration;
  IMUData latestImuData;
  bool isFirstPoseInitialized = false;
  std::atomic<bool> isRunning{false};
  std::shared_ptr<VINSOptions> options;
  std::ofstream replayRotorFactorLedger;
  std::mutex replayRotorFactorLedgerMutex;
  std::uint64_t replayRotorFactorSolveSequence = 0;
  std::ofstream motorFactorTimeline;
  std::mutex motorFactorTimelineMutex;
  std::uint64_t motorFactorSolveSequence = 0;
};
