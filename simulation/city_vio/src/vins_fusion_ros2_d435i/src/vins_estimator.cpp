#include <cv_bridge/cv_bridge.h>
#include <vins_fusion_ros2/vins_estimator.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>

namespace {
uint64_t imageFingerprint(const cv::Mat& image) {
  // FNV-1a is only a diagnostic fingerprint.  Include shape/type so equal
  // byte streams with different image layouts cannot compare equal here.
  uint64_t hash = 1469598103934665603ULL;
  const auto mix = [&hash](uint8_t value) {
    hash ^= value;
    hash *= 1099511628211ULL;
  };
  for (int shift = 0; shift < 32; shift += 8) {
    mix(static_cast<uint8_t>((image.rows >> shift) & 0xff));
    mix(static_cast<uint8_t>((image.cols >> shift) & 0xff));
    mix(static_cast<uint8_t>((image.type() >> shift) & 0xff));
  }
  if (image.empty()) return hash;
  const size_t row_bytes = image.cols * image.elemSize();
  for (int row = 0; row < image.rows; ++row) {
    const uint8_t* data = image.ptr<uint8_t>(row);
    for (size_t col = 0; col < row_bytes; ++col) mix(data[col]);
  }
  return hash;
}

uint64_t scalarFingerprint(const void* data, size_t size) {
  uint64_t hash = 1469598103934665603ULL;
  const auto* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}
}  // namespace

VinsEstimator::VinsEstimator() : rclcpp::Node("vins_estimator") {
  const char* deterministic_env = std::getenv("VINS_DETERMINISTIC_REPLAY");
  deterministic_replay_output_ =
      deterministic_env && std::string(deterministic_env) == "1";
  options = std::make_shared<VINSOptions>();
  estimator_ = std::make_shared<Estimator>();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  initialize();
}

VinsEstimator::~VinsEstimator() {
  estimator_->setDeterministicCommitOutputCallback({});
}

void VinsEstimator::initialize() {
  initializeParamters();
  initializeSubscribers();
  initializerPublishers();
}
void VinsEstimator::initializeParamters() {
  auto config_file = readParam<std::string>(this, "config_file");
  world_frame_id = readParam<std::string>(this, "world_frame_id", "world");
  body_frame_id = readParam<std::string>(this, "body_frame_id", "body");
  camera_frame_id = readParam<std::string>(this, "camera_frame_id", "camera");
  options->readParameters(config_file);
  use_source_neutral_motor_input_ = this->declare_parameter<bool>(
      "use_source_neutral_motor_input", false);
  effective_motor_state_topic_ = this->declare_parameter<std::string>(
      "effective_motor_state_topic", effective_motor_state_topic_);
  effective_motor_quality_topic_ = this->declare_parameter<std::string>(
      "effective_motor_quality_topic", effective_motor_quality_topic_);
  if (use_source_neutral_motor_input_) {
    const auto mode = vins_fusion_ros2::ParseMotorInputMode(
        this->declare_parameter<std::string>("motor_input_mode", "MOTOR_OFF"));
    if (mode == vins_fusion_ros2::MotorInputMode::kOff)
      throw std::invalid_argument(
          "source-neutral motor input requires an explicit admitted mode");
    effective_motor_join_.reset(
        new vins_fusion_ros2::EffectiveMotorQualityJoin(mode));
    VINS_INFO << "R4 D435i source-neutral motor input enabled: state="
              << effective_motor_state_topic_ << ", quality="
              << effective_motor_quality_topic_;
  }
  const int rotor_drag_dynamics_override =
      this->declare_parameter<int>(
          "use_rotor_drag_dynamics_override", -1);
  if (rotor_drag_dynamics_override == 0 ||
      rotor_drag_dynamics_override == 1) {
    options->use_rotor_drag_dynamics =
        rotor_drag_dynamics_override;
    VINS_INFO << "Rotor-drag dynamics runtime override: "
              << options->use_rotor_drag_dynamics;
  } else if (rotor_drag_dynamics_override != -1) {
    throw std::invalid_argument(
        "use_rotor_drag_dynamics_override must be -1, 0, or 1");
  }
  const double rotor_drag_dynamics_accel_noise_override =
      this->declare_parameter<double>(
          "rotor_drag_dynamics_accel_noise_override", -1.0);
  if (rotor_drag_dynamics_accel_noise_override > 0.0 &&
      std::isfinite(rotor_drag_dynamics_accel_noise_override)) {
    options->dynamics_accel_noise =
        rotor_drag_dynamics_accel_noise_override;
    VINS_INFO << "Rotor-drag dynamics acceleration-noise override: "
              << options->dynamics_accel_noise;
  } else if (rotor_drag_dynamics_accel_noise_override != -1.0) {
    throw std::invalid_argument(
        "rotor_drag_dynamics_accel_noise_override must be -1 or a "
        "finite positive value");
  }
  const int gps_dropout_gyro_bias_prior_override =
      this->declare_parameter<int>(
          "use_gps_dropout_gyro_bias_prior_override", -1);
  if (gps_dropout_gyro_bias_prior_override == 0 ||
      gps_dropout_gyro_bias_prior_override == 1) {
    options->use_gps_dropout_gyro_bias_prior =
        gps_dropout_gyro_bias_prior_override;
    VINS_INFO << "GPS-dropout gyro-bias prior runtime override: "
              << options->use_gps_dropout_gyro_bias_prior;
  } else if (gps_dropout_gyro_bias_prior_override != -1) {
    throw std::invalid_argument(
        "use_gps_dropout_gyro_bias_prior_override must be -1, 0, or 1");
  }
  const double gps_dropout_gyro_bias_sigma_override =
      this->declare_parameter<double>(
          "gps_dropout_gyro_bias_sigma_override", -1.0);
  if (gps_dropout_gyro_bias_sigma_override > 0.0 &&
      std::isfinite(gps_dropout_gyro_bias_sigma_override)) {
    options->gps_dropout_gyro_bias_sigma =
        gps_dropout_gyro_bias_sigma_override;
    VINS_INFO << "GPS-dropout gyro-bias sigma override: "
              << options->gps_dropout_gyro_bias_sigma;
  } else if (gps_dropout_gyro_bias_sigma_override != -1.0) {
    throw std::invalid_argument(
        "gps_dropout_gyro_bias_sigma_override must be -1 or a finite "
        "positive value");
  }
  const int pose_only_visual_override =
      this->declare_parameter<int>(
          "use_pose_only_visual_override", -1);
  if (pose_only_visual_override == 0 ||
      pose_only_visual_override == 1 ||
      pose_only_visual_override == 2) {
    options->use_pose_only_visual = pose_only_visual_override;
    VINS_INFO << "Pose-only visual runtime override: "
              << options->use_pose_only_visual;
  } else if (pose_only_visual_override != -1) {
    throw std::invalid_argument(
        "use_pose_only_visual_override must be -1, 0, 1, or 2");
  }
  const double pose_only_visual_weight_scale_override =
      this->declare_parameter<double>(
          "pose_only_visual_weight_scale_override", -1.0);
  if (pose_only_visual_weight_scale_override > 0.0 &&
      std::isfinite(pose_only_visual_weight_scale_override)) {
    options->pose_only_visual_weight_scale =
        pose_only_visual_weight_scale_override;
    VINS_INFO << "Pose-only visual weight scale override: "
              << options->pose_only_visual_weight_scale;
  } else if (pose_only_visual_weight_scale_override != -1.0) {
    throw std::invalid_argument(
        "pose_only_visual_weight_scale_override must be -1 or a finite "
        "positive value");
  }
  estimator_->initialize(options);
  const int input_downsample_phase =
      this->declare_parameter<int>("input_downsample_phase", 0);
  estimator_->setInputDownsamplePhase(input_downsample_phase);
  const bool replay_position_jump_quarantine =
      this->declare_parameter<bool>("replay_position_jump_quarantine", false);
  const int replay_jump_fallback_iterations =
      this->declare_parameter<int>("replay_jump_fallback_iterations", 0);
  const bool replay_jump_transaction_reject =
      this->declare_parameter<bool>("replay_jump_transaction_reject", false);
  const bool replay_jump_prior_reset =
      this->declare_parameter<bool>("replay_jump_prior_reset", false);
  const double replay_reject_persistent_innovation_timestamp =
      this->declare_parameter<double>(
          "replay_reject_persistent_innovation_timestamp", -1.0);
  const double replay_reject_accepted_update_timestamp =
      this->declare_parameter<double>(
          "replay_reject_accepted_update_timestamp", -1.0);
  const double replay_velocity_trace_start_timestamp =
      this->declare_parameter<double>(
          "replay_velocity_trace_start_timestamp", -1.0);
  const double replay_velocity_trace_end_timestamp =
      this->declare_parameter<double>(
          "replay_velocity_trace_end_timestamp", -1.0);
  const double replay_backend_health_trace_start_timestamp =
      this->declare_parameter<double>(
          "replay_backend_health_trace_start_timestamp", -1.0);
  const double replay_backend_health_trace_end_timestamp =
      this->declare_parameter<double>(
          "replay_backend_health_trace_end_timestamp", -1.0);
  const bool replay_dual_state_backend =
      this->declare_parameter<bool>(
          "replay_dual_state_backend", false);
  const int replay_dual_state_required_healthy_updates =
      this->declare_parameter<int>(
          "replay_dual_state_required_healthy_updates", 3);
  const bool replay_multi_checkpoint_recovery =
      this->declare_parameter<bool>(
          "replay_multi_checkpoint_recovery", false);
  const bool replay_warm_reinitialization =
      this->declare_parameter<bool>(
          "replay_warm_reinitialization", false);
  const bool replay_motion_onset_warm_reinitialization =
      this->declare_parameter<bool>(
          "replay_motion_onset_warm_reinitialization", false);
  const bool replay_prior_scale_projection =
      this->declare_parameter<bool>(
          "replay_prior_scale_projection", false);
  const double replay_prior_scale_projection_start_timestamp =
      this->declare_parameter<double>(
          "replay_prior_scale_projection_start_timestamp", -1.0);
  const double replay_prior_scale_projection_minimum_directional_gradient =
      this->declare_parameter<double>(
          "replay_prior_scale_projection_minimum_directional_gradient",
          -std::numeric_limits<double>::infinity());
  const int replay_prior_scale_projection_maximum_count =
      this->declare_parameter<int>(
          "replay_prior_scale_projection_maximum_count", 0);
  const double replay_skip_marginalization_prior_timestamp =
      this->declare_parameter<double>(
          "replay_skip_marginalization_prior_timestamp", -1.0);
  const bool replay_include_long_imu_factors =
      this->declare_parameter<bool>(
          "replay_include_long_imu_factors", false);
  const double replay_maximum_imu_window_interval =
      this->declare_parameter<double>(
          "replay_maximum_imu_window_interval", -1.0);
  const bool replay_gt_accel_bias_zero_oracle =
      this->declare_parameter<bool>(
          "replay_gt_accel_bias_zero_oracle", false);
  const bool replay_gt_initial_velocity_zero_oracle =
      this->declare_parameter<bool>(
          "replay_gt_initial_velocity_zero_oracle", false);
  const int replay_diagnostic_modes =
      (replay_position_jump_quarantine ? 1 : 0) +
      (replay_jump_fallback_iterations > 0 ? 1 : 0) +
      (replay_jump_transaction_reject ? 1 : 0) +
      (replay_jump_prior_reset ? 1 : 0) +
      (replay_reject_accepted_update_timestamp >= 0.0 ? 1 : 0);
  if (replay_diagnostic_modes > 1) {
    throw std::invalid_argument(
        "replay jump diagnostics are mutually exclusive");
  }
  estimator_->setReplayPositionJumpQuarantine(
      replay_position_jump_quarantine);
  estimator_->setReplayJumpFallbackIterations(
      replay_jump_fallback_iterations);
  estimator_->setReplayJumpTransactionReject(
      replay_jump_transaction_reject);
  estimator_->setReplayJumpPriorReset(replay_jump_prior_reset);
  estimator_->setReplayRejectPersistentInnovationTimestamp(
      replay_reject_persistent_innovation_timestamp);
  estimator_->setReplayRejectAcceptedUpdateTimestamp(
      replay_reject_accepted_update_timestamp);
  estimator_->setReplayVelocityTraceWindow(
      replay_velocity_trace_start_timestamp,
      replay_velocity_trace_end_timestamp);
  estimator_->setReplayBackendHealthTraceWindow(
      replay_backend_health_trace_start_timestamp,
      replay_backend_health_trace_end_timestamp);
  estimator_->setReplayDualStateBackend(
      replay_dual_state_backend,
      replay_dual_state_required_healthy_updates);
  estimator_->setReplayMultiCheckpointRecovery(
      replay_multi_checkpoint_recovery);
  estimator_->setReplayWarmReinitialization(
      replay_warm_reinitialization);
  estimator_->setReplayMotionOnsetWarmReinitialization(
      replay_motion_onset_warm_reinitialization);
  estimator_->setReplayPriorScaleProjection(
      replay_prior_scale_projection,
      replay_prior_scale_projection_start_timestamp,
      replay_prior_scale_projection_minimum_directional_gradient,
      replay_prior_scale_projection_maximum_count);
  estimator_->setReplaySkipMarginalizationPriorTimestamp(
      replay_skip_marginalization_prior_timestamp);
  estimator_->setReplayIncludeLongImuFactors(
      replay_include_long_imu_factors);
  estimator_->setReplayMaximumImuWindowInterval(
      replay_maximum_imu_window_interval);
  estimator_->setReplayGtAccelBiasZeroOracle(
      replay_gt_accel_bias_zero_oracle);
  estimator_->setReplayGtInitialVelocityZeroOracle(
      replay_gt_initial_velocity_zero_oracle);
  replay_skip_initial_images_remaining_ = static_cast<int>(std::max<int64_t>(
      0, this->declare_parameter<int>("replay_skip_initial_images", 0)));
}
void VinsEstimator::initializeSubscribers() {
  imu_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  image_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  feature_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  range_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt_imu;
  sub_opt_imu.callback_group = imu_callback_group_;

  rclcpp::SubscriptionOptions sub_opt_image;
  sub_opt_image.callback_group = image_callback_group_;

  rclcpp::SubscriptionOptions sub_opt_feature;
  sub_opt_feature.callback_group = feature_callback_group_;
  rclcpp::SubscriptionOptions sub_opt_range;
  sub_opt_range.callback_group = range_callback_group_;

  if (std::getenv("VINS_OBSERVABILITY_EXPORT_TIMESTAMPS")) {
    auto ground_truth = this->create_subscription<nav_msgs::msg::Odometry>(
        "/vio/ground_truth/odometry", rclcpp::QoS(rclcpp::KeepLast(500)),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          OdomData data;
          data.timestamp = fromMsg(msg->header.stamp);
          data.position = Eigen::Vector3d(
              msg->pose.pose.position.x, msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          data.orientation = Eigen::Quaterniond(
              msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .normalized();
          estimator_->inputReplayGroundTruth(data);
        },
        sub_opt_image);
    subs_.push_back(ground_truth);
  }

  auto stationary_hint =
      this->create_subscription<std_msgs::msg::Bool>(
          "/vio/stationary_hint", rclcpp::QoS(rclcpp::KeepLast(10)),
          [this](const std_msgs::msg::Bool::SharedPtr msg) {
            estimator_->inputStationaryHint(msg->data);
          });
  subs_.push_back(stationary_hint);

  // Offline replay can consume the recorded reset marker in the same
  // mutually-exclusive callback group as images.  This preserves the reset
  // boundary relative to camera frames; routing it through a separate helper
  // node and service was observed to execute three frames late at 2x replay.
  const bool replay_reset_events =
      this->declare_parameter<bool>("replay_reset_events", false);
  if (replay_reset_events) {
    auto reset_event = this->create_subscription<std_msgs::msg::String>(
        "/vio/preflight_event", rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data != "VINS_RESET_REQUEST") return;
          estimator_->restart();
          path.poses.clear();
          VINS_INFO << "Applied recorded VINS reset event in image callback group.";
        },
        sub_opt_image);
    subs_.push_back(reset_event);
  }

  if (options->hasImu()) {
    rclcpp::QoS imu_qos(rclcpp::KeepLast(100));
    if (this->declare_parameter<bool>("imu_best_effort", false)) {
      imu_qos.best_effort();
    }
    auto imu = this->create_subscription<sensor_msgs::msg::Imu>(
        options->imuTopic(), imu_qos,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          auto imu_msg = fromMsg(*msg);
          imu_msg.source_sequence = ++imu_callback_sequence_;
          const double values[] = {
              imu_msg.timestamp,
              imu_msg.angular_velocity.x(), imu_msg.angular_velocity.y(),
              imu_msg.angular_velocity.z(), imu_msg.linear_acceleration.x(),
              imu_msg.linear_acceleration.y(), imu_msg.linear_acceleration.z()};
          imu_msg.payload_fingerprint =
              scalarFingerprint(values, sizeof(values));
          const uint64_t callback_sequence =
              callback_delivery_sequence_.fetch_add(1) + 1;
          VINS_INFO << "Input callback trace: callback_sequence="
                    << callback_sequence << ", topic=" << options->imuTopic()
                    << ", topic_sequence=" << imu_msg.source_sequence
                    << ", timestamp=" << std::fixed << std::setprecision(9)
                    << imu_msg.timestamp << ", fingerprint=0x" << std::hex
                    << imu_msg.payload_fingerprint << std::dec;
          estimator_->inputIMU(imu_msg);
        },
        sub_opt_imu);
    subs_.push_back(imu);
  }

  if (options->hasRange()) {
    auto range = this->create_subscription<sensor_msgs::msg::Range>(
        options->range_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Range::SharedPtr msg) {
          RangeData data;
          data.timestamp = fromMsg(msg->header.stamp);
          data.source_sequence = ++range_callback_sequence_;
          data.range = msg->range;
          data.valid = std::isfinite(data.range) &&
                       data.range >= msg->min_range &&
                       data.range >= options->range_min_valid &&
                       data.range <= msg->max_range;
          const double values[] = {data.timestamp, data.range,
                                   static_cast<double>(data.valid)};
          data.payload_fingerprint = scalarFingerprint(values, sizeof(values));
          const uint64_t callback_sequence =
              callback_delivery_sequence_.fetch_add(1) + 1;
          VINS_INFO << "Input callback trace: callback_sequence="
                    << callback_sequence << ", topic=" << options->range_topic
                    << ", topic_sequence=" << data.source_sequence
                    << ", timestamp=" << std::fixed << std::setprecision(9)
                    << data.timestamp << ", fingerprint=0x" << std::hex
                    << data.payload_fingerprint << std::dec
                    << ", valid=" << data.valid;
          estimator_->inputRange(data);
        },
        sub_opt_range);
    subs_.push_back(range);
  }

  if (options->hasGpsVelocity()) {
    auto gps_velocity = this->create_subscription<px4_msgs::msg::SensorGps>(
        options->gps_velocity_topic, rclcpp::SensorDataQoS(),
        [this](const px4_msgs::msg::SensorGps::SharedPtr msg) {
          MetricVelocityData data;
          data.timestamp = this->get_clock()->now().seconds();
          // Gazebo ENU/VINS world X/Y corresponds to GPS east/north.
          data.horizontal_velocity =
              Eigen::Vector2d(msg->vel_e_m_s, msg->vel_n_m_s);
          data.sigma = msg->s_variance_m_s;
          data.valid =
              msg->fix_type >= 3 && msg->vel_ned_valid &&
              data.horizontal_velocity.allFinite() &&
              std::isfinite(data.sigma) && data.sigma >= 0.0;
          if (!data.valid) {
            gps_velocity_integration_initialized_ = false;
            return;
          }
          const double sample_timestamp =
              static_cast<double>(msg->timestamp) * 1e-6;
          const double delta_time =
              sample_timestamp - previous_gps_velocity_timestamp_;
          if (!gps_velocity_integration_initialized_ ||
              !std::isfinite(delta_time) ||
              delta_time <= 0.0 || delta_time > 1.0) {
            ++gps_velocity_integration_segment_;
            gps_segment_origin_latitude_deg_ =
                static_cast<double>(msg->lat) * 1e-7;
            gps_segment_origin_longitude_deg_ =
                static_cast<double>(msg->lon) * 1e-7;
          }
          gps_velocity_integration_initialized_ = true;
          previous_gps_velocity_timestamp_ = sample_timestamp;
          constexpr double kEarthRadiusMeters = 6378137.0;
          constexpr double kDegreesToRadians =
              3.14159265358979323846 / 180.0;
          const double latitude_deg =
              static_cast<double>(msg->lat) * 1e-7;
          const double longitude_deg =
              static_cast<double>(msg->lon) * 1e-7;
          data.horizontal_position.x() =
              (longitude_deg - gps_segment_origin_longitude_deg_) *
              kDegreesToRadians * kEarthRadiusMeters *
              std::cos(gps_segment_origin_latitude_deg_ *
                       kDegreesToRadians);
          data.horizontal_position.y() =
              (latitude_deg - gps_segment_origin_latitude_deg_) *
              kDegreesToRadians * kEarthRadiusMeters;
          data.integration_segment = gps_velocity_integration_segment_;
          estimator_->inputMetricVelocity(data);
        },
        sub_opt_range);
    subs_.push_back(gps_velocity);
  }

  if (options->hasRotorDragDynamics() && !use_source_neutral_motor_input_) {
    auto actuator_motors =
        this->create_subscription<px4_msgs::msg::ActuatorMotors>(
            options->actuator_motors_topic, rclcpp::SensorDataQoS(),
            [this](const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
              const double sample_timestamp =
                  static_cast<double>(msg->timestamp_sample) * 1e-6;
              const double dt =
                  sample_timestamp - previous_motor_sample_timestamp_;
              if (!motor_model_initialized_ || !std::isfinite(dt) ||
                  dt <= 0.0 || dt > 1.0) {
                rotor_speed_rad_s_.fill(0.0);
                motor_model_initialized_ = true;
              }
              previous_motor_sample_timestamp_ = sample_timestamp;

              bool armed = true;
              std::array<double, 4> reference_speed{};
              for (std::size_t i = 0; i < reference_speed.size(); ++i) {
                const double control = static_cast<double>(msg->control[i]);
                if (!std::isfinite(control) || control <= 0.05) armed = false;
                reference_speed[i] = armed
                    ? std::clamp(
                          control *
                                  options->dynamics_motor_input_scaling +
                              options->dynamics_motor_zero_armed,
                          0.0, options->dynamics_motor_max_speed)
                    : 0.0;
              }
              // Re-evaluate all references after the shared armed decision so
              // a partially invalid motor vector cannot become a valid input.
              if (armed) {
                for (std::size_t i = 0; i < reference_speed.size(); ++i) {
                  const double control =
                      static_cast<double>(msg->control[i]);
                  reference_speed[i] = std::clamp(
                      control * options->dynamics_motor_input_scaling +
                          options->dynamics_motor_zero_armed,
                      0.0, options->dynamics_motor_max_speed);
                }
              } else {
                reference_speed.fill(0.0);
              }

              const double filter_dt =
                  std::isfinite(dt) && dt > 0.0 && dt <= 1.0 ? dt : 0.0;
              for (std::size_t i = 0; i < rotor_speed_rad_s_.size(); ++i) {
                const double tau =
                    reference_speed[i] >= rotor_speed_rad_s_[i]
                        ? options->dynamics_motor_time_constant_up
                        : options->dynamics_motor_time_constant_down;
                if (filter_dt > 0.0 && std::isfinite(tau) && tau > 0.0) {
                  const double alpha =
                      1.0 - std::exp(-filter_dt / tau);
                  rotor_speed_rad_s_[i] +=
                      alpha * (reference_speed[i] -
                               rotor_speed_rad_s_[i]);
                }
              }

              RotorDynamicsData data;
              data.timestamp = this->get_clock()->now().seconds();
              for (const double rotor_speed : rotor_speed_rad_s_) {
                data.total_thrust_newton +=
                    options->dynamics_motor_constant *
                    rotor_speed * rotor_speed;
                data.summed_rotor_speed_rad_s += std::abs(rotor_speed);
              }
              data.valid =
                  armed && data.total_thrust_newton > 0.0 &&
                  data.summed_rotor_speed_rad_s > 0.0;
              if (!motor_callback_logged_) {
                VINS_INFO << "First actuator-motors callback: clock_s="
                          << data.timestamp
                          << ", sample_s=" << sample_timestamp
                          << ", controls="
                          << msg->control[0] << ","
                          << msg->control[1] << ","
                          << msg->control[2] << ","
                          << msg->control[3]
                          << ", valid=" << data.valid;
                motor_callback_logged_ = true;
              }
              if (data.valid && !valid_motor_callback_logged_) {
                VINS_INFO << "First valid actuator-motors callback: clock_s="
                          << data.timestamp
                          << ", sample_s=" << sample_timestamp
                          << ", thrust_n=" << data.total_thrust_newton
                          << ", rotor_speed_sum_rad_s="
                          << data.summed_rotor_speed_rad_s;
                valid_motor_callback_logged_ = true;
              }
              estimator_->inputRotorDynamics(data);
            },
            sub_opt_range);
    subs_.push_back(actuator_motors);
  }

  if (options->hasRotorDragDynamics() && use_source_neutral_motor_input_) {
    auto effective_quality = this->create_subscription<std_msgs::msg::String>(
        effective_motor_quality_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          ++effective_motor_quality_callback_sequence_;
          int64_t timestamp_ns = 0;
          {
            std::lock_guard<std::mutex> lock(effective_motor_join_mutex_);
            timestamp_ns = effective_motor_join_->PushQuality(msg->data)
                               .sample_timestamp_ns;
          }
          tryConsumeEffectiveMotor(timestamp_ns);
        });
    subs_.push_back(effective_quality);
    auto effective_state =
        this->create_subscription<sensor_msgs::msg::JointState>(
            effective_motor_state_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
              ++effective_motor_state_callback_sequence_;
              const int64_t timestamp_ns =
                  static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
                  static_cast<int64_t>(msg->header.stamp.nanosec);
              if (msg->name != std::vector<std::string>(
                                   {"motor_1", "motor_2", "motor_3", "motor_4"}) ||
                  msg->velocity.size() != 4 || !msg->position.empty() ||
                  !msg->effort.empty())
                throw std::invalid_argument(
                    "effective motor JointState schema mismatch");
              std::array<double, 4> rad_s{{msg->velocity[0], msg->velocity[1],
                                           msg->velocity[2], msg->velocity[3]}};
              {
                std::lock_guard<std::mutex> lock(effective_motor_join_mutex_);
                effective_motor_join_->PushState(timestamp_ns, rad_s);
              }
              tryConsumeEffectiveMotor(timestamp_ns);
            });
    subs_.push_back(effective_state);
  }

  if (options->isUsingStereo()) {
    sub_img0_filter_ = std::make_shared<Subscriber<Image>>(
        this, options->imageTopic(), rmw_qos_profile_sensor_data,
        sub_opt_image);

    sub_img1_filter_ = std::make_shared<Subscriber<Image>>(
        this, options->image1Topic(), rmw_qos_profile_sensor_data,
        sub_opt_image);

    sync_img_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *sub_img0_filter_, *sub_img1_filter_);

    sync_img_->registerCallback(std::bind(&VinsEstimator::stereoCallback, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
  } else {
    auto sub_img0 = this->create_subscription<sensor_msgs::msg::Image>(
        options->imageTopic(), rclcpp::QoS(rclcpp::KeepLast(100)),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          if (replay_skip_initial_images_remaining_ > 0) {
            --replay_skip_initial_images_remaining_;
            VINS_INFO << "Replay diagnostic skipped initial camera image: "
                      << "timestamp=" << std::fixed << std::setprecision(9)
                      << fromMsg(msg->header.stamp)
                      << ", remaining="
                      << replay_skip_initial_images_remaining_;
            return;
          }
          ImageData image;
          image.image0 = fromMsg(*msg);
          image.timestamp = fromMsg(msg->header.stamp);
          image.sequence = ++image_callback_sequence_;
          VINS_INFO << "Camera callback trace: sequence=" << image.sequence
                    << ", timestamp=" << std::fixed << std::setprecision(9)
                    << image.timestamp << ", fingerprint=0x" << std::hex
                    << imageFingerprint(image.image0) << std::dec
                    << ", size=" << image.image0.cols << "x"
                    << image.image0.rows << ", encoding=" << msg->encoding;
          estimator_->inputImage(image);
        },
        sub_opt_image);
    subs_.push_back(sub_img0);
  }

  auto sub_feature = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(100)),
      [this](const sensor_msgs::msg::PointCloud::SharedPtr msg) {
        estimator_->inputFeature(fromMsg(msg->header.stamp), fromMsg(*msg));
      },
      sub_opt_feature);
  subs_.push_back(sub_feature);

  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        estimator_->restart();
        path.poses.clear();
        response->success = true;
        response->message = "VINS estimator and sensor buffers reset";
      });

  publish_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&VinsEstimator::timeCallback, this));
}
void VinsEstimator::tryConsumeEffectiveMotor(int64_t timestamp_ns) {
  vins_fusion_ros2::VerifiedEffectiveMotorInput input;
  {
    std::lock_guard<std::mutex> lock(effective_motor_join_mutex_);
    if (!effective_motor_join_->TryPop(timestamp_ns, &input)) return;
  }
  RotorDynamicsData data;
  data.timestamp = static_cast<double>(input.sample_timestamp_ns) * 1e-9;
  data.source_sequence = input.source_sequence;
  data.source_hash = input.source_hash;
  data.quality_state = vins_fusion_ros2::ToString(input.quality_state);
  data.factor_weight_scale = input.factor_weight_scale;
  for (const double rotor_speed : input.rad_s) {
    data.total_thrust_newton +=
        options->dynamics_motor_constant * rotor_speed * rotor_speed;
    data.summed_rotor_speed_rad_s += std::abs(rotor_speed);
  }
  data.valid = data.total_thrust_newton > 0.0 &&
      data.summed_rotor_speed_rad_s > 0.0;
  estimator_->inputRotorDynamics(data);
}

void VinsEstimator::initializerPublishers() {
  pub_latest_odometry =
      this->create_publisher<nav_msgs::msg::Odometry>("imu_propagate", 1);
  pub_path = this->create_publisher<nav_msgs::msg::Path>("path", 1);
  pub_odometry = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
  pub_image_track =
      this->create_publisher<sensor_msgs::msg::Image>("image_track", 1);
  pub_point_cloud =
      this->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1);
  pub_margin_cloud =
      this->create_publisher<sensor_msgs::msg::PointCloud>("margin_cloud", 1);
  pub_keyframe_point =
      this->create_publisher<sensor_msgs::msg::PointCloud>("keyframe_point", 1);
  pub_keyframe_pose =
      this->create_publisher<nav_msgs::msg::Odometry>("keyframe_pose", 1);
  if (deterministic_replay_output_) {
    estimator_->setDeterministicCommitOutputCallback(
        [this](const OdomData& output) {
          auto message = toMsg(output);
          message.header.frame_id = world_frame_id;
          message.child_frame_id = body_frame_id;
          pub_odometry->publish(message);
          const double values[] = {
              output.timestamp,
              output.position.x(), output.position.y(), output.position.z(),
              output.orientation.w(), output.orientation.x(),
              output.orientation.y(), output.orientation.z(),
              output.velocity.x(), output.velocity.y(), output.velocity.z()};
          const uint64_t sequence =
              output_publication_sequence_.fetch_add(1) + 1;
          VINS_INFO << "Output publication trace: sequence=" << sequence
                    << ", timestamp=" << std::fixed << std::setprecision(9)
                    << output.timestamp << ", state_fingerprint=0x"
                    << std::hex << scalarFingerprint(values, sizeof(values))
                    << std::dec << ", position="
                    << output.position.transpose() << ", orientation="
                    << output.orientation.w() << " "
                    << output.orientation.x() << " "
                    << output.orientation.y() << " "
                    << output.orientation.z() << ", velocity="
                    << output.velocity.transpose();
        });
  }
}

void VinsEstimator::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img0,
    const sensor_msgs::msg::Image::ConstSharedPtr& img1) {
  const double timestamp = fromMsg(img0->header.stamp);
  // Gazebo can emit the same stamped stereo pair twice around a rendering
  // scheduling boundary.  Passing dt==0 into the tracker makes feature
  // velocity NaN and causes Ceres to fail the next projection residual.
  if (timestamp <= last_stereo_timestamp_) {
    VINS_WARN << "Dropping non-monotonic stereo pair: timestamp="
              << std::fixed << std::setprecision(9) << timestamp
              << ", previous=" << last_stereo_timestamp_;
    return;
  }
  last_stereo_timestamp_ = timestamp;
  ImageData image;
  image.image0 = fromMsg(*img0);
  image.image1 = fromMsg(*img1);
  image.timestamp = timestamp;
  image.sequence = ++image_callback_sequence_;
  image.left_fingerprint = imageFingerprint(image.image0);
  image.right_fingerprint = imageFingerprint(image.image1);
  const double right_timestamp = fromMsg(img1->header.stamp);
  const uint64_t callback_sequence =
      callback_delivery_sequence_.fetch_add(1) + 1;
  VINS_INFO << "Stereo pair trace: callback_sequence=" << callback_sequence
            << ", pair_sequence=" << image.sequence
            << ", left_timestamp=" << std::fixed << std::setprecision(9)
            << image.timestamp << ", right_timestamp=" << right_timestamp
            << ", timestamp_delta=" << right_timestamp - image.timestamp
            << ", left_fingerprint=0x" << std::hex
            << image.left_fingerprint << ", right_fingerprint=0x"
            << image.right_fingerprint << std::dec;
  VINS_INFO << "Camera callback trace: sequence=" << image.sequence
            << ", timestamp=" << std::fixed << std::setprecision(9)
            << image.timestamp << ", fingerprint=0x" << std::hex
            << image.left_fingerprint << std::dec
            << ", size=" << image.image0.cols << "x" << image.image0.rows
            << ", encoding=" << img0->encoding;
  estimator_->inputImage(image);
}

void VinsEstimator::timeCallback() {
  publishImuData();
  if (!deterministic_replay_output_) publishOdometry();
  publishKeyFrameData();
  publishImage();
  publishPointCloud();
}

void VinsEstimator::publishPointCloud() {
  PointCloudData cloud;
  if (estimator_->getMainCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_point_cloud->publish(msg);
  }

  if (estimator_->getMarginCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_margin_cloud->publish(msg);
  }

  if (estimator_->getkeyframeCloud(cloud)) {
    auto msg = toMsg(cloud);
    msg.header.frame_id = world_frame_id;
    pub_keyframe_point->publish(msg);
  }
}

void VinsEstimator::publishImage() {
  ImageData image;
  if (estimator_->getTrackImage(image)) {
    auto img = toMsg(image.image0);
    img.header.frame_id = world_frame_id;
    img.header.stamp = toMsg(image.timestamp);
    pub_image_track->publish(img);
  }
}
void VinsEstimator::publishImuData() {
  OdomData imu_odom;
  if (estimator_->getIntegratedImuOdom(imu_odom)) {
    nav_msgs::msg::Odometry odometry = toMsg(imu_odom);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;
    pub_latest_odometry->publish(odometry);
  }
}
void VinsEstimator::publishOdometry() {
  OdomData vio_odom;
  if (estimator_->getVisualInertialOdom(vio_odom)) {
    nav_msgs::msg::Odometry odometry = toMsg(vio_odom);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;

    pub_odometry->publish(odometry);
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odometry.header;
    pose_stamped.header.frame_id = world_frame_id;
    pose_stamped.pose = odometry.pose.pose;
    path.header = odometry.header;
    path.header.frame_id = world_frame_id;
    path.poses.push_back(pose_stamped);
    pub_path->publish(path);

    // world-->body
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odometry.header.stamp;
    tf_msg.header.frame_id = world_frame_id;
    tf_msg.child_frame_id = body_frame_id;
    tf_msg.transform.translation.x = odometry.pose.pose.position.x;
    tf_msg.transform.translation.y = odometry.pose.pose.position.y;
    tf_msg.transform.translation.z = odometry.pose.pose.position.z;
    tf_msg.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
    PoseData camera_pose;
    estimator_->getCameraPose(0, camera_pose);

    // body-->camera
    tf_msg.header.frame_id = body_frame_id;
    tf_msg.child_frame_id = camera_frame_id;
    tf_msg.transform.translation.x = camera_pose.position.x();
    tf_msg.transform.translation.y = camera_pose.position.y();
    tf_msg.transform.translation.z = camera_pose.position.z();
    tf_msg.transform.rotation.x = camera_pose.orientation.x();
    tf_msg.transform.rotation.y = camera_pose.orientation.y();
    tf_msg.transform.rotation.z = camera_pose.orientation.z();
    tf_msg.transform.rotation.w = camera_pose.orientation.w();
    tf_broadcaster_->sendTransform(tf_msg);
  }
}

void VinsEstimator::publishKeyFrameData() {
  PoseData pose;
  if (estimator_->getkeyframePose(pose)) {
    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = toMsg(pose.timestamp);
    odometry.header.frame_id = world_frame_id;
    odometry.child_frame_id = body_frame_id;
    odometry.pose.pose.position.x = pose.position.x();
    odometry.pose.pose.position.y = pose.position.y();
    odometry.pose.pose.position.z = pose.position.z();
    odometry.pose.pose.orientation.x = pose.orientation.x();
    odometry.pose.pose.orientation.y = pose.orientation.y();
    odometry.pose.pose.orientation.z = pose.orientation.z();
    odometry.pose.pose.orientation.w = pose.orientation.w();
    pub_keyframe_pose->publish(odometry);
  }
}
