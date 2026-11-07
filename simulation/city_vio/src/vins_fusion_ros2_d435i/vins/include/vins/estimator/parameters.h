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

#include <vins/logger/logger.h>
#include <vins/utility/utility.h>

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;
const int NUM_OF_F = 1000;

enum class ExtrinsicEstimationMode {
  FIXED = 0,  // 0: Do not estimate extrinsic parameters, use fixed
  APPROXIMATE =
      1,  // 1: Extrinsic parameters already roughly estimated, refine only
  INITIALIZE = 2  // 2: Estimate from scratch using motion
};

struct ImuOptions {
  double accelNoiseDensity = 0.0, accelRandomWalk = 0.0;
  double gyroNoiseDensity = 0.0, gyroRandomWalk = 0.0;
  Eigen::Vector3d gravityVector{0.0, 0.0, 9.8};
  std::string imu_topic;
  int useImu = 0;
  bool hasImu() const { return useImu; }
  std::string imuTopic() const { return imu_topic; }
  Eigen::Vector3d gravity() const { return gravityVector; }
  Eigen::Vector3d& gravity() { return gravityVector; }
};

struct VINSOptions {
  ImuOptions imu;
  double init_estimated_depth = 5.0;
  double min_parllaax_num = 0.0;

  std::vector<Eigen::Matrix3d> RIC;
  std::vector<Eigen::Vector3d> TIC;

  int USE_GPU = 0;
  int USE_GPU_ACC_FLOW = 0;
  int USE_GPU_CERES = 0;
  std::string EX_CALIB_RESULT_PATH;
  std::string VINS_RESULT_PATH;
  std::string OUTPUT_FOLDER;
  //////////////////////////////////////////////////////////////////////////////
  double solver_time = 0.0;
  int max_iterations = 0;
  ExtrinsicEstimationMode extrinsic_estimation_mode =
      ExtrinsicEstimationMode::FIXED;
  int estimate_td_mode = 0;

  double time_delay = 0.0;
  int num_of_camera = 0;
  int stereo = 0;
  int use_range = 0;
  std::string range_topic;
  double range_noise = 0.03;
  double range_min_valid = 0.0;
  double range_max_age = 0.1;
  Eigen::Vector3d range_translation = Eigen::Vector3d::Zero();
  int use_gps_velocity = 0;
  std::string gps_velocity_topic;
  double gps_velocity_noise = 0.05;
  double gps_position_noise = 0.02;
  double gps_velocity_max_age = 0.5;
  double gps_dropout_speed_max = 1.10;
  double gps_dropout_vertical_speed_max = 0.50;
  double gps_dropout_speed_noise = 0.05;
  int use_gps_dropout_gyro_bias_prior = 0;
  double gps_dropout_gyro_bias_sigma = 0.00005;
  int use_pose_only_visual = 0;
  double pose_only_visual_weight_scale = 0.05;
  int use_rotor_drag_dynamics = 0;
  std::string actuator_motors_topic = "/fmu/out/actuator_motors";
  double dynamics_mass_kg = 1.57334;
  double dynamics_rotor_drag_coefficient = 0.000175;
  double dynamics_accel_noise = 0.20;
  double dynamics_max_age = 0.05;
  double dynamics_motor_constant = 5.84e-6;
  double dynamics_motor_input_scaling = 1000.0;
  double dynamics_motor_zero_armed = 100.0;
  double dynamics_motor_max_speed = 1100.0;
  double dynamics_motor_time_constant_up = 0.0125;
  double dynamics_motor_time_constant_down = 0.025;
  std::string image0_topic, image1_topic;
  std::vector<std::string> camera_names;

  int max_feature_count = 0;
  int min_feature_distance = 0;
  double ransac_reproj_threshold = 0.0;
  // Keep the upstream 3 px behavior unless a camera/environment config opts
  // into a measured threshold.  SITL uses 4 px to avoid repeatedly deleting
  // geometrically consistent long tracks during rendered hover.
  double outlier_rejection_threshold_px = 3.0;
  int show_track = 0;
  int enable_reverse_optical_flow_check = 0;
  int enable_stereo_track_diversity = 0;
  int stereo_track_grid_rows = 3;
  int stereo_track_grid_cols = 4;
  int stereo_track_max_per_cell = 18;
  int stereo_track_minimum_count = 60;
  double stereo_track_near_depth_m = 2.0;
  double stereo_track_mid_depth_m = 5.0;
  double stereo_track_far_depth_m = 10.0;
  int enable_feature_frame_trace = 0;
  double feature_frame_trace_start_timestamp = -1.0;
  double feature_frame_trace_end_timestamp = -1.0;
  ///////////////////////////////////////////////////////////////////////////

  // 读取参数函数
  void readParameters(const std::string& config_file) {
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
      VINS_ERROR << "Wrong path to settings";
      return;
    }

    fsSettings["image0_topic"] >> this->image0_topic;
    fsSettings["image1_topic"] >> this->image1_topic;
    this->max_feature_count = (int)fsSettings["max_cnt"];
    this->min_feature_distance = (int)fsSettings["min_dist"];
    this->ransac_reproj_threshold = (double)fsSettings["F_threshold"];
    if (!fsSettings["outlier_rejection_threshold_px"].empty()) {
      const double configured_threshold =
          (double)fsSettings["outlier_rejection_threshold_px"];
      if (std::isfinite(configured_threshold) && configured_threshold > 0.0)
        this->outlier_rejection_threshold_px = configured_threshold;
    }
    this->show_track = (int)fsSettings["show_track"];
    this->enable_reverse_optical_flow_check = (int)fsSettings["flow_back"];
    if (!fsSettings["enable_stereo_track_diversity"].empty()) {
      this->enable_stereo_track_diversity =
          (int)fsSettings["enable_stereo_track_diversity"];
    }
    if (!fsSettings["stereo_track_grid_rows"].empty()) {
      this->stereo_track_grid_rows =
          std::max(1, (int)fsSettings["stereo_track_grid_rows"]);
    }
    if (!fsSettings["stereo_track_grid_cols"].empty()) {
      this->stereo_track_grid_cols =
          std::max(1, (int)fsSettings["stereo_track_grid_cols"]);
    }
    if (!fsSettings["stereo_track_max_per_cell"].empty()) {
      this->stereo_track_max_per_cell =
          std::max(1, (int)fsSettings["stereo_track_max_per_cell"]);
    }
    if (!fsSettings["stereo_track_minimum_count"].empty()) {
      this->stereo_track_minimum_count =
          std::max(0, (int)fsSettings["stereo_track_minimum_count"]);
    }
    if (!fsSettings["stereo_track_near_depth_m"].empty()) {
      this->stereo_track_near_depth_m =
          (double)fsSettings["stereo_track_near_depth_m"];
    }
    if (!fsSettings["stereo_track_mid_depth_m"].empty()) {
      this->stereo_track_mid_depth_m =
          (double)fsSettings["stereo_track_mid_depth_m"];
    }
    if (!fsSettings["stereo_track_far_depth_m"].empty()) {
      this->stereo_track_far_depth_m =
          (double)fsSettings["stereo_track_far_depth_m"];
    }
    if (!fsSettings["enable_feature_frame_trace"].empty()) {
      this->enable_feature_frame_trace =
          (int)fsSettings["enable_feature_frame_trace"];
    }
    if (!fsSettings["feature_frame_trace_start_timestamp"].empty()) {
      this->feature_frame_trace_start_timestamp =
          (double)fsSettings["feature_frame_trace_start_timestamp"];
    }
    if (!fsSettings["feature_frame_trace_end_timestamp"].empty()) {
      this->feature_frame_trace_end_timestamp =
          (double)fsSettings["feature_frame_trace_end_timestamp"];
    }
    if (!(this->stereo_track_near_depth_m > 0.0 &&
          this->stereo_track_mid_depth_m >
              this->stereo_track_near_depth_m &&
          this->stereo_track_far_depth_m >
              this->stereo_track_mid_depth_m)) {
      VINS_WARN << "Invalid stereo track depth boundaries; using "
                   "2/5/10 m defaults";
      this->stereo_track_near_depth_m = 2.0;
      this->stereo_track_mid_depth_m = 5.0;
      this->stereo_track_far_depth_m = 10.0;
    }

    if (!fsSettings["use_range"].empty())
      this->use_range = (int)fsSettings["use_range"];
    if (this->use_range) {
      fsSettings["range_topic"] >> this->range_topic;
      if (!fsSettings["range_noise"].empty())
        this->range_noise = (double)fsSettings["range_noise"];
      if (!fsSettings["range_min_valid"].empty())
        this->range_min_valid = (double)fsSettings["range_min_valid"];
      if (!fsSettings["range_max_age"].empty())
        this->range_max_age = (double)fsSettings["range_max_age"];
      cv::Mat cv_t;
      fsSettings["body_T_range"] >> cv_t;
      if (!cv_t.empty()) {
        Eigen::Matrix4d T_range;
        cv::cv2eigen(cv_t, T_range);
        this->range_translation = T_range.block<3, 1>(0, 3);
      }
      VINS_INFO << "Range config: topic=" << this->range_topic
                << ", noise=" << this->range_noise
                << ", min_valid=" << this->range_min_valid
                << ", max_age=" << this->range_max_age;
    }
    if (!fsSettings["use_gps_velocity"].empty())
      this->use_gps_velocity = (int)fsSettings["use_gps_velocity"];
    if (this->use_gps_velocity) {
      fsSettings["gps_velocity_topic"] >> this->gps_velocity_topic;
      if (!fsSettings["gps_velocity_noise"].empty())
        this->gps_velocity_noise =
            (double)fsSettings["gps_velocity_noise"];
      if (!fsSettings["gps_position_noise"].empty())
        this->gps_position_noise =
            (double)fsSettings["gps_position_noise"];
      if (!fsSettings["gps_velocity_max_age"].empty())
        this->gps_velocity_max_age =
            (double)fsSettings["gps_velocity_max_age"];
      if (!fsSettings["gps_dropout_speed_max"].empty())
        this->gps_dropout_speed_max =
            (double)fsSettings["gps_dropout_speed_max"];
      if (!fsSettings["gps_dropout_vertical_speed_max"].empty())
        this->gps_dropout_vertical_speed_max =
            (double)fsSettings["gps_dropout_vertical_speed_max"];
      if (!fsSettings["gps_dropout_speed_noise"].empty())
        this->gps_dropout_speed_noise =
            (double)fsSettings["gps_dropout_speed_noise"];
      VINS_INFO << "GPS velocity config: topic="
                << this->gps_velocity_topic
                << ", noise=" << this->gps_velocity_noise
                << ", position_noise="
                << this->gps_position_noise
                << ", max_age=" << this->gps_velocity_max_age
                << ", dropout_speed_max="
                << this->gps_dropout_speed_max
                << ", dropout_vertical_speed_max="
                << this->gps_dropout_vertical_speed_max
                << ", dropout_speed_noise="
                << this->gps_dropout_speed_noise;
      if (!fsSettings["use_gps_dropout_gyro_bias_prior"].empty())
        this->use_gps_dropout_gyro_bias_prior =
            (int)fsSettings["use_gps_dropout_gyro_bias_prior"];
      if (!fsSettings["gps_dropout_gyro_bias_sigma"].empty())
        this->gps_dropout_gyro_bias_sigma =
            (double)fsSettings["gps_dropout_gyro_bias_sigma"];
    }
    if (!fsSettings["use_rotor_drag_dynamics"].empty())
      this->use_rotor_drag_dynamics =
          (int)fsSettings["use_rotor_drag_dynamics"];
    if (!fsSettings["use_pose_only_visual"].empty())
      this->use_pose_only_visual =
          (int)fsSettings["use_pose_only_visual"];
    if (!fsSettings["pose_only_visual_weight_scale"].empty())
      this->pose_only_visual_weight_scale =
          (double)fsSettings["pose_only_visual_weight_scale"];
    if (this->use_rotor_drag_dynamics) {
      if (!fsSettings["actuator_motors_topic"].empty())
        fsSettings["actuator_motors_topic"] >>
            this->actuator_motors_topic;
      if (!fsSettings["dynamics_mass_kg"].empty())
        this->dynamics_mass_kg =
            (double)fsSettings["dynamics_mass_kg"];
      if (!fsSettings["dynamics_rotor_drag_coefficient"].empty())
        this->dynamics_rotor_drag_coefficient =
            (double)fsSettings["dynamics_rotor_drag_coefficient"];
      if (!fsSettings["dynamics_accel_noise"].empty())
        this->dynamics_accel_noise =
            (double)fsSettings["dynamics_accel_noise"];
      if (!fsSettings["dynamics_max_age"].empty())
        this->dynamics_max_age =
            (double)fsSettings["dynamics_max_age"];
      if (!fsSettings["dynamics_motor_constant"].empty())
        this->dynamics_motor_constant =
            (double)fsSettings["dynamics_motor_constant"];
      if (!fsSettings["dynamics_motor_input_scaling"].empty())
        this->dynamics_motor_input_scaling =
            (double)fsSettings["dynamics_motor_input_scaling"];
      if (!fsSettings["dynamics_motor_zero_armed"].empty())
        this->dynamics_motor_zero_armed =
            (double)fsSettings["dynamics_motor_zero_armed"];
      if (!fsSettings["dynamics_motor_max_speed"].empty())
        this->dynamics_motor_max_speed =
            (double)fsSettings["dynamics_motor_max_speed"];
      if (!fsSettings["dynamics_motor_time_constant_up"].empty())
        this->dynamics_motor_time_constant_up =
            (double)fsSettings["dynamics_motor_time_constant_up"];
      if (!fsSettings["dynamics_motor_time_constant_down"].empty())
        this->dynamics_motor_time_constant_down =
            (double)fsSettings["dynamics_motor_time_constant_down"];
      VINS_INFO << "Rotor-drag dynamics config: topic="
                << this->actuator_motors_topic
                << ", mass_kg=" << this->dynamics_mass_kg
                << ", drag_coefficient="
                << this->dynamics_rotor_drag_coefficient
                << ", accel_noise=" << this->dynamics_accel_noise
                << ", max_age=" << this->dynamics_max_age;
    }

    this->USE_GPU = (int)fsSettings["use_gpu"];
    this->USE_GPU_ACC_FLOW = (int)fsSettings["use_gpu_acc_flow"];
    this->USE_GPU_CERES = (int)fsSettings["use_gpu_ceres"];

    this->imu.useImu = (int)fsSettings["imu"];
    VINS_INFO << "USE_IMU: " << this->hasImu() << std::endl;
    if (this->hasImu()) {
      fsSettings["imu_topic"] >> this->imu.imu_topic;
      this->imu.accelNoiseDensity = (double)fsSettings["acc_n"];
      this->imu.accelRandomWalk = (double)fsSettings["acc_w"];
      this->imu.gyroNoiseDensity = (double)fsSettings["gyr_n"];
      this->imu.gyroRandomWalk = (double)fsSettings["gyr_w"];
      this->imu.gravity().z() = (double)fsSettings["g_norm"];
    }

    this->solver_time = (double)fsSettings["max_solver_time"];
    this->max_iterations = (int)fsSettings["max_num_iterations"];
    this->min_parllaax_num = (double)fsSettings["keyframe_parallax"];
    this->min_parllaax_num = this->min_parllaax_num / FOCAL_LENGTH;

    fsSettings["output_path"] >> this->OUTPUT_FOLDER;
    this->VINS_RESULT_PATH = this->OUTPUT_FOLDER + "/vio.csv";
    VINS_INFO << "result path " << this->VINS_RESULT_PATH;
    std::ofstream fout(this->VINS_RESULT_PATH, std::ios::out);
    fout.close();

    this->extrinsic_estimation_mode = static_cast<ExtrinsicEstimationMode>(
        (int)fsSettings["estimate_extrinsic"]);
    if (isInitializingExtrinsic()) {
      this->RIC.push_back(Eigen::Matrix3d::Identity());
      this->TIC.push_back(Eigen::Vector3d::Zero());
      this->EX_CALIB_RESULT_PATH =
          this->OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    } else {
      if (isExtrinsicEstimationApproximate()) {
        this->EX_CALIB_RESULT_PATH =
            this->OUTPUT_FOLDER + "/extrinsic_parameter.csv";
      }
      cv::Mat cv_T;
      fsSettings["body_T_cam0"] >> cv_T;
      Eigen::Matrix4d T;
      cv::cv2eigen(cv_T, T);
      this->RIC.push_back(T.block<3, 3>(0, 0));
      this->TIC.push_back(T.block<3, 1>(0, 3));
    }

    this->num_of_camera = (int)fsSettings["num_of_cam"];
    if (this->num_of_camera != 1 && this->num_of_camera != 2) {
      VINS_ERROR << "num of cam should be 1 or 2 ";
    }

    int pn = (int)config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    this->camera_names.push_back(cam0Path);

    if (this->num_of_camera == 2) {
      this->stereo = 1;
      std::string cam1Calib;
      fsSettings["cam1_calib"] >> cam1Calib;
      std::string cam1Path = configPath + "/" + cam1Calib;
      this->camera_names.push_back(cam1Path);

      cv::Mat cv_T;
      fsSettings["body_T_cam1"] >> cv_T;
      Eigen::Matrix4d T;
      cv::cv2eigen(cv_T, T);
      this->RIC.push_back(T.block<3, 3>(0, 0));
      this->TIC.push_back(T.block<3, 1>(0, 3));
    }
    VINS_INFO << "STEREO: " << this->stereo;

    this->time_delay = (double)fsSettings["td"];
    this->estimate_td_mode = (int)fsSettings["estimate_td"];
    if (!this->hasImu()) {
      this->extrinsic_estimation_mode = ExtrinsicEstimationMode::FIXED;
      this->estimate_td_mode = 0;
    }

    fsSettings.release();
  }

  std::string imuTopic() const { return this->imu.imuTopic(); }
  std::string imageTopic() const { return this->image0_topic; }
  std::string image1Topic() const { return this->image1_topic; }
  int max_num_iterations() const { return max_iterations; }
  double max_solver_time() const { return solver_time; }
  /// @brief Get the number of cameras in the current configuration
  int getNumCameras() const { return num_of_camera; }
  /// @brief Check whether the system is configured for stereo camera input
  bool isUsingStereo() const { return stereo; }
  /// @brief Check whether the current configuration uses IMU
  bool hasImu() const { return imu.hasImu(); }
  /// @brief Check if the configuration is monocular without IMU
  bool isMonoWithoutImu() const { return !isUsingStereo() && !hasImu(); }
  /// @brief Check if the configuration is monocular with IMU
  bool isMonoWithImu() const { return !isUsingStereo() && hasImu(); }
  /// @brief Check if the configuration is stereo without IMU
  bool isStereoWithoutImu() const { return isUsingStereo() && !hasImu(); }
  /// @brief Check if the configuration is stereo with IMU
  bool isStereoWithImu() const { return isUsingStereo() && hasImu(); }
  bool shouldShowTrack() const { return show_track; }
  bool hasStereoTrackDiversity() const {
    return isUsingStereo() && enable_stereo_track_diversity != 0;
  }
  bool shouldTraceFeatureFrame(double timestamp) const {
    return enable_feature_frame_trace != 0 &&
           (feature_frame_trace_start_timestamp < 0.0 ||
            timestamp >= feature_frame_trace_start_timestamp) &&
           (feature_frame_trace_end_timestamp < 0.0 ||
            timestamp <= feature_frame_trace_end_timestamp);
  }
  bool isExtrinsicEstimationApproximate() const {
    return extrinsic_estimation_mode == ExtrinsicEstimationMode::APPROXIMATE;
  }
  bool isInitializingExtrinsic() const {
    return extrinsic_estimation_mode == ExtrinsicEstimationMode::INITIALIZE;
  }
  bool shouldEstimateTD() const { return estimate_td_mode; }
  bool hasRange() const { return use_range != 0; }
  bool hasGpsVelocity() const { return use_gps_velocity != 0; }
  bool hasGpsDropoutGyroBiasPrior() const {
    return use_gps_dropout_gyro_bias_prior != 0;
  }
  bool hasPoseOnlyVisual() const {
    return use_pose_only_visual != 0;
  }
  bool replacesFeatureVisual() const {
    return use_pose_only_visual == 1;
  }
  bool augmentsFeatureVisual() const {
    return use_pose_only_visual == 2;
  }
  bool hasRotorDragDynamics() const {
    return use_rotor_drag_dynamics != 0;
  }
};

enum SIZE_PARAMETERIZATION {
  SIZE_POSE = 7,
  SIZE_SPEEDBIAS = 9,
  SIZE_FEATURE = 1
};

enum StateOrder { O_P = 0, O_R = 3, O_V = 6, O_BA = 9, O_BG = 12 };

enum NoiseOrder { O_AN = 0, O_GN = 3, O_AW = 6, O_GW = 9 };
