#ifndef VINS_ESTIMATOR_H
#define VINS_ESTIMATOR_H
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <vins/estimator/estimator.h>
#include <vins_fusion_ros2/visualization.h>
#include <vins_fusion_ros2/effective_motor_quality_join.h>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cstdint>
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <fstream>

using sensor_msgs::msg::Image;
using namespace message_filters;

class VinsEstimator : public rclcpp::Node {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<Image, Image>;

  VinsEstimator();
  ~VinsEstimator();

  void initialize();
  void initializeParamters();
  void initializeSubscribers();
  void initializerPublishers();

 private:
  void timeCallback();
  void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img0,
                      const sensor_msgs::msg::Image::ConstSharedPtr& img1);

  void publishPointCloud();
  void publishImuData();
  void publishOdometry();
  void publishImage();
  void publishKeyFrameData();
  void recordReplayInputConsumed(const std::string& topic,
                                 uint64_t topic_sequence,
                                 int64_t timestamp_ns,
                                 uint64_t payload_fingerprint,
                                 int64_t canonical_timestamp_ns = -1);
  void tryConsumeEffectiveMotor(int64_t timestamp_ns);

 private:
  std::shared_ptr<Estimator> estimator_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::shared_ptr<Subscriber<Image>> sub_img0_filter_;
  std::shared_ptr<Subscriber<Image>> sub_img1_filter_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_img_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr feature_callback_group_;
  rclcpp::CallbackGroup::SharedPtr range_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_track;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry,
      pub_latest_odometry, pub_keyframe_pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_point_cloud,
      pub_margin_cloud, pub_keyframe_point;

  //
  std::string world_frame_id;
  std::string body_frame_id;
  std::string camera_frame_id;

  nav_msgs::msg::Path path;
  std::shared_ptr<VINSOptions> options;
  uint64_t image_callback_sequence_ = 0;
  uint64_t imu_callback_sequence_ = 0;
  uint64_t range_callback_sequence_ = 0;
  std::atomic<uint64_t> callback_delivery_sequence_{0};
  int replay_skip_initial_images_remaining_ = 0;
  bool gps_velocity_integration_initialized_ = false;
  uint64_t gps_velocity_integration_segment_ = 0;
  double previous_gps_velocity_timestamp_ = 0.0;
  double gps_segment_origin_latitude_deg_ = 0.0;
  double gps_segment_origin_longitude_deg_ = 0.0;
  bool motor_model_initialized_ = false;
  bool motor_callback_logged_ = false;
  bool valid_motor_callback_logged_ = false;
  double previous_motor_sample_timestamp_ = 0.0;
  std::array<double, 4> rotor_speed_rad_s_{{0.0, 0.0, 0.0, 0.0}};
  bool use_source_neutral_motor_input_ = false;
  std::string effective_motor_state_topic_ =
      "/vio/motor_input/effective_motor_state";
  std::string effective_motor_quality_topic_ =
      "/vio/motor_input/effective_motor_quality";
  std::unique_ptr<vins_fusion_ros2::EffectiveMotorQualityJoin>
      effective_motor_join_;
  std::mutex effective_motor_join_mutex_;
  uint64_t effective_motor_state_callback_sequence_ = 0;
  uint64_t effective_motor_quality_callback_sequence_ = 0;
  uint64_t motor_callback_sequence_ = 0;
  uint64_t gps_callback_sequence_ = 0;
  uint64_t replay_consumed_sequence_ = 0;
  std::ofstream replay_input_ledger_stream_;
  std::mutex replay_input_ledger_mutex_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      replay_input_ack_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      replay_clock_barrier_ack_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      replay_input_contract_subscription_;
  std::mutex replay_input_contract_mutex_;
  uint64_t replay_last_clock_barrier_sequence_ = 0;
  std::string replay_expected_topic_;
  uint64_t replay_expected_topic_sequence_ = 0;
  int64_t replay_expected_canonical_timestamp_ns_ = -1;
};

#endif  // VINS_ESTIMATOR_H
