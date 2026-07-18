#include <cv_bridge/cv_bridge.h>
#include <vins_fusion_ros2/vins_estimator.h>

#include <cstdint>

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
}  // namespace

VinsEstimator::VinsEstimator() : rclcpp::Node("vins_estimator") {
  options = std::make_shared<VINSOptions>();
  estimator_ = std::make_shared<Estimator>();
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  initialize();
}

VinsEstimator::~VinsEstimator() {}

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
  estimator_->initialize(options);
  const int input_downsample_phase =
      this->declare_parameter<int>("input_downsample_phase", 0);
  estimator_->setInputDownsamplePhase(input_downsample_phase);
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
    auto imu = this->create_subscription<sensor_msgs::msg::Imu>(
        options->imuTopic(), rclcpp::QoS(rclcpp::KeepLast(100)),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          auto imu_msg = fromMsg(*msg);
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
          data.range = msg->range;
          data.valid = std::isfinite(data.range) &&
                       data.range >= msg->min_range &&
                       data.range >= options->range_min_valid &&
                       data.range <= msg->max_range;
          estimator_->inputRange(data);
        },
        sub_opt_range);
    subs_.push_back(range);
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
}

void VinsEstimator::stereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img0,
    const sensor_msgs::msg::Image::ConstSharedPtr& img1) {
  ImageData image;
  image.image0 = fromMsg(*img0);
  image.image1 = fromMsg(*img1);
  image.timestamp = fromMsg(img0->header.stamp);
  image.sequence = ++image_callback_sequence_;
  VINS_INFO << "Camera callback trace: sequence=" << image.sequence
            << ", timestamp=" << std::fixed << std::setprecision(9)
            << image.timestamp << ", fingerprint=0x" << std::hex
            << imageFingerprint(image.image0) << std::dec
            << ", size=" << image.image0.cols << "x" << image.image0.rows
            << ", encoding=" << img0->encoding;
  estimator_->inputImage(image);
}

void VinsEstimator::timeCallback() {
  publishImuData();
  publishOdometry();
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
