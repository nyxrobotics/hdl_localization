#include <nodelet/hdl_localization_nodelet.hpp>

namespace hdl_localization
{
HdlLocalizationNodelet::HdlLocalizationNodelet() : tf_buffer_(), tf_listener_(tf_buffer_)
{
  init_pose_.setZero();
  init_orientation_.x() = 0.0;
  init_orientation_.y() = 0.0;
  init_orientation_.z() = 0.0;
  init_orientation_.w() = 1.0;
}

HdlLocalizationNodelet::~HdlLocalizationNodelet()
{
}

void HdlLocalizationNodelet::onInit()
{
  nh_ = getNodeHandle();
  mt_nh_ = getMTNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  odom_stamp_last_ = ros::Time::now();

  initializeParams();
  map_frame_ = private_nh_.param<std::string>("map_frame", "map");
  odom_frame_ = private_nh_.param<std::string>("odom_frame", "odom");
  base_frame_ = private_nh_.param<std::string>("base_frame", "base_link");

  specify_init_pose_ = private_nh_.param<bool>("specify_init_pose", false);
  use_odom_frame_ = private_nh_.param<bool>("use_odom_frame", false);
  if (specify_init_pose_)
  {
    init_pose_.x() = private_nh_.param<double>("init_pos_x", 0.0);
    init_pose_.y() = private_nh_.param<double>("init_pos_y", 0.0);
    init_pose_.z() = private_nh_.param<double>("init_pos_z", 0.0);
    init_orientation_.x() = private_nh_.param<double>("init_ori_x", 0.0);
    init_orientation_.y() = private_nh_.param<double>("init_ori_y", 0.0);
    init_orientation_.z() = private_nh_.param<double>("init_ori_z", 0.0);
    init_orientation_.w() = private_nh_.param<double>("init_ori_w", 1.0);
  }
  else if (use_odom_frame_)
  {
    odom_ready_ = false;
    initialize_on_odom_ = private_nh_.param<bool>("initialize_on_odom", true);
    tf_buffer_.canTransform(map_frame_, odom_frame_, ros::Time(0), ros::Duration(1.0));
    if (initialize_on_odom_)
    {
      if (tf_buffer_.canTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(10.0)))
      {
        geometry_msgs::TransformStamped tf_map2base =
            tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(10.0));
        init_pose_.x() = tf_map2base.transform.translation.x;
        init_pose_.y() = tf_map2base.transform.translation.y;
        init_pose_.z() = tf_map2base.transform.translation.z;
        init_orientation_.x() = tf_map2base.transform.rotation.x;
        init_orientation_.y() = tf_map2base.transform.rotation.y;
        init_orientation_.z() = tf_map2base.transform.rotation.z;
        init_orientation_.w() = tf_map2base.transform.rotation.w;
      }
      else
      {
        NODELET_ERROR("Lookup transform failed %s -> %s", map_frame_.c_str(), base_frame_.c_str());
        initialize_on_odom_ = false;
      }
    }
  }

  publish_tf_ = private_nh_.param<bool>("publish_tf", true);
  use_imu_ = private_nh_.param<bool>("use_imu", true);
  if (use_odom_frame_ && use_imu_)
  {
    NODELET_WARN("[HdlLocalizationNodelet] Both use_odom_frame_ and use_imu enabled -> disabling use_imu");
    use_imu_ = false;
  }
  invert_acc_ = private_nh_.param<bool>("invert_acc", false);
  invert_gyro_ = private_nh_.param<bool>("invert_gyro", false);

  // global localization
  use_global_localization_ = private_nh_.param<bool>("use_global_localization", true);
  if (use_global_localization_)
  {
    NODELET_INFO_STREAM("wait for global localization services");
    ros::service::waitForService("/hdl_global_localization/set_global_map");
    ros::service::waitForService("/hdl_global_localization/query");

    set_global_map_service_ = nh_.serviceClient<hdl_global_localization::SetGlobalMap>("/hdl_global_localization/"
                                                                                       "set_global_map");
    query_global_localization_service_ = nh_.serviceClient<hdl_global_localization::QueryGlobalLocalization>("/hdl_"
                                                                                                             "global_"
                                                                                                             "localizat"
                                                                                                             "ion/"
                                                                                                             "query");

    relocalize_server_ = nh_.advertiseService("/relocalize", &HdlLocalizationNodelet::relocalize, this);
  }
  // initialize pose estimator
  NODELET_INFO("initialize pose estimator with specified parameters!!");
  pose_estimator_.reset(new hdl_localization::PoseEstimator(
      registration_, init_pose_, init_orientation_, private_nh_.param<double>("cool_time_duration", 0.1),
      private_nh_.param<double>("fitness_reject", 100.0), private_nh_.param<double>("fitness_reliable", 1.0),
      private_nh_.param<double>("linear_correction_gain", 1.0),
      private_nh_.param<double>("angular_correction_gain", 0.1),
      private_nh_.param<double>("angular_correction_distance_reject", 10.0),
      private_nh_.param<double>("angular_correction_distance_reliable", 0.1)));
  if (use_imu_)
  {
    NODELET_INFO("enable imu-based prediction");
    imu_sub_ = mt_nh_.subscribe("/gpsimu_driver/imu_data", 5, &HdlLocalizationNodelet::imuCallback, this);
  }
  points_sub_ = mt_nh_.subscribe("/velodyne_points", 5, &HdlLocalizationNodelet::pointsCallback, this);
  globalmap_sub_ = nh_.subscribe("/globalmap", 1, &HdlLocalizationNodelet::globalmapCallback, this);
  initialpose_sub_ = nh_.subscribe("/initialpose", 5, &HdlLocalizationNodelet::initialposeCallback, this);

  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 5, false);
  aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);
  status_pub_ = nh_.advertise<ScanMatchingStatus>("/status", 5, false);
}

pclomp::NormalDistributionsTransform<HdlLocalizationNodelet::PointT, HdlLocalizationNodelet::PointT>::Ptr
HdlLocalizationNodelet::createRegistration() const
{
  std::string reg_method = private_nh_.param<std::string>("reg_method", "NDT_OMP");
  std::string ndt_neighbor_search_method = private_nh_.param<std::string>("ndt_neighbor_search_method", "DIRECT7");
  double ndt_neighbor_search_radius = private_nh_.param<double>("ndt_neighbor_search_radius", 2.0);
  double ndt_resolution = private_nh_.param<double>("ndt_resolution", 1.0);

  if (reg_method == "NDT_OMP")
  {
    NODELET_INFO("NDT_OMP is selected");
    pclomp::NormalDistributionsTransform<HdlLocalizationNodelet::PointT, HdlLocalizationNodelet::PointT>::Ptr ndt(
        new pclomp::NormalDistributionsTransform<HdlLocalizationNodelet::PointT, HdlLocalizationNodelet::PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(ndt_resolution);
    if (ndt_neighbor_search_method == "DIRECT1")
    {
      NODELET_INFO("search_method DIRECT1 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    }
    else if (ndt_neighbor_search_method == "DIRECT7")
    {
      NODELET_INFO("search_method DIRECT7 is selected");
      ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    }
    else
    {
      if (ndt_neighbor_search_method == "KDTREE")
      {
        NODELET_INFO("search_method KDTREE is selected");
      }
      else
      {
        NODELET_WARN("invalid search method was given");
        NODELET_WARN("default method is selected (KDTREE)");
      }
      ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
    return ndt;
    // } else if (reg_method.find("NDT_CUDA") != std::string::npos) {
    //   NODELET_INFO("NDT_CUDA is selected");
    //   boost::shared_ptr<fast_gicp::NDTCuda<HdlLocalizationNodelet::PointT, HdlLocalizationNodelet::PointT>> ndt(new
    //   fast_gicp::NDTCuda<HdlLocalizationNodelet::PointT, HdlLocalizationNodelet::PointT>);
    //   ndt->setResolution(ndt_resolution);

    //   if (reg_method.find("D2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    //   } else if (reg_method.find("P2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    //   }

    //   if (ndt_neighbor_search_method == "DIRECT1") {
    //     NODELET_INFO("search_method DIRECT1 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    //   } else if (ndt_neighbor_search_method == "DIRECT7") {
    //     NODELET_INFO("search_method DIRECT7 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    //   } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
    //     NODELET_INFO_STREAM("search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
    //   } else {
    //     NODELET_WARN("invalid search method was given");
    //   }
    //   return ndt;
  }

  NODELET_ERROR_STREAM("unknown registration_ method:" << reg_method);
  return nullptr;
}

void HdlLocalizationNodelet::initializeParams()
{
  // intialize scan matching method
  double downsample_resolution = private_nh_.param<double>("downsample_resolution", 0.1);
  if (downsample_resolution > 0.0)
  {
    boost::shared_ptr<pcl::VoxelGrid<HdlLocalizationNodelet::PointT>> voxelgrid(
        new pcl::VoxelGrid<HdlLocalizationNodelet::PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter_ = voxelgrid;
  }
  else
  {
    ROS_WARN("Input downsample_filter_ is disabled");
    downsample_filter_.reset();
  }

  NODELET_INFO("create registration_ method for localization");
  registration_ = createRegistration();

  // global localization
  NODELET_INFO("create registration_ method for fallback during relocalization");
  relocalizing_ = false;
  delta_estimator_.reset(new DeltaEstimator(createRegistration()));
}

/**
 * @brief callback for imu data
 * @param imu_msg
 */
void HdlLocalizationNodelet::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  std::lock_guard<std::mutex> lock(imu_data_mutex_);
  imu_data_.push_back(imu_msg);
}

/**
 * @brief callback for point cloud data
 * @param points_msg
 */
void HdlLocalizationNodelet::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
  if (!globalmap_)
  {
    NODELET_WARN_THROTTLE(10.0, "Waiting for globalmap");
    return;
  }

  const auto& stamp = points_msg->header.stamp;
  pcl::PointCloud<HdlLocalizationNodelet::PointT>::Ptr pcl_cloud(new pcl::PointCloud<HdlLocalizationNodelet::PointT>());
  pcl::fromROSMsg(*points_msg, *pcl_cloud);

  if (pcl_cloud->empty())
  {
    NODELET_ERROR("cloud is empty!!");
    return;
  }

  ros::Time last_correction_time = pose_estimator_->lastCorrectionTime();
  // Skip calculation if timestamp is wrong
  if (stamp < last_correction_time)
  {
    return;
  }
  // transform pointcloud into base_frame_
  std::string tf_error;
  pcl::PointCloud<HdlLocalizationNodelet::PointT>::Ptr cloud(new pcl::PointCloud<HdlLocalizationNodelet::PointT>());
  if (this->tf_buffer_.canTransform(base_frame_, pcl_cloud->header.frame_id, stamp, ros::Duration(0.1), &tf_error))
  {
    if (!pcl_ros::transformPointCloud(base_frame_, *pcl_cloud, *cloud, this->tf_buffer_))
    {
      NODELET_ERROR("point cloud cannot be transformed into target frame!!");
      return;
    }
  }
  else
  {
    NODELET_ERROR(tf_error.c_str());
    return;
  }

  auto filtered = downsample(cloud);
  last_scan_ = filtered;

  if (relocalizing_)
  {
    delta_estimator_->addFrame(filtered);
  }

  std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex_);
  if (!pose_estimator_)
  {
    NODELET_ERROR("waiting for initial pose input!!");
    return;
  }
  Eigen::Matrix4f before = pose_estimator_->matrix();

  if (use_imu_)
  {
    // PointClouds + IMU prediction
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    auto imu_iter = imu_data_.begin();
    for (imu_iter; imu_iter != imu_data_.end(); imu_iter++)
    {
      if (stamp < (*imu_iter)->header.stamp)
      {
        break;
      }
      const auto& acc = (*imu_iter)->linear_acceleration;
      const auto& gyro = (*imu_iter)->angular_velocity;
      double acc_sign = invert_acc_ ? -1.0 : 1.0;
      double gyro_sign = invert_gyro_ ? -1.0 : 1.0;
      pose_estimator_->predictImu((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
                                  gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
    }
    imu_data_.erase(imu_data_.begin(), imu_iter);
  }
  else if (use_odom_frame_)
  {
    if (!odom_ready_)
    {
      usleep(100000);
      odom_ready_ = tf_buffer_.canTransform(map_frame_, odom_frame_, ros::Time::now(), ros::Duration(10.0));
      if (!odom_ready_)
      {
        NODELET_ERROR("Waiting for %s -> %s transform", base_frame_.c_str(), odom_frame_.c_str());
        return;
      }
    }
    // PointClouds + Oodometry prediction
    if (tf_buffer_.canTransform(base_frame_, odom_stamp_last_, base_frame_, ros::Time(0), odom_frame_,
                                ros::Duration(0.1)))
    {
      // Get the amount of odometry movement since the last calculation
      // Coordinate system where the front of the robot is x
      geometry_msgs::TransformStamped odom_delta =
          tf_buffer_.lookupTransform(base_frame_, odom_stamp_last_, base_frame_, ros::Time(0), odom_frame_);
      // Get the latest base_frame_ to get the time
      geometry_msgs::TransformStamped odom_now = tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0));
      ros::Time odom_stamp = odom_now.header.stamp;
      ros::Duration odom_time_diff = odom_stamp - odom_stamp_last_;
      double odom_time_diff_sec = odom_time_diff.toSec();
      if (odom_delta.header.stamp.isZero() || odom_time_diff_sec <= 0)
      {
        NODELET_WARN_THROTTLE(10.0, "Wrong timestamp detected: odom_time_diff_sec = %f", odom_time_diff_sec);
      }
      else
      {
        Eigen::Vector3f odom_travel_linear(odom_delta.transform.translation.x, odom_delta.transform.translation.y,
                                           odom_delta.transform.translation.z);
        Eigen::Vector3f odom_twist_linear = odom_travel_linear / odom_time_diff_sec;
        tf::Quaternion odom_travel_angular(odom_delta.transform.rotation.x, odom_delta.transform.rotation.y,
                                           odom_delta.transform.rotation.z, odom_delta.transform.rotation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(odom_travel_angular).getRPY(roll, pitch, yaw);
        Eigen::Vector3f odom_twist_angular(roll / odom_time_diff_sec, pitch / odom_time_diff_sec,
                                           yaw / odom_time_diff_sec);
        pose_estimator_->predictOdom(odom_stamp, odom_twist_linear, odom_twist_angular);
      }
      odom_stamp_last_ = odom_stamp;
    }
    else
    {
      if (tf_buffer_.canTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(0.1)))
      {
        NODELET_WARN_THROTTLE(10.0, "The last timestamp is wrong, skip localization");
        // Get the latest base_frame_ to get the time
        geometry_msgs::TransformStamped odom_now = tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0));
        odom_stamp_last_ = odom_now.header.stamp;
      }
      else
      {
        NODELET_WARN_STREAM("Failed to look up transform between " << cloud->header.frame_id << " and " << odom_frame_);
      }
    }
  }
  else
  {
    // PointClouds-only prediction
    pose_estimator_->predict(stamp);
  }
  // Perform scan matching using the calculated position as the initial value
  double pose_covariance[36];
  auto aligned = pose_estimator_->correct(stamp, filtered, pose_covariance);

  if (aligned_pub_.getNumSubscribers())
  {
    aligned->header.frame_id = map_frame_;
    aligned->header.stamp = cloud->header.stamp;
    aligned_pub_.publish(aligned);
  }

  if (status_pub_.getNumSubscribers())
  {
    publishScanMatchingStatus(points_msg->header, aligned);
  }

  publishOdometry(points_msg->header.stamp, pose_estimator_->matrix(), pose_covariance);
}

/**
 * @brief callback for globalmap_ input
 * @param points_msg
 */
void HdlLocalizationNodelet::globalmapCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
  NODELET_INFO("globalmap received!");
  pcl::PointCloud<HdlLocalizationNodelet::PointT>::Ptr cloud(new pcl::PointCloud<HdlLocalizationNodelet::PointT>());
  pcl::fromROSMsg(*points_msg, *cloud);
  globalmap_ = cloud;

  registration_->setInputTarget(globalmap_);

  if (use_global_localization_)
  {
    NODELET_INFO("set globalmap for global localization!");
    hdl_global_localization::SetGlobalMap srv;
    pcl::toROSMsg(*globalmap_, srv.request.global_map);

    if (!set_global_map_service_.call(srv))
    {
      NODELET_INFO("failed to set global map");
    }
    else
    {
      NODELET_INFO("done");
    }
  }
}

/**
 * @brief perform global localization to relocalize the sensor position
 * @param
 */
bool HdlLocalizationNodelet::relocalize(std_srvs::EmptyRequest& /*req*/, std_srvs::EmptyResponse& /*res*/)
{
  if (last_scan_ == nullptr)
  {
    NODELET_INFO_STREAM("no scan has been received");
    return false;
  }

  relocalizing_ = true;
  delta_estimator_->reset();
  pcl::PointCloud<HdlLocalizationNodelet::PointT>::ConstPtr scan = last_scan_;

  hdl_global_localization::QueryGlobalLocalization srv;
  pcl::toROSMsg(*scan, srv.request.cloud);
  srv.request.max_num_candidates = 1;

  if (!query_global_localization_service_.call(srv) || srv.response.poses.empty())
  {
    relocalizing_ = false;
    NODELET_INFO_STREAM("global localization failed");
    return false;
  }

  const auto& result = srv.response.poses[0];

  NODELET_INFO_STREAM("--- Global localization result ---");
  NODELET_INFO_STREAM("Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
  NODELET_INFO_STREAM("Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z
                                << " " << result.orientation.w);
  NODELET_INFO_STREAM("Error :" << srv.response.errors[0]);
  NODELET_INFO_STREAM("Inlier:" << srv.response.inlier_fractions[0]);

  Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
  pose.linear() =
      Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z)
          .toRotationMatrix();
  pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
  pose = pose * delta_estimator_->estimatedDelta();

  std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
  pose_estimator_.reset(new hdl_localization::PoseEstimator(
      registration_, pose.translation(), Eigen::Quaternionf(pose.linear()),
      private_nh_.param<double>("cool_time_duration", 0.1), private_nh_.param<double>("fitness_reject", 100.0),
      private_nh_.param<double>("fitness_reliable", 1.0), private_nh_.param<double>("linear_correction_gain", 1.0),
      private_nh_.param<double>("angular_correction_gain", 0.1),
      private_nh_.param<double>("angular_correction_distance_reject", 10.0),
      private_nh_.param<double>("angular_correction_distance_reliable", 0.1)));

  relocalizing_ = false;

  return true;
}

/**
 * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
 * @param pose_msg
 */
void HdlLocalizationNodelet::initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  NODELET_INFO("[hdl_localization] initial pose received");
  geometry_msgs::TransformStamped tf_map2global;
  if (map_frame_ != msg->header.frame_id)
  {
    tf_map2global = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
    if (tf_map2global.header.frame_id != map_frame_)
    {
      ROS_ERROR("[hdl_localization] Failed to get transform from %s to %s", map_frame_.c_str(),
                msg->header.frame_id.c_str());
      tf_map2global.transform.translation.x = 0.0;
      tf_map2global.transform.translation.y = 0.0;
      tf_map2global.transform.translation.z = 0.0;
      tf_map2global.transform.rotation.x = 0.0;
      tf_map2global.transform.rotation.y = 0.0;
      tf_map2global.transform.rotation.z = 0.0;
      tf_map2global.transform.rotation.w = 1.0;
    }
  }
  else
  {
    tf_map2global.transform.translation.x = 0.0;
    tf_map2global.transform.translation.y = 0.0;
    tf_map2global.transform.translation.z = 0.0;
    tf_map2global.transform.rotation.x = 0.0;
    tf_map2global.transform.rotation.y = 0.0;
    tf_map2global.transform.rotation.z = 0.0;
    tf_map2global.transform.rotation.w = 1.0;
  }
  tf2::Vector3 vec_global2robot(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  tf2::Quaternion q_global2robot(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // Get vector from odom to robot
  tf2::Vector3 vec_map2global(tf_map2global.transform.translation.x, tf_map2global.transform.translation.y,
                              tf_map2global.transform.translation.z);
  tf2::Quaternion q_map2global(tf_map2global.transform.rotation.x, tf_map2global.transform.rotation.y,
                               tf_map2global.transform.rotation.z, tf_map2global.transform.rotation.w);
  tf2::Vector3 vec_map2robot = vec_map2global + (tf2::Matrix3x3(q_map2global) * vec_global2robot);
  tf2::Quaternion q_map2robot = q_map2global * q_global2robot;
  init_pose_.x() = vec_map2robot.x();
  init_pose_.y() = vec_map2robot.y();
  init_pose_.z() = vec_map2robot.z();

  init_orientation_.x() = q_map2robot.x();
  init_orientation_.y() = q_map2robot.y();
  init_orientation_.z() = q_map2robot.z();
  init_orientation_.w() = q_map2robot.w();
  if (use_odom_frame_)
  {
    odom_stamp_last_ = msg->header.stamp;
    odom_ready_ = false;
  }
  if (use_imu_)
  {
    imu_data_.clear();
  }
  std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
  pose_estimator_.reset(new hdl_localization::PoseEstimator(
      registration_, init_pose_, init_orientation_, private_nh_.param<double>("cool_time_duration", 0.1),
      private_nh_.param<double>("fitness_reject", 100.0), private_nh_.param<double>("fitness_reliable", 1.0),
      private_nh_.param<double>("linear_correction_gain", 1.0),
      private_nh_.param<double>("angular_correction_gain", 0.1),
      private_nh_.param<double>("angular_correction_distance_reject", 10.0),
      private_nh_.param<double>("angular_correction_distance_reliable", 0.1)));
}

/**
 * @brief downsampling
 * @param cloud   input cloud
 * @return downsampled cloud
 */
pcl::PointCloud<HdlLocalizationNodelet::PointT>::ConstPtr
HdlLocalizationNodelet::downsample(const pcl::PointCloud<HdlLocalizationNodelet::PointT>::ConstPtr& cloud) const
{
  if (!downsample_filter_)
  {
    return cloud;
  }

  pcl::PointCloud<HdlLocalizationNodelet::PointT>::Ptr filtered(new pcl::PointCloud<HdlLocalizationNodelet::PointT>());
  downsample_filter_->setInputCloud(cloud);
  downsample_filter_->filter(*filtered);
  filtered->header = cloud->header;

  return filtered;
}

/**
 * @brief publish odometry
 * @param stamp  timestamp
 * @param pose   odometry pose to be published
 */
void HdlLocalizationNodelet::publishOdometry(const ros::Time& stamp, const Eigen::Matrix4f& pose,
                                             double pose_covariance[36])
{
  // broadcast the transform over tf
  if (publish_tf_)
  {
    if (tf_buffer_.canTransform(odom_frame_, base_frame_, ros::Time(0)))
    {
      geometry_msgs::TransformStamped map_wrt_frame =
          tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
      map_wrt_frame.header.stamp = stamp;
      map_wrt_frame.header.frame_id = base_frame_;
      map_wrt_frame.child_frame_id = map_frame_;

      geometry_msgs::TransformStamped frame_wrt_odom =
          tf_buffer_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
      Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

      geometry_msgs::TransformStamped map_wrt_odom;
      tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

      tf2::Transform odom_wrt_map;
      tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
      odom_wrt_map = odom_wrt_map.inverse();

      geometry_msgs::TransformStamped odom_trans;
      odom_trans.transform = tf2::toMsg(odom_wrt_map);
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = map_frame_;
      odom_trans.child_frame_id = odom_frame_;

      tf_broadcaster_.sendTransform(odom_trans);
    }
    else
    {
      geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
      odom_trans.header.stamp = stamp;
      odom_trans.header.frame_id = map_frame_;
      odom_trans.child_frame_id = base_frame_;
      tf_broadcaster_.sendTransform(odom_trans);
    }
  }

  // publish the transform
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;

  tf::poseEigenToMsg(Eigen::Isometry3d(pose.cast<double>()), odom.pose.pose);
  odom.pose.covariance[0] = private_nh_.param<double>("cov_scaling_factor_x", 1.0) * pose_covariance[0];
  odom.pose.covariance[7] = private_nh_.param<double>("cov_scaling_factor_y", 1.0) * pose_covariance[7];
  odom.pose.covariance[14] = private_nh_.param<double>("cov_scaling_factor_z", 1.0) * pose_covariance[14];
  odom.pose.covariance[21] = private_nh_.param<double>("cov_scaling_factor_roll", 1.0) * pose_covariance[21];
  odom.pose.covariance[28] = private_nh_.param<double>("cov_scaling_factor_pitch", 1.0) * pose_covariance[28];
  odom.pose.covariance[35] = private_nh_.param<double>("cov_scaling_factor_yaw", 1.0) * pose_covariance[35];

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;

  pose_pub_.publish(odom);
}

/**
 * @brief publish scan matching status information
 */
void HdlLocalizationNodelet::publishScanMatchingStatus(const std_msgs::Header& header,
                                                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned)
{
  ScanMatchingStatus status;
  status.header = header;

  status.has_converged = registration_->hasConverged();
  status.matching_error = 0.0;

  const double max_correspondence_dist = private_nh_.param<double>("status_max_correspondence_dist", 0.5);
  const double max_valid_point_dist = private_nh_.param<double>("status_max_valid_point_dist", 25.0);

  int num_inliers = 0;
  int num_valid_points = 0;
  std::vector<int> k_indices;
  std::vector<float> k_sq_dists;
  for (int i = 0; i < aligned->size(); i++)
  {
    const auto& pt = aligned->at(i);
    if (pt.getVector3fMap().norm() > max_valid_point_dist)
    {
      continue;
    }
    num_valid_points++;

    registration_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
    if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist)
    {
      status.matching_error += k_sq_dists[0];
      num_inliers++;
    }
  }

  status.matching_error /= num_inliers;
  status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
  status.relative_pose =
      tf2::eigenToTransform(Eigen::Isometry3d(registration_->getFinalTransformation().cast<double>())).transform;

  status.prediction_labels.reserve(2);
  status.prediction_errors.reserve(2);

  std::vector<double> errors(6, 0.0);

  if (pose_estimator_->withoutPredError())
  {
    status.prediction_labels.push_back(std_msgs::String());
    status.prediction_labels.back().data = "without_prediction";
    status.prediction_errors.push_back(
        tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->withoutPredError().get().cast<double>())).transform);
  }

  if (pose_estimator_->motionPredError())
  {
    status.prediction_labels.push_back(std_msgs::String());
    if (use_imu_)
    {
      status.prediction_labels.back().data = "imu_prediction";
    }
    else if (use_odom_frame_)
    {
      status.prediction_labels.back().data = "odom_prediction";
    }
    else
    {
      status.prediction_labels.back().data = "motion_model";
    }
    status.prediction_errors.push_back(
        tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->motionPredError().get().cast<double>())).transform);
  }

  status_pub_.publish(status);
}

}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::HdlLocalizationNodelet, nodelet::Nodelet)
