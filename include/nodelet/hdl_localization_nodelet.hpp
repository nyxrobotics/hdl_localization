#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/ScanMatchingStatus.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>

namespace hdl_localization {

class HdlLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet();
  ~HdlLocalizationNodelet() override;

  void onInit() override;

private:
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr createRegistration() const;
  void initializeParams();

private:
  /**
   * @brief callback for imu data
   * @param imu_msg
   */
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmapCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);

  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  bool relocalize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  /**
   * @brief downsampling
   * @param cloud   input cloud
   * @return downsampled cloud
   */
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const;
  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publishOdometry(const ros::Time& stamp, const Eigen::Matrix4f& pose, double pose_covariance[36]);

  /**
   * @brief publish scan matching status information
   */
  void publishScanMatchingStatus(const std_msgs::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned);

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle private_nh_;

  bool use_odom_frame_;
  bool odom_ready_;
  bool initialize_on_odom_;
  bool specify_init_pose_;
  Eigen::Vector3f init_pose_;
  Eigen::Quaternionf init_orientation_;
  ros::Time odom_stamp_last_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_frame_;
  bool publish_tf_;

  bool use_imu_;
  bool invert_acc_;
  bool invert_gyro_;
  ros::Subscriber imu_sub_;
  ros::Subscriber points_sub_;
  ros::Subscriber globalmap_sub_;
  ros::Subscriber initialpose_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher status_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // imu input buffer
  std::mutex imu_data_mutex_;
  std::vector<sensor_msgs::ImuConstPtr> imu_data_;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap_;
  pcl::Filter<PointT>::Ptr downsample_filter_;
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr registration_;

  // pose estimator
  std::mutex pose_estimator_mutex_;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator_;

  // global localization
  bool use_global_localization_;
  std::atomic_bool relocalizing_;
  std::unique_ptr<DeltaEstimater> delta_estimater_;

  pcl::PointCloud<PointT>::ConstPtr last_scan_;
  ros::ServiceServer relocalize_server_;
  ros::ServiceClient set_global_map_service_;
  ros::ServiceClient query_global_localization_service_;
};  // namespace hdl_localization
}  // namespace hdl_localization
