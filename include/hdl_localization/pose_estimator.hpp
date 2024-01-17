#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include <vector>
#include <robot_localization/robot_localization_estimator.h>
#include <robot_localization/ros_filter_utilities.h>

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <robot_localization/ukf.h>
#include "robot_localization/filter_common.h"
#include "tf2/LinearMath/Quaternion.h"

namespace kkl
{
namespace alg
{
template <typename T, class System>
class UnscentedKalmanFilterX;
}
}  // namespace kkl

namespace hdl_localization
{
/**
 * @brief scan matching-based pose estimator
 */
class PoseEstimator
{
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief constructor
   * @param registration        registration method
   * @param pos                 initial position
   * @param quat                initial orientation
   * @param cool_time_duration  during "cool time", prediction is not performed
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration,
                const geometry_msgs::PoseWithCovarianceStamped initial_pose, double alpha = 0.001, double kappa = 0.0,
                double beta = 2.0, double cool_time_duration = 1.0);
  ~PoseEstimator();
  void runNdtLocalization(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud,
                          const RobotLocalization::Measurement& measurement);
  void ukfReset(const geometry_msgs::PoseWithCovarianceStamped initial_pose, const Eigen::MatrixXd& process_noise,
                double alpha, double kappa, double beta);
  void setUkfInitialState(RobotLocalization::Measurement initial_state);
  void ukfPredict(ros::Time current_time);
  void ukfCorrect(const RobotLocalization::Measurement& measurement);
  RobotLocalization::Measurement imu2UkfMeasurement(const sensor_msgs::Imu);
  RobotLocalization::Measurement odom2UkfMeasurement(const nav_msgs::Odometry);
  RobotLocalization::Measurement initialPose2Measurement(const geometry_msgs::PoseWithCovarianceStamped initial_pose);
  RobotLocalization::Measurement registration2Measurement(pcl::Registration<PointT, PointT>::Ptr& registration);
  RobotLocalization::Measurement ndtRegistration(const ros::Time& stamp,
                                                 const pcl::PointCloud<PointT>::ConstPtr& cloud);
  RobotLocalization::Measurement combineMeasurements(RobotLocalization::Measurement ndt_measurement,
                                                     RobotLocalization::Measurement motion_measurement);

  /* getters */
  nav_msgs::Odometry getOdometry();
  ros::Time lastMeasurementTime();
  Eigen::Vector3f getPose();
  Eigen::Vector3f getVelocity();
  Eigen::Quaternionf getQuaternion();
  Eigen::Matrix3f getRotationMatrix();
  Eigen::Matrix4f getTransformationMatrix();
  double getFitnessScore();
  double getTransformationProbability();
  bool isAligned();
  pcl::PointCloud<PointT>::Ptr getAlignedPoints();

  const boost::optional<Eigen::Matrix4f>& getNdtTravel() const;
  const boost::optional<Eigen::Matrix4f>& getUkfTravel() const;
  const boost::optional<Eigen::Matrix4f>& getNdtCorrect() const;

private:
  std::unique_ptr<RobotLocalization::Ukf> ukf_;
  void setUkfProcessNoise(const Eigen::MatrixXd& process_noise);
  void setUkfAlpha(double alpla, double kappa, double beta);
  ros::Time predict_current_stamp_;  // when the estimator performed the correction step
  ros::Time predict_prev_stamp_;     // when the estimator was updated last time
  ros::Time measurement_stamp_;      // when the estimator was updated last time
  ros::Time ndt_prev_stamp_;         // when the estimator was updated last time
  double cool_time_duration_;        //
  Eigen::MatrixXd process_noise_, odom_process_noise_;
  double fitness_score_;
  double transformation_probability_;
  bool aligned_;
  pcl::PointCloud<PointT>::Ptr aligned_points_;

  Eigen::Matrix4f last_observation_;
  boost::optional<Eigen::Matrix4f> ukf_travel_;
  boost::optional<Eigen::Matrix4f> ndt_correct_;
  boost::optional<Eigen::Matrix4f> ndt_travel_;

  pcl::Registration<PointT, PointT>::Ptr registration_;
};

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP
