#include <hdl_localization/pose_estimator.hpp>

namespace hdl_localization
{
PoseEstimator::PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration,
                             const geometry_msgs::PoseWithCovarianceStamped initial_pose, double alpha, double kappa,
                             double beta, double cool_time_duration)
  : registration_(registration)
  , cool_time_duration_(cool_time_duration)
  , fitness_score_(0.0)
  , transformation_probability_(0.0)
  , aligned_(false)
  , converged_(false)
{
  // Set the initialization timestamp
  predict_current_stamp_ = ros::Time::now();
  predict_prev_stamp_ = predict_current_stamp_;
  // Initialize UKF
  const Eigen::MatrixXd process_noise =
      Eigen::MatrixXd::Identity(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE) * 0.01;

  std::vector<double> args = { alpha, kappa, beta };
  // Set the initial state

  ROS_WARN_STREAM("UKF alpha: " << alpha << ", kappa: " << kappa << ", beta: " << beta);
  ukfReset(initial_pose, process_noise, args[0], args[1], args[2]);
  ROS_WARN_STREAM("UKF initialized with initial pose: "
                  << initial_pose.pose.pose.position.x << ", " << initial_pose.pose.pose.position.y << ", "
                  << initial_pose.pose.pose.position.z << ", " << initial_pose.pose.pose.orientation.x << ", "
                  << initial_pose.pose.pose.orientation.y << ", " << initial_pose.pose.pose.orientation.z << ", "
                  << initial_pose.pose.pose.orientation.w);
}

PoseEstimator::~PoseEstimator()
{
}

void PoseEstimator::runNdtLocalization(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud,
                                       const RobotLocalization::Measurement& measurement)
{
  // Predict
  ukfPredict(stamp);
  RobotLocalization::Measurement ndt_measurement = ndtRegistration(stamp, cloud);
  RobotLocalization::Measurement combined_measurement = combineMeasurements(ndt_measurement, measurement);
  // Correct
  ukfCorrect(combined_measurement);
}

void PoseEstimator::ukfReset(const geometry_msgs::PoseWithCovarianceStamped initial_pose,
                             const Eigen::MatrixXd& process_noise, double alpha, double kappa, double beta)
{
  ROS_WARN_STREAM("UKF Set alpha");
  setUkfAlpha(alpha, kappa, beta);
  if (process_noise.rows() != RobotLocalization::STATE_SIZE || process_noise.cols() != RobotLocalization::STATE_SIZE)
  {
    ROS_ERROR_STREAM("Invalid process noise matrix size");
  }
  else
  {
    ROS_WARN_STREAM("UKF Set process_noise");
    setUkfProcessNoise(process_noise);
  }
  ROS_WARN_STREAM("UKF get initial_state");
  RobotLocalization::Measurement initial_state = initialPose2Measurement(initial_pose);
  ROS_WARN_STREAM("UKF Set initial_state");
  setUkfInitialState(initial_state);
  ROS_WARN_STREAM("UKF Reset finished");
}

void PoseEstimator::setUkfInitialState(RobotLocalization::Measurement initial_state)
{
  ukf_->correct(initial_state);
}

void PoseEstimator::ukfPredict(ros::Time current_time)
{
  double delta = (current_time - predict_current_stamp_).toSec();
  predict_current_stamp_ = current_time;
  predict_prev_stamp_ = predict_current_stamp_;
  Eigen::Vector3f last_pose = getPose();
  Eigen::Quaternionf last_quaternion = getQuaternion();
  ukf_->predict(predict_current_stamp_.toSec(), delta);
  Eigen::Vector3f current_pose = getPose();
  Eigen::Quaternionf current_quaternion = getQuaternion();
  ukf_predict_trans_ = Eigen::Matrix4f::Identity();
  ukf_predict_trans_->block<3, 1>(0, 3) = current_pose - last_pose;
  ukf_predict_trans_->block<3, 3>(0, 0) =
      current_quaternion.toRotationMatrix() * last_quaternion.toRotationMatrix().inverse();
}

void PoseEstimator::ukfCorrect(const RobotLocalization::Measurement& measurement)
{
  Eigen::Vector3f last_pose = getPose();
  Eigen::Quaternionf last_quaternion = getQuaternion();
  ukf_->correct(measurement);
  Eigen::Vector3f current_pose = getPose();
  Eigen::Quaternionf current_quaternion = getQuaternion();
  ukf_correct_trans_ = Eigen::Matrix4f::Identity();
  ukf_correct_trans_->block<3, 1>(0, 3) = current_pose - last_pose;
  ukf_correct_trans_->block<3, 3>(0, 0) =
      current_quaternion.toRotationMatrix() * last_quaternion.toRotationMatrix().inverse();
}

RobotLocalization::Measurement PoseEstimator::imu2UkfMeasurement(const sensor_msgs::Imu imu_msg)
{
  RobotLocalization::Measurement measurement;
  measurement.measurement_.resize(RobotLocalization::STATE_SIZE, 1);
  measurement.covariance_.resize(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  measurement.updateVector_.resize(RobotLocalization::STATE_SIZE, 0);
  measurement.time_ = imu_msg.header.stamp.toSec();
  measurement.measurement_[RobotLocalization::StateMemberAx] = imu_msg.linear_acceleration.x;
  measurement.measurement_[RobotLocalization::StateMemberAy] = imu_msg.linear_acceleration.y;
  measurement.measurement_[RobotLocalization::StateMemberAz] = imu_msg.linear_acceleration.z;
  measurement.measurement_[RobotLocalization::StateMemberVroll] = imu_msg.angular_velocity.x;
  measurement.measurement_[RobotLocalization::StateMemberVpitch] = imu_msg.angular_velocity.y;
  measurement.measurement_[RobotLocalization::StateMemberVyaw] = imu_msg.angular_velocity.z;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  covariance(RobotLocalization::StateMemberAx, RobotLocalization::StateMemberAx) =
      imu_msg.linear_acceleration_covariance[0];
  covariance(RobotLocalization::StateMemberAy, RobotLocalization::StateMemberAy) =
      imu_msg.linear_acceleration_covariance[4];
  covariance(RobotLocalization::StateMemberAz, RobotLocalization::StateMemberAz) =
      imu_msg.linear_acceleration_covariance[8];
  covariance(RobotLocalization::StateMemberVroll, RobotLocalization::StateMemberVroll) =
      imu_msg.angular_velocity_covariance[0];
  covariance(RobotLocalization::StateMemberVpitch, RobotLocalization::StateMemberVpitch) =
      imu_msg.angular_velocity_covariance[4];
  covariance(RobotLocalization::StateMemberVyaw, RobotLocalization::StateMemberVyaw) =
      imu_msg.angular_velocity_covariance[8];
  measurement.covariance_ = covariance;
  std::vector<int> update_flags(RobotLocalization::STATE_SIZE, 0);
  update_flags[RobotLocalization::StateMemberAx] = 1;
  update_flags[RobotLocalization::StateMemberAy] = 1;
  update_flags[RobotLocalization::StateMemberAz] = 1;
  update_flags[RobotLocalization::StateMemberVroll] = 1;
  update_flags[RobotLocalization::StateMemberVpitch] = 1;
  update_flags[RobotLocalization::StateMemberVyaw] = 1;
  measurement.updateVector_ = update_flags;
  return measurement;
}

RobotLocalization::Measurement PoseEstimator::odom2UkfMeasurement(const nav_msgs::Odometry odom_msg)
{
  RobotLocalization::Measurement measurement;
  measurement.measurement_.resize(RobotLocalization::STATE_SIZE, 1);
  measurement.covariance_.resize(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  measurement.updateVector_.resize(RobotLocalization::STATE_SIZE, 0);
  measurement.time_ = odom_msg.header.stamp.toSec();
  measurement.measurement_[RobotLocalization::StateMemberVx] = odom_msg.twist.twist.linear.x;
  measurement.measurement_[RobotLocalization::StateMemberVy] = odom_msg.twist.twist.linear.y;
  measurement.measurement_[RobotLocalization::StateMemberVz] = odom_msg.twist.twist.linear.z;
  measurement.measurement_[RobotLocalization::StateMemberVroll] = odom_msg.twist.twist.angular.x;
  measurement.measurement_[RobotLocalization::StateMemberVpitch] = odom_msg.twist.twist.angular.y;
  measurement.measurement_[RobotLocalization::StateMemberVyaw] = odom_msg.twist.twist.angular.z;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  covariance(RobotLocalization::StateMemberVx, RobotLocalization::StateMemberVx) = odom_msg.twist.covariance[0];
  covariance(RobotLocalization::StateMemberVy, RobotLocalization::StateMemberVy) = odom_msg.twist.covariance[7];
  covariance(RobotLocalization::StateMemberVz, RobotLocalization::StateMemberVz) = odom_msg.twist.covariance[14];
  covariance(RobotLocalization::StateMemberVroll, RobotLocalization::StateMemberVroll) = odom_msg.twist.covariance[21];
  covariance(RobotLocalization::StateMemberVpitch, RobotLocalization::StateMemberVpitch) =
      odom_msg.twist.covariance[28];
  covariance(RobotLocalization::StateMemberVyaw, RobotLocalization::StateMemberVyaw) = odom_msg.twist.covariance[35];
  measurement.covariance_ = covariance;
  std::vector<int> update_flags(RobotLocalization::STATE_SIZE, 0);
  update_flags[RobotLocalization::StateMemberVx] = 1;
  update_flags[RobotLocalization::StateMemberVy] = 1;
  update_flags[RobotLocalization::StateMemberVz] = 1;
  update_flags[RobotLocalization::StateMemberVroll] = 1;
  update_flags[RobotLocalization::StateMemberVpitch] = 1;
  update_flags[RobotLocalization::StateMemberVyaw] = 1;
  measurement.updateVector_ = update_flags;
  return measurement;
}

RobotLocalization::Measurement
PoseEstimator::initialPose2Measurement(const geometry_msgs::PoseWithCovarianceStamped initial_pose)
{
  ROS_WARN("initialPose2Measurement");
  ROS_WARN_STREAM(
      "initial_pose: " << initial_pose.pose.pose.position.x << ", " << initial_pose.pose.pose.position.y << ", "
                       << initial_pose.pose.pose.position.z << ", " << initial_pose.pose.pose.orientation.x << ", "
                       << initial_pose.pose.pose.orientation.y << ", " << initial_pose.pose.pose.orientation.z << ", "
                       << initial_pose.pose.pose.orientation.w);
  RobotLocalization::Measurement measurement;
  measurement.measurement_.resize(RobotLocalization::STATE_SIZE, 1);
  measurement.covariance_.resize(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  measurement.updateVector_.resize(RobotLocalization::STATE_SIZE, 0);
  ROS_WARN("Set time");
  measurement.time_ = ros::Time::now().toSec();
  ROS_WARN("Set measurement");
  measurement.measurement_[RobotLocalization::StateMemberX] = initial_pose.pose.pose.position.x;
  measurement.measurement_[RobotLocalization::StateMemberY] = initial_pose.pose.pose.position.y;
  measurement.measurement_[RobotLocalization::StateMemberZ] = initial_pose.pose.pose.position.z;
  ROS_WARN("Set quaternion");
  tf2::Quaternion tf2_quat(initial_pose.pose.pose.orientation.x, initial_pose.pose.pose.orientation.y,
                           initial_pose.pose.pose.orientation.z, initial_pose.pose.pose.orientation.w);
  double initial_roll, initial_pitch, initial_yaw;
  RobotLocalization::RosFilterUtilities::quatToRPY(tf2_quat, initial_roll, initial_pitch, initial_yaw);
  measurement.measurement_[RobotLocalization::StateMemberRoll] = initial_roll;
  measurement.measurement_[RobotLocalization::StateMemberPitch] = initial_pitch;
  measurement.measurement_[RobotLocalization::StateMemberYaw] = initial_yaw;
  ROS_WARN("Set covariance");
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  double initial_covariance = 1e-6;
  covariance(RobotLocalization::StateMemberX, RobotLocalization::StateMemberX) = initial_covariance;
  covariance(RobotLocalization::StateMemberY, RobotLocalization::StateMemberY) = initial_covariance;
  covariance(RobotLocalization::StateMemberZ, RobotLocalization::StateMemberZ) = initial_covariance;
  covariance(RobotLocalization::StateMemberRoll, RobotLocalization::StateMemberRoll) = initial_covariance;
  covariance(RobotLocalization::StateMemberPitch, RobotLocalization::StateMemberPitch) = initial_covariance;
  covariance(RobotLocalization::StateMemberYaw, RobotLocalization::StateMemberYaw) = initial_covariance;
  measurement.covariance_ = covariance;
  ROS_WARN("Set updateVector");
  std::vector<int> update_flags(RobotLocalization::STATE_SIZE, 0);
  update_flags[RobotLocalization::StateMemberX] = 1;
  update_flags[RobotLocalization::StateMemberY] = 1;
  update_flags[RobotLocalization::StateMemberZ] = 1;
  update_flags[RobotLocalization::StateMemberRoll] = 1;
  update_flags[RobotLocalization::StateMemberPitch] = 1;
  update_flags[RobotLocalization::StateMemberYaw] = 1;
  measurement.updateVector_ = update_flags;
  ROS_WARN("initialPose2Measurement finished");
  return measurement;
}

RobotLocalization::Measurement PoseEstimator::ndtRegistration(const ros::Time& stamp,
                                                              const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  RobotLocalization::Measurement measurement;
  measurement.measurement_.resize(RobotLocalization::STATE_SIZE, 1);
  measurement.covariance_.resize(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE);
  measurement.updateVector_.resize(RobotLocalization::STATE_SIZE, 0);
  std::vector<int> update_flags(RobotLocalization::STATE_SIZE, 0);
  measurement.updateVector_ = update_flags;
  measurement.time_ = stamp.toSec();
  if (registration_->getInputTarget()->empty())
  {
    ROS_WARN_STREAM("Global map is empty");
    return measurement;
  }
  if (ndt_prev_stamp_.is_zero())
  {
    ndt_prev_stamp_ = stamp;
    return measurement;
  }
  Eigen::Matrix4f init_guess = getTransformationMatrix();
  registration_->setInputSource(cloud);
  aligned_points_.reset(new pcl::PointCloud<PointT>());
  registration_->align(*aligned_points_, init_guess);
  fitness_score_ = registration_->getFitnessScore();
  // transformation_probability_ = registration_->getTransformationProbability();
  ndt_trans_ = registration_->getFinalTransformation();
  ndt_diff_ = init_guess.inverse() * ndt_trans_.get();
  converged_ = registration_->hasConverged();
  aligned_ = true;
  if (fitness_score_ > 0.0 || !registration_->hasConverged())
  {
    converged_ = false;
    aligned_points_.reset(new pcl::PointCloud<PointT>());
    // Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    aligned_points_.reset(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(*cloud, *aligned_points_, getTransformationMatrix());
    return measurement;
  }
  ROS_WARN_STREAM("-------------------NDT converged-----------------------------");

  Eigen::Vector3f ndt_pose = ndt_trans_->block<3, 1>(0, 3);
  Eigen::Quaternionf ndt_orientation(ndt_trans_->block<3, 3>(0, 0));

  if (getQuaternion().coeffs().dot(ndt_orientation.coeffs()) < 0.0f)
  {
    ndt_orientation.coeffs() *= -1.0f;
  }
  measurement.measurement_[RobotLocalization::StateMemberX] = ndt_pose[0];
  measurement.measurement_[RobotLocalization::StateMemberY] = ndt_pose[1];
  measurement.measurement_[RobotLocalization::StateMemberZ] = ndt_pose[2];
  tf2::Quaternion tf2_quat(ndt_orientation.x(), ndt_orientation.y(), ndt_orientation.z(), ndt_orientation.w());
  double roll, pitch, yaw;
  RobotLocalization::RosFilterUtilities::quatToRPY(tf2_quat, roll, pitch, yaw);
  measurement.measurement_[RobotLocalization::StateMemberRoll] = roll;
  measurement.measurement_[RobotLocalization::StateMemberPitch] = pitch;
  measurement.measurement_[RobotLocalization::StateMemberYaw] = yaw;
  Eigen::MatrixXd covariance =
      Eigen::MatrixXd::Identity(RobotLocalization::STATE_SIZE, RobotLocalization::STATE_SIZE) * fitness_score_ * 1000.0;
  measurement.covariance_ = covariance;
  update_flags[RobotLocalization::StateMemberX] = 1;
  update_flags[RobotLocalization::StateMemberY] = 1;
  update_flags[RobotLocalization::StateMemberZ] = 1;
  update_flags[RobotLocalization::StateMemberRoll] = 1;
  update_flags[RobotLocalization::StateMemberPitch] = 1;
  update_flags[RobotLocalization::StateMemberYaw] = 1;
  measurement.updateVector_ = update_flags;
  return measurement;
}

RobotLocalization::Measurement PoseEstimator::combineMeasurements(RobotLocalization::Measurement ndt_measurement,
                                                                  RobotLocalization::Measurement motion_measurement)
{
  RobotLocalization::Measurement measurement = ndt_measurement;
  for (int i = 0; i < RobotLocalization::STATE_SIZE; i++)
  {
    if (motion_measurement.updateVector_[i] == 1 && ndt_measurement.updateVector_[i] == 0)
    {
      measurement.measurement_[i] = motion_measurement.measurement_[i];
      measurement.covariance_(i, i) = motion_measurement.covariance_(i, i);
      measurement.updateVector_[i] = 1;
    }
  }
  return measurement;
}

nav_msgs::Odometry PoseEstimator::getOdometry()
{
  tf2::Transform state_tf;
  RobotLocalization::RosFilterUtilities::stateToTF(ukf_->getState(), state_tf);
  nav_msgs::Odometry odom;
  odom.header.stamp = predict_current_stamp_;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = state_tf.getOrigin().getX();
  odom.pose.pose.position.y = state_tf.getOrigin().getY();
  odom.pose.pose.position.z = state_tf.getOrigin().getZ();
  odom.pose.pose.orientation.x = state_tf.getRotation().getX();
  odom.pose.pose.orientation.y = state_tf.getRotation().getY();
  odom.pose.pose.orientation.z = state_tf.getRotation().getZ();
  odom.pose.pose.orientation.w = state_tf.getRotation().getW();
  odom.twist.twist.linear.x = ukf_->getState()[RobotLocalization::StateMemberVx];
  odom.twist.twist.linear.y = ukf_->getState()[RobotLocalization::StateMemberVy];
  odom.twist.twist.linear.z = ukf_->getState()[RobotLocalization::StateMemberVz];
  odom.twist.twist.angular.x = ukf_->getState()[RobotLocalization::StateMemberVroll];
  odom.twist.twist.angular.y = ukf_->getState()[RobotLocalization::StateMemberVpitch];
  odom.twist.twist.angular.z = ukf_->getState()[RobotLocalization::StateMemberVyaw];
  return odom;
}

ros::Time PoseEstimator::lastMeasurementTime()
{
  return predict_current_stamp_;
}

Eigen::Vector3f PoseEstimator::getPose()
{
  tf2::Transform state_tf;
  RobotLocalization::RosFilterUtilities::stateToTF(ukf_->getState(), state_tf);
  Eigen::Vector3f pose;
  pose[0] = state_tf.getOrigin().getX();
  pose[1] = state_tf.getOrigin().getY();
  pose[2] = state_tf.getOrigin().getZ();
  return pose;
}

Eigen::Vector3f PoseEstimator::getVelocity()
{
  Eigen::Vector3f velocity;
  velocity[0] = ukf_->getState()[RobotLocalization::StateMemberVx];
  velocity[1] = ukf_->getState()[RobotLocalization::StateMemberVy];
  velocity[2] = ukf_->getState()[RobotLocalization::StateMemberVz];
  return velocity;
}

Eigen::Quaternionf PoseEstimator::getQuaternion()
{
  tf2::Transform state_tf;
  RobotLocalization::RosFilterUtilities::stateToTF(ukf_->getState(), state_tf);
  Eigen::Quaternionf quaternion;
  quaternion.x() = state_tf.getRotation().getX();
  quaternion.y() = state_tf.getRotation().getY();
  quaternion.z() = state_tf.getRotation().getZ();
  quaternion.w() = state_tf.getRotation().getW();
  quaternion.normalize();
  return quaternion;
}

Eigen::Matrix3f PoseEstimator::getRotationMatrix()
{
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix = getQuaternion().toRotationMatrix();
  return rotation_matrix;
}

Eigen::Matrix4f PoseEstimator::getTransformationMatrix()
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  // matrix.block<3, 3>(0, 0) = getRotationMatrix();
  // matrix.block<3, 1>(0, 3) = getPose();
  return matrix;
}

double PoseEstimator::getFitnessScore()
{
  return fitness_score_;
}

double PoseEstimator::getTransformationProbability()
{
  return transformation_probability_;
}

bool PoseEstimator::isAligned()
{
  return aligned_;
}

bool PoseEstimator::isConverged()
{
  return converged_;
}

pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::getAlignedPoints()
{
  return aligned_points_;
}

pcl::PointCloud<PoseEstimator::PointT>::Ptr
PoseEstimator::getTransformedPoints(const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  // pcl::PointCloud<PointT>::Ptr transformed_points(new pcl::PointCloud<PointT>(*cloud));
  pcl::PointCloud<PointT>::Ptr transformed_points(new pcl::PointCloud<PointT>());
  Eigen::Matrix4f transform = getTransformationMatrix();
  pcl::transformPointCloud(*cloud, *transformed_points, transform);
  return transformed_points;
}

void PoseEstimator::setUkfProcessNoise(const Eigen::MatrixXd& process_noise)
{
  ukf_->setProcessNoiseCovariance(process_noise);
}

void PoseEstimator::setUkfAlpha(double alpha, double kappa, double beta)
{
  std::vector<double> args = { alpha, kappa, beta };
  ukf_.reset(new RobotLocalization::Ukf(args));
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::getUkfPredictTrans() const
{
  return ukf_predict_trans_;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::getUkfCorectTrans() const
{
  return ukf_correct_trans_;
}
const boost::optional<Eigen::Matrix4f>& PoseEstimator::getNdtTrans() const
{
  return ndt_trans_;
}
const boost::optional<Eigen::Matrix4f>& PoseEstimator::getNdtDiff() const
{
  return ndt_diff_;
}

}  // namespace hdl_localization
