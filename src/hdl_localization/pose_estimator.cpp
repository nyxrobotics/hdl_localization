#include <hdl_localization/pose_estimator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <hdl_localization/pose_system.hpp>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief constructor
 * @param registration        registration method
 * @param pos                 initial position
 * @param quat                initial orientation
 * @param cool_time_duration  during "cool time", prediction is not performed
 * @param fitness_reject     Do not process localization when scan matching fitness score is low
 */
PoseEstimator::PoseEstimator(
  pcl::Registration<PointT, PointT>::Ptr& registration_global,
  pcl::Registration<PointT, PointT>::Ptr& registration_local,
  const Eigen::Vector3f& pos,
  const Eigen::Quaternionf& quat,
  double cool_time_duration,
  double fitness_reject,
  double fitness_reliable,
  double linear_correction_gain,
  double angular_correction_gain,
  double angular_correction_distance_reject,
  double angular_correction_distance_reliable)
: registration_global(registration_global),
  registration_local(registration_local),
  cool_time_duration(cool_time_duration),
  fitness_reject(fitness_reject),
  fitness_reliable(fitness_reliable),
  linear_correction_gain(linear_correction_gain),
  angular_correction_gain(angular_correction_gain),
  angular_correction_distance_reject(angular_correction_distance_reject),
  angular_correction_distance_reliable(angular_correction_distance_reliable) {
  last_observation = Eigen::Matrix4f::Identity();
  last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
  last_observation.block<3, 1>(0, 3) = pos;

  process_noise = Eigen::MatrixXf::Identity(16, 16);
  process_noise.middleRows(0, 3) *= 1.0;    // Position
  process_noise.middleRows(3, 3) *= 1.0;    // Velocity
  process_noise.middleRows(6, 4) *= 0.5;    // Orientation
  process_noise.middleRows(10, 3) *= 1e-6;  // Acceleration
  process_noise.middleRows(13, 3) *= 1e-6;  // Angular velocity

  // Scan matching measurement covariance
  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  measurement_noise.middleRows(0, 3) *= 0.01;   // Position
  measurement_noise.middleRows(3, 4) *= 0.001;  // Orientation

  Eigen::VectorXf mean(16);
  mean.middleRows(0, 3) = pos;
  mean.middleRows(3, 3).setZero();
  mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z()).normalized();
  mean.middleRows(10, 3).setZero();
  mean.middleRows(13, 3).setZero();

  // TODO: Change odom covariance constants to ROS params
  // or subscribe an odometry topic and use it's covariance
  // Odometry linear velocity covariance
  odom_process_noise = Eigen::MatrixXf::Identity(16, 16);
  odom_process_noise.middleRows(0, 3) *= 1e-3;    // Position
  odom_process_noise.middleRows(3, 3) *= 1e-9;    // Velocity
  odom_process_noise.middleRows(6, 4) *= 1e-6;    // Orientation
  odom_process_noise.middleRows(13, 3) *= 1e-12;  // Angular velocity

  // IMU angular velocity covariance
  imu_process_noise = Eigen::MatrixXf::Identity(16, 16);
  imu_process_noise.middleRows(6, 4) *= 0.5;    // Orientation
  imu_process_noise.middleRows(10, 3) *= 1e-6;  // Acceleration
  imu_process_noise.middleRows(13, 3) *= 1e-6;  // Angular velocity

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;
  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 16, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator() {}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param acc      acceleration
 * @param gyro     angular velocity
 */
void PoseEstimator::predict(const ros::Time& stamp) {
  if (init_stamp.is_zero()) {
    init_stamp = stamp;
  }

  if ((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;
  ukf->predict();
}

/**
 * @brief predict
 * @param stamp    timestamp
 * @param imu_acc      acceleration
 * @param imu_gyro     angular velocity
 */
void PoseEstimator::predict_imu(const ros::Time& stamp, const Eigen::Vector3f& imu_acc, const Eigen::Vector3f& imu_gyro) {
  if (init_stamp.is_zero()) {
    init_stamp = stamp;
  }

  if ((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;
  ukf->setProcessNoiseCov(imu_process_noise * dt);
  ukf->system.dt = dt;
  ukf->predict_imu(imu_acc, imu_gyro);
}

/**
 * @brief predict_odom
 * @param stamp    timestamp
 * @param odom_twist_linear   linear velocity
 * @param odom_twist_angular  angular velocity
 */
void PoseEstimator::predict_odom(const ros::Time& stamp, const Eigen::Vector3f& odom_twist_linear, const Eigen::Vector3f& odom_twist_angular) {
  if ((stamp - init_stamp).toSec() < cool_time_duration || prev_stamp.is_zero() || prev_stamp == stamp) {
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;
  ukf->setProcessNoiseCov(odom_process_noise * dt);
  ukf->system.dt = dt;
  ukf->predict_odom(odom_twist_linear, odom_twist_angular);
}

/**
 * @brief correct
 * @param cloud   input cloud
 * @return cloud aligned to the globalmap
 */
pcl::PointCloud<PoseEstimator::PointT>::Ptr PoseEstimator::correct(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud, double& fitness_score) {
  if (init_stamp.is_zero()) {
    init_stamp = stamp;
  }
  double local_fitness;
  Eigen::Matrix4f local_matrix = local_travel(cloud, local_fitness);
  ROS_WARN("local_matrix: [%f, %f, %f, %f; %f, %f, %f, %f; %f, %f, %f, %f]", local_matrix(0, 0), local_matrix(0, 1), local_matrix(0, 2), local_matrix(0, 3), local_matrix(1, 0), local_matrix(1, 1), local_matrix(1, 2), local_matrix(1, 3), local_matrix(2, 0), local_matrix(2, 1), local_matrix(2, 2), local_matrix(2, 3));

  last_correction_stamp = stamp;
  Eigen::Matrix4f no_guess = last_observation;
  Eigen::Matrix4f init_guess = matrix() * local_matrix;
  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration_global->setInputSource(cloud);
  registration_global->align(*aligned, init_guess);
  fitness_score = registration_global->getFitnessScore();
  ROS_WARN("fitness_score: %f", fitness_score);
  if (fitness_score > fitness_reject) {
    return aligned;
  }
  Eigen::Matrix4f trans = registration_global->getFinalTransformation();
  Eigen::Vector3f p_measure = trans.block<3, 1>(0, 3);
  Eigen::Quaternionf q_measure(trans.block<3, 3>(0, 0));
  if (quat().coeffs().dot(q_measure.coeffs()) < 0.0f) {
    q_measure.coeffs() *= -1.0f;
  }
  // Get current estimation pose
  Eigen::Vector3f p_estimate = ukf->mean.head<3>();
  Eigen::Quaternionf q_estimate(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
  // Get difference between predicted and measured
  Eigen::Vector3f p_diff = p_measure - p_estimate;
  double diff_linear_norm = (p_measure - p_estimate).norm();
  double diff_angular_norm = fabs(q_estimate.angularDistance(q_measure));
  // Devide difference by fitness_score
  double p_diff_scaling = linear_correction_gain;
  double q_diff_scaling = angular_correction_gain;
  if (diff_linear_norm > angular_correction_distance_reject) {
    q_diff_scaling = 0.0;
  } else {
    q_diff_scaling = angular_correction_distance_reliable / (angular_correction_distance_reliable + diff_linear_norm / angular_correction_distance_reject);
  }
  // Limit max correction to prevent jumping
  if (diff_linear_norm > 1.0) {
    p_diff_scaling /= diff_linear_norm;
    q_diff_scaling /= diff_linear_norm;
  }
  if (diff_angular_norm > 1.0) {
    p_diff_scaling /= diff_angular_norm;
    q_diff_scaling /= diff_angular_norm;
  }
  p_diff_scaling *= (fitness_reliable / (fitness_reliable + fitness_score));
  q_diff_scaling *= (fitness_reliable / (fitness_reliable + fitness_score));
  // Add difference to current estimation
  p_measure = p_estimate + p_diff * p_diff_scaling;
  q_measure = q_estimate.slerp(q_diff_scaling, q_measure);
  // Update kalman filter
  Eigen::VectorXf observation(7);
  observation.middleRows(0, 3) = p_measure;
  observation.middleRows(3, 4) = Eigen::Vector4f(q_measure.w(), q_measure.x(), q_measure.y(), q_measure.z());
  last_observation = trans;
  // Fill data
  wo_pred_error = no_guess.inverse() * trans;
  imu_pred_error = init_guess.inverse() * trans;
  odom_pred_error = imu_pred_error;
  Eigen::MatrixXf registration_measurement_noise = Eigen::MatrixXf::Identity(7, 7);
  registration_measurement_noise.middleRows(0, 3) *= 0.001 * fitness_score;  // Position
  registration_measurement_noise.middleRows(3, 4) *= 0.001 * fitness_score;  // Orientation
  ukf->setMeasurementNoiseCov(registration_measurement_noise);
  ukf->correct(observation);
  return aligned;
}
Eigen::Matrix4f PoseEstimator::local_travel(const pcl::PointCloud<PointT>::ConstPtr& cloud, double& fitness_score) {
  Eigen::Matrix4f output_transform = Eigen::Matrix4f::Identity();
  if (init_stamp.is_zero()) {
    registration_local->setInputTarget(cloud);
    return output_transform;
  }
  registration_local->setInputSource(cloud);
  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
  registration_local->align(*aligned,output_transform);
  output_transform = registration_local->getFinalTransformation();
  registration_local->setInputTarget(cloud);
  return output_transform;
}

/* getters */
ros::Time PoseEstimator::last_correction_time() const {
  return last_correction_stamp;
}

Eigen::Vector3f PoseEstimator::pos() const {
  return Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
}

Eigen::Vector3f PoseEstimator::vel() const {
  return Eigen::Vector3f(ukf->mean[3], ukf->mean[4], ukf->mean[5]);
}

Eigen::Quaternionf PoseEstimator::quat() const {
  return Eigen::Quaternionf(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]).normalized();
}

Eigen::Matrix4f PoseEstimator::matrix() const {
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  m.block<3, 3>(0, 0) = quat().toRotationMatrix();
  m.block<3, 1>(0, 3) = pos();
  return m;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::wo_prediction_error() const {
  return wo_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::imu_prediction_error() const {
  return imu_pred_error;
}

const boost::optional<Eigen::Matrix4f>& PoseEstimator::odom_prediction_error() const {
  return odom_pred_error;
}
}  // namespace hdl_localization
