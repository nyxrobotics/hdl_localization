#ifndef POSE_ESTIMATION_SYSTEM_HPP
#define POSE_ESTIMATION_SYSTEM_HPP

#include <Eigen/Dense>
#include <ukf/unscented_kalman_filter.hpp>

namespace hdl_localization {

class PoseEstimationSystem {
public:
  typedef Eigen::Matrix<float, 3, 1> Vector3;
  typedef Eigen::Matrix<float, 4, 4> Matrix4;
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Quaternion<float> Quaternion;

public:
  double time_step_;
  PoseEstimationSystem();
  VectorX computeNextState(const VectorX& current_state) const;
  VectorX computeNextStateWithIMU(const VectorX& current_state, const Eigen::Vector3f& imu_acc, const Eigen::Vector3f& imu_gyro) const;
  VectorX computeNextStateWithOdom(const VectorX& current_state, const Eigen::Vector3f& odom_twist_lin, const Eigen::Vector3f& odom_twist_ang) const;
  VectorX computeObservation(const VectorX& current_state) const;
};

}  // namespace hdl_localization

#endif  // POSE_ESTIMATION_SYSTEM_HPP