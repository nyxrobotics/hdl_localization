#ifndef POSE_ESTIMATION_SYSTEM_HPP
#define POSE_ESTIMATION_SYSTEM_HPP

#include <Eigen/Dense>
#include <ukf/unscented_kalman_filter.hpp>

namespace hdl_localization {
const int STATE_SIZE = 16;
enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberQw,
  StateMemberQx,
  StateMemberQy,
  StateMemberQz,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};
const int MEASUREMENT_SIZE = 7;
enum MeasurementMembers
{
  MeasurementMemberX = 0,
  MeasurementMemberY,
  MeasurementMemberZ,
  MeasurementMemberQw,
  MeasurementMemberQx,
  MeasurementMemberQy,
  MeasurementMemberQz
};


class PoseEstimationSystem {
public:
  typedef Eigen::Matrix<float, 3, 1> Vector3;
  typedef Eigen::Matrix<float, 4, 4> Matrix4;
  typedef Eigen::Matrix<float, Eigen::Dynamic, 1> VectorX;
  typedef Eigen::Quaternion<float> Quaternion;

public:
  PoseEstimationSystem();
  VectorX predictNextState(const VectorX& current_state, double time_delta) const;
  VectorX predictNextStateWithIMU(const VectorX& current_state, const Eigen::Vector3f& imu_acc, const Eigen::Vector3f& imu_gyro, double time_delta) const;
  VectorX predictNextStateWithOdom(const VectorX& current_state, const Eigen::Vector3f& odom_twist_lin, const Eigen::Vector3f& odom_twist_ang, double time_delta) const;
  VectorX computeObservation(const VectorX& current_state) const;
};

}  // namespace hdl_localization

#endif  // POSE_ESTIMATION_SYSTEM_HPP