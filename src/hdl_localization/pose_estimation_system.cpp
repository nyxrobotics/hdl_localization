#include <hdl_localization/pose_estimation_system.hpp>

namespace hdl_localization {

PoseEstimationSystem::PoseEstimationSystem() {}

// System equation (without input)
// state = [X, Y, Z, Qw, Qx, Qy, Qz, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
PoseEstimationSystem::VectorX PoseEstimationSystem::predictNextState(const PoseEstimationSystem::VectorX& current_state, double time_delta) const {
  Eigen::VectorXf next_state(STATE_SIZE);
  next_state = current_state;
  Eigen::Vector3f position = current_state.middleRows(StateMemberX, 3);
  Eigen::Vector3f velocity = current_state.middleRows(StateMemberVx, 3);
  Eigen::Vector3f acceleration = current_state.middleRows(StateMemberAx, 3);
  Eigen::Vector3f angular_velocity = current_state.middleRows(StateMemberVroll, 3);
  // Update orientation
  Eigen::Quaternionf angle_quaternion(current_state[StateMemberQw], current_state[StateMemberQx], current_state[StateMemberQy], current_state[StateMemberQz]);
  Eigen::Quaternionf angle_quaternion_delta;
  angle_quaternion_delta = Eigen::AngleAxisf(angular_velocity[0] * time_delta, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(angular_velocity[1] * time_delta, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(angular_velocity[2] * time_delta, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf angle_quaternion_next = angle_quaternion_delta * angle_quaternion;
  // Update velocity
  Eigen::Vector3f velocity_next = velocity + acceleration * time_delta;
  // Update position
  Eigen::Matrix3f rotation_matrix = angle_quaternion.toRotationMatrix();
  Eigen::Matrix3f rotation_matrix_next = angle_quaternion_next.toRotationMatrix();
  Eigen::Vector3f position_next = position + rotation_matrix * (time_delta / 2.0 * velocity) + rotation_matrix_next * (time_delta / 2.0 * velocity_next);
  next_state.middleRows(StateMemberX, 3) = position_next;
  angle_quaternion_next.normalize();
  next_state.middleRows(StateMemberQw, 4) << angle_quaternion_next.w(), angle_quaternion_next.x(), angle_quaternion_next.y(), angle_quaternion_next.z();
  next_state.middleRows(StateMemberVx, 3) = velocity_next;
  return next_state;
}

// System equation with IMU input
// state = [X, Y, Z, Qw, Qx, Qy, Qz, Vx, Vy, Vz, Vroll, Vpitch, Vyaw, Ax, Ay, Az]
PoseEstimationSystem::VectorX PoseEstimationSystem::predictNextStateWithIMU(
  const PoseEstimationSystem::VectorX& current_state,
  const Eigen::Vector3f& imu_acc,
  const Eigen::Vector3f& imu_gyro,
  double time_delta) const {
  Eigen::VectorXf next_state(STATE_SIZE);
  next_state = current_state;
  Eigen::Vector3f position = current_state.middleRows(StateMemberX, 3);
  Eigen::Vector3f velocity = current_state.middleRows(StateMemberVx, 3);
  Eigen::Vector3f acceleration = current_state.middleRows(StateMemberAx, 3);
  Eigen::Vector3f angular_velocity = current_state.middleRows(StateMemberVroll, 3);
  // Update orientation
  Eigen::Vector3f angular_velocity_next(imu_gyro.x(), imu_gyro.y(), imu_gyro.z());
  Eigen::Quaternionf angle_quaternion(current_state[StateMemberQw], current_state[StateMemberQx], current_state[StateMemberQy], current_state[StateMemberQz]);
  Eigen::Quaternionf angle_quaternion_delta;
  angle_quaternion_delta = Eigen::AngleAxisf(angular_velocity_next.x() * time_delta, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(angular_velocity_next.y() * time_delta, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(angular_velocity_next.z() * time_delta, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf angle_quaternion_next = angle_quaternion_delta * angle_quaternion;
  // Update velocity
  Eigen::Vector3f acceleration_next(imu_acc.x(), imu_acc.y(), imu_acc.z() - 9.80665f);
  Eigen::Vector3f velocity_next = velocity + acceleration_next * time_delta;
  // Update position
  Eigen::Matrix3f rotation_matrix = angle_quaternion.toRotationMatrix();
  Eigen::Matrix3f rotation_matrix_next = angle_quaternion_next.toRotationMatrix();
  Eigen::Vector3f position_next = position + rotation_matrix * (time_delta / 2.0 * velocity) + rotation_matrix_next * (time_delta / 2.0 * velocity_next);
  next_state.middleRows(StateMemberX, 3) = position_next;
  angle_quaternion_next.normalize();
  next_state.middleRows(StateMemberQw, 4) << angle_quaternion_next.w(), angle_quaternion_next.x(), angle_quaternion_next.y(), angle_quaternion_next.z();
  next_state.middleRows(StateMemberVx, 3) = velocity_next;
  next_state.middleRows(StateMemberVroll, 3) = angular_velocity_next;
  next_state.middleRows(StateMemberAx, 3) = acceleration_next;
  return next_state;
}

// System equation with odometry input
PoseEstimationSystem::VectorX PoseEstimationSystem::predictNextStateWithOdom(
  const PoseEstimationSystem::VectorX& current_state,
  const Eigen::Vector3f& odom_twist_lin,
  const Eigen::Vector3f& odom_twist_ang,
  double time_delta) const {
  Eigen::VectorXf next_state(STATE_SIZE);
  next_state = current_state;
  Eigen::Vector3f position = current_state.middleRows(StateMemberX, 3);
  Eigen::Vector3f velocity = current_state.middleRows(StateMemberVx, 3);
  Eigen::Vector3f acceleration = current_state.middleRows(StateMemberAx, 3);
  Eigen::Vector3f angular_velocity = current_state.middleRows(StateMemberVroll, 3);
  // Update orientation
  Eigen::Vector3f angular_velocity_next(odom_twist_ang.x(), odom_twist_ang.y(), odom_twist_ang.z());
  Eigen::Quaternionf angle_quaternion(current_state[StateMemberQw], current_state[StateMemberQx], current_state[StateMemberQy], current_state[StateMemberQz]);
  Eigen::Quaternionf angle_quaternion_delta;
  angle_quaternion_delta = Eigen::AngleAxisf(angular_velocity_next.x() * time_delta, Eigen::Vector3f::UnitX()) *
                           Eigen::AngleAxisf(angular_velocity_next.y() * time_delta, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(angular_velocity_next.z() * time_delta, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf angle_quaternion_next = angle_quaternion_delta * angle_quaternion;
  // Update velocity
  Eigen::Vector3f velocity_next = odom_twist_lin;
  Eigen::Vector3f acceleration_next = (velocity_next - velocity) / time_delta;
  // Update position
  Eigen::Matrix3f rotation_matrix = angle_quaternion.toRotationMatrix();
  Eigen::Matrix3f rotation_matrix_next = angle_quaternion_next.toRotationMatrix();
  Eigen::Vector3f position_next = position + rotation_matrix * (time_delta / 2.0 * velocity_next) + rotation_matrix_next * (time_delta / 2.0 * velocity_next);
  next_state.middleRows(StateMemberX, 3) = position_next;
  angle_quaternion_next.normalize();
  next_state.middleRows(StateMemberQw, 4) << angle_quaternion_next.w(), angle_quaternion_next.x(), angle_quaternion_next.y(), angle_quaternion_next.z();
  next_state.middleRows(StateMemberVx, 3) = velocity_next;
  next_state.middleRows(StateMemberVroll, 3) = angular_velocity_next;
  next_state.middleRows(StateMemberAx, 3) = acceleration_next;
  return next_state;
}

// Observation equation
PoseEstimationSystem::VectorX PoseEstimationSystem::computeObservation(const PoseEstimationSystem::VectorX& current_state) const {
  PoseEstimationSystem::VectorX observation(MEASUREMENT_SIZE);
  observation.middleRows(MeasurementMemberX, 3) = current_state.middleRows(StateMemberX, 3);
  observation.middleRows(MeasurementMemberQw, 4) = current_state.middleRows(MeasurementMemberQw, 4).normalized();
  return observation;
}

}  // namespace hdl_localization
