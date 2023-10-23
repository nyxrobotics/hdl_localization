#include <hdl_localization/pose_estimation_system.hpp>

namespace hdl_localization {

PoseEstimationSystem::PoseEstimationSystem() {
  time_step_ = 0.01;
}

// System equation (without input)
PoseEstimationSystem::VectorX PoseEstimationSystem::computeNextState(const PoseEstimationSystem::VectorX& current_state) const {
  PoseEstimationSystem::VectorX next_state(16);

  Vector3 position = current_state.middleRows(0, 3);
  Vector3 velocity = current_state.middleRows(3, 3);
  Quaternion quaternion(current_state[6], current_state[7], current_state[8], current_state[9]);
  quaternion.normalize();

  Vector3 acceleration_bias = current_state.middleRows(10, 3);
  Vector3 angular_velocity_bias = current_state.middleRows(13, 3);

  // Update position
  next_state.middleRows(0, 3) = position + velocity * time_step_;

  // Update velocity
  next_state.middleRows(3, 3) = velocity;

  // Update orientation
  next_state.middleRows(6, 4) << quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z();
  next_state.middleRows(10, 3) = current_state.middleRows(10, 3);  // Constant bias on acceleration
  next_state.middleRows(13, 3) = current_state.middleRows(13, 3);  // Constant bias on angular velocity

  return next_state;
}

// System equation with IMU input
PoseEstimationSystem::VectorX PoseEstimationSystem::computeNextStateWithIMU(const PoseEstimationSystem::VectorX& current_state, const Eigen::Vector3f& imu_acc, const Eigen::Vector3f& imu_gyro) const {
  PoseEstimationSystem::VectorX next_state(16);

  Vector3 pt = current_state.middleRows(0, 3);
  Vector3 vt = current_state.middleRows(3, 3);
  Quaternion qt(current_state[6], current_state[7], current_state[8], current_state[9]);
  qt.normalize();

  Vector3 acc_bias = current_state.middleRows(10, 3);
  Vector3 gyro_bias = current_state.middleRows(13, 3);

  // Position
  Vector3 next_pt = pt + vt * time_step_;
  next_state.middleRows(0, 3) = next_pt;

  // Velocity (vel_z = 0);
  Vector3 g(0.0f, 0.0f, 9.80665f);
  Vector3 acc = qt * (imu_acc - acc_bias - g);
  Vector3 next_vt = vt + acc * time_step_;
  next_vt.z() = 0.0f;
  next_state.middleRows(3, 3) = next_vt;

  // Orientation
  Vector3 gyro = imu_gyro - gyro_bias;
  Quaternion dq(1, gyro.x() * time_step_, gyro.y() * time_step_, gyro.z() * time_step_);
  dq.normalize();
  Quaternion next_qt = (qt * dq).normalized();
  next_state.middleRows(6, 4) << next_qt.w(), next_qt.x(), next_qt.y(), next_qt.z();
  next_state.middleRows(10, 3) = current_state.middleRows(10, 3);  // Constant bias on acceleration
  next_state.middleRows(13, 3) = current_state.middleRows(13, 3);  // Constant bias on angular velocity

  return next_state;
}

// System equation with odometry input
PoseEstimationSystem::VectorX PoseEstimationSystem::computeNextStateWithOdom(const PoseEstimationSystem::VectorX& current_state, const Eigen::Vector3f& odom_twist_lin, const Eigen::Vector3f& odom_twist_ang) const {
  PoseEstimationSystem::VectorX next_state(16);
  Vector3 pt = current_state.middleRows(0, 3);
  Vector3 vt = current_state.middleRows(3, 3);
  Quaternion qt(current_state[6], current_state[7], current_state[8], current_state[9]);
  qt.normalize();

  const Vector3& raw_lin_vel = odom_twist_lin;
  Vector3 raw_ang_vel = odom_twist_ang;

  // Position
  next_state.middleRows(0, 3) = pt + vt * time_step_;

  // Velocity (vel_z = 0);
  Vector3 vel = qt * raw_lin_vel;
  vel.z() = 0.0f;
  next_state.middleRows(3, 3) = vel;

  // Orientation
  Quaternion dq(1, raw_ang_vel[0] * time_step_ / 2.0, raw_ang_vel[1] * time_step_ / 2.0, raw_ang_vel[2] * time_step_ / 2.0);
  dq.normalize();
  Quaternion next_qt = (qt * dq).normalized();
  next_state.middleRows(6, 4) << next_qt.w(), next_qt.x(), next_qt.y(), next_qt.z();
  next_state.middleRows(10, 3) = current_state.middleRows(10, 3);  // Constant bias on acceleration
  next_state.middleRows(13, 3) = current_state.middleRows(13, 3);  // Constant bias on angular velocity
  return next_state;
}

// Observation equation
PoseEstimationSystem::VectorX PoseEstimationSystem::computeObservation(const PoseEstimationSystem::VectorX& current_state) const {
  PoseEstimationSystem::VectorX observation(7);
  observation.middleRows(0, 3) = current_state.middleRows(0, 3);
  observation.middleRows(3, 4) = current_state.middleRows(6, 4).normalized();

  return observation;
}

}  // namespace hdl_localization
