/**
 * UnscentedKalmanFilterX.hpp
 * @author koide
 * 16/02/01
 **/
#ifndef KKL_UNSCENTED_KALMAN_FILTER_X_HPP
#define KKL_UNSCENTED_KALMAN_FILTER_X_HPP

#include <random>
#include <Eigen/Dense>

namespace kkl {
namespace alg {

/**
 * @brief Unscented Kalman Filter class
 * @param T        scalar type
 * @param System   system class to be estimated
 */

template <typename T, class System>
class UnscentedKalmanFilterX {
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXt;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UnscentedKalmanFilterX(
    const System& system,
    int state_dim,
    int measurement_dim,
    const MatrixXt& process_noise,
    const MatrixXt& measurement_noise,
    const VectorXt& init_mean,
    const MatrixXt& init_cov);

  void predict();

  void predictImu(const Eigen::Vector3f& imu_acc, const Eigen::Vector3f& imu_gyro);
  void predictOdom(const Eigen::Vector3f& odom_twist_linear, const Eigen::Vector3f& odom_twist_angular);
  void correct(const VectorXt& measurement);

  /* getter */
  const VectorXt& getMean() const;
  const MatrixXt& getCov() const;
  const MatrixXt& getSigmaPoints() const;
  System& getSystem();
  const System& getSystem() const;
  const MatrixXt& getProcessNoiseCov() const;
  const MatrixXt& getMeasurementNoiseCov() const;
  const MatrixXt& getKalmanGain() const;

  /* setter */
  UnscentedKalmanFilterX& setMean(const VectorXt& m);
  UnscentedKalmanFilterX& setCov(const MatrixXt& s);
  UnscentedKalmanFilterX& setProcessNoiseCov(const MatrixXt& p);
  UnscentedKalmanFilterX& setMeasurementNoiseCov(const MatrixXt& m);

  void computeSigmaPoints(const VectorXt& mean, const MatrixXt& cov, MatrixXt& sigma_points_);
  void ensurePositiveFinite(MatrixXt& cov);

private:
  const int state_dim_;
  const int measurement_dim_;
  const int sigma_points_samples_;

public:
  VectorXt mean_;
  MatrixXt cov_;
  System system_;
  MatrixXt process_noise_;
  MatrixXt measurement_noise_;
  T lambda_;
  VectorXt weights_;
  MatrixXt sigma_points_;
  VectorXt ext_weights_;
  MatrixXt ext_sigma_points_;
  MatrixXt expected_measurements_;
  MatrixXt kalman_gain_;
  std::mt19937 mt_;
  std::normal_distribution<T> normal_dist_;
};

}  // namespace alg
}  // namespace kkl

#endif