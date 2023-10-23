#include "DeltaEstimater.hpp"

namespace hdl_localization {

DeltaEstimater::DeltaEstimater(pcl::Registration<PointT, PointT>::Ptr reg) : delta_(Eigen::Isometry3f::Identity()), reg_(reg) {}

DeltaEstimater::~DeltaEstimater() {}

void DeltaEstimater::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  delta_.setIdentity();
  last_frame_.reset();
}

void DeltaEstimater::addFrame(pcl::PointCloud<PointT>::ConstPtr frame) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (last_frame_ == nullptr) {
    last_frame_ = frame;
    return;
  }

  reg_->setInputTarget(last_frame_);
  reg_->setInputSource(frame);
  lock.unlock();

  pcl::PointCloud<PointT> aligned;
  reg_->align(aligned);

  lock.lock();
  last_frame_ = frame;
  delta_ = delta_ * Eigen::Isometry3f(reg_->getFinalTransformation());
}

Eigen::Isometry3f DeltaEstimater::estimatedDelta() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return delta_;
}

}  // namespace hdl_localization