#ifndef HDL_LOCALIZATION_DELTA_ESTIMATER_HPP
#define HDL_LOCALIZATION_DELTA_ESTIMATER_HPP

#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace hdl_localization {
class DeltaEstimater {
public:
  using PointT = pcl::PointXYZI;

  DeltaEstimater(pcl::Registration<PointT, PointT>::Ptr reg) : delta_(Eigen::Isometry3f::Identity()), reg_(reg) {}
  ~DeltaEstimater() {}

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    delta_.setIdentity();
    last_frame_.reset();
  }

  void addFrame(pcl::PointCloud<PointT>::ConstPtr frame) {
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

  Eigen::Isometry3f estimatedDelta() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return delta_;
  }

private:
  mutable std::mutex mutex_;
  Eigen::Isometry3f delta_;
  pcl::Registration<PointT, PointT>::Ptr reg_;

  pcl::PointCloud<PointT>::ConstPtr last_frame_;
};

}  // namespace hdl_localization

#endif