#ifndef GLOBALMAP_SERVER_NODELET_H
#define GLOBALMAP_SERVER_NODELET_H

#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

struct EIGEN_ALIGN16 PointXYZRGBI {
  PCL_ADD_POINT4D
  PCL_ADD_RGB;
  PCL_ADD_INTENSITY;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI, (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity, intensity))

namespace hdl_localization {

class GlobalmapServerNodelet : public nodelet::Nodelet {
public:
  using PointT = PointXYZRGBI;

  GlobalmapServerNodelet();
  virtual ~GlobalmapServerNodelet();

  void onInit() override;

private:
  void initialize_params();
  void pub_once_cb(const ros::WallTimerEvent& event);
  void map_update_callback(const std_msgs::String& msg);

  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher globalmap_pub;
  ros::Subscriber map_update_sub;

  ros::WallTimer globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;
};

}  // namespace hdl_localization

#endif  // GLOBALMAP_SERVER_NODELET_H