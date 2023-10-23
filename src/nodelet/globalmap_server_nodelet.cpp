#define PCL_NO_PRECOMPILE
#include <nodelet/globalmap_server_nodelet.hpp>

namespace hdl_localization {

GlobalmapServerNodelet::GlobalmapServerNodelet() {}

GlobalmapServerNodelet::~GlobalmapServerNodelet() {}

void GlobalmapServerNodelet::onInit() {
  nh = getNodeHandle();
  mt_nh = getMTNodeHandle();
  private_nh = getPrivateNodeHandle();

  initialize_params();

  globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5, true);
  map_update_sub = nh.subscribe("/map_request/pcd", 10, &GlobalmapServerNodelet::map_update_callback, this);

  globalmap_pub_timer = nh.createWallTimer(ros::WallDuration(1.0), &GlobalmapServerNodelet::pub_once_cb, this, true, true);
}

void GlobalmapServerNodelet::initialize_params() {
  std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd", "");
  globalmap.reset(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
  globalmap->header.frame_id = "map";

  std::ifstream utm_file(globalmap_pcd + ".utm");
  if (utm_file.is_open() && private_nh.param<bool>("convert_utm_to_local", true)) {
    double utm_easting;
    double utm_northing;
    double altitude;
    utm_file >> utm_easting >> utm_northing >> altitude;
    for (auto& pt : globalmap->points) {
      pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
    }
    ROS_INFO_STREAM("Global map offset by UTM reference coordinates (x = " << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
  }

  double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
  if (downsample_resolution > 0.0) {
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);
    globalmap = filtered;
  } else {
    ROS_WARN_STREAM("Globalmap will not be downsampled");
  }
}

void GlobalmapServerNodelet::pub_once_cb(const ros::WallTimerEvent& event) {
  globalmap_pub.publish(globalmap);
}

void GlobalmapServerNodelet::map_update_callback(const std_msgs::String& msg) {
  ROS_INFO_STREAM("Received map request, map path : " << msg.data);
  std::string globalmap_pcd = msg.data;
  globalmap.reset(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
  globalmap->header.frame_id = "map";

  double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
  if (downsample_resolution > 0) {
    boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);
    globalmap = filtered;
  } else {
    ROS_WARN("Globalmap will not be downsampled");
  }
  globalmap_pub.publish(globalmap);
}

}  // namespace hdl_localization

PLUGINLIB_EXPORT_CLASS(hdl_localization::GlobalmapServerNodelet, nodelet::Nodelet)
