#ifndef MODULES_Lidar_Mapping_Lidar_Mapping_H_
#define MODULES_Lidar_Mapping_Lidar_Mapping_H_
#include <set>
#include <map>

#include <nav_msgs/Odometry.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_mapping/mapping_jacobian_calculate.h"
#include "lidar_mapping/common.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h> 
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace sensor_msgs;
using namespace message_filters;

namespace beyond
{
typedef pcl::PointXYZI point;
typedef pcl::PointCloud<point> pointCloud;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudN;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
namespace lidar_mapping
{
struct intpoint
{
  int x, y, z;
  intpoint(int x, int y, int z) : x(x), y(y), z(z) {}
  friend bool operator<(const intpoint &l, const intpoint &r)
  {
    return l.x < r.x || (l.x == r.x && l.y < r.y) || (l.x == r.x && l.y == r.y && l.z < r.z);
  }
};
class lidarMapping
{
public:
  lidarMapping(ros::NodeHandle& nh);
  void process(const sensor_msgs::PointCloud2ConstPtr &receive_edge_cloud,
               const sensor_msgs::PointCloud2ConstPtr &receive_surf_points,
               const sensor_msgs::PointCloud2ConstPtr &receive_all_points,
               const nav_msgs::Odometry::ConstPtr &input_odometry);
  void set_pub_surrond_cloud(ros::Publisher pub_cloud);
  void set_pub_full_cloud(ros::Publisher pub_cloud);
  void set_pub_odom(ros::Publisher pub_odom);
  void set_pub_path(ros::Publisher pub_path);

private:
  double odom_time;
  float scanPeriod;
  float cornerLeafSize;
  float surfLeafSize;
  float mapLeafSize;
  float bisquareWeight;
  float outlier_threshold;
  int map_cell_size;
  int edge_threshold;
  int surf_threshold;
  int maxIterNum;
  int validPointsThreshold;

  pcl::PointCloud<point>::Ptr edge_point_cloud;
  pcl::PointCloud<point>::Ptr laserCloudCornerAll;
  pcl::PointCloud<point>::Ptr surf_point_cloud;
  pcl::PointCloud<point>::Ptr cur_edge_cloud;
  pcl::PointCloud<point>::Ptr cur_sur_cloud;
  pcl::PointCloud<point>::Ptr cur_edge_cloud_tmp;
  pcl::PointCloud<point>::Ptr cur_surf_cloud_tmp;
  pcl::PointCloud<point>::Ptr point_cloud_in_lidar;
  pcl::PointCloud<point>::Ptr all_point_cloud;
  pcl::PointCloud<point>::Ptr publish_cloud;
  pcl::KdTreeFLANN<point>::Ptr edge_map_tree;
  pcl::KdTreeFLANN<point>::Ptr surf_map_tree;
  pcl::VoxelGrid<point> surf_points_filter;
  pcl::VoxelGrid<point> corner_points_filter;
  pcl::VoxelGrid<point> all_points_filter;
  float transformSum[6] = {0};
  float transformIncre[6] = {0};
  float transformTobeMapped[6] = {0};
  float transformBefMapped[6] = {0};
  float transformAftMapped[6] = {0};
  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubLidarPath;
  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform aftMappedTrans;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose_stamped;

  void receive_messages(const sensor_msgs::PointCloud2ConstPtr &receive_edge_cloud,
                        const sensor_msgs::PointCloud2ConstPtr &receive_surf_points,
                        const sensor_msgs::PointCloud2ConstPtr &receive_all_points,
                        const nav_msgs::Odometry::ConstPtr &input_odometry);
  Eigen::Matrix4f vecToMatrix(float *x);
  Eigen::Matrix4d get_trans_to_map();
  void matrixToVec(float *x, Eigen::Matrix4f dtrans1);
  void transformAssociateToMap();
  void transformUpdate();
  void trans_lidar_to_map(point const *const pi, point *const po, Eigen::Matrix4d dtrans);
  void trans_map_to_lidar(point const *const pi, point *const po, Eigen::Matrix4d dtrans);
  void update_cells(PointCloudI::Ptr edge_cloud, PointCloudI::Ptr plane_cloud);
  void update_neighbor(PointCloudI::Ptr edge_cloud, PointCloudI::Ptr plane_cloud);
  void cal_edge_points_jacobian(std::vector<double> &errors,
                                edgePlaneJacobian &lidar_jacobian, PointCloudN &jacobians);
  void cal_surf_points_jacobian(std::vector<double> &errors,
                                            edgePlaneJacobian &lidar_jacobian, PointCloudN &jacobians);               
  void optimizeTransformTobeMapped();            
  std::set<intpoint> edges_map_cells;
  std::set<intpoint> planes_map_cells;
  std::set<intpoint> map_cells;
  pcl::KdTreeFLANN<pcl::PointXYZI> edges_map_kdtree;
  pcl::KdTreeFLANN<pcl::PointXYZI> planes_map_kdtree;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> edge_voxel_filter;
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> plane_voxel_filter;
  intpoint get_map_cell(const pcl::PointXYZI curPlaneFeature);
  std::map<intpoint, PointCloudI::Ptr> _global_edges, _global_planes;
  std::map<intpoint, PointCloudI::Ptr> _global_edges1, _global_planes1;
  PointCloudI::Ptr edges_map_near;
  PointCloudI::Ptr planes_map_near;
  PointCloudI::Ptr trans_edge;
  PointCloudI::Ptr trans_surf;
  PointCloudI _laserCloudOri;
  PointCloudI _coeffSel;
  int lidar_id;
};
} // namespace lidar_mapping
} // namespace beyond

#endif
