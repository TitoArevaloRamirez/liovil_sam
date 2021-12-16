#ifndef MODULES_Lidar_Mapping_Odometry_Select_H_
#define MODULES_Lidar_Mapping_Odometry_Select_H_
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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <lidar_mapping/time_based_retriever.h>
using namespace sensor_msgs;
using namespace message_filters;
typedef pcl::PointXYZI point;
typedef pcl::PointCloud<point> lidarCloud;
namespace beyond
{
namespace lidar_mapping
{
class odometrySelect
{
  public:
    odometrySelect(ros::NodeHandle &nh);
    void set_pub_all_cloud(ros::Publisher pub_cloud);
    void set_pub_edge_cloud(ros::Publisher pub_cloud);
    void set_pub_surf_cloud(ros::Publisher pub_cloud);
    void set_pub_ground_cloud(ros::Publisher pub_cloud);
    void set_pub_odometry(ros::Publisher pub_odom);
    void set_pub_odometry1(ros::Publisher pub_odom);
    void set_pub_loop_edge(ros::Publisher pub_cloud);
    void set_pub_loop_surf(ros::Publisher pub_cloud);

    void full_cloud_lidar_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void odom_edge_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void odom_plane_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void lidar_odom_handler(const nav_msgs::Odometry::ConstPtr &odom);
    void optimized_lidar_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void optimized_edge_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void optimize_plane_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void optimized_odom_handler(const nav_msgs::Odometry::ConstPtr &odom);
    void smooth_lidar_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void smooth_edge_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void smooth_plane_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void smooth_odom_handler(const nav_msgs::Odometry::ConstPtr &odom);
    void ground_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void process(const sensor_msgs::PointCloud2ConstPtr &receive_edge_cloud,
               const sensor_msgs::PointCloud2ConstPtr &receive_surf_points,
               const nav_msgs::Odometry::ConstPtr &input_odometry,
               const sensor_msgs::PointCloud2ConstPtr &optimized_edge_cloud,
               const sensor_msgs::PointCloud2ConstPtr &optimized_surf_points,
               const nav_msgs::Odometry::ConstPtr &optimized_odometry,
               const sensor_msgs::PointCloud2ConstPtr &smooth_edge_cloud,
               const sensor_msgs::PointCloud2ConstPtr &smooth_surf_points);
};
} // namespace lidar_mapping
} // namespace beyond

pcl::PointCloud<point>::Ptr lidar_odom_cloud;
pcl::PointCloud<point>::Ptr lidar_odom_edge;
pcl::PointCloud<point>::Ptr lidar_odom_surf;
pcl::PointCloud<point>::Ptr ground_cloud;
pcl::PointCloud<point>::Ptr prev_prev_edge;
pcl::PointCloud<point>::Ptr prev_edge;
pcl::PointCloud<point>::Ptr prev_prev_surf;
pcl::PointCloud<point>::Ptr prev_surf;
pcl::PointCloud<point>::Ptr prev_prev_full_cloud;
pcl::PointCloud<point>::Ptr prev_full_cloud;
Eigen::Matrix4f lidar_odom;
pcl::PointCloud<point>::Ptr lidar_optimized_cloud;
pcl::PointCloud<point>::Ptr lidar_optimized_edge;
pcl::PointCloud<point>::Ptr lidar_optimized_surf;
Eigen::Matrix4f lidar_optimized_odom;
pcl::PointCloud<point>::Ptr lidar_smooth_cloud;
pcl::PointCloud<point>::Ptr lidar_smooth_edge;
pcl::PointCloud<point>::Ptr lidar_smooth_surf;
pcl::PointCloud<point>::Ptr dewarped_edge_points;
pcl::PointCloud<point>::Ptr dewarped_surf_points;
pcl::PointCloud<point>::Ptr dewarped_ground_points;
Eigen::Matrix4f lidar_smooth_odom;
double lidar_odom_time;
double optimized_odom_time;
double smooth_odom_time;
bool lidar_odom_receive = false;
bool optimized_odom_receive = false;
bool smooth_odom_receive = false;
int featureThreshold = 100;
int featureConsequetive = 0;
int cur_status = 0;
ros::Publisher pubAllCloud;
ros::Publisher pubEdgeCloud;
ros::Publisher pubSurfCloud;
ros::Publisher pubGroundCloud;
ros::Publisher pubOdometry;
ros::Publisher pubOdometry1;
ros::Publisher pubLoopEdge;
ros::Publisher pubLoopSurf;
double prev_x;
double prev_y;
double prev_z;
double cur_x;
double cur_y;
double cur_z;
double v_x;
double v_y;
double v_z;
double prev_time;
double prev_lidar_time;
double cur_time;
double lidar_time;
double lidar_edge_time;
double lidar_surf_time;
double smooth_edge_time;
double smooth_surf_time;
int cur_feature_num;
int frameCount = 0;
nav_msgs::Odometry prevLidarOdom;
nav_msgs::Odometry lidarOdom;
nav_msgs::Odometry optimizedOdom;
nav_msgs::Odometry smoothOdom;
ros::Time cur_ros_time;
sensor_msgs::PointCloud2 laserCloudOutMsg;
sensor_msgs::PointCloud2 laserCloudOutMsg1;
sensor_msgs::PointCloud2 laserCloudOutMsg2;
sensor_msgs::PointCloud2 laserCloudOutMsg3;
sensor_msgs::PointCloud2 laserCloudOutMsg4;
sensor_msgs::PointCloud2 laserCloudOutMsg5;
int lidar_id;
TimeBasedRetriever<Eigen::Matrix4f> lidar_odoMsg(false);
TimeBasedRetriever<Eigen::Matrix4f> vio_optimize_odoMsg(false);
TimeBasedRetriever<Eigen::Matrix4f> vio_smooth_odoMsg(false);
TimeBasedRetriever<pcl::PointCloud<point>::Ptr> full_point_msg(false);
TimeBasedRetriever<pcl::PointCloud<point>::Ptr> edge_point_msg(false);
TimeBasedRetriever<pcl::PointCloud<point>::Ptr> surf_point_msg(false);
TimeBasedRetriever<pcl::PointCloud<point>::Ptr> ground_cloud_msg(false);
TimeBasedRetriever<double> smooth_velocity(false);
TimeBasedRetriever<double> optimized_velocity(false);
double smooth_prev_time = 0.0;
double optimize_prev_time = 0.0;
Eigen::Matrix4f prev_lidar_odom, cur_lidar_odom;
Eigen::Matrix4f prev_vio_optimized, cur_vio_optimized;
Eigen::Matrix4f prev_vio_smooth, cur_vio_smooth;
bool new_point_receive = false;
bool new_edge_receive = false;
bool new_surf_receive = false;
bool prev_imu_receive = false;
std::vector<float> imu2lidar1;
Eigen::Matrix4f imu2lidar;
int retrieve_count = 0;
#endif
