#ifndef MODULES_Lidar_Mapping_Lidar_Odometry_H_
#define MODULES_Lidar_Mapping_Lidar_Odometry_H_
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "lidar_mapping/jacobian_calculate.h" 
#include "lidar_mapping/common.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/common/transforms.h>
using namespace sensor_msgs;
using namespace message_filters;

namespace beyond
{
typedef pcl::PointXYZI point;
typedef pcl::PointCloud<point> pointCloud;
typedef pcl::PointCloud<point>::Ptr pointCloudPtr;
typedef pcl::PointCloud< pcl::PointNormal > PointCloudN;
namespace lidar_mapping
{
class lidarOdometry
{
  public:
    lidarOdometry(ros::NodeHandle &nh);
    void process(const sensor_msgs::PointCloud2ConstPtr &edge_point_cloud,
                 const sensor_msgs::PointCloud2ConstPtr &edge_less_point_cloud,
                 const sensor_msgs::PointCloud2ConstPtr &surf_point_cloud,
                 const sensor_msgs::PointCloud2ConstPtr &surf_less_point_cloud,
                 const sensor_msgs::PointCloud2ConstPtr &full_point_cloud);
    void setPubEdgeCloud(ros::Publisher pub_cloud); 
    void setPubSurfCloud(ros::Publisher pub_cloud);   
    void setPubFullCloud(ros::Publisher pub_cloud);       
    void setPubOdom(ros::Publisher pub_cloud);      

  private:
    float scanPeriod;
    int skipFrameNum;
    bool systemInited = false;
    int laserCloudCornerLastNum;
    int laserCloudSurfLastNum;
    int frameCount;

    float transform[6] = {0};
    float transformSum[6] = {0};

    pointCloudPtr cornerPointsSharp;
    pointCloudPtr cornerPointsLessSharp;
    pointCloudPtr surfPointsFlat;
    pointCloudPtr surfPointsLessFlat;
    pointCloudPtr laserCloudCornerLast;
    pointCloudPtr laserCloudSurfLast;
    pointCloudPtr laserCloudOri;
    pointCloudPtr coeffSel;
    pointCloudPtr laserCloudFullRes;
    pcl::KdTreeFLANN<point>::Ptr kdtreeCornerLast;
    pcl::KdTreeFLANN<point>::Ptr kdtreeSurfLast;
    ros::Publisher pubLaserCloudCornerLast;
    ros::Publisher pubLaserCloudSurfLast;
    ros::Publisher pubLaserCloudFullRes;
    ros::Publisher pubLaserOdometry;

    void TransformToStart(point const * const pi, point * const po);
    void TransformToEnd(point const * const pi, point * const po);
    Eigen::Matrix4f vecToMatrix(float *x);
    void matrixToVec(float *x, Eigen::Matrix4f H);
};
} // namespace lidar_mapping
} // namespace beyond
#endif
