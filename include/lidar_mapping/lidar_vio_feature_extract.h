#include <cmath>
#include <vector>
#include <unordered_set>
#include <queue>

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include "lidar_mapping/time_based_retriever.h"
namespace beyond
{
namespace lidar_mapping
{
class lidarFeatureExtract
{
    typedef pcl::PointXYZI point;
    typedef pcl::PointCloud<point> lidarCloud;

  public:
    lidarFeatureExtract(ros::NodeHandle &nh, bool _auto_delete);
    void setPubLidarCloud(ros::Publisher pub_cloud);
    void setPubCorner(ros::Publisher pub_cloud);
    void setPubLessCorner(ros::Publisher pub_cloud);
    void setPubFlat(ros::Publisher pub_cloud);
    void setPubLessFlat(ros::Publisher pub_cloud);
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odom);
    void setPubOdom(ros::Publisher pub_cloud); 

  private:
    double scanPeriod;
    int systemDelay;
    int N_SCANS;
    int num;
    int edgeThre1;
    int edgeThre2;
    int surfThre;
    float leafSize;
    float lowerBound;
    float upperBound;
    float edgeCurvatureThre;
    Eigen::Matrix4f imu2lidar;
    std::vector<float> imu2lidar1;
    std::vector<double> cloudCurvature;
    std::vector<double> cloudLabel;
    std::vector<lidarCloud> orderedLidarCloud;
    std::unordered_set<int> pickedPoints;
    TimeBasedRetriever<Eigen::Matrix4f> odoMsg;
    ros::Publisher pubLaserCloud;
    ros::Publisher pubCornerPointsSharp;
    ros::Publisher pubCornerPointsLessSharp;
    ros::Publisher pubSurfPointsFlat;
    ros::Publisher pubSurfPointsLessFlat;
    ros::Publisher pubOdom;
    lidarCloud prev_edge;
    lidarCloud prev_surf;

    lidarCloud prev_prev_edge;
    lidarCloud prev_prev_surf;
    int lidar_id = 0;

    lidarCloud::Ptr prev_lidar_cloud;
    lidarCloud::Ptr prev_prev_lidar_cloud;
    double getYawAngle(const double y, const double x,
                       const double start_yaw_angle, const double end_yaw_angle, bool &passed_half);
    void getScanInd(std::vector<int> &scanStartInd, std::vector<int> &scanEndInd);
    double caculateCurvature(lidarCloud::Ptr laserCloud, int i);
    void removeInvalidPoints(lidarCloud::Ptr laserCloud);
    void pickNeighborPoints(lidarCloud::Ptr laserCloud, const int point_ind);
    void publishMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, lidarCloud::Ptr laserCloud, lidarCloud cornerPointsSharp,
                        lidarCloud cornerPointsLessSharp, lidarCloud surfPointsFlat, lidarCloud surfPointsLessFlat);
    void transform_to_end(point pi, point &po, Eigen::Matrix4f dtrans);
    void get_dewarped_points(lidarCloud point_cloud, double cur_time,
                             Eigen::Matrix4f cur_close_odometry, lidarCloud::Ptr dewarped_edge_points);                            
};
} // namespace lidar_mapping
} // namespace beyond
