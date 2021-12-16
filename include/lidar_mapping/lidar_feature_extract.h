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
namespace beyond
{
namespace lidar_mapping
{
class lidarFeatureExtract
{
  typedef pcl::PointXYZI point;
  typedef pcl::PointCloud<point> lidarCloud;

public:
  lidarFeatureExtract(ros::NodeHandle &nh);
  void setPubLidarCloud(ros::Publisher pub_cloud);
  void setPubCorner(ros::Publisher pub_cloud);
  void setPubLessCorner(ros::Publisher pub_cloud);
  void setPubFlat(ros::Publisher pub_cloud);
  void setPubLessFlat(ros::Publisher pub_cloud);
  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

private:
  double scanPeriod;
  int systemDelay;
  int N_SCANS;
  int num;
  int edgeThre1;
  int edgeThre2;
  int surfThre;
  float leafSize;
  float edgeCurvatureThre;
  float upperBound;
  float lowerBound;
  std::vector<double> cloudCurvature;
  std::vector<double> cloudLabel;
  std::vector<lidarCloud> orderedLidarCloud;
  std::unordered_set<int> pickedPoints;
  ros::Publisher pubLaserCloud;
  ros::Publisher pubCornerPointsSharp;
  ros::Publisher pubCornerPointsLessSharp;
  ros::Publisher pubSurfPointsFlat;
  ros::Publisher pubSurfPointsLessFlat;
  double getYawAngle(const double y, const double x,
                     const double start_yaw_angle, const double end_yaw_angle, bool &passed_half);
  void getScanInd(std::vector<int> &scanStartInd, std::vector<int> &scanEndInd);
  double caculateCurvature(lidarCloud::Ptr laserCloud, int i);
  void removeInvalidPoints(lidarCloud::Ptr laserCloud);
  void pickNeighborPoints(lidarCloud::Ptr laserCloud, const int point_ind);
  void publishMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, lidarCloud::Ptr laserCloud, lidarCloud cornerPointsSharp,
                      lidarCloud cornerPointsLessSharp, lidarCloud surfPointsFlat, lidarCloud surfPointsLessFlat);
};
} // namespace lidar_mapping
} // namespace beyond