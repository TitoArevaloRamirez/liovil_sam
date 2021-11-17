#include "lidar_mapping/lidar_feature_extract.h"

namespace beyond
{
namespace lidar_mapping
{
lidarFeatureExtract::lidarFeatureExtract(ros::NodeHandle &nh) {
  nh.getParam("lidar/scanPeriod", scanPeriod);
  std::cout << "scanPeriod: " << scanPeriod << std::endl;
  nh.getParam("lidar/N_SCANS", N_SCANS);
  std::cout << "N_SCANS: " << N_SCANS << std::endl;
  nh.getParam("lidar/systemDelay", systemDelay);
  std::cout << "systemDelay: " << systemDelay << std::endl;
  nh.getParam("lidar/num", num);
  std::cout << "num: " << num << std::endl;
  nh.getParam("lidar/leafSize", leafSize);
  std::cout << "leafSize: " << leafSize << std::endl;
  nh.getParam("lidar/edgeThre1", edgeThre1);
  std::cout << "edgeThre1: " << edgeThre1 << std::endl;
  nh.getParam("lidar/edgeThre2", edgeThre2);
  std::cout << "edgeThre2: " << edgeThre2 << std::endl;
  nh.getParam("lidar/surfThre", surfThre);
  std::cout << "surfThre: " << surfThre << std::endl;
  nh.getParam("lidar/edgeCurvatureThre", edgeCurvatureThre);
  std::cout << "edgeCurvatureThre: " << edgeCurvatureThre << std::endl;
}  
void lidarFeatureExtract::setPubLidarCloud(ros::Publisher pub_cloud)
{
  pubLaserCloud = pub_cloud;
}

void lidarFeatureExtract::setPubCorner(ros::Publisher pub_cloud)
{
  pubCornerPointsSharp = pub_cloud;
}

void lidarFeatureExtract::setPubLessCorner(ros::Publisher pub_cloud)
{
  pubCornerPointsLessSharp = pub_cloud;
}

void lidarFeatureExtract::setPubFlat(ros::Publisher pub_cloud)
{
  pubSurfPointsFlat = pub_cloud;
}

void lidarFeatureExtract::setPubLessFlat(ros::Publisher pub_cloud)
{
  pubSurfPointsLessFlat = pub_cloud;
}

double lidarFeatureExtract::getYawAngle(const double y, const double x,
                                        const double start_yaw_angle, const double end_yaw_angle, bool &passed_half)
{
  double cur_yaw_angle = -atan2(y, x);
  if (!passed_half)
  {
    if (cur_yaw_angle < start_yaw_angle - M_PI / 2)
    {
      cur_yaw_angle += 2 * M_PI;
    }

    if (cur_yaw_angle - start_yaw_angle > M_PI)
    {
      passed_half = true;
    }
  }
  else
  {

    if (cur_yaw_angle < end_yaw_angle - M_PI * 1.2)
    {
      cur_yaw_angle += 2 * M_PI;
    }
  }
  return cur_yaw_angle;
}

void lidarFeatureExtract::getScanInd(std::vector<int> &scanStartInd, std::vector<int> &scanEndInd)
{
  size_t points_num = 0;
  for (size_t i = 0; i < N_SCANS; i++)
  {
    scanStartInd[i] = points_num + 5;
    points_num += orderedLidarCloud[i].size();
    scanEndInd[i] = points_num - 5;
  }
}

double lidarFeatureExtract::caculateCurvature(lidarCloud::Ptr laserCloud, int i)
{
  double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0;
  double x = laserCloud->at(i).x;
  double y = laserCloud->at(i).y;
  double z = laserCloud->at(i).z;
  for (int ind = i - 1; ind >= i - 5; ind--)
  {
    diff_x += x - laserCloud->at(ind).x;
    diff_y += y - laserCloud->at(ind).y;
    diff_z += z - laserCloud->at(ind).z;
  }
  for (int ind = i + 1; ind <= i + 5; ind++)
  {
    diff_x += x - laserCloud->at(ind).x;
    diff_y += y - laserCloud->at(ind).y;
    diff_z += z - laserCloud->at(ind).z;
  }
  return pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2);
}

void lidarFeatureExtract::removeInvalidPoints(lidarCloud::Ptr laserCloud)
{
  size_t cloudSize = laserCloud->size();
  for (size_t i = 5; i < cloudSize - 6; i++)
  {
    double x0 = laserCloud->at(i).x, y0 = laserCloud->at(i).y, z0 = laserCloud->at(i).z;
    double x1 = laserCloud->at(i + 1).x, y1 = laserCloud->at(i + 1).y, z1 = laserCloud->at(i + 1).z;
    double x2 = laserCloud->at(i - 1).x, y2 = laserCloud->at(i - 1).y, z2 = laserCloud->at(i - 1).z;
    double diff_x = x1 - x0;
    double diff_y = y1 - y0;
    double diff_z = z1 - z0;
    double diff = pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2);
    double depth0 = sqrt(pow(x0, 2) + pow(y0, 2) + pow(z0, 2));
    double depth1 = sqrt(pow(x1, 2) + pow(y1, 2) + pow(z1, 2));

    if (diff > edgeCurvatureThre)
    {
      if (depth0 > depth1)
      {
        diff_x = x1 - x0 * depth1 / depth0;
        diff_y = y1 - y0 * depth1 / depth0;
        diff_z = z1 - z0 * depth1 / depth0;

        if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / depth1 < edgeCurvatureThre)
        {
          for (size_t ind = 0; ind <= 5; ind++)
          {
            pickedPoints.insert(i - ind);
          }
        }
      }
      else
      {
        diff_x = x1 * depth0 / depth1 - x0;
        diff_y = y1 * depth0 / depth1 - y0;
        diff_z = z1 * depth0 / depth1 - z0;

        if (sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z) / depth0 < edgeCurvatureThre)
        {
          for (size_t ind = 1; ind <= 6; ind++)
          {
            pickedPoints.insert(i + ind);
          }
        }
      }
    }
    diff_x = x0 - x2;
    diff_y = y0 - y2;
    diff_z = z0 - z2;
    double diff2 = pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2);

    if (diff > 0.0002 * depth0 * depth0 && diff2 > 0.0002 * depth0 * depth0)
    {
      pickedPoints.insert(i);
    }
  }
}

void lidarFeatureExtract::pickNeighborPoints(lidarCloud::Ptr laserCloud, const int point_ind)
{
  pickedPoints.insert(point_ind);
  for (int l = 1; l <= 5; l++)
  {
    float diff_x = laserCloud->points[point_ind + l].x - laserCloud->points[point_ind + l - 1].x;
    float diff_y = laserCloud->points[point_ind + l].y - laserCloud->points[point_ind + l - 1].y;
    float diff_z = laserCloud->points[point_ind + l].z - laserCloud->points[point_ind + l - 1].z;
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
    {
      break;
    }

    pickedPoints.insert(point_ind + l);
  }
  for (int l = -1; l >= -5; l--)
  {
    float diff_x = laserCloud->points[point_ind + l].x - laserCloud->points[point_ind + l + 1].x;
    float diff_y = laserCloud->points[point_ind + l].y - laserCloud->points[point_ind + l + 1].y;
    float diff_z = laserCloud->points[point_ind + l].z - laserCloud->points[point_ind + l + 1].z;
    if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05)
    {
      break;
    }

    pickedPoints.insert(point_ind + l);
  }
}

void lidarFeatureExtract::publishMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, lidarCloud::Ptr laserCloud, lidarCloud cornerPointsSharp,
                                         lidarCloud cornerPointsLessSharp, lidarCloud surfPointsFlat, lidarCloud surfPointsLessFlat)
{
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/lidar";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/lidar";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/lidar";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/lidar";
  pubSurfPointsFlat.publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/lidar";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
}

void lidarFeatureExtract::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (systemDelay > 0)
  {
    systemDelay--;
    return;
  }
  orderedLidarCloud.clear();
  orderedLidarCloud.resize(N_SCANS);

  cloudCurvature.clear();
  cloudCurvature.resize(num);

  cloudLabel.clear();
  cloudLabel.resize(num);

  pickedPoints.clear();

  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);
  pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> laserCloudIn_vel;

  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn_vel);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);

  size_t cloudSize = laserCloudIn.size();
  point first_point = laserCloudIn[0];
  point last_point = laserCloudIn[cloudSize - 1];
  double start_yaw_angle = -std::atan2(first_point.y, first_point.x);
  double end_yaw_angle = -std::atan2(last_point.y, last_point.x);

  if (end_yaw_angle - start_yaw_angle > 3 * M_PI)
  {
    end_yaw_angle -= 2 * M_PI;
  }
  else if (end_yaw_angle - start_yaw_angle < M_PI)
  {
    end_yaw_angle += 2 * M_PI;
  }
  bool halfPassed = false;
  lidarCloud::Ptr laserCloud(new lidarCloud());
  for (int i = 0; i < cloudSize; i++)
  {
    double x = laserCloudIn.at(i).x;
    double y = laserCloudIn.at(i).y;
    double z = laserCloudIn.at(i).z; 
    int scanID;
    scanID = laserCloudIn_vel.points[indices[i]].ring;
    double cur_yaw_angle = getYawAngle(y, x, start_yaw_angle, end_yaw_angle, halfPassed);

    float relTime = (cur_yaw_angle - start_yaw_angle) / (end_yaw_angle - start_yaw_angle);
    point cur_point;
    cur_point.x = x;
    cur_point.y = y;
    cur_point.z = z;
    cur_point.intensity = scanID + scanPeriod * relTime;
    orderedLidarCloud[scanID].push_back(cur_point);
  }

  getScanInd(scanStartInd, scanEndInd);

  for (int i = 0; i < N_SCANS; i++)
  {
    *laserCloud += orderedLidarCloud[i];
  }
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++)
  {
    cloudCurvature[i] = caculateCurvature(laserCloud, i);
    cloudLabel[i] = 0;
  }

  removeInvalidPoints(laserCloud);
  lidarCloud cornerPointsSharp;
  lidarCloud cornerPointsLessSharp;
  lidarCloud surfPointsFlat;
  lidarCloud surfPointsLessFlat;

  for (int i = 0; i < N_SCANS; i++)
  {
    lidarCloud::Ptr surfPointsLessFlatScan(new lidarCloud);
    for (int j = 0; j < 6; j++)
    {
      int lo = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;
      int hi = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

      std::priority_queue<std::pair<double, int>> edge_queue;
      std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> surface_queue;

      int largestPickedNum = 0;
      for (size_t k = lo; k <= hi; k++)
      {
        edge_queue.push({cloudCurvature[k], k});
        surface_queue.push({cloudCurvature[k], k});
      }
      size_t cur_edge_num = 0, cur_suf_num = 0;
      while (!edge_queue.empty() && cur_edge_num < 20)
      {
        std::pair<double, int> cur_point = edge_queue.top();
        double point_curvature = cur_point.first;
        int point_ind = cur_point.second;
        edge_queue.pop();
        if (pickedPoints.count(point_ind) == 0 && point_curvature > edgeCurvatureThre)
        {
          cur_edge_num++;
          if (cur_edge_num <= edgeThre1)
          {
            cloudLabel[point_ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[point_ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[point_ind]);
          }
          else if (cur_edge_num <= edgeThre2)
          {
            cloudLabel[point_ind] = 1;
            cornerPointsLessSharp.push_back(laserCloud->points[point_ind]);
          }
          else
          {
            break;
          }
          pickNeighborPoints(laserCloud, point_ind);
        }
      }

      while (!surface_queue.empty() && cur_suf_num <= 4)
      {
        std::pair<double, int> cur_point = surface_queue.top();
        double point_curvature = cur_point.first;
        int point_ind = cur_point.second;
        surface_queue.pop();
        if (pickedPoints.count(point_ind) == 0 && point_curvature < edgeCurvatureThre)
        {
          cur_suf_num++;
          if (cur_suf_num <= surfThre)
          {
            cloudLabel[point_ind] = -1;
            surfPointsFlat.push_back(laserCloud->points[point_ind]);
          }
          else
          {
            break;
          }
          pickNeighborPoints(laserCloud, point_ind);
        }
      }

      for (int k = lo; k <= hi; k++)
      {
        if (cloudLabel[k] <= 0)
        {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<point> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<point> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(leafSize, leafSize, leafSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;
  }

  publishMessage(laserCloudMsg, laserCloud, cornerPointsSharp,
   cornerPointsLessSharp,surfPointsFlat, surfPointsLessFlat);
}

} // namespace lidar_mapping
} // namespace beyond

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarFeatureExtract");
  ros::NodeHandle nh;
  beyond::lidar_mapping::lidarFeatureExtract *lidar_feature_extract =
      new beyond::lidar_mapping::lidarFeatureExtract(nh);
  lidar_feature_extract->setPubLidarCloud(nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100));
  lidar_feature_extract->setPubCorner(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100));
  lidar_feature_extract->setPubLessCorner(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100));
  lidar_feature_extract->setPubFlat(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100));
  lidar_feature_extract->setPubLessFlat(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100));
  std::string lidar_topic;
  nh.getParam("lidar/lidar_topic", lidar_topic);
  std::cout << "lidar_topic: " << lidar_topic << std::endl;
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, &beyond::lidar_mapping::lidarFeatureExtract::laserCloudHandler, lidar_feature_extract);

  ros::spin();

  return 0;
}
