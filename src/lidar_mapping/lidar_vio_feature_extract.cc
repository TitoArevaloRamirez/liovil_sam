#include "lidar_mapping/lidar_vio_feature_extract.h"

namespace beyond
{
namespace lidar_mapping
{
lidarFeatureExtract::lidarFeatureExtract(ros::NodeHandle &nh, bool _auto_delete) : odoMsg(_auto_delete)
{
  prev_lidar_cloud = lidarCloud::Ptr(new lidarCloud);
  prev_prev_lidar_cloud = lidarCloud::Ptr(new lidarCloud);
  nh.getParam("lidar/scanPeriod", scanPeriod);
  std::cout << "scanPeriod: " << scanPeriod << std::endl;
  nh.getParam("lidar/systemDelay", systemDelay);
  std::cout << "systemDelay: " << systemDelay << std::endl;
  nh.getParam("lidar/N_SCANS", N_SCANS);
  std::cout << "N_SCANS: " << N_SCANS << std::endl;
  nh.getParam("lidar/num", num);
  std::cout << "num: " << num << std::endl;
  nh.getParam("lidar/leafSize", leafSize);
  std::cout << "leafSize: " << leafSize << std::endl;
  nh.getParam("lidar/lowerBound", lowerBound);
  std::cout << "lowerBound: " << lowerBound << std::endl;
  nh.getParam("lidar/upperBound", upperBound);
  std::cout << "upperBound: " << upperBound << std::endl;
  nh.getParam("lidar/edgeThre1", edgeThre1);
  std::cout << "edgeThre1: " << edgeThre1 << std::endl;
  nh.getParam("lidar/edgeThre2", edgeThre2);
  std::cout << "edgeThre2: " << edgeThre2 << std::endl;
  nh.getParam("lidar/surfThre", surfThre);
  std::cout << "surfThre: " << surfThre << std::endl;
  nh.getParam("lidar/edgeCurvatureThre", edgeCurvatureThre);
  std::cout << "edgeCurvatureThre: " << edgeCurvatureThre << std::endl;
  nh.getParam("lidar/imu2lidar", imu2lidar1);
  imu2lidar(0, 0) = imu2lidar1[0];
  imu2lidar(0, 1) = imu2lidar1[1];
  imu2lidar(0, 2) = imu2lidar1[2];
  imu2lidar(0, 3) = imu2lidar1[3];
  imu2lidar(1, 0) = imu2lidar1[4];
  imu2lidar(1, 1) = imu2lidar1[5];
  imu2lidar(1, 2) = imu2lidar1[6];
  imu2lidar(1, 3) = imu2lidar1[7];
  imu2lidar(2, 0) = imu2lidar1[8];
  imu2lidar(2, 1) = imu2lidar1[9];
  imu2lidar(2, 2) = imu2lidar1[10];
  imu2lidar(2, 3) = imu2lidar1[11];
  imu2lidar(3, 0) = 0;
  imu2lidar(3, 1) = 0;
  imu2lidar(3, 2) = 0;
  imu2lidar(3, 3) = 1;
  std::cout << "imu2lidar" << imu2lidar << std::endl;
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

void lidarFeatureExtract::setPubOdom(ros::Publisher pub_cloud)
{
  pubOdom = pub_cloud;
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
    if (orderedLidarCloud[i].size() < 100) {
      continue;
    }
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

// transform the lidar point to the end of scan
void lidarFeatureExtract::transform_to_end(point pi, point &po, Eigen::Matrix4f dtrans)
{
  po.x = dtrans(0, 0) * pi.x + dtrans(0, 1) * pi.y + dtrans(0, 2) * pi.z + dtrans(0, 3);
  po.y = dtrans(1, 0) * pi.x + dtrans(1, 1) * pi.y + dtrans(1, 2) * pi.z + dtrans(1, 3);
  po.z = dtrans(2, 0) * pi.x + dtrans(2, 1) * pi.y + dtrans(2, 2) * pi.z + dtrans(2, 3);
  po.intensity = pi.intensity;
}

void lidarFeatureExtract::get_dewarped_points(lidarCloud point_cloud, double cur_time,
                                              Eigen::Matrix4f cur_close_odometry, lidarCloud::Ptr dewarped_points)
{
  for (unsigned int i = 0; i < point_cloud.size(); i++)
  {
    point cur_point;
    cur_point = point_cloud.points[i];
    bool cur_odometry_success = false;
    double point_retrieve_time = 0;
    // the lidar points belong to the previous preivous sacn, so we should eliminate the offset(two scan time)
    double point_time = cur_time - 2 * scanPeriod + (cur_point.intensity - int(cur_point.intensity));
    ros::Time cur_point_time = (ros::Time)point_time;
    // retrieve the odometry information
    Eigen::Matrix4f cur_point_odometry =
        odoMsg.GetClosestEntry(cur_point_time, cur_odometry_success, point_retrieve_time);
    // calculate transformation from this point to the end of scan
    Eigen::Matrix4f dtrans = cur_close_odometry.inverse() * cur_point_odometry;
    // calculate time difference
    double df = point_time - point_retrieve_time;
    // transform this point to the end of scan
    transform_to_end(cur_point, cur_point, dtrans);
    if (cur_odometry_success && fabs(df) < 0.005)
    {
      dewarped_points->push_back(cur_point);
    }
  }
}

void lidarFeatureExtract::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  lidar_id++;
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
  //pcl::fromROSMsg(*laserCloudMsg, laserCloudIn_vel);

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
  size_t point_num = cloudSize;
  lidarCloud::Ptr laserCloud(new lidarCloud());
  for (int i = 0; i < cloudSize; i++)
  {
    double x = laserCloudIn.at(i).x;
    double y = laserCloudIn.at(i).y;
    double z = laserCloudIn.at(i).z;
    float angle = std::atan(z/ std::sqrt(x * x + y * y));
    float _factor = (N_SCANS - 1) / (upperBound - lowerBound);

    int scanID = int(((angle * 180 / M_PI) - lowerBound) * _factor + 0.5);
    if (scanID >= N_SCANS || scanID < 0 ){
      point_num--;
      continue;
    }
    double cur_yaw_angle = getYawAngle(y, x, start_yaw_angle, end_yaw_angle, halfPassed);

    float relTime = (cur_yaw_angle - start_yaw_angle) / (end_yaw_angle - start_yaw_angle);
    point cur_point;
    cur_point.x = x;
    cur_point.y = y;
    cur_point.z = z;
    cur_point.intensity = scanID + scanPeriod * relTime;
    orderedLidarCloud[scanID].push_back(cur_point);
  }
  cloudSize = point_num;

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
    if (orderedLidarCloud[i].size() < 100) {
      continue;
    }
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

  //dewarp lidar feature points
  lidarCloud::Ptr dewarped_cloud(new lidarCloud);
  // dewarped edge points
  lidarCloud::Ptr dewarped_edge_points(new lidarCloud);
  // dewarped surface points
  lidarCloud::Ptr dewarped_surf_points(new lidarCloud);

  bool odom_retrieve_res = false;
  double cur_time = laserCloudMsg->header.stamp.toSec() - scanPeriod;
  ros::Time retrieve_ros_time = (ros::Time)cur_time;
  double retrived_time = 0;
  Eigen::Matrix4f cur_close_odometry =
      odoMsg.GetClosestEntry(retrieve_ros_time, odom_retrieve_res, retrived_time);

  double time_diff = cur_time - retrived_time;

  Eigen::Matrix4f dtrans(Eigen::Matrix4f::Identity());

  if (fabs(time_diff) < 0.05 && odom_retrieve_res)
  {
    std::cout << "time diff: " << time_diff << std::endl;
    // dewarp edge feature points
    get_dewarped_points(prev_prev_edge, cur_time, cur_close_odometry, dewarped_edge_points);
    // dewarp plane feature points
    get_dewarped_points(prev_prev_surf, cur_time, cur_close_odometry, dewarped_surf_points);
    // dewarp full points
    get_dewarped_points(*prev_prev_lidar_cloud, cur_time, cur_close_odometry, dewarped_cloud);

    dewarped_edge_points->header.stamp = (cur_time)*1000000;
    dewarped_surf_points->header.stamp = (cur_time)*1000000;
    dewarped_cloud->header.stamp = (cur_time)*1000000;


    Eigen::Matrix3f mat = cur_close_odometry.matrix().topLeftCorner<3, 3>();
    Eigen::Quaternionf q_b(mat);
    nav_msgs::Odometry velocity_vehicle_odom;

    //velocity_vehicle_odom.header.stamp = (ros::Time) time_count;
    velocity_vehicle_odom.header.stamp =  laserCloudMsg->header.stamp;
    velocity_vehicle_odom.header.frame_id = "/vehicle_init";
    velocity_vehicle_odom.child_frame_id = "/vehicle";
    velocity_vehicle_odom.pose.pose.orientation.x = q_b.x();
    velocity_vehicle_odom.pose.pose.orientation.y = q_b.y();
    velocity_vehicle_odom.pose.pose.orientation.z = q_b.z();
    velocity_vehicle_odom.pose.pose.orientation.w = q_b.w();
    velocity_vehicle_odom.pose.pose.position.x = cur_close_odometry(0, 3);
    velocity_vehicle_odom.pose.pose.position.y = cur_close_odometry(1, 3);
    velocity_vehicle_odom.pose.pose.position.z = cur_close_odometry(2, 3);
    velocity_vehicle_odom.twist.twist.linear.x = lidar_id;
    pubOdom.publish(velocity_vehicle_odom);

    //save the feature point
    prev_prev_edge = prev_edge;
    prev_prev_surf = prev_surf;

    prev_edge = cornerPointsLessSharp;
    prev_surf = surfPointsLessFlat;

    prev_prev_lidar_cloud = prev_lidar_cloud;
    prev_lidar_cloud = laserCloud;
    cur_time = laserCloudMsg->header.stamp.toSec() - 2 * scanPeriod;
    ros::Time cur_laser_time = (ros::Time)cur_time;
    publishMessage(laserCloudMsg, dewarped_cloud, cornerPointsSharp,
                 *dewarped_edge_points, surfPointsFlat, *dewarped_surf_points);

    odoMsg.DeletePrevious(cur_laser_time);
  }
}

void lidarFeatureExtract::odometryHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
  Eigen::Matrix4f cur_vio_odom(Eigen::Matrix4f::Identity());
  Eigen::Quaternionf q_b;
  Eigen::Matrix3f rot_b2o;
  Eigen::Matrix3f rot_lidar2imu;
  Eigen::Quaternionf q_lidar2imu;

  q_b.x() = odom->pose.pose.orientation.x;
  q_b.y() = odom->pose.pose.orientation.y;
  q_b.z() = odom->pose.pose.orientation.z;
  q_b.w() = odom->pose.pose.orientation.w;
  rot_b2o = q_b.toRotationMatrix();
  cur_vio_odom.matrix().topLeftCorner<3, 3>() = rot_b2o;
  cur_vio_odom(0, 3) = odom->pose.pose.position.x;
  cur_vio_odom(1, 3) = odom->pose.pose.position.y;
  cur_vio_odom(2, 3) = odom->pose.pose.position.z;
  cur_vio_odom = cur_vio_odom * imu2lidar;
  odoMsg.AddEntry(odom->header.stamp, cur_vio_odom);
}

} // namespace lidar_mapping
} // namespace beyond

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarFeatureExtract");
  ros::NodeHandle nh;


  beyond::lidar_mapping::lidarFeatureExtract *lidar_feature_extract =
      new beyond::lidar_mapping::lidarFeatureExtract(nh, false);

  lidar_feature_extract->setPubLidarCloud(nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100));
  lidar_feature_extract->setPubCorner(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100));
  lidar_feature_extract->setPubLessCorner(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100));
  lidar_feature_extract->setPubFlat(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100));
  lidar_feature_extract->setPubLessFlat(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100));
  lidar_feature_extract->setPubOdom(nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100));

  std::string lidar_topic, vio_odom_topic;
  nh.getParam("lidar/lidar_topic", lidar_topic);
  std::cout << "lidar_topic: " << lidar_topic << std::endl;
  nh.getParam("lidar/vio_odom_topic", vio_odom_topic);
  std::cout << "vio_odom_topic: " << vio_odom_topic << std::endl;


  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, &beyond::lidar_mapping::lidarFeatureExtract::laserCloudHandler, lidar_feature_extract);
  ros::Subscriber subOdometry = nh.subscribe(vio_odom_topic, 1000, &beyond::lidar_mapping::lidarFeatureExtract::odometryHandler, lidar_feature_extract);

  ROS_INFO("\033[1;32m----> LiDAR VIO feature extract Started.\033[0m");

  ros::spin();

  return 0;
}
