#include <lidar_mapping/lidar_odometry.h>
namespace beyond
{
namespace lidar_mapping
{
lidarOdometry::lidarOdometry(ros::NodeHandle &nh)
{
  cornerPointsSharp = pointCloud::Ptr(new pointCloud);
  cornerPointsLessSharp = pointCloud::Ptr(new pointCloud);
  surfPointsFlat = pointCloud::Ptr(new pointCloud);
  surfPointsLessFlat = pointCloud::Ptr(new pointCloud);
  laserCloudCornerLast = pointCloud::Ptr(new pointCloud);
  laserCloudSurfLast = pointCloud::Ptr(new pointCloud);
  laserCloudOri = pointCloud::Ptr(new pointCloud);
  laserCloudFullRes = pointCloud::Ptr(new pointCloud);
  kdtreeCornerLast = pcl::KdTreeFLANN<point>::Ptr(new pcl::KdTreeFLANN<point>);
  kdtreeSurfLast = pcl::KdTreeFLANN<point>::Ptr(new pcl::KdTreeFLANN<point>);
  nh.getParam("lidar/skipFrameNum", skipFrameNum);
  std::cout << "skipFrameNum: " << skipFrameNum << std::endl;
  frameCount = skipFrameNum;
}

/*
   transform the 1 by 6 vector to transformation matrix
*/
Eigen::Matrix4f lidarOdometry::vecToMatrix(float *x)
{
  float rx = x[0], ry = x[1], rz = x[2], tx = x[3], ty = x[4], tz = x[5];
  Eigen::Matrix4f H(Eigen::Matrix4f::Identity());

  Eigen::AngleAxisd rollAngle(rz, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(ry, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(rx, Eigen::Vector3d::UnitX());

  Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

  Eigen::Matrix3d rotationMatrix = q.matrix();

  Eigen::Matrix4f dtrans;

  dtrans(0, 0) = rotationMatrix(0, 0);
  dtrans(0, 1) = rotationMatrix(0, 1);
  dtrans(0, 2) = rotationMatrix(0, 2);
  dtrans(0, 3) = tx;
  dtrans(1, 0) = rotationMatrix(1, 0);
  dtrans(1, 1) = rotationMatrix(1, 1);
  dtrans(1, 2) = rotationMatrix(1, 2);
  dtrans(1, 3) = ty;
  dtrans(2, 0) = rotationMatrix(2, 0);
  dtrans(2, 1) = rotationMatrix(2, 1);
  dtrans(2, 2) = rotationMatrix(2, 2);
  dtrans(2, 3) = tz;
  dtrans(3, 0) = 0;
  dtrans(3, 1) = 0;
  dtrans(3, 2) = 0;
  dtrans(3, 3) = 1;

  return dtrans;
}

void lidarOdometry::matrixToVec(float *x, Eigen::Matrix4f dtrans1)
{
  Eigen::Matrix3f rotationMat;
  rotationMat << dtrans1(0, 0), dtrans1(0, 1), dtrans1(0, 2),
      dtrans1(1, 0), dtrans1(1, 1), dtrans1(1, 2),
      dtrans1(2, 0), dtrans1(2, 1), dtrans1(2, 2);
  Eigen::Vector3f ea = rotationMat.eulerAngles(0, 1, 2);

  x[0] = ea[0];
  x[1] = ea[1];
  x[2] = ea[2];
  x[3] = dtrans1(0, 3);
  x[4] = dtrans1(1, 3);
  x[5] = dtrans1(2, 3);
}


void lidarOdometry::TransformToStart(point const *const pi, point *const po)
{
  float s = 10 * (pi->intensity - int(pi->intensity));

  float transformVec[6] = {0};

  transformVec[0] = s * transform[0];
  transformVec[1] = s * transform[1];
  transformVec[2] = s * transform[2];
  transformVec[3] = s * transform[3];
  transformVec[4] = s * transform[4];
  transformVec[5] = s * transform[5];

  Eigen::Matrix4f dtrans = vecToMatrix(transformVec);
  dtrans = dtrans.inverse();

  po->x = dtrans(0, 0) * pi->x + dtrans(0, 1) * pi->y + dtrans(0, 2) * pi->z + dtrans(0, 3);
  po->y = dtrans(1, 0) * pi->x + dtrans(1, 1) * pi->y + dtrans(1, 2) * pi->z + dtrans(1, 3);
  po->z = dtrans(2, 0) * pi->x + dtrans(2, 1) * pi->y + dtrans(2, 2) * pi->z + dtrans(2, 3);
  po->intensity = pi->intensity;
}

void lidarOdometry::TransformToEnd(point const *const pi, point *const po)
{
  float s = 1 - 10 * (pi->intensity - int(pi->intensity));
  float transformVec[6] = {0};

  transformVec[0] = s * transform[0];
  transformVec[1] = s * transform[1];
  transformVec[2] = s * transform[2];
  transformVec[3] = s * transform[3];
  transformVec[4] = s * transform[4];
  transformVec[5] = s * transform[5];
  Eigen::Matrix4f dtrans = vecToMatrix(transformVec);

  po->x = dtrans(0, 0) * pi->x + dtrans(0, 1) * pi->y + dtrans(0, 2) * pi->z + dtrans(0, 3);
  po->y = dtrans(1, 0) * pi->x + dtrans(1, 1) * pi->y + dtrans(1, 2) * pi->z + dtrans(1, 3);
  po->z = dtrans(2, 0) * pi->x + dtrans(2, 1) * pi->y + dtrans(2, 2) * pi->z + dtrans(2, 3);
  po->intensity = pi->intensity;
}

void lidarOdometry::setPubEdgeCloud(ros::Publisher pub_cloud)
{
  pubLaserCloudCornerLast = pub_cloud;
}

void lidarOdometry::setPubSurfCloud(ros::Publisher pub_cloud)
{
  pubLaserCloudSurfLast = pub_cloud;
}

void lidarOdometry::setPubFullCloud(ros::Publisher pub_cloud)
{
  pubLaserCloudFullRes = pub_cloud;
}

void lidarOdometry::setPubOdom(ros::Publisher pub_cloud)
{
  pubLaserOdometry = pub_cloud;
}

void lidarOdometry::process(const sensor_msgs::PointCloud2ConstPtr &edge_point_cloud,
                            const sensor_msgs::PointCloud2ConstPtr &edge_less_point_cloud,
                            const sensor_msgs::PointCloud2ConstPtr &surf_point_cloud,
                            const sensor_msgs::PointCloud2ConstPtr &surf_less_point_cloud,
                            const sensor_msgs::PointCloud2ConstPtr &full_point_cloud)
{
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();
  laserCloudCornerLast->clear();
  laserCloudSurfLast->clear();
  laserCloudOri->clear();
  laserCloudFullRes->clear();
  pcl::fromROSMsg(*edge_point_cloud, *cornerPointsSharp);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
  pcl::fromROSMsg(*edge_less_point_cloud, *cornerPointsLessSharp);
  pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
  pcl::fromROSMsg(*surf_point_cloud, *surfPointsFlat);
  pcl::removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
  pcl::fromROSMsg(*surf_less_point_cloud, *surfPointsLessFlat);
  pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
  pcl::fromROSMsg(*full_point_cloud, *laserCloudFullRes);
  pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
  double timeSurfPointsLessFlat = full_point_cloud->header.stamp.toSec();
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/laser_odom";
  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  std::vector<double> errors;
  point pointOri, pointSel, tripod1, tripod2, tripod3;
  bool isDegenerate = false;
  Eigen::Matrix<float, 6, 6> matP;
  if (!systemInited)
  {
    pcl::PointCloud<point>::Ptr laserCloudTemp = cornerPointsLessSharp;
    cornerPointsLessSharp = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat;
    surfPointsLessFlat = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudTemp;

    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    systemInited = true;
    return;
  }
  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
  {
    PointCloudN jacobians;
    edgePlaneJacobian lidar_jacobian;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
    int cornerPointsSharpNum = cornerPointsSharp->points.size();
    int surfPointsFlatNum = surfPointsFlat->points.size();
    for (int iterCount = 0; iterCount < 25; iterCount++)
    {
      for (int i = 0; i < cornerPointsSharpNum; i++)
      {
        TransformToStart(&cornerPointsSharp->points[i], &pointSel);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);

        kdtreeCornerLast->nearestKSearch(pointSel, 2, pointSearchInd, pointSearchSqDis);

        if (pointSearchInd[1] >= 0 && pointSearchSqDis[1] < 10)
        {
          tripod1 = laserCloudCornerLast->points[pointSearchInd[0]];
          tripod2 = laserCloudCornerLast->points[pointSearchInd[1]];
          float x00 = cornerPointsSharp->points[i].x, y00 = cornerPointsSharp->points[i].y, z00 = cornerPointsSharp->points[i].z;
          float x0 = pointSel.x, y0 = pointSel.y, z0 = pointSel.z;
          float x1 = tripod1.x, y1 = tripod1.y, z1 = tripod1.z;
          float x2 = tripod2.x, y2 = tripod2.y, z2 = tripod2.z;

          pcl::PointNormal temp_jacobian = lidar_jacobian.calc_edge_jacobians(x00, y00, z00, x1, y1, z1, x2, y2, z2, x0, y0, z0, transform);

          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

          float ld2 = a012 / l12;

          float s = 1;
          if (iterCount >= 5)
          {
            s = 1 - 1.8 * fabs(ld2);
          }

          double error = s * ld2;

          if (s > 0.1 && ld2 != 0)
          {
            laserCloudOri->push_back(cornerPointsSharp->points[i]);
            errors.emplace_back(error);
            jacobians.push_back(temp_jacobian);
          }
        }
      }

      for (int i = 0; i < surfPointsFlatNum; i++)
      {
        TransformToStart(&surfPointsFlat->points[i], &pointSel);

        kdtreeSurfLast->nearestKSearch(pointSel, 3, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

        if (pointSearchSqDis[2] < 10 && pointSearchInd[2] > 0)
        {
          tripod1 = laserCloudSurfLast->points[pointSearchInd[0]];
          tripod2 = laserCloudSurfLast->points[pointSearchInd[1]];
          tripod3 = laserCloudSurfLast->points[pointSearchInd[2]];
          float x00 = surfPointsFlat->points[i].x, y00 = surfPointsFlat->points[i].y, z00 = surfPointsFlat->points[i].z;
          float x1 = tripod1.x, y1 = tripod1.y, z1 = tripod1.z;
          float x2 = tripod2.x, y2 = tripod2.y, z2 = tripod2.z;
          float x3 = tripod3.x, y3 = tripod3.y, z3 = tripod3.z;

          pcl::PointNormal temp_jacobian = lidar_jacobian.calc_plane_jacobians(x00, y00, z00,
                                                                               x1, y1, z1, x2, y2, z2, x3, y3, z3, transform);

          float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
          float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
          float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
          float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

          float ps = sqrt(pa * pa + pb * pb + pc * pc);
          pa /= ps;
          pb /= ps;
          pc /= ps;
          pd /= ps;

          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          float s = 1;
          if (iterCount >= 5)
          {
            s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
          }

          double error = s * pd2;

          if (s > 0.1 && pd2 != 0)
          {
            laserCloudOri->push_back(surfPointsFlat->points[i]);
            errors.emplace_back(error);
            jacobians.push_back(temp_jacobian);
          }
        }
      }

      int pointSelNum = laserCloudOri->points.size();
      if (pointSelNum < 10)
      {
        continue;
      }

      Eigen::VectorXf residuals(pointSelNum);
      Eigen::MatrixXf J(pointSelNum, 6);
      Eigen::MatrixXf JtJ;
      Eigen::VectorXf diag;
      Eigen::DiagonalMatrix<float, Eigen::Dynamic> diagonal;
      for (int i = 0; i < pointSelNum; i++)
      {

        float d2 = errors[i];
        J(i, 0) = jacobians[i].x;
        J(i, 1) = jacobians[i].y;
        J(i, 2) = jacobians[i].z;
        J(i, 3) = jacobians[i].normal_x;
        J(i, 4) = jacobians[i].normal_y;
        J(i, 5) = jacobians[i].normal_z;
        residuals(i) =  0.05 * d2;
      }

      JtJ = J.transpose() * J;
      diag = JtJ.diagonal();
      diagonal.diagonal() = diag;
      Eigen::Matrix<float, 6, 1> curr_delta;
      curr_delta = JtJ.colPivHouseholderQr().solve(J.transpose() * residuals);

      if (iterCount == 0)
      {
        isDegenerate = false;
        Eigen::Matrix<float, 6, 6> matV;
        Eigen::Matrix<float, 6, 6> matV2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(JtJ);
        Eigen::Matrix<float, 1, 6> matE = esolver.eigenvalues().real();
        matV = esolver.eigenvectors().real();
        matV2 = matV;
        for (int i = 0; i < 6; i++)
        {
          if (matE(0, i) < 10)
          {
            isDegenerate = true;
            for (int j = 0; j < 6; j++)
            {
              matV2(i, j) = 0;
            }
            isDegenerate = true;
          }
          else
          {
            break;
          }
        }
        matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
        Eigen::Matrix<float, 6, 1> matX2;
        for (int i = 0; i < 6; i++)
        {
          matX2(0, i) = curr_delta(0, i);
        }
        curr_delta = matP * matX2;
      }

      if (std::isnan(curr_delta(0,0)) || std::isnan(curr_delta(1, 0)) || std::isnan(curr_delta(2, 0)) ||
          std::isnan(curr_delta(3, 0)) || std::isnan(curr_delta(4, 0)) || std::isnan(curr_delta(5, 0)))
      {
        printf("optimization fail at this time");
      }
      else
      {
        transform[0] -= curr_delta(0, 0);
        transform[1] -= curr_delta(1, 0);
        transform[2] -= curr_delta(2, 0);
        transform[3] -= curr_delta(3, 0);
        transform[4] -= curr_delta(4, 0);
        transform[5] -= curr_delta(5, 0);
      }
      float deltaR = sqrt(
          pow(rad2deg(curr_delta(0, 0)), 2) +
          pow(rad2deg(curr_delta(1, 0)), 2) +
          pow(rad2deg(curr_delta(2, 0)), 2));
      float deltaT = sqrt(
          pow(curr_delta(3, 0) * 100, 2) +
          pow(curr_delta(4, 0) * 100, 2) +
          pow(curr_delta(5, 0) * 100, 2));

      if (deltaR < 0.1 && deltaT < 0.1)
      {
        break;
      }
    }
  }

  float rx, ry, rz, tx, ty, tz;

  Eigen::Matrix4f dtrans = vecToMatrix(transform);
  Eigen::Matrix4f dtrans1 = vecToMatrix(transformSum);

  dtrans1 = dtrans1 * dtrans.inverse();

  matrixToVec(transformSum, dtrans1);
  rx = transformSum[0];
  ry = transformSum[1];
  rz = transformSum[2];
  tx = transformSum[3];
  ty = transformSum[4];
  tz = transformSum[5];

  Eigen::Matrix3f rot_b2o = dtrans1.matrix().topLeftCorner<3, 3>(); 
  Eigen::Quaternionf q_b(rot_b2o);

  laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
  laserOdometry.pose.pose.orientation.x = q_b.x();
  laserOdometry.pose.pose.orientation.y = q_b.y();
  laserOdometry.pose.pose.orientation.z = q_b.z();
  laserOdometry.pose.pose.orientation.w = q_b.w();
  laserOdometry.pose.pose.position.x = tx;
  laserOdometry.pose.pose.position.y = ty;
  laserOdometry.pose.pose.position.z = tz;
  pubLaserOdometry.publish(laserOdometry);


  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++)
  {
    TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
  }

  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++)
  {
    TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
  }

  frameCount++;
  if (frameCount >= skipFrameNum + 1)
  {
    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
      TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
    }
  }

  pcl::PointCloud<point>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();
  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
  {

    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
  }

  if (frameCount >= skipFrameNum + 1) 
  {
    frameCount = 0;

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudCornerLast2.header.frame_id = "/camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudSurfLast2.header.frame_id = "/camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudFullRes3.header.frame_id = "/camera";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);
  }
}
} // namespace lidar_mapping
} // namespace beyond

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarOdometry");
  ros::NodeHandle nh;
  beyond::lidar_mapping::lidarOdometry *lidar_odom =
      new beyond::lidar_mapping::lidarOdometry(nh);
  lidar_odom->setPubEdgeCloud(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100));
  lidar_odom->setPubSurfCloud(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100));
  lidar_odom->setPubFullCloud(nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100));
  lidar_odom->setPubOdom(nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100));

  message_filters::Subscriber<sensor_msgs::PointCloud2> edge_sub(nh, "/laser_cloud_sharp", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> edge_less_sub(nh, "/laser_cloud_less_sharp", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> surf_sub(nh, "/laser_cloud_flat", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> surf_less_sub(nh, "/laser_cloud_less_flat", 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> full_point_sub(nh, "/velodyne_cloud_2", 100);

  
  TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      sync(edge_sub, edge_less_sub, surf_sub, surf_less_sub, full_point_sub, 100);
  sync.registerCallback(boost::bind(&beyond::lidar_mapping::lidarOdometry::process, lidar_odom, _1, _2, _3, _4, _5));

  ROS_INFO("\033[1;32m----> LiDAR Odometry Started.\033[0m");

  ros::spin();
  return 0;
}
