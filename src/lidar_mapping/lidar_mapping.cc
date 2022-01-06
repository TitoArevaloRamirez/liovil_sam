#include "lidar_mapping/lidar_mapping.h"
#include <sstream>
#include "lidar_mapping/nanoflann_pcl.h"

std::ofstream myfile;

namespace beyond
{
namespace lidar_mapping
{
lidarMapping::lidarMapping(ros::NodeHandle &nh)
{
    nh.getParam("lidar/scanPeriod", scanPeriod);
    std::cout << "scanPeriod: " << scanPeriod << std::endl;
    nh.getParam("lidar/cornerLeafSize", cornerLeafSize);
    std::cout << "cornerLeafSize: " << cornerLeafSize << std::endl;
    nh.getParam("lidar/surfLeafSize", surfLeafSize);
    std::cout << "surfLeafSize: " << surfLeafSize << std::endl;
    nh.getParam("lidar/mapLeafSize", mapLeafSize);
    std::cout << "mapLeafSize: " << mapLeafSize << std::endl;
    nh.getParam("lidar/bisquareWeight", bisquareWeight);
    bisquareWeight = 0.9;
    std::cout << "bisquareWeight: " << bisquareWeight << std::endl;
    nh.getParam("lidar/outlier_threshold", outlier_threshold);
    outlier_threshold = 0.1;
    std::cout << "outlier_threshold: " << outlier_threshold << std::endl;
    nh.getParam("lidar/edge_threshold", edge_threshold);
    std::cout << "edge_threshold: " << edge_threshold << std::endl;
    nh.getParam("lidar/surf_threshold", surf_threshold);
    std::cout << "surf_threshold: " << surf_threshold << std::endl;
    nh.getParam("lidar/maxIterNum", maxIterNum);
    std::cout << "maxIterNum: " << maxIterNum << std::endl;
    nh.getParam("lidar/validPointsThreshold", validPointsThreshold);
    validPointsThreshold = 50;
    std::cout << "validPointsThreshold: " << validPointsThreshold << std::endl;

    edge_point_cloud = pointCloud::Ptr(new pointCloud);
    surf_point_cloud = pointCloud::Ptr(new pointCloud);
    cur_edge_cloud = pointCloud::Ptr(new pointCloud);
    cur_sur_cloud = pointCloud::Ptr(new pointCloud);
    cur_edge_cloud_tmp = pointCloud::Ptr(new pointCloud);
    cur_surf_cloud_tmp = pointCloud::Ptr(new pointCloud);
    point_cloud_in_lidar = pointCloud::Ptr(new pointCloud);
    all_point_cloud = pointCloud::Ptr(new pointCloud);
    edge_map_tree = pcl::KdTreeFLANN<point>::Ptr(new pcl::KdTreeFLANN<point>);
    surf_map_tree = pcl::KdTreeFLANN<point>::Ptr(new pcl::KdTreeFLANN<point>);
    edges_map_near = pointCloud::Ptr(new pointCloud);
    planes_map_near = pointCloud::Ptr(new pointCloud);
    publish_cloud = pointCloud::Ptr(new pointCloud);

    corner_points_filter.setLeafSize(0.2, 0.2, 0.2);

    surf_points_filter.setLeafSize(0.4, 0.4, 0.4);

    all_points_filter.setLeafSize(mapLeafSize, mapLeafSize, mapLeafSize);
}

void lidarMapping::set_pub_surrond_cloud(ros::Publisher pub_cloud)
{
    pubLaserCloudSurround = pub_cloud;
}

void lidarMapping::set_pub_full_cloud(ros::Publisher pub_cloud)
{
    pubLaserCloudFullRes = pub_cloud;
}

void lidarMapping::set_pub_odom(ros::Publisher pub_odom)
{
    pubOdomAftMapped = pub_odom;
}

void lidarMapping::set_pub_path(ros::Publisher pub_path)
{
    pubLidarPath = pub_path;
}

// calculate the cube index of the point
intpoint lidarMapping::get_map_cell(const point cur_point)
{

    int x = floor(cur_point.x / 5),
        y = floor(cur_point.y / 5),
        z = floor(cur_point.z / 5);
    return intpoint(x, y, z);
}

void lidarMapping::update_cells(PointCloudI::Ptr edge_cloud, PointCloudI::Ptr plane_cloud)
{

    map_cells.clear();

    // update map_cells, edges_map_cells and _global_edges
    for (unsigned int i = 0; i < edge_cloud->size(); i++)
    {

        point cur_edge_point = edge_cloud->at(i);
        intpoint map_cell_ind = get_map_cell(cur_edge_point);
        map_cells.insert(map_cell_ind);
        if (!_global_edges.count(map_cell_ind))
        {
            _global_edges[map_cell_ind] = PointCloudI::Ptr(new PointCloudI);
            _global_edges1[map_cell_ind] = PointCloudI::Ptr(new PointCloudI);
            edges_map_cells.insert(map_cell_ind);
        }
        _global_edges[map_cell_ind]->push_back(cur_edge_point);
    }

    // filter global edge points
    for (auto iter = map_cells.begin();
         iter != map_cells.end();
         iter++)
    {

        intpoint map_cell_ind = *iter;
        corner_points_filter.setInputCloud(_global_edges[map_cell_ind]);
        corner_points_filter.filter(*(_global_edges1[map_cell_ind])); // can't use the input as output
        std::swap(_global_edges[map_cell_ind], _global_edges1[map_cell_ind]);
    }

    map_cells.clear();

    // update map_cells, planes_map_cells and _global_planes
    for (unsigned int i = 0; i < plane_cloud->size(); i++)
    {

        point cur_surf_point = plane_cloud->at(i);
        intpoint map_cell_ind = get_map_cell(cur_surf_point);
        map_cells.insert(map_cell_ind);
        if (!_global_planes.count(map_cell_ind))
        {
            _global_planes[map_cell_ind] = PointCloudI::Ptr(new PointCloudI);
            _global_planes1[map_cell_ind] = PointCloudI::Ptr(new PointCloudI);
            planes_map_cells.insert(map_cell_ind);
        }
        _global_planes[map_cell_ind]->push_back(cur_surf_point);
    }

    // filter global plane points
    for (auto iter = map_cells.begin();
         iter != map_cells.end();
         iter++)
    {
        intpoint map_cell_ind = *iter;

        surf_points_filter.setInputCloud(_global_planes[map_cell_ind]);
        surf_points_filter.filter(*(_global_planes1[map_cell_ind]));
        std::swap(_global_planes[map_cell_ind], _global_planes1[map_cell_ind]);
    }
}

void lidarMapping::update_neighbor(PointCloudI::Ptr edge_cloud, PointCloudI::Ptr plane_cloud)
{
    edges_map_near->clear();
    planes_map_near->clear();

    map_cells.clear();

    // find cubes containing edge feature points
    for (unsigned int i = 0; i < edge_cloud->size(); i++)
    {
        point cur_edge_point = edge_cloud->at(i);
        intpoint map_cell_ind = get_map_cell(cur_edge_point);
        map_cells.insert(map_cell_ind);
    }

    // update neighbor map of edge feature points
    for (auto iter = map_cells.begin();
         iter != map_cells.end();
         iter++)
    {
        intpoint map_cell_ind = *iter;
        if (_global_edges.count(map_cell_ind))
        {
            *edges_map_near += *_global_edges[map_cell_ind];
        }
    }

    // find cubes containing plane feature points
    map_cells.clear();
    for (unsigned int i = 0; i < plane_cloud->size(); i++)
    {
        point cur_surf_point = plane_cloud->at(i);
        intpoint map_cell_ind = get_map_cell(cur_surf_point);
        map_cells.insert(map_cell_ind);
    }

    // ipdate neighbor map of plane feature points
    for (auto iter = map_cells.begin();
         iter != map_cells.end();
         iter++)
    {
        intpoint map_cell_ind = *iter;
        if (_global_planes.count(map_cell_ind))
        {
            *planes_map_near += *_global_planes[map_cell_ind];
        }
    }
}

void lidarMapping::receive_messages(const sensor_msgs::PointCloud2ConstPtr &receive_edge_cloud,
                                    const sensor_msgs::PointCloud2ConstPtr &receive_surf_points,
                                    const sensor_msgs::PointCloud2ConstPtr &receive_all_points,
                                    const nav_msgs::Odometry::ConstPtr &input_odometry)
{
    odom_time = input_odometry->header.stamp.toSec();
    edge_point_cloud->clear();
    pcl::fromROSMsg(*receive_edge_cloud, *edge_point_cloud);
    surf_point_cloud->clear();
    pcl::fromROSMsg(*receive_surf_points, *surf_point_cloud);
    all_point_cloud->clear();
    pcl::fromROSMsg(*receive_all_points, *all_point_cloud);
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = input_odometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, geoQuat.x, geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
    transformSum[0] = pitch;
    transformSum[1] = yaw;
    transformSum[2] = roll;
    transformSum[3] = input_odometry->pose.pose.position.x;
    transformSum[4] = input_odometry->pose.pose.position.y;
    transformSum[5] = input_odometry->pose.pose.position.z;
    lidar_id = input_odometry->twist.twist.linear.x;
}

Eigen::Matrix4f lidarMapping::vecToMatrix(float *x)
{
    float rx = x[0], ry = x[1], rz = x[2], tx = x[3], ty = x[4], tz = x[5];

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, rx, ry);

    Eigen::Quaterniond q;

    q.x() = geoQuat.y;
    q.y() = geoQuat.z;
    q.z() = geoQuat.x;
    q.w() = geoQuat.w;

    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

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

void lidarMapping::matrixToVec(float *x, Eigen::Matrix4f dtrans1)
{

    Eigen::Matrix3f rotation_mat;
    rotation_mat << dtrans1(0, 0), dtrans1(0, 1), dtrans1(0, 2),
        dtrans1(1, 0), dtrans1(1, 1), dtrans1(1, 2),
        dtrans1(2, 0), dtrans1(2, 1), dtrans1(2, 2);
    Eigen::Quaternionf q(rotation_mat);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf::Quaternion(q.z(), q.x(), q.y(), q.w())).getRPY(roll, pitch, yaw);

    x[0] = pitch;
    x[1] = yaw;
    x[2] = roll;
    x[3] = dtrans1(0, 3);
    x[4] = dtrans1(1, 3);
    x[5] = dtrans1(2, 3);
}

void lidarMapping::transformAssociateToMap()
{
    Eigen::Matrix4f prev_odom = vecToMatrix(transformBefMapped);
    Eigen::Matrix4f cur_odom = vecToMatrix(transformSum);
    Eigen::Matrix4f dtrans_odom = prev_odom.inverse() * cur_odom;
    Eigen::Matrix4f prev_trans_map = vecToMatrix(transformAftMapped);
    Eigen::Matrix4f cur_trans_map = prev_trans_map * dtrans_odom;
    matrixToVec(transformTobeMapped, cur_trans_map);
}

nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;

void lidarMapping::cal_edge_points_jacobian(std::vector<double> &errors,
                                            edgePlaneJacobian &lidar_jacobian, PointCloudN &jacobians)
{
    point point_in_lidar, transformed_point;
    std::vector<int> nearest_points_ind;
    std::vector<float> nearest_points_dist;
    Eigen::Matrix3f A1;
    A1.setZero();
    Eigen::Matrix<float, 1, 3> D1;
    D1.setZero();
    Eigen::Matrix3f V1;
    V1.setZero();

    Eigen::Matrix4f trans_map = vecToMatrix(transformTobeMapped);

    trans_edge = pcl::PointCloud<point>::Ptr(new pcl::PointCloud<point>());
    trans_edge->clear();
    pcl::transformPointCloud(*cur_edge_cloud, *trans_edge, trans_map);

    for (int i = 0; i < cur_edge_cloud->size(); i++)
    {
        point_in_lidar = cur_edge_cloud->points[i];
        transformed_point = trans_edge->points[i];
        kdtreeCornerFromMap.nearestKSearch(transformed_point, 5, nearest_points_ind, nearest_points_dist);

        if (nearest_points_dist[4] < 1.0)
        {
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++)
            {
                cx += edges_map_near->points[nearest_points_ind[j]].x / 5;
                cy += edges_map_near->points[nearest_points_ind[j]].y / 5;
                cz += edges_map_near->points[nearest_points_ind[j]].z / 5;
            }

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++)
            {
                float ax = edges_map_near->points[nearest_points_ind[j]].x - cx;
                float ay = edges_map_near->points[nearest_points_ind[j]].y - cy;
                float az = edges_map_near->points[nearest_points_ind[j]].z - cz;

                a11 += ax * ax / 5;
                a12 += ax * ay / 5;
                a13 += ax * az / 5;
                a22 += ay * ay / 5;
                a23 += ay * az / 5;
                a33 += az * az / 5;
            }

            A1 << a11, a12, a13,
                a12, a22, a23, 
                a13, a23, a33;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(A1);
            D1 = esolver.eigenvalues().real();
            V1 = esolver.eigenvectors().real();

            if (D1(0, 2) > 3 * D1(0, 1))
            {

                float x0 = transformed_point.x;
                float y0 = transformed_point.y;
                float z0 = transformed_point.z;
                float x1 = cx + 0.1 * V1(0, 2);
                float y1 = cy + 0.1 * V1(1, 2);
                float z1 = cz + 0.1 * V1(2, 2);
                float x2 = cx - 0.1 * V1(0, 2);
                float y2 = cy - 0.1 * V1(1, 2); 
                float z2 = cz - 0.1 * V1(2, 2);

                float x00 = point_in_lidar.x, y00 = point_in_lidar.y, z00 = point_in_lidar.z;
                pcl::PointNormal temp_jacobian = lidar_jacobian.calc_edge_jacobians(x00, y00, z00, x1, y1, z1, x2, y2, z2, x0, y0, z0, transformTobeMapped);

                float edge_error_term1 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                float edge_error_term2 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                float edge_error_term = edge_error_term1 / edge_error_term2;

                float s = 1 - bisquareWeight * fabs(edge_error_term);

                double error = s * edge_error_term;

                if (s > outlier_threshold)
                {
                    point_cloud_in_lidar->push_back(point_in_lidar);
                    errors.emplace_back(error);
                    jacobians.push_back(temp_jacobian);
                }
            }
        }
    }
}

void lidarMapping::cal_surf_points_jacobian(std::vector<double> &errors,
                                            edgePlaneJacobian &lidar_jacobian, PointCloudN &jacobians)
{
    std::vector<int> nearest_points_ind;
    std::vector<float> nearest_points_dist;

    point point_in_lidar, transformed_point;

    Eigen::Matrix<float, 5, 3> A0;
    A0.setZero();
    Eigen::Matrix<float, 5, 1> B0;
    B0.setConstant(-1);
    Eigen::Matrix<float, 3, 1> X0;
    X0.setZero();
    Eigen::Matrix4f trans_map = vecToMatrix(transformTobeMapped);

    trans_surf = pcl::PointCloud<point>::Ptr(new pcl::PointCloud<point>());
    trans_surf->clear();
    pcl::transformPointCloud(*cur_sur_cloud, *trans_surf, trans_map);

    for (int i = 0; i < cur_sur_cloud->size(); i++)
    {
        point_in_lidar = cur_sur_cloud->points[i];
        transformed_point = trans_surf->points[i];
        kdtreeSurfFromMap.nearestKSearch(transformed_point, 5, nearest_points_ind, nearest_points_dist);

        if (nearest_points_dist[4] < 1.0)
        {
            for (int j = 0; j < 5; j++)
            {
                A0(j, 0) = planes_map_near->points[nearest_points_ind[j]].x;
                A0(j, 1) = planes_map_near->points[nearest_points_ind[j]].y;
                A0(j, 2) = planes_map_near->points[nearest_points_ind[j]].z;
            }
            X0 = A0.colPivHouseholderQr().solve(B0);

            float t1 = X0(0, 0);
            float t2 = X0(1, 0);
            float t3 = X0(2, 0);
            float t4 = 1;

            float ps = sqrt(t1 * t1 + t2 * t2 + t3 * t3);
            t1 /= ps;
            t2 /= ps;
            t3 /= ps;
            t4 /= ps;

            bool valid_surf = true;
            for (int j = 0; j < 5; j++)
            {
                if (fabs(t1 * planes_map_near->points[nearest_points_ind[j]].x +
                         t2 * planes_map_near->points[nearest_points_ind[j]].y +
                         t3 * planes_map_near->points[nearest_points_ind[j]].z + t4) > 0.2)
                {
                    valid_surf = false;
                    break;
                }
            } 

            if (valid_surf)
            {
                float pd2 = t1 * transformed_point.x + t2 * transformed_point.y + t3 * transformed_point.z + t4;

                float s = 1 - bisquareWeight * fabs(pd2) / sqrt(sqrt(transformed_point.x * transformed_point.x + transformed_point.y * transformed_point.y + transformed_point.z * transformed_point.z));
                float x00 = point_in_lidar.x, y00 = point_in_lidar.y, z00 = point_in_lidar.z;
                pcl::PointNormal temp_jacobian = lidar_jacobian.calc_plane_jacobians(x00, y00, z00,
                                                                                     t1, t2, t3, s, transformTobeMapped);
                double error = s * pd2;

                if (s > outlier_threshold)
                {
                    point_cloud_in_lidar->push_back(point_in_lidar);
                    errors.emplace_back(error);
                    jacobians.push_back(temp_jacobian);
                }
            }
        }
    }
}

void lidarMapping::optimizeTransformTobeMapped()
{
    std::cout << "edge size: " << edges_map_near->size() << std::endl;
    std::cout << "plane size: " << planes_map_near->size() << std::endl;
    if (edges_map_near->size() <= 10 || planes_map_near->size() <= 100)
        return;
    edgePlaneJacobian lidar_jacobian;
    kdtreeCornerFromMap.setInputCloud(edges_map_near);
    kdtreeSurfFromMap.setInputCloud(planes_map_near);

    for (size_t iterCount = 0; iterCount < 10; iterCount++)
    {
        std::vector<double> errors;
        PointCloudN jacobians;
        point_cloud_in_lidar->clear();
        cal_edge_points_jacobian(errors, lidar_jacobian, jacobians);
        cal_surf_points_jacobian(errors, lidar_jacobian, jacobians);
        int valid_point_num = point_cloud_in_lidar->size();
        if (valid_point_num < 50)
        {
            continue;
        }

        Eigen::MatrixXf J(valid_point_num, 6);
        Eigen::MatrixXf JtJ;
        Eigen::VectorXf residuals(valid_point_num);
        Eigen::Matrix<float, 6, 1> curr_delta;
        for (int i = 0; i < valid_point_num; i++)
        {

            J(i, 0) = jacobians[i].normal_x;
            J(i, 1) = jacobians[i].normal_y;
            J(i, 2) = jacobians[i].normal_z;
            J(i, 3) = jacobians[i].x;
            J(i, 4) = jacobians[i].y;
            J(i, 5) = jacobians[i].z;
            residuals(i) = errors[i];
        }
        JtJ = J.transpose() * J;
        curr_delta = JtJ.colPivHouseholderQr().solve(J.transpose() * residuals);
        transformTobeMapped[0] -= curr_delta(0, 0);
        transformTobeMapped[1] -= curr_delta(1, 0);
        transformTobeMapped[2] -= curr_delta(2, 0);
        transformTobeMapped[3] -= curr_delta(3, 0);
        transformTobeMapped[4] -= curr_delta(4, 0);
        transformTobeMapped[5] -= curr_delta(5, 0);

        float deltaR = sqrt(pow(curr_delta(0, 0) * M_PI / 180, 2) +
                            pow(curr_delta(1, 0) * M_PI / 180, 2) +
                            pow(curr_delta(2, 0) * M_PI / 180, 2));
        float deltaT = sqrt(pow(curr_delta(3, 0) * 100, 2) +
                            pow(curr_delta(4, 0) * 100, 2) +
                            pow(curr_delta(5, 0) * 100, 2));
        std::cout << "deltaR: " << deltaR << std::endl;                    

        if (deltaR < 0.01 && deltaT < 0.05)
            break;
    }

    transformUpdate();
}

void lidarMapping::transformUpdate()
{
    for (int i = 0; i < 6; i++)
    {
        transformBefMapped[i] = transformSum[i];
        transformAftMapped[i] = transformTobeMapped[i];
    }
}

void lidarMapping::process(const sensor_msgs::PointCloud2ConstPtr &receive_edge_cloud,
                           const sensor_msgs::PointCloud2ConstPtr &receive_surf_points,
                           const sensor_msgs::PointCloud2ConstPtr &receive_all_points,
                           const nav_msgs::Odometry::ConstPtr &input_odometry)
{
    clock_t filter = clock();
    std::cout << "receive!" << std::endl;
    receive_messages(receive_edge_cloud, receive_surf_points, receive_all_points, input_odometry);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map"; // usr world";
    aftMappedTrans.frame_id_ = "map"; // usr world";
    aftMappedTrans.child_frame_id_ = "/lidar_mapping";
    pose_stamped.header.frame_id = "map"; // usr world";
    path.header.frame_id = "map"; // usr world";

    point point_in_lidar, transformed_point;

    transformAssociateToMap();

    pcl::PointXYZI pointSel;
    Eigen::Matrix4f trans_map = vecToMatrix(transformTobeMapped);
    pcl::transformPointCloud(*edge_point_cloud, *cur_edge_cloud_tmp, trans_map);
    pcl::transformPointCloud(*surf_point_cloud, *cur_surf_cloud_tmp, trans_map);
    update_neighbor(cur_edge_cloud_tmp, cur_surf_cloud_tmp);

    // prepare valid map corner and surface cloud for pose optimization
    Eigen::Matrix4f trans_mapped = trans_map.inverse();
    pcl::transformPointCloud(*cur_edge_cloud_tmp, *cur_edge_cloud_tmp, trans_mapped);
    pcl::transformPointCloud(*cur_surf_cloud_tmp, *cur_surf_cloud_tmp, trans_mapped);

    cur_edge_cloud->clear();
    corner_points_filter.setInputCloud(cur_edge_cloud_tmp);
    corner_points_filter.filter(*cur_edge_cloud);
    int filter_edge_num = cur_edge_cloud->size();

    cur_sur_cloud->clear();
    surf_points_filter.setInputCloud(cur_surf_cloud_tmp);
    surf_points_filter.filter(*cur_sur_cloud);
    int filter_surf_num = cur_sur_cloud->size();

    cur_edge_cloud_tmp->clear();
    cur_surf_cloud_tmp->clear();
    if (lidar_id > 20)//20000
    optimizeTransformTobeMapped();
    else {
        transformUpdate();
    }

    trans_map = vecToMatrix(transformTobeMapped);
    pcl::transformPointCloud(*cur_edge_cloud, *cur_edge_cloud, trans_map);
    pcl::transformPointCloud(*cur_sur_cloud, *cur_sur_cloud, trans_map);
    update_cells(cur_edge_cloud, cur_sur_cloud);
    *publish_cloud += (*cur_edge_cloud) + (*cur_sur_cloud);
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*publish_cloud, laserCloudFullRes3);
    laserCloudFullRes3.header.frame_id = "map"; // usr world";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]);
    odomAftMapped.header.stamp = receive_all_points->header.stamp;
    odomAftMapped.pose.pose.orientation.x = geoQuat.y;
    odomAftMapped.pose.pose.orientation.y = geoQuat.z;
    odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x = transformAftMapped[3];
    odomAftMapped.pose.pose.position.y = transformAftMapped[4];
    odomAftMapped.pose.pose.position.z = transformAftMapped[5];
    pubOdomAftMapped.publish(odomAftMapped);
    pose_stamped.header.stamp = odomAftMapped.header.stamp;
    pose_stamped.pose = odomAftMapped.pose.pose;
    path.header.stamp = odomAftMapped.header.stamp;
    path.poses.push_back(pose_stamped);
    pubLidarPath.publish(path);
    myfile << std::setprecision(20) << lidar_id - 1 << " " << transformAftMapped[3] << " " << transformAftMapped[4] << " " << transformAftMapped[5] << " " << geoQuat.y << " " << geoQuat.z << " " << geoQuat.x << " " << geoQuat.w << std::endl;

    Eigen::Matrix3f rot_b2o = trans_map.matrix().topLeftCorner<3, 3>(); // body to origin
    Eigen::Quaternionf q_b(rot_b2o);

    tf::Quaternion q_tf(q_b.x(), q_b.y(), q_b.z(), q_b.w());
    ros::Time time_tf;
    time_tf.fromSec(odom_time);
    aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
    aftMappedTrans.setRotation(q_tf);
    tfBroadcaster.sendTransform(aftMappedTrans);
    clock_t filterTime = clock() - filter;
    unsigned msElapsed2 = filterTime / (CLOCKS_PER_SEC / 1000);
    std::cout << "process time****: " << msElapsed2 << " ms\n";
}

} // namespace lidar_mapping
} // namespace beyond

int main(int argc, char **argv)
{
    myfile.open("/home/wz/Desktop/odom.txt");
    ros::init(argc, argv, "lidarMapping");
    ros::NodeHandle nh;
    beyond::lidar_mapping::lidarMapping *lidar_mapping =
        new beyond::lidar_mapping::lidarMapping(nh);
    lidar_mapping->set_pub_surrond_cloud(nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100));
    lidar_mapping->set_pub_full_cloud(nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100));
    lidar_mapping->set_pub_odom(nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5));
    lidar_mapping->set_pub_path(nh.advertise<nav_msgs::Path>("/lidarPath", 1000));
    

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_corner(nh, "/laser_cloud_corner_last", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_surf(nh, "/laser_cloud_surf_last", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_full_cloud(nh, "/velodyne_cloud_3", 100);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, "/laser_odom_to_init", 100);
    TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, nav_msgs::Odometry>
        sync(sub_corner, sub_surf, sub_full_cloud, sub_odom, 100);
    sync.registerCallback(boost::bind(&beyond::lidar_mapping::lidarMapping::process, lidar_mapping, _1, _2, _3, _4));

    ROS_INFO("\033[1;32m----> LiDAR Mapping Started.\033[0m");

    ros::spin();
    return 0;
}
