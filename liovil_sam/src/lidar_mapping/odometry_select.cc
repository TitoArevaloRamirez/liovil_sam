#include "lidar_mapping/odometry_select.h"
namespace beyond
{
namespace lidar_mapping
{
odometrySelect::odometrySelect(ros::NodeHandle &nh)
{
    lidar_odom_cloud = lidarCloud::Ptr(new lidarCloud);
    lidar_odom_edge = lidarCloud::Ptr(new lidarCloud);
    lidar_odom_surf = lidarCloud::Ptr(new lidarCloud);
    lidar_optimized_cloud = lidarCloud::Ptr(new lidarCloud);
    lidar_optimized_edge = lidarCloud::Ptr(new lidarCloud);
    lidar_optimized_surf = lidarCloud::Ptr(new lidarCloud);
    lidar_smooth_cloud = lidarCloud::Ptr(new lidarCloud);
    lidar_smooth_edge = lidarCloud::Ptr(new lidarCloud);
    lidar_smooth_surf = lidarCloud::Ptr(new lidarCloud);
    lidar_id = 0;
    prev_lidar_odom = Eigen::Matrix4f::Identity();
    cur_lidar_odom = Eigen::Matrix4f::Identity();
    prev_vio_optimized = Eigen::Matrix4f::Identity();
    cur_vio_optimized = Eigen::Matrix4f::Identity();
    prev_vio_smooth = Eigen::Matrix4f::Identity();
    cur_vio_smooth = Eigen::Matrix4f::Identity();
    prev_prev_full_cloud = lidarCloud::Ptr(new lidarCloud);
    prev_full_cloud = lidarCloud::Ptr(new lidarCloud);
    prev_prev_edge = lidarCloud::Ptr(new lidarCloud);
    prev_edge = lidarCloud::Ptr(new lidarCloud);
    prev_prev_surf = lidarCloud::Ptr(new lidarCloud);
    prev_surf = lidarCloud::Ptr(new lidarCloud);
    dewarped_edge_points = lidarCloud::Ptr(new lidarCloud);
    dewarped_surf_points = lidarCloud::Ptr(new lidarCloud);
    ground_cloud = lidarCloud::Ptr(new lidarCloud);
}

void odometrySelect::set_pub_all_cloud(ros::Publisher pub_cloud)
{
    pubAllCloud = pub_cloud;
}
void odometrySelect::set_pub_edge_cloud(ros::Publisher pub_cloud)
{
    pubEdgeCloud = pub_cloud;
}
void odometrySelect::set_pub_surf_cloud(ros::Publisher pub_cloud)
{
    pubSurfCloud = pub_cloud;
}
void odometrySelect::set_pub_ground_cloud(ros::Publisher pub_cloud)
{
    pubGroundCloud = pub_cloud;
}
void odometrySelect::set_pub_odometry(ros::Publisher pub_odom)
{
    pubOdometry = pub_odom;
}
void odometrySelect::set_pub_odometry1(ros::Publisher pub_odom)
{
    pubOdometry1 = pub_odom;
}
void odometrySelect::set_pub_loop_edge(ros::Publisher pub_cloud)
{
    pubLoopEdge = pub_cloud;
}

void odometrySelect::set_pub_loop_surf(ros::Publisher pub_cloud)
{
    pubLoopSurf = pub_cloud;
}
void odometrySelect::full_cloud_lidar_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    //update lidar id, for the first initial frames, we use lidar odometry source
    lidar_time = laserCloudMsg->header.stamp.toSec();
    lidar_odom_cloud->clear();
    pcl::fromROSMsg(*laserCloudMsg, *lidar_odom_cloud);
    full_point_msg.AddEntry(laserCloudMsg->header.stamp, lidar_odom_cloud);
}

void odometrySelect::odom_edge_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    lidar_edge_time = laserCloudMsg->header.stamp.toSec();
    lidar_odom_edge->clear();
    pcl::fromROSMsg(*laserCloudMsg, *lidar_odom_edge);
    edge_point_msg.AddEntry(laserCloudMsg->header.stamp, lidar_odom_edge);
}

void odometrySelect::odom_plane_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    lidar_surf_time = laserCloudMsg->header.stamp.toSec();
    lidar_odom_surf->clear();
    pcl::fromROSMsg(*laserCloudMsg, *lidar_odom_surf);
    surf_point_msg.AddEntry(laserCloudMsg->header.stamp, lidar_odom_surf);
}

void odometrySelect::ground_handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    ground_cloud->clear();
    pcl::fromROSMsg(*laserCloudMsg, *ground_cloud);
    ground_cloud_msg.AddEntry(laserCloudMsg->header.stamp, ground_cloud);
}

void odometrySelect::lidar_odom_handler(const nav_msgs::Odometry::ConstPtr &odom)
{
    lidar_odom_time = odom->header.stamp.toSec();
    cur_ros_time = odom->header.stamp;
    lidar_odom_receive = true;
    Eigen::Quaternionf q;
    Eigen::Matrix3f rot; 
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q.w() = odom->pose.pose.orientation.w;
    rot = q.toRotationMatrix(); //Eigen::Matrix<float, 3, 3>::Identity();
    lidar_odom.matrix().topLeftCorner<3, 3>() = rot;
    lidar_odom(0, 3) = odom->pose.pose.position.x;
    lidar_odom(1, 3) = odom->pose.pose.position.y;
    lidar_odom(2, 3) = odom->pose.pose.position.z; 
    lidar_odom(3, 3) = 1; 
    lidar_odoMsg.AddEntry(odom->header.stamp, lidar_odom);
    new_point_receive = true;
    if (lidar_id < 5) {
        prev_lidar_odom = cur_lidar_odom;
        cur_lidar_odom = lidar_odom;
        Eigen::Matrix3f mat = cur_lidar_odom.matrix().topLeftCorner<3, 3>();
        Eigen::Quaternionf q1(mat);
        lidarOdom.header.stamp = odom->header.stamp;
        lidarOdom.header.frame_id = "/vehicle_init";
        lidarOdom.pose.pose.orientation.x = q1.x();
        lidarOdom.pose.pose.orientation.y = q1.y();
        lidarOdom.pose.pose.orientation.z = q1.z();
        lidarOdom.pose.pose.orientation.w = q1.w();
        lidarOdom.pose.pose.position.x = cur_lidar_odom(0, 3);
        lidarOdom.pose.pose.position.y = cur_lidar_odom(1, 3);
        lidarOdom.pose.pose.position.z = cur_lidar_odom(2, 3);

        Eigen::Matrix3f mat1 = prev_lidar_odom.matrix().topLeftCorner<3, 3>();
        Eigen::Quaternionf q2(mat1);
        prevLidarOdom.header.stamp = odom->header.stamp;
        prevLidarOdom.header.frame_id = "/vehicle_init";
        prevLidarOdom.pose.pose.orientation.x = q2.x();
        prevLidarOdom.pose.pose.orientation.y = q2.y();
        prevLidarOdom.pose.pose.orientation.z = q2.z();
        prevLidarOdom.pose.pose.orientation.w = q2.w();
        prevLidarOdom.pose.pose.position.x = prev_lidar_odom(0, 3);
        prevLidarOdom.pose.pose.position.y = prev_lidar_odom(1, 3);
        prevLidarOdom.pose.pose.position.z = prev_lidar_odom(2, 3);
    }
}

void odometrySelect::optimized_odom_handler(const nav_msgs::Odometry::ConstPtr &odom)
{
    double optimize_cur_time = odom->header.stamp.toSec();
    double dt = optimize_cur_time - optimize_prev_time;
    optimize_prev_time = optimize_cur_time;
    Eigen::Quaternionf q;
    Eigen::Matrix3f rot; 
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q.w() = odom->pose.pose.orientation.w;
    rot = q.toRotationMatrix(); //Eigen::Matrix<float, 3, 3>::Identity();
    lidar_optimized_odom.matrix().topLeftCorner<3, 3>() = rot;
    lidar_optimized_odom(0, 3) = odom->pose.pose.position.x;
    lidar_optimized_odom(1, 3) = odom->pose.pose.position.y;
    lidar_optimized_odom(2, 3) = odom->pose.pose.position.z; 
    lidar_optimized_odom(3, 3) = 1; 
    vio_optimize_odoMsg.AddEntry(odom->header.stamp, lidar_optimized_odom);
    prev_x = cur_x;
    prev_y = cur_y;
    prev_z = cur_z;
    cur_x = lidar_optimized_odom(0, 3);
    cur_y = lidar_optimized_odom(1, 3);
    cur_z = lidar_optimized_odom(2, 3);
    double cur_velocity = sqrt(pow((cur_x - prev_x) / dt, 2) + pow((cur_y - prev_y) / dt, 2) +
                                           pow((cur_z - prev_z) / dt, 2));
    optimized_velocity.AddEntry(odom->header.stamp, cur_velocity);
}


void odometrySelect::smooth_odom_handler(const nav_msgs::Odometry::ConstPtr &odom)
{
    Eigen::Quaternionf q;
    Eigen::Matrix3f rot; 
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;
    q.w() = odom->pose.pose.orientation.w;
    rot = q.toRotationMatrix(); //Eigen::Matrix<float, 3, 3>::Identity();
    lidar_smooth_odom.matrix().topLeftCorner<3, 3>() = rot;
    lidar_smooth_odom(0, 3) = odom->pose.pose.position.x;
    lidar_smooth_odom(1, 3) = odom->pose.pose.position.y;
    lidar_smooth_odom(2, 3) = odom->pose.pose.position.z; 
    lidar_smooth_odom(3, 3) = odom->twist.twist.angular.x; // store the feature number
    vio_smooth_odoMsg.AddEntry(odom->header.stamp, lidar_smooth_odom);
    v_x = odom->twist.twist.linear.x;
    v_y = odom->twist.twist.linear.y;
    v_z = odom->twist.twist.linear.z;
    double predict_velocity = sqrt(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));
    smooth_velocity.AddEntry(odom->header.stamp, predict_velocity);
}

} // namespace lidar_mapping
} // namespace beyond

void transform_to_end(point pi, point &po, Eigen::Matrix4f dtrans)
{
  po.x = dtrans(0, 0) * pi.x + dtrans(0, 1) * pi.y + dtrans(0, 2) * pi.z + dtrans(0, 3);
  po.y = dtrans(1, 0) * pi.x + dtrans(1, 1) * pi.y + dtrans(1, 2) * pi.z + dtrans(1, 3);
  po.z = dtrans(2, 0) * pi.x + dtrans(2, 1) * pi.y + dtrans(2, 2) * pi.z + dtrans(2, 3);
  po.intensity = pi.intensity;
}
void get_dewarped_points(lidarCloud::Ptr point_cloud, double cur_time,
                                              Eigen::Matrix4f cur_close_odometry, lidarCloud::Ptr dewarped_points)
{
  int num = 0;
  int dewarp_succeed_num = 0;
  for (unsigned int i = 0; i < point_cloud->size(); i++)
  {
    point cur_point;
    cur_point = point_cloud->points[i];
    bool cur_odometry_success = false;
    double point_retrieve_time = 0;
    // the lidar points belong to the previous preivous sacn, so we should eliminate the offset(two scan time)
    double point_time = cur_time - 0.1 + (cur_point.intensity - int(cur_point.intensity));
    ros::Time cur_point_time = (ros::Time)point_time;
    // retrieve the odometry information
    Eigen::Matrix4f cur_point_odometry;
    if (cur_status == 0) {
        cur_point_odometry = vio_optimize_odoMsg.GetClosestEntry(cur_point_time, cur_odometry_success, point_retrieve_time);
    }
    else if (cur_status == 1) {
        cur_point_odometry = vio_smooth_odoMsg.GetClosestEntry(cur_point_time, cur_odometry_success, point_retrieve_time);
    }
    // calculate transformation from this point to the end of scan
    cur_point_odometry(3,3) = 1;
    cur_point_odometry = cur_point_odometry;
    //cur_close_odometry(3,3) = 1;
    Eigen::Matrix4f dtrans = cur_close_odometry.inverse().eval() * cur_point_odometry;
    // calculate time difference
    double df = point_time - point_retrieve_time;
    // transform this point to the end of scan
    point transform_point;
    transform_to_end(cur_point, transform_point, dtrans);
    if (cur_odometry_success && fabs(df) < 0.002)
    {
      dewarped_points->push_back(transform_point);
      dewarp_succeed_num++;
    }
    else {
      num++;
    }
  }
  std::cout << "dewarp fail num: " << num << std::endl;
  std::cout << "dewarp_succeed_num: " << dewarp_succeed_num << std::endl;
}
static std::ofstream out_gt("/home/cong/velocity.txt");
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometrySelect");
    ros::NodeHandle nh;
    beyond::lidar_mapping::odometrySelect *odometry_select = new beyond::lidar_mapping::odometrySelect(nh);
    odometry_select->set_pub_all_cloud(nh.advertise<sensor_msgs::PointCloud2>("/mapping_all_points", 100));
    odometry_select->set_pub_edge_cloud(nh.advertise<sensor_msgs::PointCloud2>("/mapping_edge_points", 100));
    odometry_select->set_pub_surf_cloud(nh.advertise<sensor_msgs::PointCloud2>("/mapping_surf_points", 100));
    odometry_select->set_pub_odometry(nh.advertise<nav_msgs::Odometry>("/mapping_odom", 100));
    odometry_select->set_pub_odometry1(nh.advertise<nav_msgs::Odometry>("/mapping_odom_prev", 100));
    odometry_select->set_pub_ground_cloud(nh.advertise<sensor_msgs::PointCloud2>("/mapping_ground_points", 100));
    odometry_select->set_pub_loop_edge(nh.advertise<sensor_msgs::PointCloud2>("/loop_edge", 100));
    odometry_select->set_pub_loop_surf(nh.advertise<sensor_msgs::PointCloud2>("/loop_ground", 100));
    ros::Rate rate(100);

    //subscribe undewarped point cloud
    ros::Subscriber sub_full_cloud = nh.subscribe("/velodyne_cloud_2",
                                                  10000, &beyond::lidar_mapping::odometrySelect::full_cloud_lidar_handler, odometry_select);
    ros::Subscriber sub_corner = nh.subscribe("/laser_cloud_less_sharp",
                                              10000, &beyond::lidar_mapping::odometrySelect::odom_edge_handler, odometry_select);
    ros::Subscriber sub_surf = nh.subscribe("/laser_cloud_less_flat",
                                            10000, &beyond::lidar_mapping::odometrySelect::odom_plane_handler, odometry_select);
    //subscribe odometry information
    ros::Subscriber sub_odom = nh.subscribe("/laser_odom_to_init",
                                            1000, &beyond::lidar_mapping::odometrySelect::lidar_odom_handler, odometry_select);
    ros::Subscriber sub_optimized_odom = nh.subscribe("/vio/odom_imu_optim",
                                                     1000, &beyond::lidar_mapping::odometrySelect::optimized_odom_handler, odometry_select);
    ros::Subscriber sub_smooth_odom = nh.subscribe("/vio/odom_imu_smooth",
                                                     1000, &beyond::lidar_mapping::odometrySelect::smooth_odom_handler, odometry_select);
    ros::Subscriber sub_ground_cloud = nh.subscribe("/ground_cloud", 1000,  &beyond::lidar_mapping::odometrySelect::ground_handler, odometry_select);


    while (ros::ok())
    {
        // for the initial frames, just use lidar odometry
        if (lidar_id < 5 && lidar_odom_receive) {
            lidar_odom_receive = false;
            bool full_cloud_retrieve = false;
            bool edge_cloud_retrieve = false;
            bool surf_cloud_retrieve = false;
            bool ground_cloud_retrieve = false;
            double full_cloud_retrieve_time = 0;
            double edge_cloud_retrieve_time = 0;
            double surf_cloud_retrieve_time = 0;
            double ground_cloud_retrieve_time = 0;
            pcl::PointCloud<point>::Ptr closest_full_cloud = full_point_msg.GetClosestEntry(
                cur_ros_time, full_cloud_retrieve, full_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_edge_cloud = edge_point_msg.GetClosestEntry(
                cur_ros_time, edge_cloud_retrieve, edge_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_surf_cloud = surf_point_msg.GetClosestEntry(
                cur_ros_time, surf_cloud_retrieve, surf_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_ground_cloud = ground_cloud_msg.GetClosestEntry(
                cur_ros_time, ground_cloud_retrieve, ground_cloud_retrieve_time);
            cout << "diff cloud time: " << cur_ros_time.toSec() - full_cloud_retrieve_time << std::endl;
            cout << "diff edge time: " << cur_ros_time.toSec() - edge_cloud_retrieve_time << std::endl;
            cout << "diff surf time: " << cur_ros_time.toSec() - surf_cloud_retrieve_time << std::endl;
            cout << "lidar id: " << lidar_id << std::endl;
            lidar_id++;
            pcl::toROSMsg(*closest_full_cloud, laserCloudOutMsg);
            pcl::toROSMsg(*closest_edge_cloud, laserCloudOutMsg1);
            pcl::toROSMsg(*closest_surf_cloud, laserCloudOutMsg2);
            pcl::toROSMsg(*closest_ground_cloud, laserCloudOutMsg3);
            laserCloudOutMsg.header.stamp = cur_ros_time;
            laserCloudOutMsg1.header.stamp = cur_ros_time;
            laserCloudOutMsg2.header.stamp = cur_ros_time;
            laserCloudOutMsg3.header.stamp = cur_ros_time;
            lidarOdom.header.stamp = cur_ros_time;
            prevLidarOdom.header.stamp = cur_ros_time;
            pubAllCloud.publish(laserCloudOutMsg);
            pubEdgeCloud.publish(laserCloudOutMsg1);
            pubSurfCloud.publish(laserCloudOutMsg2);
            pubGroundCloud.publish(laserCloudOutMsg3);
            pubOdometry.publish(lidarOdom);
            pubOdometry1.publish(prevLidarOdom);
            pubLoopEdge.publish(laserCloudOutMsg1);
            pubLoopSurf.publish(laserCloudOutMsg2);
        }
        else if (new_point_receive && lidar_id >= 5){ 
            dewarped_edge_points->clear();
            dewarped_surf_points->clear();
            new_point_receive = false;
            std::cout << "lidar id: " << lidar_id << std::endl;
            double cur_time = lidar_odom_time;
            ros::Time retrieve_ros_time = (ros::Time)cur_time;
            bool lidar_odom_retrieve_res = false;
            bool vio_optimized_retrieve_res = false;
            bool vio_smooth_retrieve_res = false;
            double lidar_retrieve_time = 0;
            double vio_optimized_retrieve_time = 0;
            double vio_smooth_retrieve_time = 0;
            Eigen::Matrix4f closest_lidar_odom = lidar_odoMsg.GetClosestEntry(retrieve_ros_time, lidar_odom_retrieve_res, lidar_retrieve_time);
            double time_diff = cur_time - lidar_retrieve_time;

            ros::Time delete_time = (ros::Time)cur_time;
            Eigen::Matrix4f closest_vio_optimizd_odom = vio_optimize_odoMsg.GetClosestEntry(retrieve_ros_time, vio_optimized_retrieve_res, vio_optimized_retrieve_time);
            double time_diff1 = cur_time - vio_optimized_retrieve_time;

            Eigen::Matrix4f closest_vio_smooth_odom = vio_smooth_odoMsg.GetClosestEntry(retrieve_ros_time, vio_smooth_retrieve_res, vio_smooth_retrieve_time);
            double time_diff2 = cur_time - vio_smooth_retrieve_time;

            // change status based on different situations
            prev_lidar_odom = cur_lidar_odom;
            cur_lidar_odom = closest_lidar_odom;
            prev_vio_optimized = cur_vio_optimized;
            cur_vio_optimized = closest_vio_optimizd_odom;
            prev_vio_smooth = cur_vio_smooth;
            cur_vio_smooth = closest_vio_smooth_odom;

             if (lidar_id == 5) {
                lidar_id++;
                continue;
            }
        

            std::cout << "time_diff: " << time_diff << std::endl;
            std::cout << "time_diff1: " << time_diff1 << std::endl;
            std::cout << "time diff2: " << time_diff2 << std::endl;
            if (fabs(time_diff) > 0.02) {
                lidar_odom_retrieve_res = false;
            }
            if (fabs(time_diff1) > 0.02 || fabs(time_diff2) > 0.02) {
                vio_optimized_retrieve_res = false;
                vio_smooth_retrieve_res = false;
            }
            

            bool full_cloud_retrieve = false;
            bool edge_cloud_retrieve = false;
            bool surf_cloud_retrieve = false;
            bool ground_cloud_retrieve = false;
            double full_cloud_retrieve_time = 0;
            double edge_cloud_retrieve_time = 0;
            double surf_cloud_retrieve_time = 0;
            double ground_cloud_retrieve_time = 0;
            pcl::PointCloud<point>::Ptr closest_full_cloud = full_point_msg.GetClosestEntry(
                retrieve_ros_time, full_cloud_retrieve, full_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_edge_cloud = edge_point_msg.GetClosestEntry(
                retrieve_ros_time, edge_cloud_retrieve, edge_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_surf_cloud = surf_point_msg.GetClosestEntry(
                retrieve_ros_time, surf_cloud_retrieve, surf_cloud_retrieve_time);
            pcl::PointCloud<point>::Ptr closest_ground_cloud = ground_cloud_msg.GetClosestEntry(
                retrieve_ros_time, ground_cloud_retrieve, ground_cloud_retrieve_time);

            cout.precision(17);

            cout << "diff cloud time: " << retrieve_ros_time.toSec() - full_cloud_retrieve_time << std::endl;
            cout << "diff edge time: " << retrieve_ros_time.toSec() - edge_cloud_retrieve_time << std::endl;
            cout << "diff surf time: " << retrieve_ros_time.toSec() - surf_cloud_retrieve_time << std::endl;
            cout << "diff ground time: " << retrieve_ros_time.toSec() - ground_cloud_retrieve_time << std::endl;

            cur_feature_num = 0;

            double cur_velocity = 0;
            double predict_velocity = 0;

            if (vio_smooth_retrieve_res) {
                cur_feature_num = closest_vio_smooth_odom(3,3);
                retrieve_count++;
            }
            else{
                prev_imu_receive = false;
                retrieve_count = 0;
            }
            //status: 0, use optimized 1, use smooth 2, use lidar
            // feature check
            bool feature_valid = true;
            if (cur_feature_num < featureThreshold) {
                feature_valid = false;
            }
            // velocity check
            bool velocity_valid = true;
            if (!vio_optimized_retrieve_res || !vio_smooth_retrieve_res) {
                velocity_valid = false;
            }
            else{
                bool retrieved = false, retrieved1 = false;
                double retreived_time = 0.0, retreived_time1 = 0.0;
                cur_velocity = optimized_velocity.GetClosestEntry(retrieve_ros_time, retrieved, retreived_time);
                predict_velocity = smooth_velocity.GetClosestEntry(retrieve_ros_time, retrieved1, retreived_time1);
                cout << "current velocity: " << cur_velocity << endl;
                cout << "predict velocity: " << predict_velocity << endl;
                if (cur_velocity < 0.8 * predict_velocity || cur_velocity > 1.2 * predict_velocity)
                {
                    velocity_valid = false;
                }
            }
            if (!lidar_odom_retrieve_res) {
                // choose from smooth and optimized vio odometry,
                // the first situation, optimized to smooth
                if (cur_status == 0) {
                    if (!feature_valid || !velocity_valid) {
                        cur_status = 1;
                        featureConsequetive = 0;
                    }
                }
                // the second situation, smooth to optimized
                else if (cur_status == 1) {
                    if (!feature_valid || !velocity_valid) {
                        featureConsequetive = 0;
                    }
                    else {
                        featureConsequetive++;
                        if (featureConsequetive > 10) {
                            cur_status = 0;
                        }
                    }
                }
            }
            else{
                if (!vio_optimized_retrieve_res || !vio_smooth_retrieve_res) {
                    // publish lidar topic
                    cur_status = 2;
                }
                else {
                    if (cur_status == 0) {
                        if (!feature_valid || !velocity_valid) {
                            cur_status = 1;
                            featureConsequetive = 0;
                        }
                    }
                    else if (cur_status == 1) {
                        if (!velocity_valid) {
                            cur_status = 2;
                            featureConsequetive = 0;
                        }
                        else {
                            if (!feature_valid) {
                                featureConsequetive = 0;
                            }
                            else {
                                featureConsequetive++;
                                if (featureConsequetive > 10) {
                                    cur_status = 0;
                                }
                            }
                        }
                    }
                    else{
                        if (!velocity_valid) {
                            cur_status = 2;
                            featureConsequetive = 0;
                        }
                        else {
                            if (!feature_valid) {
                                featureConsequetive = 0;
                            }
                            else {
                                featureConsequetive++;
                                //if (featureConsequetive > 3)
                                cur_status = 1;
                            }
                        }
                    }
                }
            }
            std::cout << "current status: " << cur_status << std::endl; 
            prev_vio_smooth(3,3) = 1;
            //prev_vio_smooth = prev_vio_smooth * imu2lidar;
            cur_vio_smooth(3,3) = 1;
            cur_vio_smooth = cur_vio_smooth;
            //prev_vio_optimized(3, 3) = 1;
            //prev_vio_optimized = prev_vio_optimized * imu2lidar;
            cur_vio_optimized(3, 3) = 1;
            cur_vio_optimized = cur_vio_optimized;
            cout << prev_vio_smooth.inverse() * cur_vio_smooth << std::endl;


            out_gt << cur_status << " " << cur_feature_num << " " << cur_velocity << " " << predict_velocity << std::endl;


            //std::cout << "choose status" << std::endl;

            // if (lidar_id < 70) {
            //     cur_status = 2;
            // }

            if (lidar_id < 10) {
                cur_status = 2;
            }

            if (vio_smooth_retrieve_res && retrieve_count > 2) {
                cur_status = 1;
            }
            else{
                cur_status = 2;
            }
       
            if (cur_status == 0) {
                Eigen::Matrix4f rel_vio_optimizd_odom = prev_vio_optimized.inverse() * cur_vio_optimized;
                Eigen::Matrix3f mat = cur_vio_optimized.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q(mat);
                lidarOdom.header.stamp = delete_time;
                lidarOdom.header.frame_id = "/vehicle_init";
                lidarOdom.pose.pose.orientation.x = q.x();
                lidarOdom.pose.pose.orientation.y = q.y();
                lidarOdom.pose.pose.orientation.z = q.z();
                lidarOdom.pose.pose.orientation.w = q.w();
                lidarOdom.pose.pose.position.x = cur_vio_optimized(0, 3);
                lidarOdom.pose.pose.position.y = cur_vio_optimized(1, 3);
                lidarOdom.pose.pose.position.z = cur_vio_optimized(2, 3);
                lidarOdom.twist.twist.linear.x = lidar_id;

                Eigen::Matrix3f mat1 = prev_vio_optimized.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q1(mat1);
                prevLidarOdom.header.stamp = delete_time;
                prevLidarOdom.header.frame_id = "/vehicle_init";
                prevLidarOdom.pose.pose.orientation.x = q1.x();
                prevLidarOdom.pose.pose.orientation.y = q1.y();
                prevLidarOdom.pose.pose.orientation.z = q1.z();
                prevLidarOdom.pose.pose.orientation.w = q1.w();
                prevLidarOdom.pose.pose.position.x = prev_vio_optimized(0, 3);
                prevLidarOdom.pose.pose.position.y = prev_vio_optimized(1, 3);
                prevLidarOdom.pose.pose.position.z = prev_vio_optimized(2, 3);

                get_dewarped_points(closest_edge_cloud, cur_time, cur_vio_optimized, dewarped_edge_points);
                get_dewarped_points(closest_surf_cloud, cur_time, cur_vio_optimized, dewarped_surf_points);
                get_dewarped_points(closest_ground_cloud, cur_time, cur_vio_optimized, dewarped_ground_points);
                pcl::toROSMsg(*closest_full_cloud, laserCloudOutMsg);
                pcl::toROSMsg(*dewarped_edge_points, laserCloudOutMsg1);
                pcl::toROSMsg(*dewarped_surf_points, laserCloudOutMsg2);
                pcl::toROSMsg(*dewarped_ground_points, laserCloudOutMsg3);
                laserCloudOutMsg.header.stamp = delete_time;
                laserCloudOutMsg1.header.stamp = delete_time;
                laserCloudOutMsg2.header.stamp = delete_time;
                laserCloudOutMsg3.header.stamp = delete_time;
                pubAllCloud.publish(laserCloudOutMsg);
                pubEdgeCloud.publish(laserCloudOutMsg1);
                pubSurfCloud.publish(laserCloudOutMsg2);
                pubGroundCloud.publish(laserCloudOutMsg3);
                pubOdometry.publish(lidarOdom);
                pubOdometry1.publish(prevLidarOdom);
                
            }
            if (cur_status == 1) {
                std::cout << "prev_vio_smooth: " << prev_vio_smooth << std::endl;
                std::cout << "cur_vio_smooth: " << cur_vio_smooth << std::endl;
                Eigen::Matrix4f rel_vio_smooth_odom = prev_vio_smooth.inverse() * cur_vio_smooth;
                Eigen::Matrix3f mat = cur_vio_smooth.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q(mat);
                lidarOdom.header.stamp = delete_time;
                lidarOdom.header.frame_id = "/vehicle_init";
                lidarOdom.pose.pose.orientation.x = q.x();
                lidarOdom.pose.pose.orientation.y = q.y();
                lidarOdom.pose.pose.orientation.z = q.z();
                lidarOdom.pose.pose.orientation.w = q.w();
                lidarOdom.pose.pose.position.x = cur_vio_smooth(0, 3);
                lidarOdom.pose.pose.position.y = cur_vio_smooth(1, 3);
                lidarOdom.pose.pose.position.z = cur_vio_smooth(2, 3);
                lidarOdom.twist.twist.linear.x = lidar_id;

                Eigen::Matrix3f mat1 = prev_vio_smooth.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q1(mat1);
                prevLidarOdom.header.stamp = delete_time;
                prevLidarOdom.header.frame_id = "/vehicle_init";
                prevLidarOdom.pose.pose.orientation.x = q1.x();
                prevLidarOdom.pose.pose.orientation.y = q1.y();
                prevLidarOdom.pose.pose.orientation.z = q1.z();
                prevLidarOdom.pose.pose.orientation.w = q1.w();
                prevLidarOdom.pose.pose.position.x = prev_vio_smooth(0, 3);
                prevLidarOdom.pose.pose.position.y = prev_vio_smooth(1, 3);
                prevLidarOdom.pose.pose.position.z = prev_vio_smooth(2, 3);
                //dewarp point cloud
                get_dewarped_points(closest_edge_cloud, cur_time, cur_vio_smooth, dewarped_edge_points);
                get_dewarped_points(closest_surf_cloud, cur_time, cur_vio_smooth, dewarped_surf_points);
                //get_dewarped_points(closest_ground_cloud, cur_time, cur_vio_smooth, dewarped_ground_points);
                pcl::toROSMsg(*closest_full_cloud, laserCloudOutMsg);
                pcl::toROSMsg(*dewarped_edge_points, laserCloudOutMsg1);
                pcl::toROSMsg(*dewarped_surf_points, laserCloudOutMsg2);
                pcl::toROSMsg(*closest_ground_cloud, laserCloudOutMsg3);
                pcl::toROSMsg(*closest_edge_cloud, laserCloudOutMsg4);
                pcl::toROSMsg(*closest_surf_cloud, laserCloudOutMsg5);
                laserCloudOutMsg.header.stamp = delete_time;
                laserCloudOutMsg1.header.stamp = delete_time;
                laserCloudOutMsg2.header.stamp = delete_time;
                laserCloudOutMsg3.header.stamp = delete_time;
                laserCloudOutMsg4.header.stamp = delete_time;
                laserCloudOutMsg5.header.stamp = delete_time;
                pubAllCloud.publish(laserCloudOutMsg);
                pubEdgeCloud.publish(laserCloudOutMsg1);
                pubSurfCloud.publish(laserCloudOutMsg2);
                pubGroundCloud.publish(laserCloudOutMsg3);
                pubOdometry.publish(lidarOdom);
                pubOdometry1.publish(prevLidarOdom);
                pubLoopEdge.publish(laserCloudOutMsg4);
                pubLoopSurf.publish(laserCloudOutMsg5);
            }
            if (cur_status == 2) {
                Eigen::Matrix4f rel_lidar_odom = prev_lidar_odom.inverse() * cur_lidar_odom;
                Eigen::Matrix3f mat = cur_lidar_odom.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q(mat);
                lidarOdom.header.stamp = delete_time;
                lidarOdom.header.frame_id = "/vehicle_init";
                lidarOdom.pose.pose.orientation.x = q.x();
                lidarOdom.pose.pose.orientation.y = q.y();
                lidarOdom.pose.pose.orientation.z = q.z();
                lidarOdom.pose.pose.orientation.w = q.w();
                lidarOdom.pose.pose.position.x = cur_lidar_odom(0, 3);
                lidarOdom.pose.pose.position.y = cur_lidar_odom(1, 3);
                lidarOdom.pose.pose.position.z = cur_lidar_odom(2, 3);
                lidarOdom.twist.twist.linear.x = lidar_id;


                Eigen::Matrix3f mat1 = prev_lidar_odom.matrix().topLeftCorner<3, 3>();
                Eigen::Quaternionf q1(mat1);
                prevLidarOdom.header.stamp = delete_time;
                prevLidarOdom.header.frame_id = "/vehicle_init";
                prevLidarOdom.pose.pose.orientation.x = q1.x();
                prevLidarOdom.pose.pose.orientation.y = q1.y();
                prevLidarOdom.pose.pose.orientation.z = q1.z();
                prevLidarOdom.pose.pose.orientation.w = q1.w();
                prevLidarOdom.pose.pose.position.x = prev_lidar_odom(0, 3);
                prevLidarOdom.pose.pose.position.y = prev_lidar_odom(1, 3);
                prevLidarOdom.pose.pose.position.z = prev_lidar_odom(2, 3);
                pcl::toROSMsg(*closest_full_cloud, laserCloudOutMsg);
                pcl::toROSMsg(*closest_edge_cloud, laserCloudOutMsg1);
                pcl::toROSMsg(*closest_surf_cloud, laserCloudOutMsg2);
                pcl::toROSMsg(*closest_ground_cloud, laserCloudOutMsg3);
                laserCloudOutMsg.header.stamp = delete_time;
                laserCloudOutMsg1.header.stamp = delete_time;
                laserCloudOutMsg2.header.stamp = delete_time;
                laserCloudOutMsg3.header.stamp = delete_time;
                pubAllCloud.publish(laserCloudOutMsg);
                pubEdgeCloud.publish(laserCloudOutMsg1);
                pubSurfCloud.publish(laserCloudOutMsg2);
                pubOdometry.publish(lidarOdom);
                pubOdometry1.publish(prevLidarOdom);
                pubGroundCloud.publish(laserCloudOutMsg3);
                pubLoopEdge.publish(laserCloudOutMsg1);
                pubLoopSurf.publish(laserCloudOutMsg2);
            }
            if (vio_smooth_retrieve_res) {
                prev_imu_receive = true;
            }
            lidar_id++;

            cur_time = lidar_odom_time - 0.2;
            delete_time = (ros::Time)cur_time;

            lidar_odoMsg.DeletePrevious(delete_time);
            vio_optimize_odoMsg.DeletePrevious(delete_time);
            vio_smooth_odoMsg.DeletePrevious(delete_time);
            full_point_msg.DeletePrevious(delete_time);
            edge_point_msg.DeletePrevious(delete_time);
            surf_point_msg.DeletePrevious(delete_time);
            ground_cloud_msg.DeletePrevious(delete_time);
            smooth_velocity.DeletePrevious(delete_time);
            optimized_velocity.DeletePrevious(delete_time);
        } 
        ros::getGlobalCallbackQueue()->callOne(ros::WallDuration(0));
    }
}
