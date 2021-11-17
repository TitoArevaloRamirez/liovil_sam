#include "lidar_mapping/lidar_gps.h"
namespace beyond
{
namespace lidar_mapping
{
lidarGPS::lidarGPS(ros::NodeHandle &nh) : gps_msg_buffer(false)
{
    scanPeriod = 0.1;
    lidar_scan_id = 0;
    loop_constriant_optimized_id = 0;
    graph = new gtsam::NonlinearFactorGraph();
    paramISAM2.optimizationParams = gtsam::ISAM2GaussNewtonParams();
    isam2 = gtsam::ISAM2(paramISAM2);
    first_scan_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.01, 0.01, 0.01,
         0.005, 0.005, 0.005)
            .finished()); // rad,rad,rad,m, m, m
    pose_between_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.001, 0.001, 0.001,
         0.01, 0.01, 0.01)
            .finished()); // rad,rad,rad,m, m, m
    loop_constraint_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.001, 0.001, 0.001,
         0.01, 0.01, 0.01)
            .finished()); // rad,rad,rad,m, m, m

    nh.getParam("imu2lidar", imu2lidar1);
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

    gps_noise_model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.05));
    return;
}

void lidarGPS::add_gps_factor(int frame_id, const GPS_MSG& position, bool provided)
{
  // position is available as an argument
  if (provided) {
    std::cout << "add gps factor frame_id: " << frame_id << std::endl;
    graph->add(gtsam::GPSFactor(X(frame_id), position, gps_noise_model));
    std::cout << "add gps factor succeed: " << frame_id << std::endl;
  }
  // need to retrieve from the buffer
  else {

  }
  return;
}

void lidarGPS::odom_lidar_callback(const nav_msgs::Odometry::ConstPtr &input_lidar_odometry)
{
    if (optimize) {
        return;
    }
    // initialize and set time
    gtsam::Pose3 lidar_pose;
    double cur_time = input_lidar_odometry->header.stamp.toSec() - 0.1;
    ros::Time lidar_time = (ros::Time)cur_time;
    std::cout << std::setprecision(20) << "receive odom_lidar_time: " << lidar_time.toNSec() << "\n";
    Eigen::Matrix4d cur_lidar_pose(Eigen::Matrix4d::Identity());
    Eigen::Quaterniond q_b;
    Eigen::Matrix3d rot_b2o;
    q_b.x() = input_lidar_odometry->pose.pose.orientation.x;
    q_b.y() = input_lidar_odometry->pose.pose.orientation.y;
    q_b.z() = input_lidar_odometry->pose.pose.orientation.z;
    q_b.w() = input_lidar_odometry->pose.pose.orientation.w;
    rot_b2o = q_b.toRotationMatrix();
    cur_lidar_pose.matrix().topLeftCorner<3, 3>() = rot_b2o;
    cur_lidar_pose(0, 3) = input_lidar_odometry->pose.pose.position.x;
    cur_lidar_pose(1, 3) = input_lidar_odometry->pose.pose.position.y;
    cur_lidar_pose(2, 3) = input_lidar_odometry->pose.pose.position.z;
    lidar_pose = gtsam::Pose3(cur_lidar_pose);
    initial_values.insert(X(lidar_scan_id), lidar_pose);
    std::cout << "intit x: " << cur_lidar_pose(0, 3) << std::endl;
    std::cout << "intit y: " << cur_lidar_pose(1, 3) << std::endl;
    std::cout << "intit z: " << cur_lidar_pose(2, 3) << std::endl;
    // add in factor
    if (lidar_scan_id == 0)
    {
        graph->add(gtsam::PriorFactor<gtsam::Pose3>(X(lidar_scan_id), lidar_pose, first_scan_noise));
        lidar_scan_id++;
        return;
    }
    double retrived_time = 0;
    bool retrive_res = false;
    Eigen::Matrix4f cur_close_odometry =
        gps_msg_buffer.GetClosestEntry(lidar_time, retrive_res, retrived_time);
    float time_diff = cur_time - retrived_time;
    std::cout << "time_diff****: " << time_diff << std::endl;
    optimize = false;
    if (fabs(time_diff) < 0.001) {
        std::cout << "***************optimize****************" << std::endl;
        optimize = true;
    }
    gps_msg_buffer.DeletePrevious(lidar_time);
    gtsam::Pose3 delta_transformation = last_lidar_pose.inverse() * lidar_pose;
        graph->add(gtsam::BetweenFactor<gtsam::Pose3>(X(lidar_scan_id - 1),
                                                      X(lidar_scan_id),
                                                      delta_transformation,
                                                      pose_between_noise));
    if (optimize) {
        //add_gps_factor(X(lidar_scan_id), gtsam::Point3(cur_close_odometry(0, 3),
        //cur_close_odometry(1, 3),cur_close_odometry(2, 3)), true); 
        Eigen::Matrix4d pose_tmp;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                pose_tmp(i, j) = (double)(cur_close_odometry(i, j));
            }
        }
        gtsam::Pose3 closest_pose;
        closest_pose = gtsam::Pose3(pose_tmp);
        GPS_MSG cur_gps_pose;
        cur_gps_pose(0) = pose_tmp(0, 3);
        cur_gps_pose(1) = pose_tmp(1, 3);
        cur_gps_pose(2) = pose_tmp(2, 3);
        std::cout << "add x: " << cur_gps_pose(0) << std::endl;
        std::cout << "add y: " << cur_gps_pose(1) << std::endl;
        std::cout << "add z: " << cur_gps_pose(2) << std::endl;
        //graph->add(gtsam::PriorFactor<gtsam::Pose3>(X(lidar_scan_id), closest_pose, first_scan_noise));
        add_gps_factor(lidar_scan_id, cur_gps_pose, true);
        isam2.update(*graph, initial_values);
        isam2.update();
        isam2.update();
        isam2.update();
        post_values = isam2.calculateEstimate(); 
        gtsam::Vector3 position = post_values.at<gtsam::Pose3>(X(lidar_scan_id)).translation();
        std::cout << "x: " << position(0) << std::endl;
        std::cout << "y: " << position(1) << std::endl;
        std::cout << "z: " << position(2) << std::endl;
        graph->resize(0);
            initial_values.clear();
    }
    else
    {
        // form pose between factor, and add to the graph
    }
    last_lidar_pose = lidar_pose;
    optimize = false;
    lidar_scan_id++;
}

void lidarGPS::odom_gps_callback(const nav_msgs::Odometry::ConstPtr &input_gps_odometry)
{
    // initialize and set time
    gtsam::Pose3 gps_pose;
    ros::Time gps_time = input_gps_odometry->header.stamp;
    Eigen::Matrix4f cur_vio_odom(Eigen::Matrix4f::Identity());
    Eigen::Quaternionf q_b;
    Eigen::Matrix3f rot_b2o;
    q_b.x() = input_gps_odometry->pose.pose.orientation.x;
    q_b.y() = input_gps_odometry->pose.pose.orientation.y;
    q_b.z() = input_gps_odometry->pose.pose.orientation.z;
    q_b.w() = input_gps_odometry->pose.pose.orientation.w;
    rot_b2o = q_b.toRotationMatrix();
    cur_vio_odom.matrix().topLeftCorner<3, 3>() = rot_b2o;
    cur_vio_odom(0, 3) = input_gps_odometry->pose.pose.position.x;
    cur_vio_odom(1, 3) = input_gps_odometry->pose.pose.position.y;
    cur_vio_odom(2, 3) = input_gps_odometry->pose.pose.position.z;
    cur_vio_odom = cur_vio_odom * imu2lidar;
    gps_msg_buffer.AddEntry(input_gps_odometry->header.stamp, cur_vio_odom);
    std::cout << std::setprecision(20) << "receive gps time: " << gps_time.toNSec() << "\n";
}

} // namespace lidar_mapping
} // namespace beyond
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarMapping");
    ros::NodeHandle nh;
    beyond::lidar_mapping::lidarGPS *lidar_gps = new beyond::lidar_mapping::lidarGPS(nh);
    ros::Subscriber odom_lidar_sub = nh.subscribe("/aft_mapped_to_init", 1000,
                                                  &beyond::lidar_mapping::lidarGPS::odom_lidar_callback, lidar_gps);
    ros::Subscriber odom_gps_sub = nh.subscribe("/vio/odom_gps", 1000,
                                                &beyond::lidar_mapping::lidarGPS::odom_gps_callback, lidar_gps);
    ros::spin();
    return 0;
}
