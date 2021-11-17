cmake_minimum_required(VERSION 3.0.2)
project(liovil_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  #liosam original
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  #visual frontend
  image_transport
  pcl_ros
  genmsg
  #vio
  stereo_msgs
  message_filters
  #image_geometry
)

find_package(OpenCV 3.3.1 REQUIRED HINTS /home/frc-ag/vilsam_dependencies/opencv/opencv_install)  #visual frontend
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
#find_package(OpenCV REQUIRED QUIET) #liosam original
find_package(Boost REQUIRED COMPONENTS system)
find_package(GTSAM REQUIRED QUIET)
find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
  StereoFeatureMatches.msg  #visual frontend
  landmarks.msg             #vio
)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
  visualization_msgs
  image_transport
  #cv_bridge     #vio
  stereo_msgs   #vio
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

#vio

set(SOURCES  include/vio/FixedLagSmoother.h
             src/vio/FixedLagSmoother.cpp
             include/vio/BatchFixedLagSmoother.h
             src/vio/BatchFixedLagSmoother.cpp
             include/vio/time_based_retriever.h
             include/vio/CameraPoseVisualization.h
             src/vio/CameraPoseVisualization.cpp
             include/vio/anms.h
	         include/vio/Header.h
	         include/vio/gms_matcher.h)

###########
## Build ##
###########

############################## LIOSAM ##############################

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/liosam/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/liosam/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/liosam/mapOptmization.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/liosam/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

############################## Visual Frontend ##############################

add_executable(${PROJECT_NAME}_visual_frontend
  src/visual_frontend/feature_operations.cc
  src/visual_frontend/stereo_tracker.cc
  src/visual_frontend/frame.cc
  src/visual_frontend/landmark.cc
  src/visual_frontend/stereo_pair.cc
  src/visual_frontend/parameters.cc
  src/visual_frontend/optimizer.cc
  src/visual_frontend/reprojection_fns.cc
  src/visual_frontend/visual_frontend.cc)

add_dependencies(${PROJECT_NAME}_visual_frontend ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_visual_frontend ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_visual_frontend ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_show_frame_node src/visual_frontend/show_frame.cc)
add_dependencies(${PROJECT_NAME}_show_frame_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(${PROJECT_NAME}_show_frame_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_show_frame_node ${OpenCV_LIBS} ${catkin_LIBRARIES})


############################## VIO ##############################
add_executable(${PROJECT_NAME}_smart_smoother src/vio/VINSmart_estimator.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME}_smart_smoother ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_smart_smoother ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME}_visual_kf src/vio/show_kf.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME}_visual_kf ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_visual_kf ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

############################## LiDAR mapping ##############################

add_executable(${PROJECT_NAME}_lidarFeatureExtract src/lidar_mapping/lidar_feature_extract.cc)
target_link_libraries(${PROJECT_NAME}_lidarFeatureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_lidarVioFeatureExtract src/lidar_mapping/lidar_vio_feature_extract.cc)
target_link_libraries(${PROJECT_NAME}_lidarVioFeatureExtract ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_lidarOdometry src/lidar_mapping/lidar_odometry.cc)
target_link_libraries(${PROJECT_NAME}_lidarOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_odometrySelect src/lidar_mapping/odometry_select.cc)
target_link_libraries(${PROJECT_NAME}_odometrySelect ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} )

add_executable(${PROJECT_NAME}_lidarMapping src/lidar_mapping/lidar_mapping.cc)
target_link_libraries(${PROJECT_NAME}_lidarMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} yaml-cpp)

############################## Misc ##############################
# IMU Preintegration
add_executable(${PROJECT_NAME}_imu_ned2enu src/ned2enu.cpp)
target_link_libraries(${PROJECT_NAME}_imu_ned2enu ${catkin_LIBRARIES})
