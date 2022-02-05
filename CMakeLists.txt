cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_laserscan_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL 1.10 REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf2 tf2_ros tf2_sensor_msgs 
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(fuse_pointcloud_laserscan src/fuse_voxel.cpp)
target_link_libraries(fuse_pointcloud_laserscan ${catkin_LIBRARIES} ${PCL_LIBRARIES})





