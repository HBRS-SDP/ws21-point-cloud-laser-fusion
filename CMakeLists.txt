cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_laserscan_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf2 tf2_ros tf2_sensor_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(fuse_pointcloud_laserscan src/fuse_pointcloud_laserscan.cpp)
target_link_libraries(fuse_pointcloud_laserscan ${catkin_LIBRARIES})


