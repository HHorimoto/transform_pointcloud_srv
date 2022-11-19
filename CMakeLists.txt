cmake_minimum_required(VERSION 3.0.2)
project(transform_pointcloud_srv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TransformPointcloud.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_pointcloud_srv_node src/transform_pointcloud_srv_node.cpp)
target_link_libraries(transform_pointcloud_srv_node ${catkin_LIBRARIES})
