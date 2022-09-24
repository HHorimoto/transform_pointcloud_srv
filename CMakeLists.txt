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

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES transform_pointcloud_srv
  # CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs pcl_ros                    
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_pointcloud_srv src/transform_pointcloud_srv.cpp)
target_link_libraries(transform_pointcloud_srv ${catkin_LIBRARIES})
add_dependencies(transform_pointcloud_srv transform_pointcloud_srv_gencpp)
