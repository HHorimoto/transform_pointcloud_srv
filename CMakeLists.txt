cmake_minimum_required(VERSION 3.5)
project(transform_pointcloud_srv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-- Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#--- for service
set(srv_files
  "srv/TransformPointcloud.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES sensor_msgs
)
#--- end service

#--- for C++
add_executable(transform_pointcloud_srv_node src/transform_pointcloud_srv_node.cpp)
ament_target_dependencies(transform_pointcloud_srv_node rclcpp pcl_ros tf2 tf2_ros sensor_msgs)
rosidl_target_interfaces(transform_pointcloud_srv_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS transform_pointcloud_srv_node DESTINATION lib/${PROJECT_NAME})
target_include_directories(transform_pointcloud_srv_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
#--- end C++

#--- for launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
#--- end launch

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/test_talker.py  
  scripts/test_client.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)
ament_package()
