cmake_minimum_required(VERSION 3.8)
project(cloud_to_scan)
cmake_policy(SET CMP0074 NEW)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_to_scan_node 
  src/cloud_to_scan_node.cpp
)
ament_target_dependencies(cloud_to_scan_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

# 显式链接 tf2 库
target_link_libraries(cloud_to_scan_node
  ${tf2_LIBRARIES}
)
target_link_libraries(cloud_to_scan_node ${PCL_LIBRARIES})
install(TARGETS
  cloud_to_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
