cmake_minimum_required(VERSION 3.8)
project(gps_to_utm)

# 强制使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找 GeographicLib 包（关键修改）
find_package(GeographicLib REQUIRED) # 使用 REQUIRED 确保必须找到

# 2. 查找 ROS 2 相关包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# 添加可执行文件
add_executable(utm_odometry_node src/utm_odometry_node.cpp)

target_include_directories(utm_odometry_node PUBLIC
  ${GeographicLib_INCLUDE_DIRS}  # 添加头文件路径
)

target_link_libraries(utm_odometry_node
  rclcpp::rclcpp
  ${GeographicLib_LIBRARIES}  # 替换为库变量
  m
) 
    
ament_target_dependencies(utm_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 安装和其余部分保持不变
install(TARGETS
  utm_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()