cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
set(srv_files
  "srv/SetPose.srv"
)

# 生成自定义接口
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# 地图加载节点
add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node
  rclcpp
  nav_msgs
  tf2
  yaml-cpp
)
target_link_libraries(map_loader_node yaml-cpp)

# A*路径规划节点
add_executable(path_planner_node src/path_planner_node.cpp)
ament_target_dependencies(path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# PID控制器节点
add_executable(pid_controller_node src/pid_controller_node.cpp)
ament_target_dependencies(pid_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 链接自定义接口
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(map_loader_node ${cpp_typesupport_target})
target_link_libraries(path_planner_node ${cpp_typesupport_target})
target_link_libraries(pid_controller_node ${cpp_typesupport_target})

# 安装可执行文件
install(TARGETS
  map_loader_node
  path_planner_node
  pid_controller_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件
install(DIRECTORY
  launch srv maps
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
