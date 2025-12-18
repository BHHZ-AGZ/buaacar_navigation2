cmake_minimum_required(VERSION 3.8)
project(ros2_beast_bridge)

# 指定C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包（包含导航相关依赖）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS system)
find_package(sensor_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)  # 全局路径消息依赖
find_package(nlohmann_json REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # TF与消息转换依赖
find_package(geometry_msgs REQUIRED)  # 导航目标消息依赖
find_package(nav2_msgs REQUIRED)

# 包含头文件目录（确保所有自定义头文件可被找到）
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../jwt-cpp/include  # jwt-cpp头文件目录
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

# 1. 定义base_link_pose_lib共享库（地图和位姿计算）
add_library(base_link_pose_lib SHARED
  src/base_link_pose.cpp  # 位姿计算实现
)

# 配置base_link_pose_lib的依赖和链接
target_include_directories(base_link_pose_lib PUBLIC
  include  # 公开头文件，允许其他模块引用
)

ament_target_dependencies(base_link_pose_lib
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
)

target_link_libraries(base_link_pose_lib
  rclcpp::rclcpp
  ${Boost_LIBRARIES}
)

# 2. 定义navigation_goal_lib共享库（导航目标处理）
add_library(navigation_goal_lib SHARED
  src/navigation_goal.cpp  # 新增：导航目标处理实现
)

# 配置navigation_goal_lib的依赖和链接
target_include_directories(navigation_goal_lib PUBLIC
  include
)

ament_target_dependencies(navigation_goal_lib
  rclcpp
  geometry_msgs  # 导航目标消息（PoseStamped）
  nav_msgs       # 全局路径消息（Path）
  tf2            # 四元数转换
  nlohmann_json  # JSON处理
  rclcpp_action
  nav2_msgs
)

target_link_libraries(navigation_goal_lib
  base_link_pose_lib  # 依赖位姿计算库（获取地图参数）
  rclcpp::rclcpp
  ${Boost_LIBRARIES}
)

# 3. 定义主可执行文件http_bridge_server
add_executable(http_bridge_server
  src/http_server.cpp  # HTTP服务器实现
  src/main.cpp         # 程序入口
)

# 配置主程序的依赖和链接
target_include_directories(http_bridge_server PRIVATE
  include
)

target_link_libraries(http_bridge_server
  base_link_pose_lib      # 位姿计算库
  navigation_goal_lib     # 新增：导航目标处理库
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${std_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(http_bridge_server
  rclcpp
  std_msgs
  ament_index_cpp
  sensor_msgs
  geometry_msgs  # 补充导航消息依赖
  nav2_msgs
)

# 安装配置（统一安装路径，确保ROS能找到可执行文件和库）
install(TARGETS
  http_bridge_server
  base_link_pose_lib
  navigation_goal_lib  # 新增：安装导航目标处理库
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装资源文件（配置、网页等）
install(DIRECTORY web/
  DESTINATION share/${PROJECT_NAME}/web
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装头文件（供其他包引用）
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# 导出依赖信息，确保其他包能正确链接
ament_export_include_directories(include)
ament_export_libraries(
  base_link_pose_lib
  navigation_goal_lib  # 新增：导出导航目标处理库
)
ament_export_dependencies(
  rclcpp 
  tf2 
  tf2_ros 
  nav_msgs 
  geometry_msgs  # 新增：导出导航消息依赖
  tf2_geometry_msgs 
  nlohmann_json
  nav2_msgs
)

ament_package()
