cmake_minimum_required(VERSION 3.8)
project(unitree_move_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(unitree_sdk2 REQUIRED)
find_package(geometry_msgs REQUIRED) 

add_executable(unitree_b2_move src/unitree_b2_move.cpp)

ament_target_dependencies(
  unitree_b2_move
  rclcpp
  # unitree_sdk2
  geometry_msgs
)

target_link_libraries(unitree_b2_move
  libb2w_interface.so
)
install(TARGETS unitree_b2_move
  DESTINATION lib/${PROJECT_NAME} 
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()