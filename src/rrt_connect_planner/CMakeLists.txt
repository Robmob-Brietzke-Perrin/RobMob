cmake_minimum_required(VERSION 3.8)
project(rrt_connect_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robmob_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(planner_node src/planner_node.cpp src/map_helper.cpp src/rrt_connect.cpp)
# add_executable(mock_robot_node src/mock_robot_node.cpp)
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  planner_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  robmob_interfaces
  tf2_geometry_msgs
)

# ament_target_dependencies(
#   mock_robot_node
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   robmob_interfaces
# )

# install(
#   DIRECTORY include rviz
#   DESTINATION share/${PROJECT_NAME}
# )
install(DIRECTORY rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS planner_node # mock_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
