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

# find_package(geometry_msgs REQUIRED) # Some usefull msgs
# find_package(visualization_msgs REQUIRED) # TODO: use this to publish current traj to rviz?
# find_package(tf2 REQUIRED) # consider using this as well

add_executable(planner_node src/planner_node.cpp)
target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  planner_node
  rclcpp
  rclcpp_action
  nav_msgs
  robmob_interfaces
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
