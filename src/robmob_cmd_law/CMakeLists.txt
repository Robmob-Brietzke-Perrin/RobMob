cmake_minimum_required(VERSION 3.8)
project(robmob_cmd_law)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Differentiate target
if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  message(STATUS "Building for ROS 2 Jazzy (Gazebo Harmonic)")
  add_compile_definitions(ROS_DISTRO_JAZZY)

elseif("$ENV{ROS_DISTRO}" STREQUAL "humble")
  message(STATUS "Building for ROS 2 Humble (Gazebo Fortress)")
  add_compile_definitions(ROS_DISTRO_HUMBLE)

else()
  message(WARNING "Unknown ROS_DISTRO: $ENV{ROS_DISTRO}. Defaulting to Humble build.")
  add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robmob_interfaces REQUIRED)

add_library(cmd_law_node SHARED
  src/fibonacci_cmd_law_node.cpp)
target_include_directories(cmd_law_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(cmd_law_node
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(cmd_law_node
  "action_tutorials_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(cmd_law_node PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_cmd_law_node)#TODO:Remplacer 
install(TARGETS
  cmd_law_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
