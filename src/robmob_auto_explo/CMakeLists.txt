cmake_minimum_required(VERSION 3.8)
project(robmob_auto_explo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robmob_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(auto_explo_node src/auto_explo_node.cpp src/explo_helper.cpp)
target_include_directories(auto_explo_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(auto_explo_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(full_turn_node src/full_turn_node.cpp)
target_include_directories(full_turn_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(auto_explo_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(auto_explo_node
  "rclcpp"
  "rclcpp_action"
  "tf2_ros"             
  "robmob_interfaces"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "std_msgs"
  "slam_toolbox") 

ament_target_dependencies(full_turn_node
  "rclcpp"
  "geometry_msgs") 

install(TARGETS auto_explo_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS full_turn_node
  DESTINATION lib/${PROJECT_NAME})

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
