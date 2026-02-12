#ifndef ROBMOB_AUTO_EXPLO__AUTO_EXPLO_NODE_HPP
#define ROBMOB_AUTO_EXPLO__AUTO_EXPLO_NODE_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//FIXME: mauvais scope
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using LaserScan = sensor_msgs::msg::LaserScan;
using PoseStamped = geometry_msgs::msg::PoseStamped;

class AutoExploNode : public rclcpp::Node
{
public:
  AutoExploNode();

private:

  void map_callback(const OccupancyGrid::SharedPtr msg) { latest_map_ = msg; }
  void scan_callback(const LaserScan::SharedPtr msg) { latest_scan_ = msg; }

  void decision_loop();
  bool get_robot_pose(double &x, double &y, double &yaw);
  void publish_goal(double x, double y, double yaw, double sector_angle, double dist);


  void stop_cb();

  // Membres
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr goal_pub_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stop_timer_;

  OccupancyGrid::SharedPtr latest_map_;
  LaserScan::SharedPtr latest_scan_;
  double last_angle_ = 0.0;

  PoseStamped initial_pose_;
  bool initial_pose_saved_ = false;
  bool goal_active_ = false;
  PoseStamped current_goal_;

  const double GOAL_THRESHOLD = 0.25;
  const double OBSTACLE_THRESHOLD = 0.5;
};

#endif // ROBMOB_AUTO_EXPLO__AUTO_EXPLO_NODE_HPP