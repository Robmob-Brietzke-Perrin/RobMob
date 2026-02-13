#ifndef RRT_CONNECT_PLANNER__PLANNER_NODE_HPP
#define RRT_CONNECT_PLANNER__PLANNER_NODE_HPP

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "robmob_interfaces/action/compute_path.hpp"
#include "rrt_connect_planner/map_helper.hpp"
#include "rrt_connect_planner/rrt_types.hpp"

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  using PathMsg = nav_msgs::msg::Path;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using ComputePath = robmob_interfaces::action::ComputePath;
  using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

  // Callbacks
  void map_callback(const OccupancyGrid::SharedPtr msg);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputePath::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputePath> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle);

  // Main execution
  void execute(const std::shared_ptr<GoalHandleComputePath> goal_handle);

  // Converters
  PathMsg to_ros_path(const std::vector<Point>& points);

  // ROS Comms
  ComputePath::Feedback feedback_msg_;
  rclcpp_action::Server<ComputePath>::SharedPtr compute_path_server_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<PathMsg>::SharedPtr path_pub_;

  // Helper
  MapHelper map_helper_;
  std::mutex map_mutex_;

  // Params
  struct Params {
    double robot_radius;
    // int max_iterations;
    // int max_tree_size;
    double smooth_cut_dist;
    bool verbose;
  } params_;
};

#endif // RRT_CONNECT_PLANNER__PLANNER_NODE_HPP