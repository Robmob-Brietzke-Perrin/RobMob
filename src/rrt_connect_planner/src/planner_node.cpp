#include "rrt_connect_planner/planner_node.hpp"
#include "rrt_connect_planner/rrt_connect.hpp"
#include <vector>

class Point;
class Tree;
class RRT;

void PlannerNode::execute(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(ALGORITHM_RATE);

    const auto goal_request = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputePath::Feedback>();
    auto result = std::make_shared<ComputePath::Result>();

    path_msg.header.frame_id = "map";

    RRT algo;
    Tree tree_a, tree_b;
    bool tree_size_ok = true;
    result->success=false;
    algo.init(goal_request->start, goal_request->goal);
    for(unsigned int i = 0;i < goal_request->max_iterations && tree_size_ok && rclcpp::ok(); ++i)
    {
      if (goal_handle->is_canceling()) {
          RCLCPP_WARN(this->get_logger(), "Canceling action...");
          goal_handle->canceled(result);
          return;
      }

      x_rand = random_state();
      if(extend(tree_a, x_rand) != TRAPPED)
      {
        feedback->tree_a_size++; // Bof vu le switch
        if(extend(tree_b, x_new) == REACHED)
        {
          std::vector<Point> raw_path = path(tree_a, tree_b);
          nav_msgs::msg::Path path_msg;
          for (auto p : raw_path)
          {
            geometry_msgs::msg::PoseStamped new_pose;
            new_pose.header.frame_id = "map";
            new_pose.header.stamp = this->now();
            new_pose.pose.position.x = i * 0.1;
            new_pose.pose.position.y = i * 0.1;
            new_pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(new_pose);

            path_msg.header.stamp = this->now();
            this->path_pub_->publish(path_msg);
          }
          result->success=true;
          break; // Au lieu de break, il faut modif feedback pour avoir un Path, et continuer la recherche avec une cond d'amelioration. Pdt ce tps le client à une certaine patience -> cancel l'action si le premier path suffisant.
        }
      }
      std::swap(&tree_a, &tree_b);

      // TODO: tree visualisation? ou edge points?

      feedback->iterations++;
      goal_handle->publish_feedback(feedback);

      if(feedback->tree_size_a > goal_request->max_tree_size ||
         feedback->tree_size_b > goal_request->max_tree_size)
         tree_size_ok = false;

      loop_rate.sleep();
    }

    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        // TODO: correctly cleanup interrupted computation?
        RCLCPP_INFO(this->get_logger(), "Goal achieved !");
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

// TESTING: ros2 topic pub /map nav_msgs/msg/OccupancyGrid "{header: {frame_id: 'map'}, info: {resolution: 1.0, width: 2, height: 2, origin: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, data: [0, 0, 0, 0]}" --once
// AND: ros2 action send_goal /compute_path robmob_interfaces/action/ComputePath "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, max_iterations: 200, max_tree_size: 500}"
// WITH RVIZ configured -> Add -> by topic -> path_verbose/ (ros2 run rviz2 rviz2)