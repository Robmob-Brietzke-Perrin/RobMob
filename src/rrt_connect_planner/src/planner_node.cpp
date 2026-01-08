#include "rrt_connect_planner/planner_node.hpp"

void PlannerNode::execute(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    // Actual computation logic
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(ALGORITHM_RATE);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ComputePath::Feedback>();
    auto result = std::make_shared<ComputePath::Result>();

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map"; // FIXME: or "odom"?
 
    for(unsigned int i = 0;i < goal->max_iterations && rclcpp::ok(); ++i)
    {
        if (goal_handle->is_canceling())
        {
            RCLCPP_WARN(this->get_logger(), "Canceling action...");
            goal_handle->canceled(result);
            return;
        }
        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header.frame_id = "map";
        new_pose.header.stamp = this->now();
        new_pose.pose.position.x = i * 0.1;
        new_pose.pose.position.y = i * 0.1;
        new_pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(new_pose);
        
        path_msg.header.stamp = this->now();
        this->path_pub_->publish(path_msg);


        feedback->iterations++;
        feedback->tree_a_size++;
        feedback->tree_b_size++;
        goal_handle->publish_feedback(feedback);

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