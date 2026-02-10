#ifndef ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP
#define ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP

#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robmob_interfaces/action/compute_path.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

#ifdef ROS_DISTRO_JAZZY
  #include "geometry_msgs/msg/twist_stamped.hpp"
#else
  #include "geometry_msgs/msg/twist.hpp"
#endif

class CmdLawNode : public rclcpp::Node
{
public:
  explicit CmdLawNode(const rclcpp::NodeOptions & options);
  void send_goal();

private:
    #ifdef ROS_DISTRO_JAZZY
        using TwistCmd = geometry_msgs::msg::TwistStamped;
    #else
        using TwistCmd = geometry_msgs::msg::Twist;
    #endif  
    
    using Path = nav_msgs::msg::Path;
    using ComputePath = robmob_interfaces::action::ComputePath;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

    // Objets ROS
    rclcpp_action::Client<ComputePath>::SharedPtr client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<TwistCmd>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Variables d'état
    geometry_msgs::msg::PoseStamped last_goal_pose_;
    nav_msgs::msg::Path current_path_;
    size_t target_index_ = 0;
    bool path_following_active_ = false;

    // Callbacks
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void goal_response_callback(const GoalHandleComputePath::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleComputePath::SharedPtr, const std::shared_ptr<const ComputePath::Feedback> feedback);
    void result_callback(const GoalHandleComputePath::WrappedResult & result);
    
    // Boucle de contrôle (Suivi de trajectoire)
    void control_loop();
    void stop_robot();

    double l1 = 0.1; 
    double k1 = 0.8; 
    double k2 = 0.8;
};

#endif