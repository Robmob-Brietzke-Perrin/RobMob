#ifndef ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP
#define ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP

#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/path.hpp"
#include "robmob_interfaces/action/compute_path.hpp"

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

    // Variables membres
    rclcpp_action::Client<ComputePath>::SharedPtr client_ptr_;

    // Signatures de fonctions (Indispensable pour le std::bind)
    void goal_response_callback(const GoalHandleComputePath::SharedPtr & goal_handle);
    void feedback_callback(GoalHandleComputePath::SharedPtr, const std::shared_ptr<const ComputePath::Feedback> feedback);
    void result_callback(const GoalHandleComputePath::WrappedResult & result);
    
    void sendcmd_vel();
};

#endif