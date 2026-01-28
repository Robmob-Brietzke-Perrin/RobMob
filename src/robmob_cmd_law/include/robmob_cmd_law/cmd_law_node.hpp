#ifndef ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP
#define ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP
#ifdef ROS_DISTRO_JAZZY
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#ifdef ROS_DISTRO_HUMBLE
#include "geometry_msgs/msg/twist_stamped.hpp"
#endif
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robmob_interfaces/action/compute_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <functional>


class CmdLawNode : public rclcpp::Node
{
    public:
        CmdLawNode();
        //sub a odom ou un truc comme ca
        void send_goal()
        {
          using namespace std::placeholders;
            
          if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
          }
      
          auto goal_msg = ComputePath::Goal();
      
          auto send_goal_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();

          send_goal_options.goal_response_callback =
            std::bind(&CmdLawNode::goal_response_callback, this, _1);

          send_goal_options.feedback_callback =
            std::bind(&CmdLawNode::feedback_callback, this, _1, _2);

          send_goal_options.result_callback =
            std::bind(&CmdLawNode::result_callback, this, _1);

          this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
        
    private:
        #ifdef ROS_DISTRO_JAZZY
        using TwistCmd = geometry_msgs::msg::TwistStamped;
        #endif
        #ifdef ROS_DISTRO_HUMBLE
            using TwistCmd = geometry_msgs::msg::Twist;
        #endif  
        using Path = naw_msgs::msg::Path;
        using ComputePath = robmob_interfaces::action::ComputePath;
        using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

        rclcpp_action::Client<ComputePath>::SharedPtr compute_path_client_;
        
        CmdLawNode goal_response_callback(){};

        void sendcmd_vel();
    //
};





#endif // ROBMOB_CMD_LAW__CMD_LAW_NODE_HPP