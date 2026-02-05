#include "robmob_cmd_law/cmd_law_node.hpp"

using namespace std::placeholders;

CmdLawNode::CmdLawNode(const rclcpp::NodeOptions & options) 
: Node("cmd_law_node", options) 
{
    this->client_ptr_ = rclcpp_action::create_client<ComputePath>(this, "compute_path");
}

void CmdLawNode::send_goal() 
{
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return;
    }

    auto goal_msg = ComputePath::Goal();
    // TODO:Configure  goal_msg

    auto send_goal_options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = std::bind(&CmdLawNode::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&CmdLawNode::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&CmdLawNode::result_callback, this, _1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void CmdLawNode::goal_response_callback(const GoalHandleComputePath::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void CmdLawNode::feedback_callback(
    GoalHandleComputePath::SharedPtr,
    const std::shared_ptr<const ComputePath::Feedback> feedback)
{
    //TODO: feedback
    RCLCPP_INFO(this->get_logger(), "Feedback received"); 
}

void CmdLawNode::result_callback(const GoalHandleComputePath::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Success!");
        // TODO: follow traj
    } else {
        RCLCPP_ERROR(this->get_logger(), "Action failed");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CmdLawNode)