#include "cmd_law_node.hpp"

CmdLawNode::CmdLawNode():
            Node("cmd_law_node"){compute_path_client_ = rclcpp_action::create_client<ComputePath>(this,"compute_path");}

void CmdLawNode::goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
{
    auto goal_handle = future.get();
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
    //TODO : ajouter un feedback
    //Cancel ou non l'action 
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void CmdLawNode::result_callback(const GoalHandleComputePath::WrappedResult & result)
  {
    if (Result.success)
        ;
        //TODO: follow path
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }