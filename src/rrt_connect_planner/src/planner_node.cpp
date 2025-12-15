#include "rclcpp/rclcpp.hpp"
#include "rrt_connect_planner/rrt_connect.hpp"
#include "rrt_connect_planner/map_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robmob_interfaces/action/compute_path.hpp"


constexpr int ALGORITHM_RATE = 10; // Equivalent to #define but cleaner in c++
constexpr int MAP_QUEUE_SIZE = 10;

// Some abreviation for painfully long typings
using namespace robmob_interfaces::action;
using namespace std::placeholders;
using namespace nav_msgs::msg;
using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

class PlannerNode : public rclcpp::Node
{

public:

    PlannerNode()
    : Node("planner_node")
    {
        /* ===== Map Subscription =====*/

        auto map_callback = [this](OccupancyGrid::UniquePtr msg)
        {
            // Use mutex so that this does not interfere with the action (in another thread)
            std::lock_guard<std::mutex> lock(map_mutex_);
            // TODO: inflate map?
            latest_map_ = std::move(msg);
        };
        map_sub_ = this->create_subscription<OccupancyGrid>("/map"/*TODO: find where to get the map*/, MAP_QUEUE_SIZE, map_callback);

        /* ===== Compute Path Action ===== */

        // Lambda function on goal recieved
        auto handle_goal = [this](
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ComputePath::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Recieved path computing request.");
            
            if (!is_goal_reachable(goal)) // TODO: implement goal point verification?
            {
                RCLCPP_WARN(this->get_logger(), "Goal position unreachable, action rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }
            if (latest_map_ == nullptr)
            {
                RCLCPP_WARN(this->get_logger(), "No map is loaded yet, action rejected");
                return rclcpp_action::GoalResponse::REJECT;
            }

            (void)uuid; // (does effectively nothing exept getting rid of unused param warn)
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        // Lambda function on goal cancel (TODO: call this from client in case too long?)
        auto handle_cancel = [this](const std::shared_ptr<GoalHandleComputePath> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            if (goal_handle->is_canceling())
            {
                // TODO: correctly cleanup interrupted computation?
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        // Lambda function on goal accepted (must finish quickly so it delegates to a thread)
        auto handle_accepted = [this](const std::shared_ptr<GoalHandleComputePath> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle]()
            { return this->execute(goal_handle); };
            std::thread{execute_in_thread}.detach();
        };

        // Create the action server
        this->compute_path_server_ = rclcpp_action::create_server<ComputePath>(
            this,
            "compute_path",
            handle_goal,
            handle_cancel,
            handle_accepted);
    }

    void execute(const std::shared_ptr<GoalHandleComputePath> goal_handle)
    {
        // Actual computation logic
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(ALGORITHM_RATE);

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ComputePath::Feedback>();
        auto result = std::make_shared<ComputePath::Result>();

        // TODO: Get the map here? Or copy the latest registered instance?

        for(unsigned int i = 0;i < goal->max_iterations && rclcpp::ok(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                RCLCPP_WARN(this->get_logger(), "Canceling action...");
                goal_handle->canceled(result);
                return;
            }

            // TODO: rrt-connect loop?

            // TODO: update and publish feedback
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

private:
    rclcpp_action::Server<ComputePath>::SharedPtr compute_path_server_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;
    OccupancyGrid::SharedPtr latest_map_ = nullptr;
    std::mutex map_mutex_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}