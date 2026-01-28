#include "rrt_connect_planner/planner_node.hpp"

using namespace std::placeholders;

PlannerNode::PlannerNode()
  : Node("rrt_planner_node")
{
    // Ros parameters
    this->declare_parameter("robot_radius", 0.3f);
    this->declare_parameter("max_iterations", 10000);
    this-declare_parameter("verbose", true);

    robot_radius_ = this->get_parameter("robot_radius").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    verbose_ = this->get_parameter("verbose").as_bool();

    // Subscription to map
    // QoS TransientLocal pour recevoir la map même si publiée avant le lancement du node
    rclcpp::QoS map_qos(1);
    map_qos.reliable();
    map_qos.durability_volatile();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", map_qos, std::bind(&PlannerNode::map_callback, this, _1));

    // Visualization path publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("Path", 10);

    feedback_msg_.iterations = 0;
    feedback_msg_.tree_size_a = 0;
    feedback_msg_.tree_size_b = 0;
    feedback_msg_.inter_path = nullptr;
  
    // Initialisation de l'Action Server
    compute_path_server_ = rclcpp_action::create_server<ComputePath>(
        this,
        "compute_path",
        std::bind(&PlannerNode::handle_goal, this, _1, _2),
        std::bind(&PlannerNode::handle_cancel, this, _1),
        std::bind(&PlannerNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "RRT Connect Planner Initialized");
}

void PlannerNode::map_callback(const OccupancyGrid::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    RCLCPP_INFO(this->get_logger(), "Map received");

    map_helper_.initialize(*msg);
    map_helper_.inflate_obstacles(robot_radius_);
}

rclcpp_action::GoalResponse PlannerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ComputePath::Goal> goal)
{
    if (map_helper_.get_state() != MapHelper::INFLATED) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: Map not ready yet");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    std::thread{std::bind(&PlannerNode::execute, this, _1), goal_handle}.detach();
}

void PlannerNode::execute(const std::shared_ptr<GoalHandleComputePath> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing RRT Connect...");

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePath::Result>();

    std::unique_lock<std::mutex> lock(map_mutex_);
    RRTConnect rrt_connect(map_helper_);

    if (!map_helper_.is_free(goal->start.pose) ||
        !map_helper_.is_free(goal->goal.pose)) {
        RCLCPP_ERROR(this->get_logger(), "Start or Goal is inside an obstacle");
        result->success = false;
        goal_handle->abort(result);
        return;
    }
    lock.unlock(); // FIXME: move this if using map later

    // Initialize trees
    Tree tree_start, tree_goal;
    tree_start.add_node(goal->start.pose, nullptr);
    tree_goal.add_node(goal->goal.pose, nullptr);

    // Interchangeble buffers
    Tree* tree_a = &tree_start;
    Tree* tree_b = &tree_goal;

    bool success = false;
    Path final_path;
    auto start_time = this->now();
    int iteration = 0;
    for(; iteration < max_iterations_ && rclcpp::ok(); ++iteration)
    {
      // Check if cancel
      if (goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
      }

      // Actual RRT algorithm
      Pose q_rand = rrt_connect.random_config();
      if (verbose_ && iteration % 100 == 0) {
          RCLCPP_INFO(this->get_logger(), "Iter %d: Random point (%.2f, %.2f)", 
              iteration, q_rand.position.x, q_rand.position.y);
      }
      std::shared_ptr<TreeNode> q_new_node;

      if(rrt_connect.extend(*tree_a, q_rand, q_new_node) != RRTConnect::TRAPPED)
      {
          std::shared_ptr<TreeNode> q_connect_node;
          if(tree_a == &tree_a) feedback_msg_.tree_size_a++;
          if(rrt_connect.connect(*tree_b, q_new_node->get_pose(), q_connect_node) == RRTConnect::REACHED)
          {
              // Get path from connected trees (from start to goal)
              if (tree_a == &tree_start)
                  final_path = rrt_connect.get_path(q_new_node, q_connect_node);
              else
                  final_path = rrt_connect.get_path(q_connect_node, q_new_node); // Inversion if swaped

              success = true;
              feedback_msg_.inter_path = final_path;
              break;
          }
      }
      // TODO: reduce jittering: try skipping points iteratively -> less points, direct, but sharp turns
      // -> Send traj as first valid in feedback
      // -> Continue action to try to smooth out the path with elastic band = minimiser longueur chemin + maximiser distance aux obstacles en régression linéaire.

      // Swap
      std::swap(tree_a, tree_b);

      feedback_msg_.iterations = iteration;
      goal_handle->publish_feedback(feedback_msgs_);
    }

    for(; iteration < max_iterations_ && rclcpp::ok(); ++iteration)
    {
      // Check annulation
      if (goal_handle->is_canceling()) {
          result->success = false;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
      }
    }

    // --- 3. Résultat ---
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Path found with %d iterations", iteration);

        final_path.header.stamp = this->now();
        final_path.header.frame_id = "map";

        // Publication pour debug/visu
        if(verbose_)
          path_pub_->publish(final_path);

        result->path = final_path;
        result->success = true;
        goal_handle->succeed(result);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to find path after %d iterations", iteration);
        result->success = false;
        goal_handle->abort(result);
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}


/*

ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/joachimperrin/turtlebot4_env_docker/shared/robmob_ws/src/robmob_bringup/config/test_map.yaml

ros2 lifecycle set /map_server configure

ros2 run rrt_connect_planner planner_node

ros2 action send_goal /compute_path robmob_interfaces/action/ComputePath "{start: {header: {frame_id: 'map'}, pose: {position: {x: -9.0, y: -9.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}, goal: {header: {frame_id: 'map'}, pose: {position: {x: 40.0, y: 40.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"



*/