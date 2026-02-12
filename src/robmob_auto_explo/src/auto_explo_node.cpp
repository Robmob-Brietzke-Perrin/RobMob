#include "robmob_auto_explo/auto_explo_node.hpp"
#include "robmob_auto_explo/explo_helper.hpp"

using namespace std::chrono_literals;

AutoExploNode::AutoExploNode() : Node("auto_explo_node") {
    // Subscriptions
    map_sub_ = this->create_subscription<OccupancyGrid>(
        "map", 10, std::bind(&AutoExploNode::map_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&AutoExploNode::scan_callback, this, std::placeholders::_1));

    // Publisher vers le planner (délègue à cmd_law_node)
    goal_pub_ = this->create_publisher<PoseStamped>("goal_pose", 10);

    // TF Buffer et Listener pour la pose du robot
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Boucle de décision (pas trop pressé, 2Hz par défault, à tune eventuellement)
    timer_ = this->create_wall_timer(1000ms, std::bind(&AutoExploNode::decision_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "Noeud d'exploration auto prêt.");
}

void AutoExploNode::decision_loop() {
    if (!latest_map_ || !latest_scan_) return;

    double x, y, yaw;
    if (!get_robot_pose(x, y, yaw)) return;

    // Appel au helper pour trouver la prochaine direction cible (combinasion lointain + unknown + inertie)
    auto best_angle = ExploHelper::getBestDirection(
        latest_scan_, latest_map_, x, y, yaw, 0.4, last_angle_); // FIXME: 40cm suffisant? 

    if (best_angle.has_value()) {
        publish_goal(x, y, yaw, best_angle.value(), 0.4);
        last_angle_ = best_angle.value();
    } else {
        RCLCPP_WARN(this->get_logger(), "Aucune bonne direction trouvée.");
        // rclcpp::shutdown(); // TODO: trouver une condition d'arrêt, pour l'instant il faut ros2 node kill /auto_explo_node
    }
}

bool AutoExploNode::get_robot_pose(double &x, double &y, double &yaw) {
    try {
        auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        x = t.transform.translation.x;
        y = t.transform.translation.y;

        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        yaw = 2.0 * atan2(qz, qw); 
        return true;
    } catch (const tf2::TransformException & ex) {
        return false;
    }
}

void AutoExploNode::publish_goal(double x, double y, double yaw, double sector_angle, double dist) {
    PoseStamped goal;
    goal.header.stamp = this->now();
    goal.header.frame_id = "map";

    goal.pose.position.x = x + dist * cos(yaw + sector_angle);
    goal.pose.position.y = y + dist * sin(yaw + sector_angle);
    goal.pose.orientation.w = 1.0; // FIXME: donner la même rotation qu'actuellement? 

    goal_pub_->publish(goal);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoExploNode>());
    rclcpp::shutdown();
    return 0;
}