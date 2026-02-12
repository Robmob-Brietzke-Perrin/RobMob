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

    stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "stop_exploration", 10, std::bind(&AutoExploNode::stop_cb, this, std::placeholders::_1));

    map_save_client_ = this->create_client<SaveMap>("/slam_toolbox/save_map");
    this->declare_parameter("map_save_path", "~/robmob_ws/src/robmob_bringup/config/map_generee");

    RCLCPP_INFO(this->get_logger(), "Noeud d'exploration auto prêt.");
}

void AutoExploNode::stop_cb(const std_msgs::msg::Empty::SharedPtr msg) {
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Fin d'exploration. Lancement de la sauvegarde...");

    // Renvoie le robot à la base
    if (initial_pose_saved_) {
        initial_pose_.header.stamp = this->now();
        goal_pub_->publish(initial_pose_);
        RCLCPP_INFO(this->get_logger(), "Ordre de retour au départ envoyé.");
    }

    // Save la map via le service de Slam Toolbox
    if (!map_save_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Service Slam Toolbox non disponible !");
    } else {
        auto request = std::make_shared<SaveMap::Request>();
        request->name.data = this->get_parameter("map_save_path").as_string();; 
        
        RCLCPP_INFO(this->get_logger(), "Sauvegarde de la carte via Slam Toolbox...");
        map_save_client_->async_send_request(request);
    }

    // 3. Délai pour laisser le temps au robot de bouger et au fichier d'être écrit
    rclcpp::sleep_for(std::chrono::seconds(3));
    rclcpp::shutdown();
}
void AutoExploNode::decision_loop() {
  if (!latest_map_ || !latest_scan_) return;

    double x, y, yaw;
    if (!get_robot_pose(x, y, yaw)) return;

    if(!initial_pose_saved_)
    {
      initial_pose_.header.frame_id = "map";
      initial_pose_.pose.position.x = x;
      initial_pose_.pose.position.y = y;
      initial_pose_.pose.orientation.w = 1.0; // Simplifié
      initial_pose_saved_ = true;
      RCLCPP_INFO(this->get_logger(), "Position de départ sauvegardée.");
    }

    // Vérifier si l'objectif actuel est toujours valide/atteint
    if (goal_active_) {
        double dist_to_goal = std::hypot(current_goal_.pose.position.x - x, 
                                         current_goal_.pose.position.y - y);
        
        // Vérifier aussi si un mur est apparu devant (via le scan) maintenant seulement si devant dans le scan
        int center_idx = latest_scan_->ranges.size() / 2;
        int window = latest_scan_->ranges.size() / 8;
        float min_front_scan = 10.0;
        
        for(int i = center_idx - window; i < center_idx + window; ++i) {
            if(std::isfinite(latest_scan_->ranges[i])) {
                min_front_scan = std::min(min_front_scan, latest_scan_->ranges[i]);
            }
        }

        if (dist_to_goal > GOAL_THRESHOLD && min_front_scan > OBSTACLE_THRESHOLD) {
            // L'objectif est toujours en cours et le chemin est libre, on attend.
            return; 
        }
        else 
        {
            goal_active_ = false;
            publish_goal(x, y, yaw, 0.0, 0.0); // Stop the robot to avoid residual mvt during recomputation
            RCLCPP_INFO(this->get_logger(), "Objectif atteint ou obstacle détecté. Recalcul...");
        }
            
    }
    else
    { 
      // Calculer une nouvelle direction
      auto best_angle = ExploHelper::getBestDirection(
          latest_scan_, latest_map_, x, y, yaw, 2.0, last_angle_);

      if (best_angle.has_value()) {
          publish_goal(x, y, yaw, best_angle.value(), 2.0);
          last_angle_ = best_angle.value();
          goal_active_ = true;
      }
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
    current_goal_.header.stamp = this->now();
    current_goal_.header.frame_id = "map";
    current_goal_.pose.position.x = x + dist * std::cos(yaw + sector_angle);
    current_goal_.pose.position.y = y + dist * std::sin(yaw + sector_angle);
    current_goal_.pose.orientation.w = 1.0;

    goal_pub_->publish(current_goal_);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoExploNode>());
    rclcpp::shutdown();
    return 0;
}