#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

using namespace std::chrono_literals;

class FullTurnNode : public rclcpp::Node {
public:
    FullTurnNode() : Node("full_turn_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Initialisation de la rotation 360°...");
        
        start_time_ = this->now();
        // On vérifie l'état toutes les 100ms
        timer_ = this->create_wall_timer(100ms, std::bind(&FullTurnNode::on_timer, this));
    }

private:
    void on_timer() {
        auto current_time = this->now();
        double elapsed = (current_time - start_time_).seconds();

        // On tourne pendant 6.3 secondes (1 rad/s ptd 6.3s ≃ 1 full turn)
        if (elapsed < 6.3) {
            geometry_msgs::msg::Twist msg;
            msg.angular.z = 1.0;
            publisher_->publish(msg);
        } else {
            // Arrêt du robot
            publisher_->publish(geometry_msgs::msg::Twist());
            RCLCPP_INFO(this->get_logger(), "Rotation terminée. Arrêt du noeud.");
            
            // shutdown quand on a fini pour passer la main à l'auto explo
            rclcpp::shutdown();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FullTurnNode>());
    return 0;
}