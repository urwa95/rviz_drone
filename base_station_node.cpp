#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BaseStationNode : public rclcpp::Node {
public:
    BaseStationNode() : Node("base_station") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("drone_cmd", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BaseStationNode::timer_callback, this));
    }

private:
    void timer_callback() {
        auto message = geometry_msgs::msg::Twist();
        // Mock command data
        message.linear.x = 1.0;
        message.angular.z = 0.5;
        publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BaseStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


