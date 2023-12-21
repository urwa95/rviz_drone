#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/LinearMath/Quaternion.h"

class DroneNode : public rclcpp::Node {
public:
    DroneNode() : Node("drone") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DroneNode::update_position, this));
    }

private:
    void update_position() {
        // Simulate circular motion
        double radius = 5.0;
        double speed = 0.1;
        double angle = now().seconds() * speed;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "world";
        t.child_frame_id = "drone";

        t.transform.translation.x = radius * cos(angle);
        t.transform.translation.y = radius * sin(angle);
        t.transform.translation.z = 1.0; // Fixed altitude

        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DroneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


