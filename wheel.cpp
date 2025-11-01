#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelMotorNode : public rclcpp::Node {
public:
  WheelMotorNode() : Node("wheel_motor_node") {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&WheelMotorNode::cmd_callback, this, std::placeholders::_1));
  }

private:
  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Motor cmd: linear=%.2f angular=%.2f",
                msg->linear.x, msg->angular.z);
    // TODO: Convert to PWM or serial commands for motor controller
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelMotorNode>());
  rclcpp::shutdown();
  return 0;
}
