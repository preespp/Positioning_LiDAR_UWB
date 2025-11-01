#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class UwbPosNode : public rclcpp::Node {
public:
  UwbPosNode() : Node("uwb_pos_node") {
    uwb_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&UwbPosNode::publish_position, this));
  }

private:
  void publish_position() {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "uwb_tag";
    // Example fake data
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 0.5;
    msg.pose.pose.position.z = 0.0;
    uwb_pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbPosNode>());
  rclcpp::shutdown();
  return 0;
}
