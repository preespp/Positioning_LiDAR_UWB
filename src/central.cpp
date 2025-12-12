#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class CentralNode : public rclcpp::Node {
public:
  CentralNode() : Node("central_node") {

    uwb_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10,
        std::bind(&CentralNode::uwb_callback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&CentralNode::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Central node started (fusion-only mode). No autonomous control.");
  }

private:

  void uwb_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "UWB: [%.2f, %.2f]",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y);
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // SLAM system receives /scan directly; EKF uses pose output of SLAM
    (void)msg;
    // (Optional) diagnostics here
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralNode>());
  rclcpp::shutdown();
  return 0;
}
