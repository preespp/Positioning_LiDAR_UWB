#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class CentralNode : public rclcpp::Node {
public:
  CentralNode() : Node("central_node") {

    uwb_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/uwb/position", 10,
      std::bind(&CentralNode::uwb_callback, this, std::placeholders::_1));

    slam_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/slam_pose", 10,
      std::bind(&CentralNode::slam_callback, this, std::placeholders::_1));

    fused_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/fused_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Central fusion node started");
  }

private:
  geometry_msgs::msg::PoseWithCovarianceStamped last_uwb;
  geometry_msgs::msg::PoseWithCovarianceStamped last_slam;
  bool have_uwb = false;
  bool have_slam = false;

  void uwb_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    last_uwb = *msg;
    have_uwb = true;
    fuse_if_ready();
  }

  void slam_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    last_slam = *msg;
    have_slam = true;
    fuse_if_ready();
  }

  void fuse_if_ready() {
    if (!have_uwb || !haveslam)
      return;

    double x_u = last_uwb_.pose.pose.position.x;
    double y_u = last_uwb_.pose.pose.position.y;
    double x_s = last_slam_.pose.pose.position.x;
    double y_s = last_slam_.pose.pose.position.y;

    // Variances
    double var_u = last_uwb_.pose.covariance[0];
    double var_s = last_slam_.pose.covariance[0];

    // Prevent division by zero
    if (var_u < 1e-6) var_u = 1e-6;
    if (var_s < 1e-6) var_s = 1e-6;

    // Covariance-weighted fusion
    double w_u = 1.0 / var_u;
    double w_s = 1.0 / var_s;

    double x_f = (w_u * x_u + w_s * x_s) / (w_u + w_s);
    double y_f = (w_u * y_u + w_s * y_s) / (w_u + w_s);

    geometry_msgs::msg::PoseWithCovarianceStamped fused;
    fused.header.stamp = this->get_clock()->now();
    fused.header.frame_id = "map";

    fused.pose.pose.position.x = x_f;
    fused.pose.pose.position.y = y_f;
    fused.pose.pose.position.z = 0.0;

    // Fused covariance (lower than either alone)
    double var_f = 1.0 / (w_u + w_s);
    fused.pose.covariance[0] = var_f;
    fused.pose.covariance[7] = var_f;

    fused_pub_->publish(fused);

    RCLCPP_INFO(this->get_logger(),
      "FUSED: x=%.2f y=%.2f | UWB(%.2f) SLAM(%.2f)",
      x_f, y_f, sqrt(var_u), sqrt(var_s));
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_sub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr fused_pub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralNode>());
  rclcpp::shutdown();
  return 0;
}
