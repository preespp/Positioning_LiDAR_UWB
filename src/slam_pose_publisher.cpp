#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <optional>

class SlamPosePublisher : public rclcpp::Node {
public:
  SlamPosePublisher() : Node("slam_pose_publisher"),
    tf_buffer(this->get_clock()),
    tf_listener(tf_buffer) {

    pub = this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/slam_pose", 10);

    uwb_sub = this->create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          last_uwb_ = *msg;
          have_uwb_ = true;
        });

    timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SlamPosePublisher::publish_pose, this));

    RCLCPP_INFO(get_logger(), "SLAM pose publisher started");
  }

private:
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;
  std::optional<double> offset_x_;
  std::optional<double> offset_y_;

  geometry_msgs::msg::PoseWithCovarianceStamped last_uwb;
  bool have_uwb = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_sub;

  void publish_pose() {
    try {
      auto tf = tf_buffer.lookupTransform(
        "map", "base_link", tf2::TimePointZero);

      double x_slam = tf.transform.translation.x;
      double y_slam = tf.transform.translation.y;

      // Initialize offset ONCE
      if (!offset_x && have_uwb) {
        offset_x = last_uwb.pose.pose.position.x - x_slam;
        offset_y = last_uwb.pose.pose.position.y - y_slam;

        RCLCPP_INFO(get_logger(),
          "SLAM aligned to UWB: offset_x=%.2f offset_y=%.2f",
          *offset_x, *offset_y);
      }

      if (!offset_x) return;  // wait until alignment

      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = "map";

      msg.pose.pose.position.x = x_slam + *offset_x;
      msg.pose.pose.position.y = y_slam + *offset_y;
      msg.pose.pose.position.z = tf.transform.translation.z;

      msg.pose.pose.orientation = tf.transform.rotation;

      msg.pose.covariance[0]  = 0.05 * 0.05;
      msg.pose.covariance[7]  = 0.05 * 0.05;
      msg.pose.covariance[35] = 0.1  * 0.1;

      pub->publish(msg);
    }
    catch (const tf2::TransformException &) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF map->base_link");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
