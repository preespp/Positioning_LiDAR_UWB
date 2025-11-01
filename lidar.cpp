#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarNode : public rclcpp::Node {
public:
  LidarNode() : Node("lidar_node") {
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&LidarNode::publish_scan, this));
  }

private:
  void publish_scan() {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->now();
    scan.header.frame_id = "lidar";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 0.01;
    scan.range_min = 0.2;
    scan.range_max = 10.0;
    scan.ranges.resize(315, 2.0); // Dummy data
    lidar_pub_->publish(scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
