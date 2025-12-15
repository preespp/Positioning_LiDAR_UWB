#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

class UWBSerialNode : public rclcpp::Node {
public:
  UWBSerialNode() : Node("uwb_serial_node") {

    publisher_ = this->create_publisher<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10);

    open_serial("/dev/ttyUSB1");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&UWBSerialNode::read_serial, this));

    RCLCPP_INFO(get_logger(), "UWB Serial Node started");
  }

  ~UWBSerialNode() {
    if (fd_ > 0) close(fd_);
  }

private:
  int fd_{-1};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void open_serial(const std::string &port) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
    }

    termios tty{};
    tcgetattr(fd_, &tty);

    cfsetospeed(&tty, B38400);
    cfsetispeed(&tty, B38400);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd_, TCSANOW, &tty);
  }

  void read_serial() {
    char buf[256];
    int n = read(fd_, buf, sizeof(buf) - 1);
    if (n <= 0) return;

    buf[n] = '\0';
    std::string line(buf);

    auto start = line.find('{');
    auto end   = line.find('}');
    if (start == std::string::npos || end == std::string::npos)
      return;

    line = line.substr(start + 1, end - start - 1);

    double x, y, rmse;
    if (!parse_json(line, x, y, rmse)) return;

    publish_pose(x, y, rmse);
  }

  bool parse_json(const std::string &s, double &x, double &y, double &rmse) {
    try {
      std::stringstream ss(s);
      std::string token;
      while (std::getline(ss, token, ',')) {
        auto sep = token.find(':');
        if (sep == std::string::npos) continue;

        std::string key = token.substr(0, sep);
        std::string val = token.substr(sep + 1);

        if (key.find("x") != std::string::npos) x = std::stod(val);
        if (key.find("y") != std::string::npos) y = std::stod(val);
        if (key.find("rmse") != std::string::npos) rmse = std::stod(val);
      }
    } catch (...) {
      return false;
    }
    return true;
  }

  void publish_pose(double x, double y, double rmse) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    // Simple covariance using RMSE
    double var = rmse * rmse;
    msg.pose.covariance[0]  = var;  // x
    msg.pose.covariance[7]  = var;  // y
    msg.pose.covariance[35] = 1e-3; // yaw (unknown)

    publisher_->publish(msg);

    RCLCPP_INFO(get_logger(),
      "UWB (x=%.2f, y=%.2f, rmse=%.2f)", x, y, rmse);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UWBSerialNode>());
  rclcpp::shutdown();
  return 0;
}
