#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <string>
#include <sstream>

using namespace std::chrono_literals;

class UwbPosNode : public rclcpp::Node {
public:
  UwbPosNode(const std::string & serial_port,
             unsigned int baud_rate)
    : Node("uwb_pos_node"),
      io_(),
      serial_(io_, serial_port)
  {
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    uwb_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10);
    start_read();
    io_thread_ = std::thread([this]() { io_.run(); });
  }

  ~UwbPosNode() {
    io_.stop();
    if (io_thread_.joinable()) io_thread_.join();
  }

private:
  void start_read() {
    boost::asio::async_read_until(serial_, buf_, '\n',
        boost::bind(&UwbPosNode::on_line_read, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
  }

  void on_line_read(const boost::system::error_code & ec,
                    std::size_t bytes_transferred)
  {
  	(void)bytes_transferred;
  	
    if (!ec) {
      std::istream is(&buf_);
      std::string line;
      std::getline(is, line);

      RCLCPP_DEBUG(this->get_logger(), "Received line: '%s'", line.c_str());

      double x=0.0, y=0.0, z=0.0;
      if (parse_xyz_line(line, x, y, z)) {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "uwb_tag";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = z;
        // leave covariance at default (zero) or set if known
        uwb_pub_->publish(msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not parse line: '%s'", line.c_str());
      }

      // Continue reading
      start_read();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error reading serial: %s", ec.message().c_str());
    }
  }

  bool parse_xyz_line(const std::string & line, double & x, double & y, double & z) {
    // Example: expect "x,y,z"
    std::istringstream ss(line);
    std::string sx, sy, sz;
    if (! std::getline(ss, sx, ',')) return false;
    if (! std::getline(ss, sy, ',')) return false;
    if (! std::getline(ss, sz, ',')) return false;
    try {
      x = std::stod(sx);
      y = std::stod(sy);
      z = std::stod(sz);
      return true;
    } catch (...) {
      return false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_pub_;
  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  boost::asio::streambuf buf_;
  std::thread io_thread_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  std::string port = "/dev/ttyUSB0";
  unsigned int baud = 115200;

  auto node = std::make_shared<UwbPosNode>(port, baud);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
