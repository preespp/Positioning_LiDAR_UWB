#include "rclcpp/rclcpp.hpp"
#include "uwb_lidar/msg/wheel_command.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class WheelMotorNode : public rclcpp::Node {
public:
  WheelMotorNode() : Node("wheel_motor_node") {
    // open serial port
    serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open /dev/ttyUSB0");
    } else {
      configure_serial();
      RCLCPP_INFO(get_logger(), "Serial ready on /dev/ttyUSB0");
    }

    cmd_sub_ = this->create_subscription<uwb_lidar::msg::WheelCommand>(
        "/wheel_cmd", 10,
        std::bind(&WheelMotorNode::cmd_callback, this, std::placeholders::_1));
  }

  ~WheelMotorNode() {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  void configure_serial() {
    struct termios tty;
    tcgetattr(serial_fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tcsetattr(serial_fd_, TCSANOW, &tty);
  }

  void cmd_callback(const uwb_lidar::msg::WheelCommand::SharedPtr msg) {
    float throttle = msg->throttle;
    float steering = msg->steering;

    char buffer[64];
    int n = snprintf(buffer, sizeof(buffer), "%.3f %.3f\n", throttle, steering);

    if (serial_fd_ >= 0) {
      write(serial_fd_, buffer, n);
    }

    RCLCPP_INFO(this->get_logger(),
                "Sent to ESP32: throttle=%.2f steering=%.2f",
                throttle, steering);
  }

  int serial_fd_;
  rclcpp::Subscription<uwb_lidar::msg::WheelCommand>::SharedPtr cmd_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelMotorNode>());
  rclcpp::shutdown();
  return 0;
}
