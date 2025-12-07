#include "rclcpp/rclcpp.hpp"
#include "your_package_name/msg/wheel_command.hpp"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardWheelTeleop : public rclcpp::Node {
public:
  KeyboardWheelTeleop() : Node("keyboard_wheel_teleop") {
    pub_ = create_publisher<your_package_name::msg::WheelCommand>("/wheel_cmd", 10);

    setup_terminal();

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&KeyboardWheelTeleop::update, this));

    RCLCPP_INFO(get_logger(),
                "Keyboard teleop started.\n"
                "W/S = throttle\n"
                "A/D = steering\n"
                "X   = stop\n"
                "Q   = quit");
  }

  ~KeyboardWheelTeleop() { restore_terminal(); }

private:
  void setup_terminal() {
    tcgetattr(STDIN_FILENO, &orig_term_);
    termios new_term = orig_term_;
    new_term.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  }

  void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term_);
  }

  void update() {
    char c;
    if (read(STDIN_FILENO, &c, 1) > 0) {
      if (c == 'q') {
        rclcpp::shutdown();
        return;
      } else if (c == 'w') throttle_ = std::min(1.0f, throttle_ + 0.1f);
      else if (c == 's') throttle_ = std::max(-1.0f, throttle_ - 0.1f);
      else if (c == 'a') steering_ = std::min(1.0f, steering_ + 0.1f);
      else if (c == 'd') steering_ = std::max(-1.0f, steering_ - 0.1f);
      else if (c == 'x') { throttle_ = 0; steering_ = 0; }

      RCLCPP_INFO(get_logger(), "Throttle: %.2f    Steering: %.2f",
                  throttle_, steering_);
    }

    publish_cmd();
  }

  void publish_cmd() {
    your_package_name::msg::WheelCommand msg;
    msg.throttle = throttle_;
    msg.steering = steering_;
    pub_->publish(msg);
  }

  rclcpp::Publisher<your_package_name::msg::WheelCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  termios orig_term_;

  float throttle_ = 0.0f;
  float steering_ = 0.0f;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardWheelTeleop>());
  rclcpp::shutdown();
  return 0;
}
