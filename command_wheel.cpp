#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "your_package_name/action/run_robot.hpp"

class CommandControlNode : public rclcpp::Node {
public:
  using RunRobot = your_package_name::action::RunRobot;
  using GoalHandleRunRobot = rclcpp_action::ClientGoalHandle<RunRobot>;

  CommandControlNode() : Node("command_control_node") {
    client_ = rclcpp_action::create_client<RunRobot>(this, "/run_robot");
    send_goal();
  }

private:
  void send_goal() {
    if (!client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available.");
      return;
    }

    auto goal_msg = RunRobot::Goal();
    goal_msg.target_x = 2.0;
    goal_msg.target_y = 1.5;

    auto options = rclcpp_action::Client<RunRobot>::SendGoalOptions();
    options.feedback_callback = [this](GoalHandleRunRobot::SharedPtr,
                                       const std::shared_ptr<const RunRobot::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(), "Progress: %.1f%%", feedback->progress);
    };
    options.result_callback = [this](const GoalHandleRunRobot::WrappedResult &result) {
      RCLCPP_INFO(this->get_logger(), "Result: success=%d", result.result->success);
    };

    client_->async_send_goal(goal_msg, options);
  }

  rclcpp_action::Client<RunRobot>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandControlNode>());
  rclcpp::shutdown();
  return 0;
}
