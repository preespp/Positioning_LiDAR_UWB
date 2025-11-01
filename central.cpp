#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "your_package_name/action/run_robot.hpp"  // Change to actual package name

class CentralNode : public rclcpp::Node {
public:
  using RunRobot = your_package_name::action::RunRobot;
  using GoalHandleRunRobot = rclcpp_action::ServerGoalHandle<RunRobot>;

  CentralNode() : Node("central_node") {
    // Subscribers
    uwb_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/uwb/position", 10, std::bind(&CentralNode::uwb_callback, this, std::placeholders::_1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&CentralNode::lidar_callback, this, std::placeholders::_1));
    
    // Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Action Server
    action_server_ = rclcpp_action::create_server<RunRobot>(
        this,
        "/run_robot",
        std::bind(&CentralNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CentralNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&CentralNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Central node started.");
  }

private:
  void uwb_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "UWB position: [%.2f, %.2f]", msg->pose.pose.position.x, msg->pose.pose.position.y);
    // TODO: Add fusion logic here
  }

  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    (void)msg;
    // TODO: Add collision detection / mapping
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &,
                                          std::shared_ptr<const RunRobot::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%.2f", goal->target_x, goal->target_y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRunRobot> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRunRobot> goal_handle) {
    std::thread{std::bind(&CentralNode::execute_goal, this, goal_handle)}.detach();
  }

  void execute_goal(const std::shared_ptr<GoalHandleRunRobot> goal_handle) {
    auto feedback = std::make_shared<RunRobot::Feedback>();
    auto result = std::make_shared<RunRobot::Result>();

    rclcpp::Rate rate(10);
    for (int i = 0; i <= 100 && rclcpp::ok(); i++) {
      feedback->progress = i;
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal execution completed.");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr uwb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp_action::Server<RunRobot>::SharedPtr action_server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CentralNode>());
  rclcpp::shutdown();
  return 0;
}
