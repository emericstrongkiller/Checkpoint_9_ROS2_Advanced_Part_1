#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class PreApproach : public rclcpp::Node {
public:
  // Robot states
  enum class State {
    APPROACHING, // Moving toward obstacle
    TURNING,     // Turning to face the shelf
    CALLING_SERVICE
  };

  PreApproach(const std::string &scan_topic, const std::string &cmd_topic,
              const std::string &odom_topic, const std::string &service_name)
      : Node("pre_approach"), current_state_(State::APPROACHING) {
    // Initialize parameters
    initializeParameters();

    // Create approach service client
    approach_service_client_ =
        this->create_client<custom_interfaces::srv::GoToLoading>(service_name);

    // Create subscriptions
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));

    // Create publisher and timer
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&PreApproach::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "PreApproach node initialized. Moving forward until %.2f m "
                "from the obstacle!",
                obstacle_distance_);
  }

private:
  // Constants
  static constexpr int SCAN_CENTER_INDEX = 540;
  static constexpr double FORWARD_SPEED = 0.5;
  static constexpr double TURN_SPEED = 0.4;

  void initializeParameters() {
    // Obstacle parameter
    auto obstacle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    obstacle_param_desc.description =
        "Distance to the obstacle at which the robot will stop (meters)";
    this->declare_parameter<float>("obstacle", 1.0, obstacle_param_desc);
    this->get_parameter("obstacle", obstacle_distance_);

    // Degrees parameter
    auto degrees_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    degrees_param_desc.description =
        "Amount of degrees the robot should turn to face the shelf";
    this->declare_parameter<float>("degrees", 90.0, degrees_param_desc);
    this->get_parameter("degrees", turn_degrees_);

    // final approach parameter
    auto approach_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    approach_param_desc.description =
        "Boolean to choose if the robot moves under shelf and lifts it or not";
    this->declare_parameter<bool>("final_approach", false, approach_param_desc);
    this->get_parameter("final_approach", attach_to_shelf);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan &msg) {
    // Safety check for index bounds
    if (SCAN_CENTER_INDEX >= msg.ranges.size()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Scan index %d is out of bounds (size: %zu)!",
                   SCAN_CENTER_INDEX, msg.ranges.size());
      return;
    }

    switch (current_state_) {
    case State::APPROACHING:
      handleApproachingState(msg);
      break;
    case State::TURNING:
      handleTurningState();
      break;
    case State::CALLING_SERVICE:
      if (!service_called) {
        approach_service_call();
      }
      break;
    }
  }

  void handleApproachingState(const sensor_msgs::msg::LaserScan &msg) {
    if (msg.ranges[SCAN_CENTER_INDEX] < obstacle_distance_) {
      // Obstacle reached, transition to turning state
      current_state_ = State::TURNING;
      initial_yaw_ = current_yaw_;

      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected at %.2f m. Turning %.2f degrees...",
                  msg.ranges[SCAN_CENTER_INDEX], turn_degrees_);

      // Stop and prepare to turn
      stopRobot();
      cmd_msg_.angular.z = calculateTurnSpeed();
    } else {
      // Continue moving forward
      cmd_msg_.linear.x = FORWARD_SPEED;
    }
  }

  void handleTurningState() {
    // Check if we've reached our target angle
    double angle_turned = getAngleTurned();
    double target_angle = std::abs(turn_degrees_ * M_PI / 180.0);

    if (angle_turned >= target_angle) {
      // Turning completed
      current_state_ = State::CALLING_SERVICE;
      RCLCPP_INFO(this->get_logger(), "Turn completed (%.2f degrees)",
                  angle_turned * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "current_state_: %d",
                  static_cast<int>(current_state_));

      stopRobot();
    }
  }

  double getAngleTurned() {
    double delta = std::abs(current_yaw_ - initial_yaw_);
    // Handle wrap-around (when crossing +/-PI boundary)
    if (delta > M_PI) {
      delta = 2.0 * M_PI - delta;
    }
    return delta;
  }

  double calculateTurnSpeed() {
    // Return speed with appropriate sign based on turn direction
    return TURN_SPEED * (turn_degrees_ > 0 ? 1.0 : -1.0);
  }

  void stopRobot() {
    cmd_msg_.linear.x = 0.0;
    cmd_msg_.linear.y = 0.0;
    cmd_msg_.angular.z = 0.0;
  }

  void odomCallback(const nav_msgs::msg::Odometry &msg) {
    tf2::Quaternion q;
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
  }

  void approach_service_call() {
    auto request =
        std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
    request->attach_to_shelf = attach_to_shelf;
    approach_service_client_->async_send_request(request);

    RCLCPP_INFO(this->get_logger(), "attach_to_shelf: %d",
                request->attach_to_shelf);

    service_called = true;
  }

  void timerCallback() {
    if (current_state_ != State::CALLING_SERVICE) {
      // send commands UNTIL service has been called
      cmd_pub_->publish(cmd_msg_);
    }
  }

  // approach service client
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_service_client_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher and timer
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Robot state
  State current_state_;
  geometry_msgs::msg::Twist cmd_msg_;

  // Configuration parameters
  float obstacle_distance_;
  float turn_degrees_;
  bool attach_to_shelf;

  // rservice call parameters
  bool service_called = false;

  // Current pose tracking
  double current_yaw_ = 0.0;
  double initial_yaw_ = 0.0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PreApproach>(
      "/scan", "/diffbot_base_controller/cmd_vel_unstamped",
      "/diffbot_base_controller/odom", "/approach_shelf");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
