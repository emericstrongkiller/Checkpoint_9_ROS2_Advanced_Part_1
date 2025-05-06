#include "my_components/preapproach_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace my_components {
PreApproach::PreApproach(const rclcpp::NodeOptions &)
    : Node("pre_approach"), current_state_(State::APPROACHING) {
  // Initialize parameters
  initializeParameters();

  // Create subscriptions
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&PreApproach::scanCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PreApproach::odomCallback, this, std::placeholders::_1));

  // Create publisher and timer
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&PreApproach::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "PreApproach node initialized. Moving forward until %.2f m "
              "from the obstacle!",
              obstacle_distance_);
}

void PreApproach::initializeParameters() {
  scan_topic_ = "/scan";
  cmd_topic_ = "/diffbot_base_controller/cmd_vel_unstamped";
  odom_topic_ = "/diffbot_base_controller/odom";
  obstacle_distance_ = 0.4;
  turn_degrees_ = -90;
}

void PreApproach::scanCallback(
    const sensor_msgs::msg::LaserScan &msg) { // Safety check for index bounds
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
  case State::COMPLETED:
    // Keep the robot stopped
    stopRobot();
    break;
  }
}

void PreApproach::handleApproachingState(
    const sensor_msgs::msg::LaserScan &msg) {
  if (msg.ranges[SCAN_CENTER_INDEX] < obstacle_distance_) {
    // Obstacle reached, transition to turning state
    current_state_ = State::TURNING;
    initial_yaw_ = current_yaw_;

    RCLCPP_INFO(this->get_logger(),
                "Obstacle detected at %.2f m. Turning %d degrees...",
                msg.ranges[SCAN_CENTER_INDEX], turn_degrees_);

    // Stop and prepare to turn
    stopRobot();
    cmd_msg_.angular.z = calculateTurnSpeed();
  } else {
    // Continue moving forward
    cmd_msg_.linear.x = FORWARD_SPEED;
  }
}

void PreApproach::handleTurningState() {
  // Check if we've reached our target angle
  double angle_turned = getAngleTurned();
  double target_angle = std::abs(turn_degrees_ * M_PI / 180.0);

  if (angle_turned >= target_angle) {
    // Turning completed
    current_state_ = State::COMPLETED;
    RCLCPP_INFO(this->get_logger(), "Turn completed (%.2f degrees) !!",
                angle_turned * 180.0 / M_PI);
    stopRobot();
  }
}

double PreApproach::getAngleTurned() {
  double delta = std::abs(current_yaw_ - initial_yaw_);
  // Handle wrap-around (when crossing +/-PI boundary)
  if (delta > M_PI) {
    delta = 2.0 * M_PI - delta;
  }
  return delta;
}

double PreApproach::calculateTurnSpeed() {
  // Return speed with appropriate sign based on turn direction
  return TURN_SPEED * (turn_degrees_ > 0 ? 1.0 : -1.0);
}

void PreApproach::stopRobot() {
  cmd_msg_.linear.x = 0.0;
  cmd_msg_.linear.y = 0.0;
  cmd_msg_.angular.z = 0.0;
}

void PreApproach::odomCallback(const nav_msgs::msg::Odometry &msg) {
  tf2::Quaternion q;
  q.setX(msg.pose.pose.orientation.x);
  q.setY(msg.pose.pose.orientation.y);
  q.setZ(msg.pose.pose.orientation.z);
  q.setW(msg.pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw_);
}

void PreApproach::timerCallback() { cmd_pub_->publish(cmd_msg_); }

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)