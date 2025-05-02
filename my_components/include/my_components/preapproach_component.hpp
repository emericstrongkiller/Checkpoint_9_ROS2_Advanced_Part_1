#ifndef COMPOSITION__MOVEROBOT_COMPONENT_HPP_
#define COMPOSITION__MOVEROBOT_COMPONENT_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  enum class State {
    APPROACHING, // Moving toward obstacle
    TURNING,     // Turning to face the shelf
    COMPLETED    // Task completed
  };

  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &);

private:
  // Constants
  static constexpr int SCAN_CENTER_INDEX = 540;
  static constexpr double FORWARD_SPEED = 0.5;
  static constexpr double TURN_SPEED = 0.4;

  void initializeParameters();
  void scanCallback(const sensor_msgs::msg::LaserScan &msg);
  void handleApproachingState(const sensor_msgs::msg::LaserScan &msg);
  void handleTurningState();
  double getAngleTurned();
  double calculateTurnSpeed();
  void stopRobot();
  void odomCallback(const nav_msgs::msg::Odometry &msg);
  void timerCallback();

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher and timer
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Robot state
  State current_state_;
  geometry_msgs::msg::Twist cmd_msg_;

  // Configuration parameters (now read from ROS 2 parameters)
  std::string scan_topic_;
  std::string cmd_topic_;
  std::string odom_topic_;
  float obstacle_distance_;
  int turn_degrees_;

  // Current pose tracking
  double current_yaw_ = 0.0;
  double initial_yaw_ = 0.0;
};

} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_