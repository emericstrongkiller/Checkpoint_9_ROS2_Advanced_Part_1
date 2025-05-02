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

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)