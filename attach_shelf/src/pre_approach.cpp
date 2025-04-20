#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class Pre_approach : public rclcpp::Node {
public:
  Pre_approach(std::string scan_topic, std::string cmd_topic,
               std::string odom_topic)
      : Node("pre_approach"), recorded_obstacle_triggered_yaw(false) {
    // Obstacle parameter setup
    auto obstacle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    obstacle_param_desc.description =
        "Sets the distance to the obstacle at which the robot will stop";
    this->declare_parameter<float>("obstacle", 1.0, obstacle_param_desc);
    this->get_parameter("obstacle", obstacle);

    // Degrees parameter setup
    auto degrees_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    degrees_param_desc.description =
        "Sets the amount of degrees the robot should turn to face the shelf";
    this->declare_parameter<float>("degrees", 90.0, degrees_param_desc);
    this->get_parameter("degrees", degrees);

    // subscribers assignation
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        std::bind(&Pre_approach::Scan_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&Pre_approach::Odom_callback, this, std::placeholders::_1));

    // publisher assignation
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Pre_approach::Cmd_callback, this));
  }

  void Scan_callback(const sensor_msgs::msg::LaserScan msg) {
    if (!obstacle_reached) {
      cmd_msg.linear.x = 0.5;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0;
      if (msg.ranges[360] < obstacle) {
        obstacle_reached = true;
        RCLCPP_INFO(
            this->get_logger(),
            "Below %.2f m of the obstacle, GOING TO TURN for %.2f degrees",
            obstacle, degrees);
      }
    } else if (obstacle_reached && !angle_reached) {
      if (!recorded_obstacle_triggered_yaw) {
        obstacle_triggered_yaw = yaw;
        recorded_obstacle_triggered_yaw = true;
      }
      cmd_msg.linear.x = 0;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0.4 * (degrees / abs(degrees));
      if (abs(obstacle_triggered_yaw - yaw) > abs(degrees * M_PI / 180)) {
        angle_reached = true;
        RCLCPP_INFO(this->get_logger(), "Angle reached ! Shutting down..");
        rclcpp::shutdown();
      }
    } else {
      cmd_msg.linear.x = 0;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0;
    }
  }

  void Odom_callback(const nav_msgs::msg::Odometry msg) {
    q.setX(msg.pose.pose.orientation.x);
    q.setY(msg.pose.pose.orientation.y);
    q.setZ(msg.pose.pose.orientation.z);
    q.setW(msg.pose.pose.orientation.w);

    m.setRotation(q);
    m.getRPY(roll, pitch, yaw);
  }

  void Cmd_callback() {
    if (!moving_forward_msg) {
      RCLCPP_INFO(this->get_logger(),
                  "Moving forward until %.2f m from the obstacle !", obstacle);
      moving_forward_msg = true;
    }
    cmd_pub->publish(cmd_msg);
  }

private:
  std::string scan_topic;
  std::string cmd_topic;
  std::string odom_topic;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

  sensor_msgs::msg::LaserScan scan_data;
  geometry_msgs::msg::Twist cmd_msg;
  nav_msgs::msg::Odometry odom_buffer;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2::Quaternion q;
  tf2::Matrix3x3 m;
  double roll, pitch, yaw;
  bool recorded_obstacle_triggered_yaw = false;

  bool obstacle_reached = false;
  bool angle_reached = false;
  bool moving_forward_msg = false;

  float obstacle;
  float obstacle_triggered_yaw;
  float degrees;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Pre_approach> pre_approach_node =
      std::make_shared<Pre_approach>(
          "/scan", "/diffbot_base_controller/cmd_vel_unstamped",
          "/diffbot_base_controller/odom");
  rclcpp::spin(pre_approach_node);
  rclcpp::shutdown();
  return 0;
}