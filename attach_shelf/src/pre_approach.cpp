#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;

class Pre_approach : public rclcpp::Node {
public:
  Pre_approach(std::string scan_topic, std::string cmd_topic)
      : Node("pre_approach") {
    // Obstacle parameter setup
    auto obstacle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    obstacle_param_desc.description =
        "Sets the distance to the obstacle at which the robot will stop";
    this->declare_parameter<float>("obstacle", 1.0, obstacle_param_desc);

    // Degrees parameter setup
    auto degrees_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    degrees_param_desc.description =
        "Sets the amount of degrees the robot should turn to face the shelf";
    this->declare_parameter<float>("degrees", 90.0, degrees_param_desc);

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 10,
        std::bind(&Pre_approach::Scan_callback, this, std::placeholders::_1));
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&Pre_approach::Cmd_callback, this));
  }

  void Scan_callback(const sensor_msgs::msg::LaserScan msg) {
    this->get_parameter("velocity", obstacle);
    if (msg.ranges[360] < obstacle) {
      cmd_msg.linear.x = 0;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0;
      RCLCPP_INFO(this->get_logger(), "STOPPING, below %f of the obstacle",
                  obstacle);
    } else {
      cmd_msg.linear.x = 0.6;
      cmd_msg.linear.y = 0;
      cmd_msg.angular.z = 0;
    }
  }
  void Cmd_callback() {
    RCLCPP_INFO(this->get_logger(),
                "publishing command: linear_x: %f, linear_y: %f, angular_z: "
                "%f, to cmd_vel",
                cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z);
    cmd_pub->publish(cmd_msg);
  }

private:
  std::string scan_topic;
  std::string cmd_topic;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

  sensor_msgs::msg::LaserScan scan_data;
  geometry_msgs::msg::Twist cmd_msg;

  rclcpp::TimerBase::SharedPtr timer_;

  float obstacle;
  float degrees;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Pre_approach> pre_approach_node =
      std::make_shared<Pre_approach>("/scan", "/cmd_vel");
  rclcpp::spin(pre_approach_node);
  rclcpp::shutdown();
  return 0;
}