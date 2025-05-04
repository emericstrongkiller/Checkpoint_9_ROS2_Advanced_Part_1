#ifndef COMPOSITION__SERVER_COMPONENT_HPP_
#define COMPOSITION__SERVER_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include <map>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

namespace my_components {

class AttachServer : public rclcpp::Node {
public:
  enum class State { MOVING_TO_SHELF, ADDITIONAL_30_CM, ATTACH_TO_SHELF };
  struct Point {
    float x;
    float y;
  };
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  void initializeParameters();
  void handle_approach_request(
      const custom_interfaces::srv::GoToLoading::Request::SharedPtr request,
      custom_interfaces::srv::GoToLoading::Response::SharedPtr response);
  void check_completion();
  void laser_callback(const sensor_msgs::msg::LaserScan &msg);
  void timer_callback();

  // constants
  static constexpr double DISTANCE_KP = 0.5;
  static constexpr double ANGLE_KP = 0.8;
  static constexpr double MAX_LINEAR_VEL = 0.4;
  static constexpr double MAX_ANGULAR_VEL = 0.4;
  static constexpr double DISTANCE_THRESHOLD = 0.2;
  static constexpr double ANGLE_THRESHOLD = 0.08;
  static constexpr double T_INTERVAL_30CM = 1.5;

  // service, publishers and subscribers
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr shelf_attach_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr completion_timer_;
  custom_interfaces::srv::GoToLoading::Response::SharedPtr current_response_;
  std::shared_ptr<rmw_request_id_t> current_request_header_;
  custom_interfaces::srv::GoToLoading::Request::SharedPtr current_request_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr service_callback_group;
  rclcpp::CallbackGroup::SharedPtr data_callback_group;

  // ADDITIONAL_30_CM variables
  double current_time;
  double last_triggered_time;
  bool last_trigger_done = false;

  // ATTACH_TO_SHELF_Variables
  bool attach_to_shelf_ = false;

  // shelf scanning logic
  bool failed_scanning = false;
  bool published_shelf_tf = false;

  // indications of end of states
  bool approach_done = false;
  bool service_called = false;
  bool shelf_attach_done = false;

  bool final_approach_done = false;
  bool process_done = false;
  bool done_scanning = false;

  // TF FOR OFFSET robot_base_link -> shelf_frame
  geometry_msgs::msg::TransformStamped transform_stamped;
  geometry_msgs::msg::TransformStamped odom_to_laser_stamped;
  geometry_msgs::msg::TransformStamped odom_to_cart_frame_stamped;

  // transforms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

  // shelf coordinates
  geometry_msgs::msg::PointStamped point_in_base_link;
  Point p_shelf;

  // cmd_vel order
  geometry_msgs::msg::Twist cmd_msg;

  State current_state_;

  std::string approach_service_name;
  std::string laser_sub_name;
  std::string cmd_vel_pub_name;
  std::string shelf_attach_pub_name;
};

} // namespace my_components

#endif // COMPOSITION__SERVER_COMPONENT_HPP_