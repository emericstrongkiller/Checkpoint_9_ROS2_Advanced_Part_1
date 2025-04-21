#include "custom_interfaces/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/point_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <map>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using namespace std::chrono_literals;

class FinalApproach : public rclcpp::Node {
public:
  enum class State { SCANNING_SHELF, HANDLING_SHELF_TF };
  struct Point {
    float x = 0.0f;
    float y = 0.0f;
  };
  FinalApproach(const std::string &approach_service_name,
                const std::string &laser_sub_name,
                const std::string &cmd_vel_pub_name)
      : Node("approach_service"), current_state_(State::SCANNING_SHELF) {
    // TF initialization
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Service definition
    approach_shelf_ = this->create_service<custom_interfaces::srv::GoToLoading>(
        approach_service_name,
        std::bind(&FinalApproach::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Scan Subscriber definition
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_sub_name, 10,
        std::bind(&FinalApproach::laser_callback, this, std::placeholders::_1));

    // Cmd_vel publisher definition
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_pub_name, 10);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&FinalApproach::timer_callback, this));
  }

private:
  void handle_approach_request(
      const custom_interfaces::srv::GoToLoading::Request::SharedPtr request,
      custom_interfaces::srv::GoToLoading::Response::SharedPtr response) {
    switch (current_state_) {
    case State::SCANNING_SHELF:
      if (p_shelf.x == 0.0f && p_shelf.y == 0.0f) {
        RCLCPP_WARN(
            this->get_logger(),
            "not enough legs || too much legs detected, service failed.");
        response->complete = false;
      }
      break;
    case State::HANDLING_SHELF_TF:
      handling_shelf_tf();
      break;
    }

    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Recieved message from service !");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan &msg) {
    std::vector<std::vector<int>> clusters;
    std::vector<int> current;

    // check all laser intensities
    for (size_t i = 0; i < msg.intensities.size(); i++) {
      // register all indexes for each clusters
      if (msg.intensities[i] > 0.0) {
        current.push_back(i);
      }
      // register each cluster in a cluster vector
      else {
        if (!current.empty()) {
          clusters.push_back(current);
          current.clear();
        }
      }
    }
    // save current cluster if the whole intensities[] passed without ending the
    // cluster
    if (!current.empty()) {
      clusters.push_back(current);
      current.clear();
    }

    // clusters results
    if (clusters.size() < 2 || clusters.size() > 2) {
      RCLCPP_WARN(this->get_logger(),
                  "Found %zu clusters instead of 2, no data will be forwarded "
                  "to handling_shelf_tf()..",
                  clusters.size());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "found %zu clusters !", clusters.size());
      // get the index at the middle of the cluster
      size_t p1_index =
          (clusters[0][0] + clusters[0][clusters[0].size() - 1]) / 2;
      size_t p2_index =
          (clusters[1][0] + clusters[1][clusters[1].size() - 1]) / 2;

      // show clusters (DEBUG)
      for (size_t i = 0; i < clusters.size(); i++) {
        for (size_t j = 0; j < clusters[i].size(); j++) {
          RCLCPP_DEBUG(this->get_logger(), "cluster [%zu] n°%zu value: %d", i,
                       j, clusters[i][j]);
        }
      }

      // show shelf legs found (DEBUG)
      RCLCPP_DEBUG(this->get_logger(), "cluster 1 at: %zu index", p1_index);
      RCLCPP_DEBUG(this->get_logger(), "cluster 2 at: %zu index", p2_index);

      // calclate shelf's legs point coordinates
      float p1_angle = msg.angle_min + p1_index * msg.angle_increment;
      float p1_range = msg.ranges[p1_index];
      float x1 = p1_range * cos(p1_angle);
      float y1 = p1_range * sin(p1_angle);

      float p2_angle = msg.angle_min + p2_index * msg.angle_increment;
      float p2_range = msg.ranges[p2_index];
      float x2 = p2_range * cos(p2_angle);
      float y2 = p2_range * sin(p2_angle);

      // calculate midpoint between shelf's legs and store it
      p_shelf.x = (x1 + x2) / 2.0;
      p_shelf.y = (y1 + y2) / 2.0;

      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.frame_id = msg.header.frame_id;
      transform_stamped.header.stamp = msg.header.stamp;
      transform_stamped.child_frame_id = "cart_frame";
      transform_stamped.transform.translation.x = p_shelf.x;
      transform_stamped.transform.translation.y = p_shelf.y;
      transform_stamped.transform.translation.z = 0.0;

      // angle correction
      // float shelf_angle = atan2(y2 - y1, x2 - x1);
      float shelf_angle = atan2(x2 - x1, y2 - y1) * (-1);
      tf2::Quaternion q;
      q.setRPY(0, 0, shelf_angle);

      // no rotation for now.. (apply later maybe)
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform_stamped);

      // Getting to the next state !
      current_state_ = State::HANDLING_SHELF_TF;
    }
  }

  void handling_shelf_tf() {}

  void timer_callback() { // calculate angle and distance offset from robot
                          // base_link to target point
    float angle_offset =
        atan2(point_in_base_link.point.y, point_in_base_link.point.x);
    float distance_offset = sqrt(pow(point_in_base_link.point.x, 2) +
                                 pow(point_in_base_link.point.y, 2));
    RCLCPP_INFO(this->get_logger(), "Distance_offset: %f", distance_offset);
    RCLCPP_INFO(this->get_logger(), "Angle_offset: %f", angle_offset);

    // define command based on offsets and user defined proportionnal
    // coefficients
    if (distance_offset < DISTANCE_THRESHOLD) {
      // ADD OTHER IF HERE TO TEST THE 30CM INCREMENT
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
    } else {
      cmd_msg.linear.x = std::min(MAX_LINEAR_VEL, distance_offset * ANGLE_KP);
      cmd_msg.angular.z = std::min(MAX_ANGULAR_VEL, angle_offset * DISTANCE_KP);
    }

    // send cmd to robot
    cmd_vel_pub_->publish(cmd_msg);
  }

  // constants
  static constexpr double DISTANCE_KP = 1.0;
  static constexpr double ANGLE_KP = 1.0;
  static constexpr double MAX_LINEAR_VEL = 0.5;
  static constexpr double MAX_ANGULAR_VEL = 0.5;
  static constexpr double DISTANCE_THRESHOLD = 0.05;
  static constexpr double ANGLE_THRESHOLD = 0.05;

  // service, publishers and subscribers
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // transforms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  // tf broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // shelf coordinates
  geometry_msgs::msg::PointStamped point_in_base_link;
  Point p_shelf;

  // cmd_vel order
  geometry_msgs::msg::Twist cmd_msg;

  State current_state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalApproach>(
      "/approach_shelf", "/scan", "/diffbot_base_controller/cmd_vel_unstamped");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
