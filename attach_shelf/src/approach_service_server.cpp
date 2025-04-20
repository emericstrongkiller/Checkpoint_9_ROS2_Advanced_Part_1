#include "custom_interfaces/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <map>
#include <string>

class FinalApproach : public rclcpp::Node {
public:
  enum class State { SCANNING_SHELF };
  FinalApproach(const std::string &approach_service_name,
                const std::string &laser_sub_name)
      : Node("approach_service"), current_state_(State::SCANNING_SHELF) {
    // Service definition
    approach_shelf_ = this->create_service<custom_interfaces::srv::GoToLoading>(
        approach_service_name,
        std::bind(&FinalApproach::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Scan Subscriber definition
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_sub_name, 10,
        std::bind(&FinalApproach::laser_callback, this, std::placeholders::_1));
  }

private:
  void handle_approach_request(
      const custom_interfaces::srv::GoToLoading::Request::SharedPtr request,
      custom_interfaces::srv::GoToLoading::Response::SharedPtr response) {
    switch (current_state_) {
    case State::SCANNING_SHELF:
      scanning_shelf();
      break;
    }

    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Recieved message from service !");
  }

  void laser_callback(const sensor_msgs::msg::LaserScan msg) {
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      if (msg.intensities[i] > 0.0) {
        shelf_results["leg_1_position"] = i;
        shelf_results["leg_1_position"] = i + 1;
      }
    }
  }

  void scanning_shelf() {
    RCLCPP_INFO(this->get_logger(), "got shelf at %d and %d indexes",
                shelf_results["leg_1_position"],
                shelf_results["leg_2_position"]);
  }

  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  std::map<std::string, int> shelf_results;

  State current_state_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalApproach>("/approach_shelf", "/scan");
  rclcpp::spin(node);
  rclcpp::shutdown();
}