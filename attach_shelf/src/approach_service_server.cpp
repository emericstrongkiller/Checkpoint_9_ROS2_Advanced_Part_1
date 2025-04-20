#include "custom_interfaces/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <string>

class FinalApproach : public rclcpp::Node {
public:
  FinalApproach(const std::string &approach_service_name)
      : Node("approach_service") {
    // Service definition
    approach_shelf_ = this->create_service<custom_interfaces::srv::GoToLoading>(
        approach_service_name,
        std::bind(&FinalApproach::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr
      approach_shelf_;

  void handle_approach_request(
      const custom_interfaces::srv::GoToLoading::Request::SharedPtr request,
      custom_interfaces::srv::GoToLoading::Response::SharedPtr response) {
    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Recieved message from service !");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalApproach>("/approach_shelf");
  rclcpp::spin(node);
  rclcpp::shutdown();
}