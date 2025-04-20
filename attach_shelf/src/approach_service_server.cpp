#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <string>

class FinalApproach : public rclcpp::Node {
public:
  FinalApproach(const std::string &approach_service_name)
      : Node("approach_service") {
    // Service definition
    approach_shelf_ = this->create_service<std_srvs::srv::Trigger>(
        approach_service_name,
        std::bind(&FinalApproach::handle_approach_request, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr approach_shelf_;

  void handle_approach_request(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
    response->success = true;
    response->message = "message_recieved !";
    RCLCPP_INFO(this->get_logger(), "Recieved message from service !");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FinalApproach>("/approach_shelf");
  rclcpp::spin(node);
  rclcpp::shutdown();
}