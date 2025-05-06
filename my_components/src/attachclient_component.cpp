#include "my_components/attachclient_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cinttypes>
#include <iostream>
#include <memory>

using namespace std::chrono_literals;
using ServiceResponseFuture =
    rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture;

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options), service_called_(false),
      service_answered_(false) {

  client_ =
      create_client<custom_interfaces::srv::GoToLoading>("approach_shelf");

  // Timer to initially call the service
  setup_timer_ =
      create_wall_timer(2s, std::bind(&AttachClient::call_service, this));

  // Timer to check for service results
  check_timer_ = create_wall_timer(
      500ms, std::bind(&AttachClient::check_service_answer, this));
}

void AttachClient::call_service() {
  if (service_called_) {
    // Already called the service, disable this timer
    setup_timer_->cancel();
    return;
  }

  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    return;
  }

  // Service is available, make the call
  auto request =
      std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
  // This client always asks for shelf attach (as permitted in checkpoint10
  // instructions)
  request->attach_to_shelf = true;

  RCLCPP_INFO(this->get_logger(), "CALLED Service with attach_to_shelf = %d",
              static_cast<int>(request->attach_to_shelf));

  service_future_ = client_->async_send_request(request).future.share();
  service_called_ = true;

  // After calling, cancel this timer as we only need to call once
  setup_timer_->cancel();
}

void AttachClient::check_service_answer() {
  if (!service_called_ || service_answered_) {
    return;
  }

  if (service_future_.wait_for(0s) == std::future_status::ready) {
    auto result = service_future_.get();

    if (result->complete) {
      RCLCPP_INFO(
          this->get_logger(),
          "Service completed successfully! Robot has approached the shelf.");
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Service failed: laser detected fewer than 2 legs, cannot "
                   "complete approach task.");
    }

    service_answered_ = true;
    check_timer_->cancel();
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
