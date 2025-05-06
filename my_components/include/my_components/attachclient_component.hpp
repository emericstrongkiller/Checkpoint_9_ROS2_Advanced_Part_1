#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <future>
#include <memory>

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  void call_service();
  void check_service_answer();

  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr setup_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;

  std::shared_future<
      std::shared_ptr<custom_interfaces::srv::GoToLoading::Response>>
      service_future_;
  bool service_called_;
  bool service_answered_;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
