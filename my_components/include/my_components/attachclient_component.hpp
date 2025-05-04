#ifndef COMPOSITION__CLIENT_COMPONENT_HPP_
#define COMPOSITION__CLIENT_COMPONENT_HPP_

#include "custom_interfaces/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void on_timer();

private:
  rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // COMPOSITION__CLIENT_COMPONENT_HPP_