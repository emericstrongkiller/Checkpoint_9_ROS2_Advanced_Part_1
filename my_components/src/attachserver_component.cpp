#include "my_components/attachserver_component.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("approach_service", options),
      current_state_(State::MOVING_TO_SHELF) {
  // Initialize parameters
  initializeParameters();

  // TF initialization
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Callback groups
  service_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  data_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = data_callback_group;

  // Service definition
  approach_shelf_ = this->create_service<custom_interfaces::srv::GoToLoading>(
      approach_service_name,
      std::bind(&AttachServer::handle_approach_request, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group);

  // Scan Subscriber definition
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      laser_sub_name, 10,
      std::bind(&AttachServer::laser_callback, this, std::placeholders::_1),
      sub_options);

  // Cmd_vel publisher definition
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_pub_name, 10);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&AttachServer::timer_callback, this),
      data_callback_group);

  // shelf attach publisher definition
  shelf_attach_pub =
      this->create_publisher<std_msgs::msg::String>(shelf_attach_pub_name, 10);
}

void AttachServer::initializeParameters() {
  approach_service_name = "/approach_shelf";
  laser_sub_name = "/scan";
  cmd_vel_pub_name = "/diffbot_base_controller/cmd_vel_unstamped";
  shelf_attach_pub_name = "/elevator_up";
}

void AttachServer::handle_approach_request(
    const custom_interfaces::srv::GoToLoading::Request::SharedPtr request,
    custom_interfaces::srv::GoToLoading::Response::SharedPtr response) {
  current_state_ = State::MOVING_TO_SHELF;
  service_called = true;
  attach_to_shelf_ = request->attach_to_shelf;
  current_response_ = response;

  current_response_->complete = true;

  completion_timer_ = create_wall_timer(
      100ms, std::bind(&AttachServer::check_completion, this));
}

void AttachServer::check_completion() {
  if (!current_response_) {
    return;
  }

  if (current_state_ == State::ATTACH_TO_SHELF && shelf_attach_done) {
    current_response_->complete = true;
    completion_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Approach service now complete");
  }

  if (failed_scanning) {
    current_response_->complete = false;
    completion_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Approach service failed");
  }
}

void AttachServer::laser_callback(const sensor_msgs::msg::LaserScan &msg) {
  if (!approach_done && service_called) {
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
    // save current cluster if the whole intensities[] passed without ending
    // the cluster
    if (!current.empty()) {
      clusters.push_back(current);
      current.clear();
    }

    // clusters results
    if (clusters.size() < 2 || clusters.size() > 2) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Found %zu clusters instead of 2, no frame created, "
                   "returning failed signal..",
                   clusters.size());
      failed_scanning = true;
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

      // 1. Get the transform from laser frame to odom (at latest time)
      geometry_msgs::msg::TransformStamped laser_to_odom;
      try {
        // Get the latest transform from laser to odom
        laser_to_odom = tf_buffer_->lookupTransform(
            "odom",                   // target frame
            msg.header.frame_id,      // source frame
            tf2::TimePointZero,       // get latest transform
            tf2::durationFromSec(0.5) // timeout
        );

        // angle correction
        // float shelf_angle = atan2(y2 - y1, x2 - x1);
        float shelf_angle = atan2(x2 - x1, y2 - y1) * (-1);
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, shelf_angle);

        // 2. Create a pose stamped for the shelf in the laser frame
        geometry_msgs::msg::PoseStamped pose_in_laser;
        pose_in_laser.header.frame_id = msg.header.frame_id;
        pose_in_laser.header.stamp =
            this->get_clock()->now(); // Use CURRENT time
        pose_in_laser.pose.position.x = p_shelf.x;
        pose_in_laser.pose.position.y = p_shelf.y;
        pose_in_laser.pose.position.z = 0.0;
        pose_in_laser.pose.orientation.x = q.x();
        pose_in_laser.pose.orientation.y = q.y();
        pose_in_laser.pose.orientation.z = q.z();
        pose_in_laser.pose.orientation.w = q.w();

        // 3. Transform the pose to odom
        geometry_msgs::msg::PoseStamped pose_in_odom;
        tf2::doTransform(pose_in_laser, pose_in_odom, laser_to_odom);

        // 4. Create the transform from odom to cart_frame
        geometry_msgs::msg::TransformStamped transform_odom_to_cart;
        transform_odom_to_cart.header.stamp = this->get_clock()->now();
        transform_odom_to_cart.header.frame_id = "odom";
        transform_odom_to_cart.child_frame_id = "cart_frame";
        transform_odom_to_cart.transform.translation.x =
            pose_in_odom.pose.position.x;
        transform_odom_to_cart.transform.translation.y =
            pose_in_odom.pose.position.y;
        transform_odom_to_cart.transform.translation.z =
            pose_in_odom.pose.position.z;
        transform_odom_to_cart.transform.rotation =
            pose_in_odom.pose.orientation;

        // 5. Broadcast the transform
        static_tf_broadcaster->sendTransform(transform_odom_to_cart);

        // let the client know the transform was brodcasted successfully
        published_shelf_tf = true;

        rclcpp::sleep_for(200ms);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to odom: %s",
                    msg.header.frame_id.c_str(), ex.what());
        rclcpp::sleep_for(100ms);
      }
    }
  }
  if (current_state_ == State::MOVING_TO_SHELF && attach_to_shelf_) {
    try {
      // Get offset between shelf's TF and robot_chassis's TF
      geometry_msgs::msg::TransformStamped transform_stamped2 =
          tf_buffer_->lookupTransform("cart_frame", "robot_base_link",
                                      tf2::TimePointZero,
                                      tf2::durationFromSec(0.5));

      // translation offsets
      float trans_x = transform_stamped2.transform.translation.x;
      float trans_y = transform_stamped2.transform.translation.y;

      // rotation offsets with quaternion
      tf2::Quaternion q2(transform_stamped2.transform.rotation.x,
                         transform_stamped2.transform.rotation.y,
                         transform_stamped2.transform.rotation.z,
                         transform_stamped2.transform.rotation.w);
      // Getting euler angles from quaternion
      tf2::Matrix3x3 m2(q2);
      double roll, pitch, yaw;
      m2.getRPY(roll, pitch, yaw);

      // Determine the cmd_vel values based on cart_frame <->
      // robot_base_link offsets
      float cart_distance = sqrt(pow(trans_x, 2) + pow(trans_y, 2));

      // print offsets (DEBUG)
      RCLCPP_DEBUG(this->get_logger(), "Distance_offset: %f", cart_distance);
      RCLCPP_DEBUG(this->get_logger(), "Angle_offset: %f", yaw);

      if (cart_distance < DISTANCE_THRESHOLD) {
        if (abs(yaw) < ANGLE_THRESHOLD) {
          // ADD OTHER IF HERE TO TEST THE 30CM INCREMENT
          cmd_msg.linear.x = 0.0;
          cmd_msg.angular.z = 0.0;
          // approach_done !!!!
          approach_done = true;
          last_triggered_time =
              this->get_clock()->now().seconds(); // Set timer here
          // GO TO THE NEXT STEP
          current_state_ = State::ADDITIONAL_30_CM;
          RCLCPP_INFO(
              this->get_logger(),
              "MOVING_TO_SHELF Finished, Transitionning to ADDITIONAL_30_CM");
        }
        // when at the right distance from the shelf's frame origin, orient
        // the robot correctly with respect to shelf's frame orientation
        // before moving to the next state
        else {
          cmd_msg.linear.x = 0.0;
          cmd_msg.angular.z = std::min(MAX_ANGULAR_VEL, yaw * ANGLE_KP * (-1));
        }

      } else if (acos(-trans_x / cart_distance) > 0.01) {
        cmd_msg.linear.x =
            std::min(MAX_LINEAR_VEL, cart_distance * DISTANCE_KP);
        // Calculation of the orientation offset between the chassis and
        // the Point of origin of the shelf To make the robot point to the
        // shelf's origin.
        float trans_x = transform_stamped2.transform.translation.x;
        cmd_msg.angular.z =
            std::min(MAX_ANGULAR_VEL, acos(-trans_x / cart_distance) * (-1.0));
      } else {
      }
    }
    // catch error if cart_frame isn't broadcasted yet (which is the case in
    // the beginning as we've just created it and brodcasted it above in
    // this code)
    catch (const tf2::LookupException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not transform robot_base_link to cart_frame: %s",
                  ex.what());
      rclcpp::sleep_for(100ms);
    }
  } else if (current_state_ == State::ADDITIONAL_30_CM && attach_to_shelf_) {
    // move forward for T_INTERVAL_30CM = (1) seconds at 0.3m/s => move 30cm
    // in a non blocking code
    approach_done = true;
    current_time = this->get_clock()->now().seconds();
    if (current_time < last_triggered_time + T_INTERVAL_30CM) {
      cmd_msg.linear.x = 0.4; // Move forward until time passes
    } else {
      cmd_msg.linear.x = 0.0; // Then stop
      RCLCPP_INFO(this->get_logger(),
                  "ADDITIONAL_30_CM Finished, going to lift shelf");
      current_state_ = State::ATTACH_TO_SHELF;
    }

  } else if (current_state_ == State::ATTACH_TO_SHELF && attach_to_shelf_) {
    // stop robot AGAIN
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    // attach to shelf if client asked
    std_msgs::msg::String msg;
    msg.data = "";
    shelf_attach_pub->publish(msg);
    shelf_attach_done = true;
  }
}

void AttachServer::timer_callback() {
  if (attach_to_shelf_) {
    // send cmd to robot IF THE SERVICE HAS BEEN CALLED
    cmd_vel_pub_->publish(cmd_msg);
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)