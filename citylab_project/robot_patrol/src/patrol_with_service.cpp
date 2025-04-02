#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono;
using namespace std;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("test_service"), state_(FORWARD) {
    // Creating callback groups for concurrency control
    this->cb_laser = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opt;
    sub_opt.callback_group = cb_laser;

    this->timer_cb = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    client_cb = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Creating service client
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service", rmw_qos_profile_services_default, client_cb);

    // Subscribing to LaserScan topic
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&PatrolWithService::laser_callback, this,
                  std::placeholders::_1),
        sub_opt);

    // Creating publisher for cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer to publish velocity commands
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(400),
        std::bind(&PatrolWithService::publish_cmd_vel, this), timer_cb);
  }

private:
  enum DIRECTIONS { FORWARD, RIGHT, LEFT };
  DIRECTIONS state_;
  DIRECTIONS last_state_ = FORWARD;

  bool laser_data_received_ = false;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_data_received_ = true;

    front_sum = 0.0;
    // left_side = msg->ranges[450];  // Left side reading
    // right_side = msg->ranges[250]; // Right side reading
    // front_sum = msg->ranges[330];
    int front_count = 0;
    // for (int i = 385; i <= 275; i--) { // Front sector (60°)
    //   if (!std::isinf(msg->ranges[i])) {
    //     front_sum += msg->ranges[i];
    //     front_count++;
    //   }
    // }
    // front_sum = 0.0;
    // int front_count = 0;

    for (int i = 300; i <= 360; i++) {
      if (!std::isinf(msg->ranges[i])) {
        front_sum += msg->ranges[i];
        front_count++;
      }
    }
    // for (int i = 265; i <= 360; i++) {
    //   if (!std::isinf(msg->ranges[i])) {
    //     front_sum += msg->ranges[i];
    //     front_count++;
    //   }
    // }
    // // Take the average if there are valid values
    // if (front_count > 0) {
    //   front_sum /= front_count;
    // }

    RCLCPP_INFO(this->get_logger(), "front %f ", front_sum);
    if (front_sum <= 35) {
      auto request =
          std::make_shared<robot_patrol::srv::GetDirection::Request>();
      request->laser_data = *msg;
      auto future = client_->async_send_request(
          request, std::bind(&PatrolWithService::service_response_callback,
                             this, std::placeholders::_1));
    }
  }
  void service_response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result) {
    std::string direction = result.get()->direction;
    RCLCPP_INFO(this->get_logger(), "Response: direction = %s",
                direction.c_str());

    update_state(direction);
  }

  void update_state(const std::string &direction) {
    // if (direction == "FORWARD") {
    //   if (state_ == RIGHT || state_ == LEFT) {
    //     RCLCPP_INFO(this->get_logger(), "Ignoring premature FORWARD
    //     state"); return;
    //   }
    //   state_ = FORWARD;
    // }

    if (direction == "RIGHT") {
      state_ = RIGHT;
      first_forward_call = false;
      bias = -1.1;
    }
    if (direction == "LEFT") {
      state_ = LEFT;
      first_forward_call = false;
      bias = 1.1;
    }
  }

  void publish_cmd_vel() {
    if (!laser_data_received_) {
      RCLCPP_WARN(this->get_logger(),
                  "No laser data received yet. Stopping robot.");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    // if (state_ != FORWARD) { // If currently turning, check if we can go
    // forward
    // Read side distances
    // float left_side = msg->ranges[450];  // Left side reading
    // float right_side = msg->ranges[250]; // Right side reading

    // Only go forward if both front and side are clear
    // if (state_ == RIGHT && front_sum >= 0.35 && right_side >= 0.4) {
    //   state_ = FORWARD;
    // } else if (state_ == LEFT && front_sum >= 0.35 && left_side >= 0.4) {
    //   state_ = FORWARD;
    // }
    // }
    if (front_sum > 35) {
      last_state_ = state_;
      state_ = FORWARD;
    }
    switch (state_) {
    case FORWARD:
      cmd.linear.x = 0.1;
      if (first_forward_call) {
        cmd.angular.z = 0;
        first_forward_call = false;
      } else if (first_forward_call)
        cmd.angular.z = 0;
      else if (last_state_ != FORWARD) {
        cmd.angular.z = bias * 0.65; // turn offset to set it straight
        first_turn_call = true;
        last_state_ = FORWARD;
      } // pub_->publish(cmd);
      else if (first_turn_call) {
        cmd.angular.z = bias * 0.0185; // drift velocity offset for
        correction
      }
      cmd.angular.z = 0;

      break;
    case RIGHT:
      cmd.linear.x = 0.1; // Reduce speed while turning
      cmd.angular.z = -0.5;
      // pub_->publish(cmd);
      //   if (delay_counter_ > 0) {
      //     delay_counter_--;
      //   } else {
      //     state_ = FORWARD; // Only switch after delay is done
      //   }
      break;
    case LEFT:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.5;
      //  pub_->publish(cmd);
      //   if (delay_counter_ > 0) {
      //     delay_counter_--;
      //   } else {
      //     state_ = FORWARD; // Only switch after delay is done
      //   }
      break;
    }

    pub_->publish(cmd);
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int delay_counter_ = 0;
  rclcpp::CallbackGroup::SharedPtr cb_laser;
  rclcpp::CallbackGroup::SharedPtr timer_cb;
  rclcpp::CallbackGroup::SharedPtr client_cb;
  std::string direction;
  bool first_forward_call = true;
  int bias = 1;
  float front_sum = 0;
  float left_side = 0;
  float right_side = 0;
  bool first_turn_call = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolWithService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
