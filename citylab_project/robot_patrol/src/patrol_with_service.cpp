#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono;
using namespace std;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service"), state_(FORWARD) {
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
        std::bind(&TestService::laser_callback, this, std::placeholders::_1),
        sub_opt);

    // Creating publisher for cmd_vel
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer to publish velocity commands
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&TestService::publish_cmd_vel, this),
        timer_cb);
  }

private:
  enum DIRECTIONS { FORWARD, RIGHT, LEFT };
  DIRECTIONS state_;
  bool laser_data_received_ = false;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_data_received_ = true;

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;

    auto future = client_->async_send_request(
        request, std::bind(&TestService::service_response_callback, this,
                           std::placeholders::_1));
  }

  void service_response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result) {
    std::string direction = result.get()->direction;
    RCLCPP_INFO(this->get_logger(), "Response: direction = %s",
                direction.c_str());

    update_state(direction);
  }

  void update_state(const std::string &direction) {

    if (direction == "FORWARD") {
      state_ = FORWARD;
      // delay_counter_ = 0;
    } else if (direction == "RIGHT") {
      state_ = RIGHT;
      // delay_counter_ = 0; // Ensure a full turn before switching
    } else if (direction == "LEFT") {
      state_ = LEFT;
      //  delay_counter_ = 0; // Ensure a full turn before switching
    }
  }

  void publish_cmd_vel() {
    if (!laser_data_received_) {
      RCLCPP_WARN(this->get_logger(),
                  "No laser data received yet. Stopping robot.");
      return;
    }

    geometry_msgs::msg::Twist cmd;

    switch (state_) {
    case FORWARD:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
      pub_->publish(cmd);
      break;
    case RIGHT:
      cmd.linear.x = 0.1; // Reduce speed while turning
      cmd.angular.z = -0.8;
      pub_->publish(cmd);
      //   if (delay_counter_ > 0) {
      //     delay_counter_--;
      //   } else {
      //     state_ = FORWARD; // Only switch after delay is done
      //   }
      break;
    case LEFT:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.8;
      pub_->publish(cmd);
      //   if (delay_counter_ > 0) {
      //     delay_counter_--;
      //   } else {
      //     state_ = FORWARD; // Only switch after delay is done
      //   }
      break;
    }

    //  pub_->publish(cmd);
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int delay_counter_ = 0;
  rclcpp::CallbackGroup::SharedPtr cb_laser;
  rclcpp::CallbackGroup::SharedPtr timer_cb;
  rclcpp::CallbackGroup::SharedPtr client_cb;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
