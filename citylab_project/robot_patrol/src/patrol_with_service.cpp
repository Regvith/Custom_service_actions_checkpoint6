#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using namespace std::chrono;
using namespace std;
class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service"), state_(FORWARD) {
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestService::laser_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&TestService::publish_cmd_vel, this));
  }

private:
  enum DIRECTIONS { FORWARD, RIGHT, LEFT };
  DIRECTIONS state_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available.");
      return;
    }

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
    } else if (direction == "RIGHT") {
      state_ = RIGHT;
    } else if (direction == "LEFT") {
      state_ = LEFT;
    }
  }

  void publish_cmd_vel() {
    geometry_msgs::msg::Twist cmd;
    // if (state_ == FORWARD) {
    //   cmd.linear.x = 0.1;
    //   cmd.angular.z = 0.0;
    // } else if (state_ == RIGHT) {
    //   cmd.linear.x = 0.1;
    //   cmd.angular.z = -0.5;
    // } else if (state_ == LEFT) {
    //   cmd.linear.x = 0.1;
    //   cmd.angular.z = 0.5;
    // }
    switch (state_) {
    case FORWARD:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
      pub_->publish(cmd);

      break;
    case RIGHT:
      cmd.linear.x = 0.1;
      cmd.angular.z = -0.5;
      pub_->publish(cmd);
      rclcpp::sleep_for(285ms);
      break;
    case LEFT:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.5;
      pub_->publish(cmd);
      rclcpp::sleep_for(285ms);
      break;
    }
    pub_->publish(cmd);
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
