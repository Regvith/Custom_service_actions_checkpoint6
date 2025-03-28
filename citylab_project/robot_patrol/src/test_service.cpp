#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <future>

using namespace std::chrono_literals;

class TestServiceNode : public rclcpp::Node {
public:
  TestServiceNode() : Node("test_service_node"), service_done_(false) {
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "/direction_service");

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&TestServiceNode::laser_callback, this,
                  std::placeholders::_1));
  }

  bool is_service_done() const { return service_done_; }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service not available.");
      return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;

    auto future = client_->async_send_request(
        request,
        [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                   result) {
          RCLCPP_INFO(this->get_logger(), "Response: direction = %s",
                      result.get()->direction.c_str());
          service_done_ = true;
        });

    future.wait_for(1s);
  }

  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  bool service_done_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestServiceNode>();

  rclcpp::Rate rate(1);
  while (rclcpp::ok() && !node->is_service_done()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
