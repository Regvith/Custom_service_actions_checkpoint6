#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

std::shared_ptr<rclcpp::Client<robot_patrol::srv::GetDirection>> client;

void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(rclcpp::get_logger("test_service"), "Service not available.");
    return;
  }

  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
  request->laser_data = *msg;

  auto future = client->async_send_request(
      request,
      [](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture result) {
        RCLCPP_INFO(rclcpp::get_logger("test_service"),
                    "Response: direction = %s",
                    result.get()->direction.c_str());
        // rclcpp::shutdown();
      });
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_service");

  client = node->create_client<robot_patrol::srv::GetDirection>(
      "/direction_service");

  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, laser_callback);

  rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}
