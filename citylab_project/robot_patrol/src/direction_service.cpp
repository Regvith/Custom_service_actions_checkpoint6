#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <memory>

class DirectionService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  DirectionService() : Node("get_direction_service_node") {
    cb_server =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    server_ = this->create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_server);
  }

private:
  void
  service_callback(const std::shared_ptr<GetDirection::Request> request,
                   const std::shared_ptr<GetDirection::Response> response) {
    size_t range_size = request->laser_data.ranges.size();
    RCLCPP_INFO(this->get_logger(), "Laser data size: %ld", range_size);

    // Example logic (modify as needed)
    response->direction = "FORWARD";
  }

  rclcpp::Service<GetDirection>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr cb_server;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DirectionService> node = std::make_shared<DirectionService>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}