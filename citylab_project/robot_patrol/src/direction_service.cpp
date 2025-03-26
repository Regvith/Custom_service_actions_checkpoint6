#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
using namespace std;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("get_direction_service_node") {}

private:
  rclcpp::Service<>::SharedPtr
};