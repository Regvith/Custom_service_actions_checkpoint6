#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

using namespace std;
using namespace chrono;

class DirectionService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  DirectionService() : Node("get_direction_service_node") {
    cb_server = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    server_ = this->create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_server);

    RCLCPP_INFO(this->get_logger(), "Service server is ready");
  }

private:
  void
  service_callback(const std::shared_ptr<GetDirection::Request> request,
                   const std::shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service request received");

    size_t range_size = request->laser_data.ranges.size();
    if (range_size < 330) {
      RCLCPP_ERROR(this->get_logger(), "Not enough laser data!");
      response->direction = "UNKNOWN";
      return;
    }

    int start_index_right = range_size / 4;
    int end_index_right = start_index_right + 110;
    int front_start_index = end_index_right + 1;
    int front_end_index = front_start_index + 110;
    int left_start_index = front_end_index + 1;
    int left_end_index = left_start_index + 110;

    double sum_left = 0, sum_front = 0, sum_right = 0;
    int count_left = 0, count_front = 0, count_right = 0;

    for (int i = start_index_right; i < left_end_index; i++) {
      if (!isinf(request->laser_data.ranges[i])) {
        if (i >= start_index_right && i < end_index_right) {
          sum_right += request->laser_data.ranges[i];
          count_right++;
        } else if (i >= front_start_index && i < front_end_index) {
          sum_front += request->laser_data.ranges[i];
          count_front++;
        } else if (i >= left_start_index && i < left_end_index) {
          sum_left += request->laser_data.ranges[i];
          count_left++;
        }
      }
    }

    double total_dist_sec_left = (count_left > 0) ? sum_left / count_left : 0;
    double total_dist_sec_front =
        (count_front > 0) ? sum_front / count_front : 0;
    double total_dist_sec_right =
        (count_right > 0) ? sum_right / count_right : 0;

    string direction;

    if (total_dist_sec_front <= 0.35) {
      if (total_dist_sec_right >= total_dist_sec_front &&
          total_dist_sec_right >= total_dist_sec_left) {
        direction = "RIGHT";
      } else if (total_dist_sec_front >= total_dist_sec_left &&
                 total_dist_sec_front >= total_dist_sec_right) {
        direction = "FORWARD";
      } else {
        direction = "LEFT";
      }
    } else {
      direction = "FORWARD";
    }

    RCLCPP_INFO(this->get_logger(),
                "Total Left: %.2f, Total Front: %.2f, Total Right: %.2f",
                total_dist_sec_left, total_dist_sec_front,
                total_dist_sec_right);
    RCLCPP_INFO(this->get_logger(), "Chosen Direction: %s", direction.c_str());

    response->direction = direction;
    RCLCPP_INFO(this->get_logger(), "Service request completed");
  }

  rclcpp::Service<GetDirection>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr cb_server;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DirectionService> node = std::make_shared<DirectionService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
