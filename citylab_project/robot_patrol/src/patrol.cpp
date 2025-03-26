#include "cmath"
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <vector>
using namespace std;
using namespace std::chrono;
class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    if (init_service)
      rclcpp::sleep_for(2s);
    init_service = false;
    this->cb_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->cb_scan = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->cb_timer = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_odom;
    options_odom.callback_group = cb_odom;
    rclcpp::SubscriptionOptions options_scan;
    options_scan.callback_group = cb_scan;

    this->pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    this->sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 100,
        std::bind(&Patrol::scan_callback, this, std::placeholders::_1),
        options_scan);
    this->timer = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), cb_timer);
    this->sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, bind(&Patrol::odom_callback, this, placeholders::_1),
        options_odom);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
    int counter = 0;
    int start_index = msg->ranges.size() / 4;
    int end_index = 3 * msg->ranges.size() / 4;
    if (front_value_init) {
      front_value = msg->ranges[330];
      front_value_init = false;
    }
    float max_value = 0;
    int max_angle_index = -1;

    int max_expected_points = end_index - start_index;
    scan_msg.resize(max_expected_points);
    scan_index.resize(max_expected_points);
    scan_angle.resize(max_expected_points);
    front_value = msg->ranges[330];

    for (int i = start_index; i < end_index; i++) {
      if (!std::isinf(msg->ranges[i])) {
        scan_msg.at(counter) = msg->ranges[i];
        scan_angle.at(counter) = msg->angle_min + i * msg->angle_increment;
        scan_angle.at(counter) =
            atan2(sin(scan_angle.at(counter)), cos(scan_angle.at(counter)));
        scan_index.at(counter) = i;
        counter++;
      }
    }
    for (long unsigned int i = 0; i < scan_msg.size(); i++) {
      if (scan_msg.at(i) > max_value) {
        max_value = scan_msg.at(i);
        max_angle_index = i;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Max range value:%f index:%d", max_value,
                max_angle_index);
    if (max_angle_index != -1) {
      direction_ = scan_angle[max_angle_index];
      RCLCPP_INFO(this->get_logger(), "Max angle value:%f index:%d", direction_,
                  max_angle_index);
    }
    // auto min_it = std::min_element(scan_angle.begin(), scan_angle.end());
    // if (min_it != scan_angle.end()) {
    //   double min_value = *min_it;
    //   RCLCPP_INFO(this->get_logger(), "Scan min: %.2f | Indices collected:
    //   %ld",
    //               min_value, scan_index.size());
    // } else {
    //   RCLCPP_WARN(this->get_logger(), "No valid scan data found.");
    // }

    // Optional: trim if you only want valid points
    scan_msg.resize(counter);
    scan_index.resize(counter);
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f", current_yaw);

    // rclcpp::sleep_for(1s);
    RCLCPP_INFO(this->get_logger(), "Front Value: %f %ld", front_value,
                scan_index.size());
    counter = 0;
    // rclcpp::sleep_for(1s);
  }
  double qtoy(const geometry_msgs::msg::Quaternion &q) {
    return atan2(2 * (q.w * q.z + q.x * q.y),
                 1 - 2 * (pow(q.x, 2) + pow(q.z, 2)));
  }
  void timer_callback() {
    switch (state_) {
    case MOVE:
      if (front_value < 0.35) {
        target_yaw = current_yaw + direction_;
        target_yaw = atan2(sin(target_yaw), cos(target_yaw));
        move.angular.z = 0; // offset for drifting
        move.linear.x = 0;
        pub_->publish(move);
        state_ = ROTATE;
      }
      if (direction_ > 0)
        bias = 1;
      if (direction_ < 0)
        bias = -1;

      move.angular.z = bias * 0.014; // offset for drifting

      move.linear.x = 0.1;
      pub_->publish(move);

      break;
    case ROTATE:
      double yaw_error = target_yaw - current_yaw;
      // yaw_error = atan2(sin(yaw_error), cos(yaw_error)); // Normalize
      yaw_error = atan2(sin(yaw_error), cos(yaw_error));
      if (fabs(yaw_error) < 0.035) {
        move.angular.z = 0.0;
        pub_->publish(move);
        state_ = MOVE;
      }
      // move.linear.x = 0.0;

      move.angular.z = (direction_ / 2);
      pub_->publish(move);

      break;
    }
  }
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) {
    const auto &qmsg = odom_msg->pose.pose.orientation;
    current_yaw = qtoy(qmsg);
    if (current_yaw > M_PI)
      current_yaw -= 2 * M_PI;
    if (current_yaw < -M_PI)
      current_yaw += 2 * M_PI;
    RCLCPP_DEBUG(this->get_logger(), "Yaw: %f", current_yaw);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::TimerBase::SharedPtr timer;
  geometry_msgs::msg::Twist move;
  rclcpp::CallbackGroup::SharedPtr cb_odom;
  rclcpp::CallbackGroup::SharedPtr cb_scan;
  rclcpp::CallbackGroup::SharedPtr cb_timer;
  float bias_offset = 0.1;
  vector<float> scan_msg{};
  double front_value;
  bool front_value_init = true;
  bool front_flag = false;
  double current_yaw = 0;
  vector<int> scan_index{};
  vector<float> scan_angle{};
  float direction_ = 0;
  double target_yaw;
  enum ROBOT_STATE { MOVE, ROTATE };
  ROBOT_STATE state_ = MOVE;
  bool init_service = true;
  int bias = 1;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> node = std::make_shared<Patrol>();
  // rclcpp::spin(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}
