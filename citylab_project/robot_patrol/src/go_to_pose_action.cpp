#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_patrol/action/go_to_pose.hpp>
using namespace std;
class GoToPoseAction : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPose>;

  GoToPoseAction(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("go_to_pose_node", node_options) {
    this->cb_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_odom;
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 100,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
          if (first_odom) {
            pose_.x = msg->pose.pose.position.x;
            pose_.y = msg->pose.pose.position.y;
            pose_.theta = qtoe(msg->pose.pose.orientation);
            first_odom = false;
          }
          pose_.x = msg->pose.pose.position.x;
          pose_.y = msg->pose.pose.position.y;
          pose_.theta = qtoe(msg->pose.pose.orientation);
          //   if (pose_.theta > M_PI)
          //     pose_.theta -= 2 * M_PI;
          //   if (pose_.theta < -M_PI)
          //     pose_.theta += 2 * M_PI;
          pose_.theta = atan2(sin(pose_.theta), cos(pose_.theta));
        },
        options);

    action_server = rclcpp_action::create_server<GoToPose>(
        this, "/go_to_pose",
        std::bind(&GoToPoseAction::goal_response, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&GoToPoseAction::cancel_response, this,
                  std::placeholders::_1),
        std::bind(&GoToPoseAction::goal_accept, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
  }

private:
  double qtoe(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }
  rclcpp_action::GoalResponse
  goal_response(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const GoToPose::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancel_response(std::shared_ptr<GoalHandle> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void goal_accept(std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&GoToPoseAction::execute, this, goal_handle)}
        .detach();
  }

  void execute(std::shared_ptr<GoalHandle> goal_handle) {

    auto goal_ = goal_handle->get_goal();
    auto result = std::make_shared<GoToPose::Result>();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    move_forward = false;
    rotate_theta = false;
    break_out_of_loop = false;
    rclcpp::Rate rate(20);
    while (rclcpp::ok()) {

      distance = std::sqrt(std::pow((pose_.x - goal_->goal_pos.x), 2) +
                           std::pow((pose_.y - goal_->goal_pos.y), 2));
      double current_yaw = pose_.theta;
      feedback->current_pos = pose_;
      goal_handle->publish_feedback(feedback);
      double target_yaw =
          atan2((goal_->goal_pos.y - pose_.y), (goal_->goal_pos.x - pose_.x));
      target_yaw = atan2(sin(target_yaw), cos(target_yaw));
      double target_error = target_yaw - current_yaw;
      target_error = atan2(sin(target_error), cos(target_error));
      if (fabs(target_error) < 0.01) {
        cmd_vel.angular.z = 0; // Stop rotating
        move_forward = true;
        // break;
      } else {
        cmd_vel.angular.z = 0.5 * target_error; // Gradual rotation
      }
      if (move_forward) {
        cmd_vel.linear.x = 0.1;
      }
      if (fabs(distance) < 0.01) {
        move_forward = false;
        rotate_theta = true;
        cmd_vel.linear.x = 0;
        //   pub_->publish(cmd_vel);
        // break;
      }

      if (rotate_theta) {
        double current_yaw_theta = atan2(sin(pose_.theta), cos(pose_.theta));
        double target_theta = atan2(sin(goal_->goal_pos.theta * (M_PI / 180)),
                                    cos(goal_->goal_pos.theta * (M_PI / 180)));
        double error = atan2(sin(target_theta - current_yaw_theta),
                             cos(target_theta - current_yaw_theta));
        if (fabs(error) < 0.005) {

          cmd_vel.angular.z = 0;
          rotate_theta = false;
          break_out_of_loop = true;
          //  pub_->publish(cmd_vel);

          break;
        } else {
          cmd_vel.angular.z = error * 0.4;
        }
      }
      if (goal_handle->is_canceling()) {
        result->status = true;
        goal_handle->succeed(result);
        goal_handle->canceled(result);

        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        pub_->publish(cmd_vel);

        break;
      }
      pub_->publish(cmd_vel);
      if (break_out_of_loop)
        break;
      RCLCPP_INFO(this->get_logger(), "%f %f distance", distance, pose_.theta);
      RCLCPP_INFO(this->get_logger(), "Final Pose: x: %f, y: %f, theta: %f",
                  pose_.x, pose_.y, pose_.theta);
      RCLCPP_INFO(this->get_logger(), "x: %f y: %f theta: %f",
                  goal_->goal_pos.x, goal_->goal_pos.y, goal_->goal_pos.theta);
      rate.sleep();
    }
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_->publish(cmd_vel); // Stop movement

    RCLCPP_INFO(this->get_logger(), "Action Completed");
    result->status = true;
    goal_handle->succeed(result);
    break_out_of_loop = false;
  }
  rclcpp_action::Server<GoToPose>::SharedPtr action_server;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::CallbackGroup::SharedPtr cb_odom;
  geometry_msgs::msg::Pose2D pose_;
  geometry_msgs::msg::Twist cmd_vel;
  bool move_forward = false;
  double distance = 0;
  double angle = 0;
  double angle_orientation = 0;
  bool first_odom = true;
  bool rotate_theta = false;
  bool break_out_of_loop = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPoseAction>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
