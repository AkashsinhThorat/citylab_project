#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using GoToPoseActionMsg = robot_patrol::action::GoToPose;
using GoalHandleGoToPoseActionMsg =
    rclcpp_action::ServerGoalHandle<GoToPoseActionMsg>;
using Pose2D = geometry_msgs::msg::Pose2D;

class GoToPose : public rclcpp::Node {
public:
  GoToPose() : Node("action_server_node") {
    using namespace std::placeholders;

    // Initialize the Action server
    this->action_server_ = rclcpp_action::create_server<GoToPoseActionMsg>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));
    RCLCPP_INFO(this->get_logger(), "Action Server Ready!!");

    // Initialize a mutually exclusive clalback group, Declare the
    // SubscriptionOptions and Initialize the subscriber
    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_subscription_options;
    odom_subscription_options.callback_group = odom_callback_group_;
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1),
        odom_subscription_options);

    // Initialize the publisher
    cmd_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  // class variables
  Pose2D desired_pos_;
  Pose2D current_pos_;
  rclcpp_action::Server<GoToPoseActionMsg>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  std::mutex curr_pos_mtx_;

  // ROS callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseActionMsg::Goal> goal) {
    float x = goal->goal_pos.x;
    float y = goal->goal_pos.y;
    float theta = deg2rad(goal->goal_pos.theta);
    RCLCPP_INFO(this->get_logger(), "Action Called \n");
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with desired goal pose x=%.3f , y=%.3f "
                ",theta=%.3f",
                x, y, theta);

    desired_pos_.x = x;
    desired_pos_.y = y;
    desired_pos_.theta = theta;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleGoToPoseActionMsg> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleGoToPoseActionMsg> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPoseActionMsg> goal_handle) {
    auto result = std::make_shared<GoToPoseActionMsg::Result>();
    auto feedback = std::make_shared<GoToPoseActionMsg::Feedback>();
    geometry_msgs::msg::Twist cmd;

    // Tunable Parmeters
    const double LIN_SPEED = 0.20;  // m/s
    const double K_ANG_GOAL = 1.2;  // P gain (face waypoint)
    const double K_ANG_ALIGN = 1.2; // P gain (final yaw align)
    const double MAX_W = 1.0;       // rad/s clamp
    const double DIST_TOL = 0.05;   // 5 cm
    const double YAW_TOL = 0.05;    // 0.05 radians

    // Helpful lambdas
    auto clamp = [](double v, double lo, double hi) {
      return std::max(lo, std::min(v, hi));
    };
    auto wrap_to_pi = [](double a) {
      while (a > M_PI)
        a -= 2.0 * M_PI;
      while (a < -M_PI)
        a += 2.0 * M_PI;
      return a;
    };

    rclcpp::Rate rate(10);                   // 10 Hz for control loop
    rclcpp::Time last_fb_time = this->now(); // feedback throttle (1 Hz)

    // ------------------------------
    // Phase 1: drive to (x,y)
    // ------------------------------
    while (rclcpp::ok()) {
      Pose2D cur;
      {
        std::lock_guard<std::mutex> lk(curr_pos_mtx_);
        cur = current_pos_;
      }

      const double dx = desired_pos_.x - cur.x;
      const double dy = desired_pos_.y - cur.y;
      const double dist_err = std::hypot(dx, dy);

      if (dist_err < DIST_TOL) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_publisher_->publish(cmd);
        break;
      }

      // Face the waypoint (also we must guard atan2 near zero)
      const double heading_to_goal =
          (dist_err > 1e-6) ? std::atan2(dy, dx) : cur.theta;
      const double yaw_err = wrap_to_pi(heading_to_goal - cur.theta);

      if (goal_handle->is_canceling()) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_publisher_->publish(cmd);
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // Control
      double ang_cmd = clamp(K_ANG_GOAL * yaw_err, -MAX_W, MAX_W);
      double lin_cmd = LIN_SPEED;
      if (std::fabs(yaw_err) > 0.6)
        lin_cmd *= 0.4; // slowdown if misaligned for more than 0.6 rad

      cmd.linear.x = lin_cmd;
      cmd.angular.z = ang_cmd;
      cmd_publisher_->publish(cmd);

      // Feedback @ 1 Hz
      auto now = this->now();
      if ((now - last_fb_time).seconds() >= 1.0) {
        feedback->current_pos = cur;
        goal_handle->publish_feedback(feedback);
        last_fb_time = now;
      }

      rate.sleep();
    }

    // ------------------------------
    // Phase 2: align final yaw
    // ------------------------------
    while (rclcpp::ok()) {
      Pose2D cur;
      {
        std::lock_guard<std::mutex> lk(curr_pos_mtx_);
        cur = current_pos_;
      }

      const double yaw_err = wrap_to_pi(desired_pos_.theta - cur.theta);
      if (std::fabs(yaw_err) < YAW_TOL)
        break;

      if (goal_handle->is_canceling()) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_publisher_->publish(cmd);
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      cmd.linear.x = 0.0;
      cmd.angular.z = clamp(K_ANG_ALIGN * yaw_err, -MAX_W, MAX_W);
      cmd_publisher_->publish(cmd);

      // Feedback @ 1 Hz
      auto now = this->now();
      if ((now - last_fb_time).seconds() >= 1.0) {
        feedback->current_pos = cur;
        goal_handle->publish_feedback(feedback);
        last_fb_time = now;
      }

      rate.sleep();
    }

    // Stop & succeed
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_publisher_->publish(cmd);
    result->status = true;
    goal_handle->succeed(result);

    Pose2D final_pose;
    {
      std::lock_guard<std::mutex> lk(curr_pos_mtx_);
      final_pose = current_pos_;
    }
    RCLCPP_INFO(this->get_logger(),
                "Goal succeeded. Final pose x=%.3f y=%.3f th=%.3f(rad)",
                final_pose.x, final_pose.y, final_pose.theta);
    RCLCPP_INFO(this->get_logger(), "Action Completed !!");
  }

  void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> odom_msg) {

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    quatToEulerRPY(odom_msg->pose.pose.orientation, roll, pitch, yaw);

    std::lock_guard<std::mutex> lk(curr_pos_mtx_);
    current_pos_.x = odom_msg->pose.pose.position.x;
    current_pos_.y = odom_msg->pose.pose.position.y;
    current_pos_.theta = yaw;
  }

  // helper functions

  // Converts a geometry_msgs quaternion to roll/pitch/yaw (radians).
  void quatToEulerRPY(const geometry_msgs::msg::Quaternion &q, double &roll,
                      double &pitch, double &yaw) {
    tf2::Quaternion tfq(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tfq).getRPY(roll, pitch, yaw);
  }

  //   // Converts radians to degrees
  //   float rad2deg(float radians) { return radians * (180.0 / M_PI); }

  // Converts degrees to radians
  float deg2rad(float degrees) { return degrees * (M_PI / 180.0); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto action_server_node = std::make_shared<GoToPose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}