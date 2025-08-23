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
    this->action_server_ = rclcpp_action::create_server<GoToPoseActionMsg>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    // Declare the SubscriptionOptions and Initialize the subscriber
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

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex curr_pos_mtx_;

  // ROS callbacks
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseActionMsg::Goal> goal) {
    float x = goal->goal_pos.x;
    float y = goal->goal_pos.y;
    float theta = deg2rad(goal->goal_pos.theta);
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with desired goal pose x=%.3f , y=%.3f "
                ",theta=%.3f",
                x, y, theta);

    desired_pos_.x = x;
    desired_pos_.y = y;
    desired_pos_.theta = theta; // is now in radians
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

  //   void execute(const std::shared_ptr<GoalHandleGoToPoseActionMsg>
  //   goal_handle) {
  //     const auto goal = goal_handle->get_goal();

  //     auto feedback = std::make_shared<GoToPoseActionMsg::Feedback>();
  //     feedback->current_pos = current_pos_;

  //     auto result = std::make_shared<GoToPoseActionMsg::Result>();
  //     auto cmd = geometry_msgs::msg::Twist();
  //     rclcpp::Rate loop_rate(1);

  //     Pose2D diff_delta;
  //     diff_delta.x = desired_pos_.x - current_pos_.x;
  //     diff_delta.y = desired_pos_.y - current_pos_.y;
  //     diff_delta.theta = desired_pos_.theta - current_pos_.theta;
  //     float err_x = 0.0;
  //     float err_y = 0.0;
  //     float err_theta = 0.0;
  //     while (abs(diff_delta.x) > err_x && abs(diff_delta.y) > err_y &&
  //            abs(diff_delta.theta) > err_theta) {
  //       // Check if there is a cancel request
  //       if (goal_handle->is_canceling()) {
  //         result->status = false;
  //         goal_handle->canceled(result);
  //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //         return;
  //       }

  //       // Move the bot towards the desired pos
  //       cmd.linear.x = 0.2;
  //       cmd.angular.z = diff_delta.theta / 2.0;
  //       cmd_publisher_->publish(cmd);
  //       feedback->current_pos = current_pos_;
  //       goal_handle->publish_feedback(feedback);
  //       RCLCPP_INFO(this->get_logger(), "Publish feedback");
  //       loop_rate.sleep();
  //     }

  //     // Check if goal is done
  //     if (rclcpp::ok()) {
  //       result->status = true;
  //       cmd.linear.x = 0.0;
  //       cmd.angular.z = 0.0;
  //       cmd_publisher_->publish(cmd);
  //       goal_handle->succeed(result);
  //       RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  //       RCLCPP_INFO(this->get_logger(),
  //                   "Final robot pose after goal execution is x=%.3f , y=%.3f
  //                   "
  //                   ",theta=%.3f",
  //                   current_pos_.x, current_pos_.y, current_pos_.theta);
  //     }
  //   }

  void execute(const std::shared_ptr<GoalHandleGoToPoseActionMsg> goal_handle) {
    // Fixed control parameters (tune as needed)
    const double lin_speed = 0.2;     // m/s (fixed per requirement)
    const double k_ang = 1;           // P gain for heading
    const double max_ang_speed = 1.0; // rad/s clamp
    const double dist_tol = 0.1;      // 3 cm stop distance
    const double yaw_tol = 0.1;       // ~1.7 deg stop heading

    // Small helpers
    auto wrap_to_pi = [](double a) {
      // wrap angle to [-pi, pi]
      while (a > M_PI)
        a -= 2.0 * M_PI;
      while (a < -M_PI)
        a += 2.0 * M_PI;
      return a;
    };
    auto clamp = [](double v, double lo, double hi) {
      return std::max(lo, std::min(v, hi));
    };

    auto feedback = std::make_shared<GoToPoseActionMsg::Feedback>();
    auto result = std::make_shared<GoToPoseActionMsg::Result>();
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate loop_rate(10); // 10 Hz control loop

    while (rclcpp::ok()) {
      // --- 1) Read the latest robot pose (snapshot) ---
      Pose2D cur;
      // If you added a mutex for thread-safety, lock around this copy:
      {
        std::lock_guard<std::mutex> lk(curr_pos_mtx_);
        cur = current_pos_;
      }

      // --- 2) Compute vector and heading to goal ---
      const double dx = desired_pos_.x - cur.x;
      const double dy = desired_pos_.y - cur.y;
      const double dist_err = std::hypot(dx, dy); // how far to the goal (m)
      const double heading_to_goal = std::atan2(dy, dx); // where we should face
      const double yaw_err =
          wrap_to_pi(heading_to_goal - cur.theta); // how much to turn

      // --- 3) Stop condition ---
      if (dist_err < dist_tol && std::fabs(yaw_err) < yaw_tol) {
        break;
      }

      // --- 4) Handle cancel request ---
      if (goal_handle->is_canceling()) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_publisher_->publish(cmd);

        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // --- 5) Control law ---
      // Turn toward the goal proportional to heading error (P controller).
      double ang_cmd = clamp(k_ang * yaw_err, -max_ang_speed, max_ang_speed);
      //   float ang_cmd = yaw_err;

      // Move forward at fixed 0.2 m/s as requested.
      // (Optional: slow down if the heading error is big)
      double lin_cmd = lin_speed;
      if (std::fabs(yaw_err) >
          0.6) { // ~34 deg: reduce forward drift when badly misaligned
        lin_cmd *= 0.4;
      }

      cmd.linear.x = lin_cmd;
      cmd.angular.z = ang_cmd;
      cmd_publisher_->publish(cmd);

      // --- 6) Publish feedback (current pose) ---
      feedback->current_pos = cur;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // --- 7) Success: stop the robot and finish ---
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_publisher_->publish(cmd);

    result->status = true;
    goal_handle->succeed(result);

    // Log final pose
    Pose2D final_pose = current_pos_;
    RCLCPP_INFO(this->get_logger(),
                "Goal succeeded. Final pose x=%.3f y=%.3f th=%.3f",
                final_pose.x, final_pose.y, final_pose.theta);
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