#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol_node") {

    subscriber_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // QoS: LaserScan is sensor data (low-latency, best-effort)
    auto laserscan_qos = rclcpp::SensorDataQoS();

    //  minimal backlog fro smd_qos:
    auto cmd_qos = rclcpp::QoS(1).reliable();

    // Initalizing the subscriber
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = subscriber_callback_group;
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", laserscan_qos,
        std::bind(&Patrol::scan_topic_callback, this, std::placeholders::_1),
        sub_options);

    // Initializing the Publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);

    // Initializing the timer object
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group);
  }

private:
  // Class Attributes
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist vel_msg;
  std::vector<float> curr_scan;
  float direction_;
  float angle_min_;
  float angle_increment_ = 0.0;
  rclcpp::CallbackGroup::SharedPtr subscriber_callback_group;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group;

  //  Class Methods
  void scan_topic_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    angle_min_ = msg->angle_min;
    angle_increment_ = msg->angle_increment;

    int scan_size = static_cast<int>(msg->ranges.size());
    int start_idx = static_cast<int>((-M_PI_2 - angle_min_) / angle_increment_);
    int end_idx = static_cast<int>((+M_PI_2 - angle_min_) / angle_increment_);

    start_idx = std::max(0, std::min(start_idx, scan_size - 1));
    end_idx = std::max(0, std::min(end_idx, scan_size - 1));

    curr_scan.assign(msg->ranges.begin() + start_idx,
                     msg->ranges.begin() + end_idx + 1);
    // RCLCPP_INFO(this->get_logger(),
    //             "LaserScan information received from the Robot!");
  }

  void set_orientation() {
    RCLCPP_INFO(this->get_logger(), "Finding the Safest Orientation!");
    float curr_largest_dist = 0.0;
    int curr_largest_idx = -1;

    for (size_t idx = 0; idx < curr_scan.size(); idx++) {
      if (std::isfinite(this->curr_scan[idx]) &&
          curr_largest_dist <= this->curr_scan[idx]) {
        curr_largest_dist = this->curr_scan[idx];
        curr_largest_idx = idx;
      }
    }

    if (curr_largest_idx >= 0) {
      float relative_angle =
          -M_PI_2 + curr_largest_idx * this->angle_increment_;
      this->direction_ = relative_angle;
      RCLCPP_INFO(this->get_logger(), "Safest direction angle: %.2f radians",
                  this->direction_);
    } else {
      RCLCPP_WARN(this->get_logger(), "No valid laser readings found!");
      this->direction_ = 0.0;
    }
  }

  void timer_callback() {
    // RCLCPP_INFO(this->get_logger(), "Publishing commands to move the
    // Robot!");

    if (this->curr_scan.empty()) {
      RCLCPP_WARN(this->get_logger(), "No scan data yet.");
      return;
    }

    // Get scan indices for -30° to +30° (−π/6 to +π/6)
    int scan_size = static_cast<int>(curr_scan.size());
    int start_idx_rel = static_cast<int>(
        std::round(((-M_PI / 6) + M_PI / 2) / angle_increment_));
    int end_idx_rel = static_cast<int>(
        std::round(((+M_PI / 6) + M_PI / 2) / angle_increment_));

    // Clamp indices to valid range
    start_idx_rel = std::max(0, std::min(start_idx_rel, scan_size - 1));
    end_idx_rel = std::max(0, std::min(end_idx_rel, scan_size - 1));

    // Find minimum distance in the front 60° sector
    float min_dist = std::numeric_limits<float>::infinity();
    for (int i = start_idx_rel; i <= end_idx_rel; ++i) {
      if (std::isfinite(curr_scan[i]) && curr_scan[i] < min_dist) {
        min_dist = curr_scan[i];
      }
    }

    RCLCPP_INFO(this->get_logger(), "Closest obstacle in front %.2f : %.2f m",
                this->direction_, min_dist);

    if (min_dist > 0.35) {
      this->vel_msg.angular.z = 0.0;
    } else {
      set_orientation();
      this->vel_msg.angular.z = (this->direction_ / 2.0);
      if (this->vel_msg.angular.z > 0.0) {
        RCLCPP_INFO(this->get_logger(), "Turning Left!");
      } else {
        RCLCPP_INFO(this->get_logger(), "Turning Right!");
      }
    }

    this->vel_msg.linear.x = 0.1;
    this->publisher_->publish(this->vel_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto patrol_node = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}