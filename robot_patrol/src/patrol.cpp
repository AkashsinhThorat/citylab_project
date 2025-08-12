#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <functional> // std::bind, std::placeholders
#include <memory>     // std::shared_ptr, std::make_shared

using namespace std::chrono_literals;
using namespace std::placeholders;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {

    // Initialize the callback groups
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // Declare the SubscriptionOptions and Initialize the subscriber
    rclcpp::SubscriptionOptions laser_subscription_options;
    laser_subscription_options.callback_group = laser_callback_group_;
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, _1),
            laser_subscription_options);

    // Initialize the timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group_);

    // Initialize the publisher
    vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    (void)scan_msg;
    RCLCPP_INFO(this->get_logger(), "Scan Received on the Laser Callback");
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Firing the Timer Callback");
  }

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  float direction_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
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