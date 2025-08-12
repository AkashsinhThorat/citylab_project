#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol()
      : rclcpp::Node("patrol_node"), direction_{0.0f},
        obstacle_threshold_m_{0.35f}, linear_speed_{0.10f} {
    // Optional parameters (topic names & loop period)
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ =
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    control_period_ms_ =
        this->declare_parameter<int>("control_period_ms", 100); // 10 Hz

    // Callback groups
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // QoS: LaserScan is sensor data (low-latency, best-effort)
    auto laser_qos = rclcpp::SensorDataQoS();

    //  minimal backlog fro smd_qos:
    auto cmd_qos = rclcpp::QoS(1).reliable();

    // Subscriber
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = laser_callback_group_;
    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, laser_qos,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
            sub_options);

    // Publisher
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, cmd_qos);

    // Timer (control loop)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_period_ms_),
        std::bind(&Patrol::timer_callback, this), timer_callback_group_);

    RCLCPP_INFO(
        get_logger(), "Patrol running: scan='%s', cmd_vel='%s', period=%dms",
        scan_topic_.c_str(), cmd_vel_topic_.c_str(), control_period_ms_);
  }

private:
  // --- Core patrol logic helpers ---

  // Clamp helper to keep angles within [-pi/2, pi/2]
  static double clamp_to_half_pi(double angle) {
    const double half_pi = M_PI_2; // pi/2
    if (angle > half_pi)
      return half_pi;
    if (angle < -half_pi)
      return -half_pi;
    return angle;
  }

  // Convert a target angle (rad) to a scan index, clamped into valid range
  static int angle_to_index(double angle, double angle_min,
                            double angle_increment, int size) {
    const int idx =
        static_cast<int>(std::round((angle - angle_min) / angle_increment));
    return std::max(0, std::min(size - 1, idx));
  }

  // Compute min finite distance around the front (±window_rad around 0 rad)
  static float min_front_distance(const sensor_msgs::msg::LaserScan &scan,
                                  double window_rad) {
    const int n = static_cast<int>(scan.ranges.size());
    if (n == 0 || scan.angle_increment == 0.0)
      return std::numeric_limits<float>::infinity();

    const int center =
        angle_to_index(0.0, scan.angle_min, scan.angle_increment, n);
    const int half_span = std::max(
        1, static_cast<int>(std::round(window_rad / scan.angle_increment)));
    const int lo = std::max(0, center - half_span);
    const int hi = std::min(n - 1, center + half_span);

    float dmin = std::numeric_limits<float>::infinity();
    for (int i = lo; i <= hi; ++i) {
      const float d = scan.ranges[i];
      if (std::isfinite(d)) {
        dmin = std::min(dmin, d);
      }
    }
    return dmin;
  }

  // Find the index of the largest finite distance within [-pi/2, +pi/2] region
  static int argmax_safest_index(const sensor_msgs::msg::LaserScan &scan) {
    const int n = static_cast<int>(scan.ranges.size());
    if (n == 0 || scan.angle_increment == 0.0)
      return -1;

    // Compute index bounds for the front 180° sector
    const int lo =
        std::max(0, static_cast<int>(std::ceil((-M_PI_2 - scan.angle_min) /
                                               scan.angle_increment)));
    const int hi =
        std::min(n - 1, static_cast<int>(std::floor((M_PI_2 - scan.angle_min) /
                                                    scan.angle_increment)));
    if (lo > hi)
      return -1;

    float best = -std::numeric_limits<float>::infinity();
    int best_idx = -1;

    for (int i = lo; i <= hi; ++i) {
      const float d = scan.ranges[i];
      if (std::isfinite(d)) { // ignore +inf/NaN as required
        if (d > best) {
          best = d;
          best_idx = i;
        }
      }
    }
    return best_idx; // may be -1 if none are finite
  }

  // --- ROS callbacks ---

  void
  laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg) {
    // 1) Move forward until an obstacle is closer than threshold at the front
    //    Use a small ±10° window to judge "front".
    const float front_min = min_front_distance(*scan_msg, 10.0 * M_PI / 180.0);

    if (front_min < obstacle_threshold_m_) {
      // 2) Obstacle detected: pick safest direction within front 180°, ignoring
      // inf
      const int idx = argmax_safest_index(*scan_msg);
      if (idx >= 0) {
        const double angle =
            scan_msg->angle_min + idx * scan_msg->angle_increment;
        direction_ = static_cast<float>(clamp_to_half_pi(angle));
      } else {
        // No finite measurements in the sector -> keep going straight
        direction_ = 0.0f;
      }
      RCLCPP_DEBUG(get_logger(),
                   "Obstacle %.2fm. Safest angle=%.2f rad, direction_=%.2f",
                   front_min,
                   scan_msg->angle_min +
                       (idx >= 0 ? idx * scan_msg->angle_increment : 0.0),
                   direction_);
    } else {
      // Path is clear: go straight
      direction_ = 0.0f;
    }
  }

  void timer_callback() {
    // Publish at 10 Hz: linear.x always 0.1 m/s, angular.z = direction_/2
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_speed_;
    cmd.angular.z = static_cast<double>(direction_) / 2.0;

    vel_publisher_->publish(cmd);

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
                          "cmd: vx=%.2f, wz=%.2f (dir=%.2f)", cmd.linear.x,
                          cmd.angular.z, direction_);
  }

  // --- Members ---
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters / state
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  int control_period_ms_;

  float direction_; // [-pi/2, +pi/2], set by laser_callback
  const float obstacle_threshold_m_;
  const float linear_speed_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
