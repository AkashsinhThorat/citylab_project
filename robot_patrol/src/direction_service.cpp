#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <functional>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("service_server_node") {
    srv_ = this->create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Direction Service Ready ...");
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  float sector_total_dist(const float sec_begin_rad, const float sec_end_rad,
                          const sensor_msgs::msg::LaserScan &scan_msg) {
    const int n = static_cast<int>(scan_msg.ranges.size());
    if (n == 0 || scan_msg.angle_increment == 0.0)
      return 0.0;

    // Compute index bounds for the sector
    const int sec_lo = std::max(
        0, static_cast<int>(std::ceil((sec_begin_rad - scan_msg.angle_min) /
                                      scan_msg.angle_increment)));
    const int sec_hi = std::min(
        n - 1, static_cast<int>(std::floor((sec_end_rad - scan_msg.angle_min) /
                                           scan_msg.angle_increment)));

    // sector outside FOV, i.e=> no contribution
    if (sec_lo > sec_hi)
      return 0.0f;

    // std::cout << "lo:" << sec_lo << "    hi:" << sec_hi << "\n";
    // std::cout << "Printing Values : [ \n";

    // sum of distances in the sector
    float total_dist_sec = 0.0;
    for (int i = sec_lo; i <= sec_hi; i++) {
      //   std::cout << "idx : " << i << "   ====>  value: " <<
      //   scan_msg.ranges[i]
      //             << "\n";
      if (std::isinf(scan_msg.ranges[i])) {
        total_dist_sec += scan_msg.range_max;
      } else if (std::isfinite(scan_msg.ranges[i])) {
        total_dist_sec += scan_msg.ranges[i];
      } else {
        total_dist_sec += 0.0;
      }
    }
    // std::cout << "] \n\n";

    return total_dist_sec;
  }

  void
  direction_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Direction Service Requested!");

    const auto &laser_data = request->laser_data;

    // sum of distances in right sector
    float total_dist_sec_right =
        sector_total_dist((-M_PI_2), (-M_PI / 6), laser_data);

    // sum of distances in front sector
    float total_dist_sec_front =
        sector_total_dist((-M_PI / 6), (M_PI / 6), laser_data);

    // sum of distances in left sector
    float total_dist_sec_left =
        sector_total_dist((M_PI / 6), (M_PI_2), laser_data);

    RCLCPP_DEBUG(
        this->get_logger(),
        "[Total Distance]  Left: %.2f     Forward: %2f     Right: %2f     ",
        total_dist_sec_left, total_dist_sec_front, total_dist_sec_right);

    // Populate the response
    if ((total_dist_sec_front >= total_dist_sec_left) &&
        (total_dist_sec_front >= total_dist_sec_right)) {
      response->direction = "Move forward";
    } else if ((total_dist_sec_left >= total_dist_sec_front) &&
               (total_dist_sec_left >= total_dist_sec_right)) {
      response->direction = "Turn left";
    } else {
      response->direction = "Turn right";
    }

    // Service Complted
    RCLCPP_INFO(this->get_logger(), "Direction Service Completed!");
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}