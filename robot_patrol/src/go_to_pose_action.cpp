#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/init_options.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/action/go_to_pose.hpp"

#include <chrono>
#include <functional>
#include <memory>

using namespace GoToPose = robot_patrol::action::GoToPose;

class GoToPose : public rclcpp::Node {
public:
private:
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPose>());
  rclcpp::shutdown();
  return 0;
}