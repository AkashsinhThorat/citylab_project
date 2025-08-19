#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <atomic>
#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;

class TestService : public rclcpp::Node {
public:
  TestService() : rclcpp::Node("test_service_node"), in_flight_(false) {
    // Parameters so you can change names from launch if needed
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");
    service_name_ = this->declare_parameter<std::string>("service_name",
                                                         "direction_service");

    // Create the service client (non-blocking; may not be ready yet)
    client_ = this->create_client<GetDirection>(service_name_);

    // Subscriber for /scan with sensor-friendly QoS
    auto qos = rclcpp::SensorDataQoS().keep_last(1);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, qos,
        std::bind(&TestService::laser_callback, this, std::placeholders::_1));
  }

private:
  // Called on every LaserScan message
  void laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    // Don’t send if the service isn’t up yet
    if (!client_->service_is_ready()) {
      RCLCPP_INFO(this->get_logger(), "Service not ready yet...");
      return;
    }

    // Back-pressure: only one request at a time
    if (in_flight_.load()) {
      return;
    }

    // Build the request and copy the scan into it
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    // Mark in-flight BEFORE sending
    in_flight_.store(true);

    // Send async request; response will arrive in response_callback()
    client_->async_send_request(request,
                                std::bind(&TestService::response_callback, this,
                                          std::placeholders::_1));
  }

  // Called when the service replies
  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    // future.get() is safe here (already ready in this callback)
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %s",
                response->direction.c_str());

    // Allow the next request
    in_flight_.store(false);
  }

private:
  // Params
  std::string scan_topic_;
  std::string service_name_;

  // ROS interfaces
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;

  // Simple back-pressure flag
  std::atomic<bool> in_flight_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>()); // no busy loop
  rclcpp::shutdown();
  return 0;
}