#include "ros2_dummy_diagnostics_publisher/ros2_dummy_diagnostics_publisher.hpp"
#include <thread>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  unsigned int publish_frequency = 1000;
  if (argc > 1) {
    publish_frequency = std::stoi(argv[1]);
  }
  if (publish_frequency == 0) {
    publish_frequency = 1;
  }

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<ros2_dummy_diagnostics_publisher::ROS2DummyDiagnosticsPublisher>(options);

  auto publish_period = std::chrono::nanoseconds(static_cast<int>(1e9 / publish_frequency));
  auto last_time = std::chrono::steady_clock::now();
  node->set_frequency(static_cast<double>(publish_frequency));


  while (rclcpp::ok()) {
    auto now = std::chrono::steady_clock::now();
    auto sleep_duration = publish_period - (now - last_time);
    if (sleep_duration > std::chrono::nanoseconds(0)) {
    } else {
      last_time = std::chrono::steady_clock::now();
      node->update();
    }
  }
}