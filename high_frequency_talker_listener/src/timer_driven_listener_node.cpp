#include "high_frequency_talker_listener/timer_driven_listener.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  double frequency = 1000.0;
  if (argc > 1) {
    frequency = std::stod(argv[1]);
  }
  if (frequency <= 0.0) {
    frequency = 1.0;
  }

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<high_frequency_talker_listener::TimerDrivenListener>(options, frequency);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}