#include "high_frequency_talker_listener/event_driven_listener.hpp"


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<high_frequency_talker_listener::EventDrivenListener>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
