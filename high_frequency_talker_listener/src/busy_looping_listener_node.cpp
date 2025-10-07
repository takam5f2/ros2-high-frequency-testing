#include "high_frequency_talker_listener/busy_looping_listener.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  unsigned int retrieve_frequency = 1000;
  if (argc > 1) {
    retrieve_frequency = std::stoi(argv[1]);
  }
  if (retrieve_frequency == 0) {
    retrieve_frequency = 1;
  }


  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<high_frequency_talker_listener::BusyLoopingListener>(options);

  auto retrieve_period = std::chrono::nanoseconds(static_cast<int>(1e9 / retrieve_frequency));
  auto last_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    auto now = std::chrono::steady_clock::now();
    auto sleep_duration = retrieve_period - (now - last_time);
    if (sleep_duration > std::chrono::nanoseconds(0)) {
    } else {
      last_time = std::chrono::steady_clock::now();
      node->retrieve_messages();
    }
  }
  rclcpp::shutdown();
  return 0;
}