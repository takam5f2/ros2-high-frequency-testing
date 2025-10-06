#include "high_frequency_callback/timer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  bool use_single_threaded = true;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--multi") {
      use_single_threaded = false;
    } else if (arg == "--single") {
      use_single_threaded = true;
    }
  }
  // argv[1] means whether to use multithreaded.
  if (!use_single_threaded) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Using MultiThreadedExecutor");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<high_frequency_callback::TimerNode>(rclcpp::NodeOptions());
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
  } else {
    auto node = std::make_shared<high_frequency_callback::TimerNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
}