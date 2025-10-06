#include "high_frequency_talker_listener/looping_talker.hpp"

namespace high_frequency_talker_listener {
  LoopingTalker::LoopingTalker(const rclcpp::NodeOptions & options)
  : Node("looping_talker", options)
  {
    rclcpp::PublisherOptions pub_options = rclcpp::PublisherOptions();
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions().with_default_policies();

    last_time_ = std::chrono::steady_clock::now();

    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("looping_message", rclcpp::QoS(10), pub_options);
  }

  LoopingTalker::~LoopingTalker() {
  }

  void LoopingTalker::publish_message() {
    auto message = std_msgs::msg::UInt64();
    message.data = per_second_publish_count_++;
    publisher_->publish(message);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
    if (elapsed >= 1) {
      RCLCPP_INFO(this->get_logger(), "Published %u messages in the last %ld seconds", per_second_publish_count_, elapsed);
      per_second_publish_count_ = 0;
      last_time_ = now;
    }
  }
}