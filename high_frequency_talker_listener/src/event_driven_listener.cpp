#include "high_frequency_talker_listener/event_driven_listener.hpp"


namespace high_frequency_talker_listener {
  EventDrivenListener::EventDrivenListener(const rclcpp::NodeOptions & options) : Node("event_driven_listener", options) {
    rclcpp::SubscriptionOptions sub_options = rclcpp::SubscriptionOptions();
    sub_options.qos_overriding_options = rclcpp::QosOverridingOptions().with_default_policies();

    last_time_ = std::chrono::steady_clock::now();

    subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
      "looping_message",
      rclcpp::QoS(10),
      std::bind(&EventDrivenListener::topic_callback, this, std::placeholders::_1),
      sub_options);
  }

  EventDrivenListener::~EventDrivenListener() {
  }

  void EventDrivenListener::topic_callback(const std_msgs::msg::UInt64::SharedPtr msg) {
    (void)msg;
    per_second_receive_count_++;

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
    if (elapsed >= 1) {
      RCLCPP_INFO(this->get_logger(), "Received %u messages in the last %ld seconds", per_second_receive_count_, elapsed);
      per_second_receive_count_ = 0;
      last_time_ = now;
    }
  }
}