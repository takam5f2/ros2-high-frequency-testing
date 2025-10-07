#include "high_frequency_talker_listener/busy_looping_listener.hpp"

namespace high_frequency_talker_listener
{
  BusyLoopingListener::BusyLoopingListener(const rclcpp::NodeOptions & options) : Node ("busy_looping_listener", options)
  {
    rclcpp::SubscriptionOptions sub_options = rclcpp::SubscriptionOptions();
    sub_options.qos_overriding_options = rclcpp::QosOverridingOptions().with_default_policies();

    rclcpp::CallbackGroup::SharedPtr callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    sub_options.callback_group = callback_group;

    last_time_ = std::chrono::steady_clock::now();

    subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
      "looping_message",
      rclcpp::QoS(10),
      std::bind(&BusyLoopingListener::dummy_subscription_callback, this, std::placeholders::_1),
      sub_options);

  }

  BusyLoopingListener::~BusyLoopingListener()
  {
  }

  void BusyLoopingListener::dummy_subscription_callback(const std_msgs::msg::UInt64::SharedPtr msg)
  {
    (void)msg;
    return;
  }

  void BusyLoopingListener::retrieve_messages()
  {
    std_msgs::msg::UInt64 msg;
    rclcpp::MessageInfo msg_info;
    while (subscription_->take(msg, msg_info)) {
      per_second_receive_count_++;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_time_).count();
    if (elapsed >= 1) {
      RCLCPP_INFO(this->get_logger(), "Received %u messages in the last %ld seconds", per_second_receive_count_, elapsed);
      per_second_receive_count_ = 0;
      last_time_ = now;
    }
  }
}
