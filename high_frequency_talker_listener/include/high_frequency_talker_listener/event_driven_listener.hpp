#ifndef __EVENT_DRIVEN_LISTENER_HPP__
#define __EVENT_DRIVEN_LISTENER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <chrono>


namespace high_frequency_talker_listener {
  class EventDrivenListener : public rclcpp::Node {
    public:
      explicit EventDrivenListener(const rclcpp::NodeOptions & options);
      ~EventDrivenListener();
    private:
      rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
      void topic_callback(const std_msgs::msg::UInt64::SharedPtr msg);
      unsigned int per_second_receive_count_ = 0;
      std::chrono::steady_clock::time_point last_time_;
  };
}

#endif // __EVENT_DRIVEN_LISTENER_HPP__
