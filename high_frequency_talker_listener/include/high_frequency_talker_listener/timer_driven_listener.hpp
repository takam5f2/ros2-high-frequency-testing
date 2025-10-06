#ifndef __TIMER_DRIVEN_LISTENER_HPP__
#define __TIMER_DRIVEN_LISTENER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <chrono>

namespace high_frequency_talker_listener {
  class TimerDrivenListener : public rclcpp::Node {
    public:
      explicit TimerDrivenListener(const rclcpp::NodeOptions & options, const double frequency);
      ~TimerDrivenListener();
    private:
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
      void timer_callback();
      void topic_callback(const std_msgs::msg::UInt64::SharedPtr msg);
      unsigned int per_second_receive_count_ = 0;
      std::chrono::steady_clock::time_point last_time_;
  };
}
#endif // __TIMER_DRIVEN_LISTENER_HPP__