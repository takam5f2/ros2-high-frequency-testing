#ifndef __BUSY_LOOPING_LISTENER_HPP__
#define __BUSY_LOOPING_LISTENER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <chrono>

namespace high_frequency_talker_listener
{
  class BusyLoopingListener : public rclcpp::Node
  {
  public:
    BusyLoopingListener(const rclcpp::NodeOptions & options);
    ~BusyLoopingListener();
    void retrieve_messages();
  private:
    void dummy_subscription_callback(const std_msgs::msg::UInt64::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
    unsigned int per_second_receive_count_ = 0;
    std::chrono::steady_clock::time_point last_time_;
  };
}
#endif // __BUSY_LOOPING_LISTENER_HPP__
