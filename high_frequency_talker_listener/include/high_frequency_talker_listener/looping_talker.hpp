#ifndef __LOOPING_TALKER_HPP__
#define __LOOPING_TALKER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <chrono>

namespace high_frequency_talker_listener {
  class LoopingTalker : public rclcpp::Node {
    public:
      explicit LoopingTalker(const rclcpp::NodeOptions & options);
      ~LoopingTalker();
      void publish_message();

    private:

      rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

      unsigned int per_second_publish_count_ = 0;
      std::chrono::steady_clock::time_point last_time_;
  };
}

#endif // __LOOPING_TALKER_HPP__
