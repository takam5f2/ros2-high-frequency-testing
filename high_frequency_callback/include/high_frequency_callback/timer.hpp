#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "high_frequency_program_interface/srv/frequency.hpp"

namespace high_frequency_callback {
  class TimerNode : public rclcpp::Node {
    public:
      explicit TimerNode(const rclcpp::NodeOptions & options);
      ~TimerNode();

    private:
      void set_frequency_callback(const std::shared_ptr<high_frequency_program_interface::srv::Frequency::Request> request,
                     std::shared_ptr<high_frequency_program_interface::srv::Frequency::Response> response);
      void timer_callback();
      void print_callback();

      rclcpp::CallbackGroup::SharedPtr timer_group_;
      rclcpp::CallbackGroup::SharedPtr sub_group_;

      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::TimerBase::SharedPtr print_timer_;
      rclcpp::Service<high_frequency_program_interface::srv::Frequency>::SharedPtr service_;

      int per_second_count_ = 0;
      double frequency_ = 0.0; // Default frequency is 1 Hz

      std::mutex mutex_;
  };
}

#endif // __TIMER_HPP__
