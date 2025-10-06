#include "high_frequency_callback/timer.hpp"

namespace high_frequency_callback {
  TimerNode::TimerNode(const rclcpp::NodeOptions & options) : Node("configurable_timer", options) {

    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    service_ = this->create_service<high_frequency_program_interface::srv::Frequency>(
      "set_frequency",
      std::bind(&TimerNode::set_frequency_callback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, sub_group_
    );

    print_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TimerNode::print_callback, this), sub_group_);
  }

  TimerNode::~TimerNode() {}

  void TimerNode::set_frequency_callback (const std::shared_ptr<high_frequency_program_interface::srv::Frequency::Request> request, std::shared_ptr<high_frequency_program_interface::srv::Frequency::Response> response) {
    if (request->frequency <= 0) {
      response->success = false;
      response->message = "Frequency must be greater than 0.";
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    frequency_ = static_cast<double>(request->frequency);
    const auto period = std::chrono::duration<double>(1.0 / frequency_);

    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }


    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&TimerNode::timer_callback, this), timer_group_);

    response->success = true;
    response->message = "Timer period set to " + std::to_string(1.0 / frequency_) + " seconds.";
  }

  void TimerNode::timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    per_second_count_++;
  }

  void TimerNode::print_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO(this->get_logger(), "Expected frequency: %.2f Hz, Actual frequency: %d Hz",
                frequency_,
                per_second_count_);
    per_second_count_ = 0;
  }
}