#include "ros2_dummy_diagnostics_publisher/ros2_dummy_diagnostics_publisher.hpp"

namespace ros2_dummy_diagnostics_publisher {
    ROS2DummyDiagnosticsPublisher::ROS2DummyDiagnosticsPublisher(const rclcpp::NodeOptions & options)
  : Node("ros2_dummy_diagnostics_publisher", options),
    updater_(this)
  {
    updater_.setHardwareID("imu_monitor");
    updater_.add("yaw_rate_status", this, &ROS2DummyDiagnosticsPublisher::run_dummy_diagnostic_task);
    updater_.setPeriod(7200.0);
  }
  
  ROS2DummyDiagnosticsPublisher::~ROS2DummyDiagnosticsPublisher() {
  }

  void ROS2DummyDiagnosticsPublisher::update() {
    updater_.force_update();
  }

  void ROS2DummyDiagnosticsPublisher::set_frequency(const double frequency) {
      frequency_ = frequency;
  }

  void ROS2DummyDiagnosticsPublisher::run_dummy_diagnostic_task(diagnostic_updater::DiagnosticStatusWrapper & stat) {

    per_second_message_count_ += 1;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    stat.addf("yaw rate from imu", "%.2f", static_cast<double>(per_second_message_count_));
    stat.addf("yaw rate from twist", "%.2f", frequency_);
    stat.summary(status.level, status.message);

    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_time_;
    if (elapsed >= std::chrono::seconds(1)) {
        last_time_ = now;
        per_second_message_count_ = 0;
    }
  }
}
