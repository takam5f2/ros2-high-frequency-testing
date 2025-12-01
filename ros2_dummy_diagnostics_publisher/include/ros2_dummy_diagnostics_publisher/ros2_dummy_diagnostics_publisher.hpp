#ifndef __ROS2_DUMMY_DIAGNOSTICS_PUBLISHER_HPP__
#define __ROS2_DUMMY_DIAGNOSTICS_PUBLISHER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <chrono>

namespace ros2_dummy_diagnostics_publisher {
  class ROS2DummyDiagnosticsPublisher : public rclcpp::Node {
    public:
      explicit ROS2DummyDiagnosticsPublisher(const rclcpp::NodeOptions & options);
      ~ROS2DummyDiagnosticsPublisher();
      void update();
      void set_frequency(const double frequency);

  private:
      diagnostic_updater::Updater updater_;
      double frequency_;
      void run_dummy_diagnostic_task(diagnostic_updater::DiagnosticStatusWrapper & stat);
  };
}

#endif // __ROS2_DUMMY_DIAGNOSTICS_PUBLISHER_HPP__
