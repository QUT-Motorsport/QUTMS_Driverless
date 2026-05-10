#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <driverless_msgs/srv/is_recording.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/recorder.hpp>

namespace rosbag_creator
{

class Rosbag2RecorderComponent : public rclcpp::Node
{
public:
  explicit Rosbag2RecorderComponent(const rclcpp::NodeOptions & options);
  ~Rosbag2RecorderComponent() override;

private:
  void reconcile_recording_state();

  void handle_is_recording(
    const std::shared_ptr<driverless_msgs::srv::IsRecording::Request> /*req*/,
    std::shared_ptr<driverless_msgs::srv::IsRecording::Response> res);

  std::shared_ptr<rosbag2_transport::Recorder> recorder_;
  bool discovery_disabled_ {false};
  std::atomic<bool> recording_state_ {false};
  rclcpp::executors::SingleThreadedExecutor recorder_executor_;
  std::thread recorder_spin_thread_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Service<driverless_msgs::srv::IsRecording>::SharedPtr is_recording_srv_;
};

}  // namespace rosbag_creator
