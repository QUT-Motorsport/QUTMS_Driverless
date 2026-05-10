#include "rosbag_creator/rosbag2_recorder_component.hpp"

#include <chrono>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_transport/config_options_from_node_params.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <utility>

namespace rosbag_creator {

Rosbag2RecorderComponent::Rosbag2RecorderComponent(const rclcpp::NodeOptions& options)
    : Node("rosbag2_recorder_component", options) {
    this->declare_parameter<bool>("start_immediately", false);
    bool start_immediately = this->get_parameter("start_immediately").as_bool();

    auto record_options = rosbag2_transport::get_record_options_from_node_params(*this);
    auto storage_options = rosbag2_transport::get_storage_options_from_node_params(*this);
    discovery_disabled_ = record_options.is_discovery_disabled;

    auto writer_unique = rosbag2_transport::ReaderWriterFactory::make_writer(record_options);
    auto writer = std::shared_ptr<rosbag2_cpp::Writer>(std::move(writer_unique));

    auto recorder_opts = rclcpp::NodeOptions();
    recorder_opts.context(this->get_node_base_interface()->get_context());
    recorder_opts.use_intra_process_comms(true);

    recorder_ = std::make_shared<rosbag2_transport::Recorder>(writer, storage_options, record_options,
                                                              "rosbag2_recorder", recorder_opts);

    recorder_->set_on_start_recording_callback([this]() { recording_state_.store(true, std::memory_order_relaxed); });

    recorder_executor_.add_node(recorder_);
    recorder_spin_thread_ = std::thread([this]() { recorder_executor_.spin(); });

    is_recording_srv_ = this->create_service<driverless_msgs::srv::IsRecording>(
        "/rosbag_creator/is_recording",
        std::bind(&Rosbag2RecorderComponent::handle_is_recording, this, std::placeholders::_1, std::placeholders::_2));

    state_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                           std::bind(&Rosbag2RecorderComponent::reconcile_recording_state, this));

    if (start_immediately) {
        recorder_->record();
        recording_state_.store(true, std::memory_order_relaxed);
        RCLCPP_INFO(this->get_logger(), "Recording started immediately");
    }
}

Rosbag2RecorderComponent::~Rosbag2RecorderComponent() {
    recorder_executor_.cancel();
    if (recorder_spin_thread_.joinable()) {
        recorder_spin_thread_.join();
    }
}

void Rosbag2RecorderComponent::handle_is_recording(const std::shared_ptr<driverless_msgs::srv::IsRecording::Request>,
                                                   std::shared_ptr<driverless_msgs::srv::IsRecording::Response> res) {
    reconcile_recording_state();

    bool recording = recording_state_.load(std::memory_order_relaxed);
    if (!recording) {
        try {
            recording = !discovery_disabled_ ? recorder_->is_discovery_running() : !recorder_->subscriptions().empty();
            if (recording) {
                recording_state_.store(true, std::memory_order_relaxed);
            }
        } catch (const std::exception& e) {
            RCLCPP_DEBUG(this->get_logger(), "live state check skipped: %s", e.what());
        }
    }

    res->recording = recording;
}

void Rosbag2RecorderComponent::reconcile_recording_state() {
    if (!recording_state_.load(std::memory_order_relaxed)) {
        return;
    }

    try {
        if (!discovery_disabled_) {
            if (!recorder_->is_discovery_running()) {
                recording_state_.store(false, std::memory_order_relaxed);
            }
            return;
        }

        // Fallback for discovery-disabled mode: stopped recorder has no subscriptions.
        if (recorder_->subscriptions().empty()) {
            recording_state_.store(false, std::memory_order_relaxed);
        }
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "state reconciliation skipped: %s", e.what());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag_creator::Rosbag2RecorderComponent)

}  // namespace rosbag_creator
