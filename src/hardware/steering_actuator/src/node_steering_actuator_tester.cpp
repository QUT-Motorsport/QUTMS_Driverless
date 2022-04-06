#include <stdio.h>

#include <chrono>
#include <future>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT& future, WaitTimeT time_to_wait) {
	auto end = std::chrono::steady_clock::now() + time_to_wait;
	std::chrono::milliseconds wait_period(100);
	std::future_status status = std::future_status::timeout;
	do {
		auto now = std::chrono::steady_clock::now();
		auto time_left = end - now;
		if (time_left <= std::chrono::seconds(0)) {
			break;
		}
		status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
	} while (rclcpp::ok() && status != std::future_status::ready);
	return status;
}

class SteeringTest : public rclcpp::Node {
   private:
	static constexpr char const* node_to_handle = "steering";
	static constexpr char const* node_get_state_topic = "steering/get_state";
	static constexpr char const* node_change_state_topic = "steering/change_state";

	rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr steering_get_state;
	rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr steering_change_state;

   public:
	SteeringTest(const std::string& node_name) : rclcpp::Node(node_name) {
		RCLCPP_INFO(this->get_logger(), "%s node alive", node_name.c_str());
	}

	void init() {
		this->steering_get_state = this->create_client<lifecycle_msgs::srv::GetState>(node_get_state_topic);
		this->steering_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(node_change_state_topic);
	}

	unsigned int get_state(std::chrono::seconds time_out = 3s) {
		auto request = lifecycle_msgs::srv::GetState::Request::SharedPtr();

		if (!this->steering_get_state->wait_for_service(time_out)) {
			RCLCPP_ERROR(this->get_logger(), "Service %s is not available",
						 this->steering_get_state->get_service_name());
			return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
		}

		auto future_result = steering_get_state->async_send_request(request);
		auto future_status = wait_for_result(future_result, time_out);

		if (future_status != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "Timeout while getting current state of node");
			return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
		}

		if (future_result.get()) {
			return future_result.get()->current_state.id;
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to get current state of node");
			return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
		}
	}

	bool change_state(uint8_t transition, std::chrono::seconds time_out = 3s) {
		auto request = lifecycle_msgs::srv::ChangeState::Request::SharedPtr();

		request->transition.id = transition;

		if (!steering_change_state->wait_for_service(time_out)) {
			RCLCPP_ERROR(this->get_logger(), "Service %s not available",
						 this->steering_change_state->get_service_name());
			return false;
		}

		auto future_result = steering_change_state->async_send_request(request);
		auto future_status = wait_for_result(future_result, time_out);

		if (future_status != std::future_status::ready) {
			RCLCPP_ERROR(this->get_logger(), "Timeout while changing current state of node");
			return false;
		}

		if (future_result.get()->success) {
			return true;
		}
		return false;
	}
};

void callee_script(std::shared_ptr<SteeringTest> steering_client) {
	rclcpp::WallRate time_between_state_changes(0.1);  // 10s

	// configure
	{
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// activate
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// deactivate
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// activate it again
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// and deactivate it again
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// we cleanup
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}

	// and finally shutdown
	// Note: We have to be precise here on which shutdown transition id to call
	// We are currently in the unconfigured state and thus have to call
	// TRANSITION_UNCONFIGURED_SHUTDOWN
	{
		time_between_state_changes.sleep();
		if (!rclcpp::ok()) {
			return;
		}
		if (!steering_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
			return;
		}
		if (!steering_client->get_state()) {
			return;
		}
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);

	auto x = std::make_shared<SteeringTest>("steering_test");
	x->init();

	rclcpp::executors::SingleThreadedExecutor exe;
	exe.add_node(x);

	std::shared_future<void> script = std::async(std::launch::async, std::bind(callee_script, x));
	exe.spin_until_future_complete(script);

	rclcpp::shutdown();

	return 0;
}