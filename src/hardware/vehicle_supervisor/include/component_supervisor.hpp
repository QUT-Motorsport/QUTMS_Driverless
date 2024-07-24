#ifndef VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_HPP_
#define VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_HPP_

#include <iostream>

#include "CAN_DVL.h"
#include "CAN_EBS_CTRL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "can_interface.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/driving_dynamics1.hpp"
#include "driverless_msgs/msg/float32_stamped.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "driverless_msgs/msg/shutdown.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "driverless_msgs/msg/system_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

namespace vehicle_supervisor {

class ASSupervisor : public rclcpp::Node, public CanInterface {
   protected:
    DVL_HeartbeatState_t DVL_heartbeat;
    VCU_HeartbeatState_t CTRL_VCU_heartbeat;
    EBS_CTRL_HeartbeatState_t EBS_heartbeat;
    SW_HeartbeatState_t SW_heartbeat;
    DVL_DrivingDynamics1_Data_u DVL_drivingDynamics1;
    DVL_SystemStatus_Data_u DVL_systemStatus;
    RES_Status_t RES_status;

    driverless_msgs::msg::State ros_state;

    // callback timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr res_alive_timer_;
    rclcpp::TimerBase::SharedPtr dataLogger_timer_;

    // subscribers
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub_;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr canopen_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr control_sub_;
    rclcpp::Subscription<driverless_msgs::msg::Shutdown>::SharedPtr shutdown_sub_;
    rclcpp::Subscription<driverless_msgs::msg::Float32Stamped>::SharedPtr steering_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lap_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steering_ready_sub_;

    // publishers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_status_pub_;
    rclcpp::Publisher<driverless_msgs::msg::DrivingDynamics1>::SharedPtr logging_drivingDynamics1_pub_;
    rclcpp::Publisher<driverless_msgs::msg::SystemStatus>::SharedPtr logging_systemStatus_pub_;

    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr ctrl_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

    bool res_alive = 0;
    float last_torque = 0;
    float last_steering_angle = 0;
    float last_velocity = 0;

    void canopen_callback(const driverless_msgs::msg::Can::SharedPtr msg);
    void can_callback(const driverless_msgs::msg::Can::SharedPtr msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void steering_angle_callback(const driverless_msgs::msg::Float32Stamped::SharedPtr msg);
    void control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void lap_counter_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    void steering_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void shutdown_callback(const driverless_msgs::msg::Shutdown::SharedPtr msg);

    void dvl_heartbeat_timer_callback();
    void res_alive_timer_callback();
    void dataLogger_timer_callback();

    void run_fsm();
    void publish_heartbeart();
    void reset_dataLogger();

   public:
    ASSupervisor(const rclcpp::NodeOptions& options, std::string name = "vehicle_supervisor_node");
};

}  // namespace vehicle_supervisor

#endif  // VEHICLE_SUPERVISOR__COMPONENT_SUPERVISOR_HPP_
