#ifndef CANBUS__COMPONENT_CANBUS_TRANSLATOR_HPP_
#define CANBUS__COMPONENT_CANBUS_TRANSLATOR_HPP_

#include <algorithm>
#include <bitset>
#include <iostream>

#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "CAN_VESC.h"
#include "QUTMS_can.h"
#include "SocketCAN.hpp"
#include "TritiumCAN.hpp"
#include "can_interface.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/float32_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "interface.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace canbus {

const int NUM_CMUS = 8;
const int NUM_VOLTAGES = 14;
const int NUM_TEMPERATURES = 16;
const float WHEEL_DIAMETER = 0.4064;
const float AXLE_WIDTH = 1.4;

// create array of CAN IDs we care about
std::vector<uint32_t> canopen_ids = {RES_BOOT_UP_ID, RES_HEARTBEAT_ID, C5E_BOOT_UP_ID, C5E_POS_ID,
                                     C5E_EMCY_ID,    C5E_STATUS_ID,    C5E_SRV_ID};
std::vector<uint32_t> can_ids = {SW_Heartbeat_ID, EBS_CTRL_Heartbeat_ID};

// names for the CAN IDs
std::vector<std::string> canopen_names = {"RES_BOOT_UP_ID", "RES_HEARTBEAT_ID", "C5E_BOOT_UP_ID", "C5E_POS_ID",
                                          "C5E_EMCY_ID",    "C5E_STATUS_ID",    "C5E_SRV_ID"};
std::vector<std::string> can_names = {"SW_Heartbeat_ID", "EBS_CTRL_Heartbeat_ID", "VCU_TransmitSteering_ID"};

class CANTranslator : public rclcpp::Node, public CanInterface {
   private:
    // can connection queue retrieval timer
    rclcpp::TimerBase::SharedPtr timer_;

    // subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub_;

    // publishers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr canopen_pub_;
    // ADD PUBS FOR CAN TOPICS HERE
    rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr steering_angle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr wss_velocity_pub1_;
    rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr wss_velocity_pub2_;
    rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr wss_velocity_pub3_;
    rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr wss_velocity_pub4_;
    std::vector<rclcpp::Publisher<driverless_msgs::msg::Float32Stamped>::SharedPtr> wheel_speed_pubs_;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

    std::string ros_base_frame_;

    // can connection
    std::shared_ptr<CANInterface> can_interface_;

    // class variables for sensor data
    float wheel_speeds_[4];

    std::vector<rclcpp::Time> last_canopen_times_{canopen_ids.size(), rclcpp::Time(0)};
    std::vector<rclcpp::Time> last_can_times_{can_names.size(), rclcpp::Time(0)};

    void canmsg_timer();
    void canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const;

   public:
    CANTranslator(const rclcpp::NodeOptions& options);
    ~CANTranslator() override;

    bool set_interface();
};

}  // namespace canbus

#endif  // CANBUS__COMPONENT_CANBUS_TRANSLATOR_HPP_
