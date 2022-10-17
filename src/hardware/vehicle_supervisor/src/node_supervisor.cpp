#include <iostream>

#include "CAN_DVL.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can_interface.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

const int RES_NODE_ID = 0x011;

using std::placeholders::_1;

typedef struct {
    bool sw_k2;
    bool bt_k3;
    bool estop;
    bool loss_of_signal_shutdown_notice;
    uint8_t radio_quality;
} RES_Status_t;

class ASSupervisor : public rclcpp::Node, public CanInterface {
   private:
    DVL_HeartbeatState_t DVL_heartbeat;
    VCU_HeartbeatState_t CTRL_VCU_heartbeat;
    VCU_HeartbeatState_t EBS_VCU_heartbeat;
    RES_Status_t RES_status;

    // Can publisher and subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann;

    rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr state_pub;
    rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_pub;

    rclcpp::TimerBase::SharedPtr timer_;

    driverless_msgs::msg::State ros_state;

    rclcpp::Time _internal_status_time;

    // Called when a new can message is recieved
    void canbus_callback(const driverless_msgs::msg::Can msg) {
        switch (msg.id) {
            case (0x700 + RES_NODE_ID): {
                /*
                RES has reported in, request state change to enable it

                Doing so will result in the RES reporting PDOs
                2000 - 20007 every 30ms with the ID 0x180 + RES_NODE_ID

                Byte 0 = state command (start up)
                Byte 1 = Node ID (0x00 = All Nodes)
                */
                uint8_t p[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                this->can_pub->publish(this->_d_2_f(0x00, false, p));

            } break;

            case (0x180 + RES_NODE_ID): {
                // RES Reciever Status Packet
                this->_internal_status_time = this->now();  // Debug information

                // Store in local state
                this->RES_status.estop = msg.data[0] & (1 << 0);                           // ESTOP = PDO 2000 Bit 0
                this->RES_status.sw_k2 = msg.data[0] & (1 << 1);                           // K2 = PDO 2000 Bit 1
                this->RES_status.bt_k3 = msg.data[0] & (1 << 2);                           // K3 = PDO 2000 Bit 2
                this->RES_status.radio_quality = msg.data[6];                              // Radio Quality = PDO 2006
                this->RES_status.loss_of_signal_shutdown_notice = msg.data[7] & (1 << 6);  // LoSSN = PDO 2007 Bit 6

                // Log RES state
                // RCLCPP_INFO(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
                // 			this->res_status.sw_k2, this->res_status.bt_k3, this->res_status.estop,
                // 			this->res_status.radio_quality);
            } break;

                // case (SW_Heartbeat_ID): { // IDK how to get this properly
                //     // Steering Wheel CAN msg
                //     if (ros_state.state == DVL_STATES::DVL_STATE_SELECT_MISSION) {
                //         // Only send mission if it's in START state
                //         this->ros_state.mission = msg.data[2]; // not sure which data byte the mission is
                //         ros_state.state = DVL_STATES::DVL_STATE_CHECK_EBS;
                //     }
                // } break;

                // case(VCU_Heartbeat_ID & ~(0xF)): { // No idea which VCU ID this is
                //     // VCU CAN msg
                //     uint8_t vcu_state = msg.data[0];

                //     // LOGIC HERE TO UPDATE AS STATE
                //     //  not this->as_status.state = vcu_state;

                // } break;

            default:
                break;
        }
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDrive msg) {
        // Validate driving state
        if (this->ros_state.state == DVL_STATES::DVL_STATE_DRIVING) {
            this->DVL_heartbeat.torqueRequest = msg.acceleration;  // Chaos
        } else {
            this->DVL_heartbeat.torqueRequest = 0;
        }
    }

    void heartbeat_callback() {
        auto heartbeat = Compose_DVL_Heartbeat(&this->DVL_heartbeat);
        this->can_pub->publish(this->_d_2_f(heartbeat.id, true, heartbeat.data));

        this->state_pub->publish(this->ros_state);
    }

    void run_fsm() {
        if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_START) {
            DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_SELECT_MISSION;
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_SELECT_MISSION) {
            /*
            SW heartbeat shit here
            */

            // MANUAL
            // DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_MANUAL;
            // SELECTED
            DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_SELECTED;

            DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_CHECK_EBS;
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_CHECK_EBS) {
            if (CTRL_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_EBS_READY) {
                DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_READY;
            }
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_READY) {
            if (RES_status.bt_k3) {
                DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_DRIVING;
            }
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_DRIVING) {
            if (EBS_VCU_heartbeat.stateID == VCU_STATE_EBS_BRAKING) {
                DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            }
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_ACTIVATE_EBS) {
            // if (stationary || time elapsed) {
            //     DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
            // }
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_FINISHED) {
            // whatever
        } else if (DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_EMERGENCY) {
            // whatever
        }
    }

   public:
    ASSupervisor() : Node("tractive_system_controller") {
        // setup states
        this->ros_state.state = DVL_STATES::DVL_STATE_START;
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;

        // Configure logger level
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // Can
        RCLCPP_DEBUG(this->get_logger(), "Initalising CAN interactants...");
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&ASSupervisor::canbus_callback, this, _1));

        if (this->can_pub == nullptr || this->can_sub == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CAN topic interactants");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Done");
        }

        // State
        RCLCPP_DEBUG(this->get_logger(), "Initalising state interactants...");
        this->state_pub = this->create_publisher<driverless_msgs::msg::State>("as_status", 10);
        RCLCPP_DEBUG(this->get_logger(), "Done");

        // RES
        RCLCPP_INFO(this->get_logger(), "Initialising RES receiver...");
        this->res_pub = this->create_publisher<driverless_msgs::msg::RES>("res_status", 10);

        RCLCPP_INFO(this->get_logger(), "Attemping to start RES");
        uint8_t p[8] = {0x80, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        this->can_pub->publish(this->_d_2_f(0x00, false, p));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        uint8_t p2[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        this->can_pub->publish(this->_d_2_f(0x00, false, p2));
        RCLCPP_INFO(this->get_logger(), "RES startup complete");

        // Ackermann
        RCLCPP_DEBUG(this->get_logger(), "Initalising ackermann interactants...");
        this->ackermann = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "ackermann", 10, std::bind(&ASSupervisor::ackermann_callback, this, _1));
        RCLCPP_DEBUG(this->get_logger(), "Done");

        // Heartbeat
        RCLCPP_DEBUG(this->get_logger(), "Creating heartbeat timer...");
        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ASSupervisor::heartbeat_callback, this));
        RCLCPP_DEBUG(this->get_logger(), "Done");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASSupervisor>());
    rclcpp::shutdown();
    return 0;
}
