#include <iostream>

#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can_interface.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "driverless_msgs/msg/steering_reading.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class ASSupervisor : public rclcpp::Node, public CanInterface {
   private:
    DVL_HeartbeatState_t DVL_heartbeat;
    VCU_HeartbeatState_t CTRL_VCU_heartbeat;
    VCU_HeartbeatState_t EBS_VCU_heartbeat;
    SW_HeartbeatState_t SW_heartbeat;
    RES_Status_t RES_status;

    // Can publisher and subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann;

    rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr state_pub;
    rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_pub;
    rclcpp::Publisher<driverless_msgs::msg::SteeringReading>::SharedPtr steering_reading_pub;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::TimerBase::SharedPtr res_alive_timer;

    driverless_msgs::msg::State ros_state;

    rclcpp::Time _internal_status_time;

    bool res_alive = 0;
    float last_torque = 0;

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

                this->res_alive = 1;
                // run state machine (will update state from "start" to "select mission")
                this->run_fsm();
                break;
            }

            case (0x180 + RES_NODE_ID): {
                // RES Reciever Status Packet
                this->_internal_status_time = this->now();  // Debug information

                Parse_RES_Heartbeat((uint8_t*)&msg.data[0], &this->RES_status);

                this->res_alive = 1;
                // Log RES state
                // RCLCPP_INFO(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
                //             this->RES_status.sw_k2, this->RES_status.bt_k3, this->RES_status.estop,
                //             this->RES_status.radio_quality);
                this->run_fsm();
                break;
            }

            default:
                break;
        }

        uint32_t qutms_masked_id = msg.id & ~0xF;
        switch (qutms_masked_id) {
            case (VCU_Heartbeat_ID): {
                // ignore type
                uint8_t VCU_ID = msg.id & 0xF;

                RCLCPP_DEBUG(this->get_logger(), "VCU ID: %u STATE: %02x", VCU_ID, msg.data[0]);

                // data vector to uint8_t array
                uint8_t data[8];
                for (int i = 0; i < 8; i++) {
                    data[i] = msg.data[i];
                }

                // update heartbeat data for this specific VCU
                if (VCU_ID == VCU_ID_CTRL) {
                    Parse_VCU_Heartbeat(data, &this->CTRL_VCU_heartbeat);
                    this->run_fsm();
                } else if (VCU_ID == VCU_ID_EBS) {
                    Parse_VCU_Heartbeat(data, &this->EBS_VCU_heartbeat);
                    this->run_fsm();
                }

                // SEEMS LIKE A GOOD IDEA TO ADD A TIMER TO MAKE SURE THE VCUs ARE CHECKING IN
                // ELSE EMERGENCY
                break;
            }
            case (VCU_TransmitSteering_ID): {
                // data vector to uint8_t array
                uint8_t data[8];
                for (int i = 0; i < 8; i++) {
                    data[i] = msg.data[i];
                }

                int16_t steering_0_raw;
                int16_t steering_1_raw;
                uint16_t adc_0;
                uint16_t adc_1;

                Parse_VCU_TransmitSteering(data, &steering_0_raw, &steering_1_raw, &adc_0, &adc_1);
                RCLCPP_DEBUG(this->get_logger(), "Steering 0: %i  Steering 1: %i ADC 0: %i ADC 1: %i", steering_0_raw,
                             steering_1_raw, adc_0, adc_1);
                double steering_0 = steering_0_raw / 10.0;
                double steering_1 = steering_1_raw / 10.0;
                if (abs(steering_0 - steering_1) < 5) {
                    driverless_msgs::msg::SteeringReading reading;
                    reading.steering_angle = steering_0;
                    reading.adc_0 = adc_0;
                    reading.adc_1 = adc_1;
                    this->steering_reading_pub->publish(reading);
                }
            }

            default:
                break;
        }
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDrive msg) {
        // input range: 0 to 1
        // torque to car range: 50 to 100
        this->last_torque = 50 + 50 * msg.acceleration;  // convert to percentage
    }

    void heartbeat_callback() {
        // CAN publisher
        auto heartbeat = Compose_DVL_Heartbeat(&this->DVL_heartbeat);
        this->can_pub->publish(this->_d_2_f(heartbeat.id, true, heartbeat.data));

        auto sw_heartbeat = Compose_SW_Heartbeat(&this->SW_heartbeat);
        this->can_pub->publish(this->_d_2_f(sw_heartbeat.id, true, sw_heartbeat.data));

        // ROScube publisher
        this->ros_state.header.stamp = this->now();
        this->ros_state.state = this->DVL_heartbeat.stateID;
        this->state_pub->publish(this->ros_state);
    }

    void res_alive_callback() {
        if (!this->res_alive) {
            RCLCPP_INFO(this->get_logger(), "Attemping to start RES");
            uint8_t p[8] = {0x80, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            this->can_pub->publish(this->_d_2_f(0x00, false, p));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            uint8_t p2[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            this->can_pub->publish(this->_d_2_f(0x00, false, p2));
        }
        this->res_alive = 0;
    }

    void run_fsm() {
        // by default, no torque
        this->DVL_heartbeat.torqueRequest = 0.0;
        // RCLCPP_INFO(this->get_logger(), "DVL STATE: %i", this->DVL_heartbeat.stateID);

        if (!this->RES_status.sw_k2) {
            // transition to finished when RES swtiched is pressed
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
        }

        if (this->RES_status.estop) {
            // transition to E-Stop state when RES reports E-Stop or loss of signal
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
        }

        // Starting state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_START) {
            // Changes to Select Mission state when RES is ready
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_SELECT_MISSION;
        }
        // Select Mission state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_SELECT_MISSION) {
            // if (this->SW_heartbeat.flags. == 1)
            // {
            //     this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_READY;
            // }

            // TRACKDRIVE
            this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_SELECTED;
            this->ros_state.mission = driverless_msgs::msg::State::TRACKDRIVE;

            // MANUAL
            // this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_MANUAL;
            // this->ros_state.mission = driverless_msgs::msg::State::MANUAL_DRIVING;

            if (ros_state.mission != driverless_msgs::msg::State::MISSION_NONE) {
                // transition to Check EBS state when mission is selected
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_CHECK_EBS;
            }
        }
        // Check EBS state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_CHECK_EBS) {
            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_EBS_READY) {
                // transition to Ready state when VCU reports EBS checks complete
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_READY;
            }
        }
        // Ready state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_READY) {
            if (this->RES_status.bt_k3) {
                // transition to Driving state when RES R2D button is pressed
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_DRIVING;
            }
        }
        // Driving state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_DRIVING) {
            if (this->EBS_VCU_heartbeat.stateID == VCU_STATE_EBS_BRAKING) {
                // transition to EBS Braking state when VCU reports EBS is braking (when it shouldn't be)
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            }

            // update torque with last saved value
            this->DVL_heartbeat.torqueRequest = this->last_torque;

            // LOGIC UPON FINISHING MISSION
        }
        // EBS Activated state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_ACTIVATE_EBS) {
            // if (stationary || time elapsed) {
            //     DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
            // }
        }
        // Finished state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_FINISHED) {
            // Do lights
            // Gracefully terminate nodes

            if (this->RES_status.sw_k2) {
                // transition to finished when RES swtiched is changed
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;
            }
        }
        // Emergency state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_EMERGENCY) {
            // Do lights and siren
            // Gracefully terminate nodes
        }
    }

   public:
    ASSupervisor() : Node("vehicle_supervisor") {
        // Setup states
        this->ros_state.state = driverless_msgs::msg::State::CHECK_EBS;
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;
        this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_NONE;
        this->SW_heartbeat.flags._SW_Flags.FAN_ENABLE = 1;

        // Configure logger level
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);

        // CAN
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&ASSupervisor::canbus_callback, this, _1));

        if (this->can_pub == nullptr || this->can_sub == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CAN topic pub sub");
        }

        // State
        this->state_pub = this->create_publisher<driverless_msgs::msg::State>("as_status", 10);

        // RES
        this->res_pub = this->create_publisher<driverless_msgs::msg::RES>("res_status", 10);

        // Steering
        this->steering_reading_pub =
            this->create_publisher<driverless_msgs::msg::SteeringReading>("steering_reading", 10);

        // Ackermann
        this->ackermann = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/driving_command", 10, std::bind(&ASSupervisor::ackermann_callback, this, _1));

        // Heartbeat
        this->heartbeat_timer =
            this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ASSupervisor::heartbeat_callback, this));

        // RES Alive
        this->res_alive_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                        std::bind(&ASSupervisor::res_alive_callback, this));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASSupervisor>());
    rclcpp::shutdown();
    return 0;
}
