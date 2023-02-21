#include <iostream>

#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "CAN_VESC.h"
#include "QUTMS_can.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "can_interface.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/driving_dynamics1.hpp"
#include "driverless_msgs/msg/motor_rpm.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "driverless_msgs/msg/reset.hpp"
#include "driverless_msgs/msg/shutdown.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "driverless_msgs/msg/steering_reading.hpp"
#include "driverless_msgs/msg/system_status.hpp"
#include "driverless_msgs/msg/wss_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

void copy_data(const std::vector<uint8_t> &vec, uint8_t *dest, size_t n) {
    for (size_t i = 0; i < n; i++) {
        dest[i] = vec[i];
    }
}

class ASSupervisor : public rclcpp::Node, public CanInterface {
   private:
    DVL_HeartbeatState_t DVL_heartbeat;
    VCU_HeartbeatState_t CTRL_VCU_heartbeat;
    VCU_HeartbeatState_t EBS_VCU_heartbeat;
    SW_HeartbeatState_t SW_heartbeat;
    RES_Status_t RES_status;
    DVL_DrivingDynamics1_Data_u DVL_drivingDynamics1;
    DVL_SystemStatus_Data_u DVL_systemStatus;

    // Can publisher and subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<driverless_msgs::msg::Shutdown>::SharedPtr shutdown_sub;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Publisher<driverless_msgs::msg::State>::SharedPtr state_pub;
    rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_pub;
    rclcpp::Publisher<driverless_msgs::msg::SteeringReading>::SharedPtr steering_reading_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_angle_pub;
    rclcpp::Publisher<driverless_msgs::msg::Reset>::SharedPtr reset_pub;
    rclcpp::Publisher<driverless_msgs::msg::MotorRPM>::SharedPtr motorRPM_pub;
    rclcpp::Publisher<driverless_msgs::msg::WSSVelocity>::SharedPtr wss_vel_pub;

    rclcpp::Publisher<driverless_msgs::msg::DrivingDynamics1>::SharedPtr logging_drivingDynamics1_pub;
    rclcpp::Publisher<driverless_msgs::msg::SystemStatus>::SharedPtr logging_systemStatus_pub;

    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::TimerBase::SharedPtr res_alive_timer;

    rclcpp::TimerBase::SharedPtr dataLogger_timer;

    driverless_msgs::msg::State ros_state;

    bool res_alive = 0;
    float last_torque = 0;

    const int NUM_MOTORS = 4;
    // velocity of each wheel in m/s
    float wheel_speeds[4];
    const int MOTOR_COUNT = 4;
    const float WHEEL_DIAMETER = 0.4064;

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
                this->can_pub->publish(this->_d_2_f(0x00, false, p, sizeof(p)));

                this->res_alive = 1;
                this->run_fsm();
                break;
            }

            case (0x180 + RES_NODE_ID): {
                // RES Reciever Status Packet
                Parse_RES_Heartbeat((uint8_t *)&msg.data[0], &this->RES_status);

                driverless_msgs::msg::RES res_msg;
                res_msg.sw_k2 = this->RES_status.sw_k2;
                res_msg.bt_k3 = this->RES_status.bt_k3;
                res_msg.estop = this->RES_status.estop;
                res_msg.radio_quality = this->RES_status.radio_quality;
                res_msg.loss_of_signal_shutdown_notice = this->RES_status.loss_of_signal_shutdown_notice;
                this->res_pub->publish(res_msg);

                this->res_alive = 1;
                // Log RES state
                RCLCPP_DEBUG(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
                             this->RES_status.sw_k2, this->RES_status.bt_k3, this->RES_status.estop,
                             this->RES_status.radio_quality);
                this->run_fsm();
                break;
            }

            default:
                break;
        }

        uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
        uint8_t vesc_id = msg.id & 0xFF;

        if (vesc_id < NUM_MOTORS) {
            switch (vesc_masked_id) {
                case VESC_CAN_PACKET_STATUS: {
                    int32_t rpm;
                    float current;
                    float duty;

                    // data vector to uint8_t array
                    uint8_t data[8];
                    for (int i = 0; i < 8; i++) {
                        data[i] = msg.data[i];
                    }

                    // extract and publish RPM
                    Parse_VESC_CANPacketStatus(data, &rpm, &current, &duty);

                    driverless_msgs::msg::MotorRPM rpmMsg;
                    rpmMsg.index = vesc_id;
                    rpmMsg.rpm = rpm;
                    this->motorRPM_pub->publish(rpmMsg);
                    this->DVL_drivingDynamics1._fields.speed_actual = rpm / (21.0 * 4.50) * M_PI * WHEEL_DIAMETER / 60;

                    wheel_speeds[vesc_id] = (rpm / (21.0 * 4.50)) * M_PI * this->WHEEL_DIAMETER / 60;

                    float av_velocity = 0;

                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        av_velocity += this->wheel_speeds[i];
                    }
                    av_velocity = av_velocity / MOTOR_COUNT;

                    driverless_msgs::msg::WSSVelocity vel_msg;
                    vel_msg.velocity = av_velocity;
                    vel_msg.header.stamp = this->now();
                    this->wss_vel_pub->publish(vel_msg);
                }
            }
        }

        uint32_t qutms_masked_id = msg.id & ~0xF;
        switch (qutms_masked_id) {
            case (VCU_Heartbeat_ID): {
                uint8_t VCU_ID = msg.id & 0xF;
                RCLCPP_DEBUG(this->get_logger(), "VCU ID: %u STATE: %02x", VCU_ID, msg.data[0]);

                // data vector to uint8_t array
                uint8_t data[8];
                copy_data(msg.data, data, 8);

                // update heartbeat data for this specific VCU
                if (VCU_ID == VCU_ID_CTRL) {
                    Parse_VCU_Heartbeat(data, &this->CTRL_VCU_heartbeat);
                    this->run_fsm();
                } else if (VCU_ID == VCU_ID_EBS) {
                    Parse_VCU_Heartbeat(data, &this->EBS_VCU_heartbeat);

                    if (this->EBS_VCU_heartbeat.otherFlags.ebs._VCU_Flags_EBS.CTRL_EBS == 1) {
                        this->DVL_systemStatus._fields.EBS_state = DVL_EBS_STATE_ARMED;
                    } else {
                        this->DVL_systemStatus._fields.EBS_state = DVL_EBS_STATE_ACTIVATED;
                    }

                    this->run_fsm();
                }

                break;
            }
            case (VCU_TransmitSteering_ID): {
                // data vector to uint8_t array
                uint8_t data[8];
                copy_data(msg.data, data, 8);

                int16_t steering_0_raw;
                int16_t steering_1_raw;
                uint16_t adc_0;
                uint16_t adc_1;

                Parse_VCU_TransmitSteering(data, &steering_0_raw, &steering_1_raw, &adc_0, &adc_1);
                RCLCPP_DEBUG(this->get_logger(), "Steering Angle 0: %i  Steering Angle 1: %i ADC 0: %i ADC 1: %i",
                             steering_0_raw, steering_1_raw, adc_0, adc_1);
                double steering_0 = steering_0_raw / 10.0;
                double steering_1 = steering_1_raw / 10.0;
                if (abs(steering_0 - steering_1) < 10) {
                    driverless_msgs::msg::SteeringReading reading;
                    reading.steering_angle = steering_0;
                    reading.adc_0 = adc_0;
                    reading.adc_1 = adc_1;
                    this->steering_reading_pub->publish(reading);

                    std_msgs::msg::Float32 angle_msg;
                    angle_msg.data = steering_0;
                    this->steering_angle_pub->publish(angle_msg);
                } else {
                    // go to emergency
                    this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
                }
                this->run_fsm();

                this->DVL_drivingDynamics1._fields.steering_angle_actual = (int8_t)steering_0;

                break;
            }
            case (SW_Heartbeat_ID): {
                // data vector to uint8_t array
                uint8_t data[4];
                copy_data(msg.data, data, 4);

                Parse_SW_Heartbeat(data, &this->SW_heartbeat);
                RCLCPP_INFO(this->get_logger(), "SW State: %02x Mission Id: %d", this->SW_heartbeat.stateID,
                            this->SW_heartbeat.missionID);

                this->run_fsm();

                break;
            }

            default:
                break;
        }
    }

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) {
        // input range: -1 to 0, 0 to 1
        // torque to car range: 50 to 100 for accel, 40 to 0 for regen

        // compute torque from accel and braking
        if (msg.drive.acceleration >= 0) {
            // accel is positive -> accel (0.0 -> 1.0)
            this->last_torque = 50 + 50 * msg.drive.acceleration;
        } else {
            // accel is negative -> braking (0.0 -> -1.0)
            this->last_torque = 40 + 40 * msg.drive.acceleration;
        }

        this->DVL_drivingDynamics1._fields.steering_angle_target = (int8_t)msg.drive.steering_angle;
        this->DVL_drivingDynamics1._fields.speed_target = (int8_t)msg.drive.speed;

        // convert requested accel to estimated motor torque
        float torqueValue = msg.drive.acceleration * 9 * 4.5 * 4;

        this->DVL_drivingDynamics1._fields.motor_moment_target = (int8_t)torqueValue;
        this->DVL_drivingDynamics1._fields.motor_moment_actual = (int8_t)torqueValue;

        this->run_fsm();
    }

    void heartbeat_callback() {
        // CAN publisher
        auto heartbeat = Compose_DVL_Heartbeat(&this->DVL_heartbeat);
        this->can_pub->publish(this->_d_2_f(heartbeat.id, true, heartbeat.data, sizeof(heartbeat.data)));

        // ROScube publisher
        this->ros_state.header.stamp = this->now();
        this->ros_state.state = this->DVL_heartbeat.stateID;
        this->state_pub->publish(this->ros_state);
    }

    void res_alive_callback() {
        if (!this->res_alive) {
            this->DVL_heartbeat.stateID = driverless_msgs::msg::State::EMERGENCY;
            RCLCPP_DEBUG(this->get_logger(), "RES TIMEOUT: Attemping to start RES");
            uint8_t p[8] = {0x80, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            this->can_pub->publish(this->_d_2_f(0x00, false, p, sizeof(p)));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            uint8_t p2[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            this->can_pub->publish(this->_d_2_f(0x00, false, p2, sizeof(p2)));
        }
        this->res_alive = 0;

        this->run_fsm();
    }

    void dataLogger_callback() {
        // system status
        auto systemStatusMsg = Compose_DVL_SystemStatus(&this->DVL_systemStatus);
        this->can_pub->publish(
            this->_d_2_f(systemStatusMsg.id, false, systemStatusMsg.data, sizeof(systemStatusMsg.data)));

        driverless_msgs::msg::SystemStatus systemStatus_ROSmsg;
        systemStatus_ROSmsg.as_state = this->DVL_systemStatus._fields.AS_state;
        systemStatus_ROSmsg.ebs_state = this->DVL_systemStatus._fields.EBS_state;
        systemStatus_ROSmsg.ami_state = this->DVL_systemStatus._fields.AMI_state;
        systemStatus_ROSmsg.steering_state = this->DVL_systemStatus._fields.steering_state;
        systemStatus_ROSmsg.service_brake_state = this->DVL_systemStatus._fields.service_brake_state;
        systemStatus_ROSmsg.lap_counter = this->DVL_systemStatus._fields.lap_counter;
        systemStatus_ROSmsg.cones_count_actual = this->DVL_systemStatus._fields.cones_count_actual;
        systemStatus_ROSmsg.cones_count_all = this->DVL_systemStatus._fields.cones_count_all;

        this->logging_systemStatus_pub->publish(systemStatus_ROSmsg);

        // very small delay between messages
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // driving dynamics 1
        auto drivingDynamics1Msg = Compose_DVL_DrivingDynamics1(&this->DVL_drivingDynamics1);
        this->can_pub->publish(
            this->_d_2_f(drivingDynamics1Msg.id, false, drivingDynamics1Msg.data, sizeof(drivingDynamics1Msg.data)));

        driverless_msgs::msg::DrivingDynamics1 drivingDynamics1_ROSmsg;
        drivingDynamics1_ROSmsg.speed_actual = this->DVL_drivingDynamics1._fields.speed_actual;
        drivingDynamics1_ROSmsg.speed_target = this->DVL_drivingDynamics1._fields.speed_target;
        drivingDynamics1_ROSmsg.steering_angle_actual = this->DVL_drivingDynamics1._fields.steering_angle_actual;
        drivingDynamics1_ROSmsg.steering_angle_target = this->DVL_drivingDynamics1._fields.steering_angle_target;
        drivingDynamics1_ROSmsg.brake_hydr_actual = this->DVL_drivingDynamics1._fields.brake_hydr_actual;
        drivingDynamics1_ROSmsg.brake_hydr_target = this->DVL_drivingDynamics1._fields.brake_hydr_target;
        drivingDynamics1_ROSmsg.motor_moment_actual = this->DVL_drivingDynamics1._fields.motor_moment_actual;
        drivingDynamics1_ROSmsg.motor_moment_target = this->DVL_drivingDynamics1._fields.motor_moment_target;

        this->logging_drivingDynamics1_pub->publish(drivingDynamics1_ROSmsg);
    }

    void shutdown_callback(const driverless_msgs::msg::Shutdown msg) {
        if (msg.emergency_shutdown) {
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            run_fsm();
        } else if (msg.finished_engage_ebs) {
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_ACTIVATE_EBS;
            run_fsm();
        } else if (msg.finished) {
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
            run_fsm();
        }
    }

    void run_fsm() {
        // by default, no torque
        this->DVL_heartbeat.torqueRequest = 0.0;

        // Starting state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_START) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;

            // reset mission selections
            this->ros_state.mission = driverless_msgs::msg::State::MISSION_NONE;
            this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_NONE;

            // transition to select mission when res switch is backwards
            if (!this->RES_status.sw_k2) {
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_SELECT_MISSION;
            }
        }

        // Select mission state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_SELECT_MISSION) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;
            if (this->SW_heartbeat.stateID == SW_STATES::SW_STATE_SELECT_MISSION) {
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_WAIT_FOR_MISSION;
            }
        }

        // Wait for mission state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_WAIT_FOR_MISSION) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;
            if (this->SW_heartbeat.stateID == SW_STATES::SW_STATE_MISSION_ACK) {
                this->ros_state.mission = this->SW_heartbeat.missionID;
            }

            if (this->ros_state.mission != driverless_msgs::msg::State::MISSION_NONE) {
                // set mission for logging
                switch (this->ros_state.mission) {
                    case DRIVERLESS_MISSIONS::MISSION_INSPECTION:
                        this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_INSPECTION;
                        break;
                    case DRIVERLESS_MISSIONS::MISSION_EBS:
                        this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_BRAKETEST;
                        break;
                    case DRIVERLESS_MISSIONS::MISSION_TRACK:
                        this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_TRACKDRIVE;
                        break;
                    default:
                        this->DVL_systemStatus._fields.AMI_state = 0;
                        break;
                }

                // transition to Check EBS state when mission is selected
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_CHECK_EBS;

                // set the DVL mission IDs according to selection
                if (this->ros_state.mission == driverless_msgs::msg::State::MANUAL_DRIVING) {
                    this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_MANUAL;
                } else {
                    this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_SELECTED;
                }
            }
        }

        // Check EBS state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_CHECK_EBS) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;

            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_EBS_READY && this->RES_status.sw_k2) {
                // transition to Ready state when VCU reports EBS checks complete and res switch is forward
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_READY;
            }
        }

        // Ready state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_READY) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_READY;

            if (this->RES_status.bt_k3) {
                // transition to Driving state when RES R2D button is pressed
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_RELEASE_EBS;
            }

            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_SHUTDOWN) {
                // transition to E-Stop state when EBS VCU reports shutdown
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            }
        }

        // Release brake state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_RELEASE_EBS) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_DRIVING;

            if (EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_EBS_DRIVE) {
                // transition to Driving state when brakes r good
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_DRIVING;
                driverless_msgs::msg::Reset reset_msg;
                reset_msg.reset = true;
                this->reset_pub->publish(reset_msg);
            }

            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_SHUTDOWN) {
                // transition to E-Stop state when EBS VCU reports shutdown
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            }
        }

        // Driving state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_DRIVING) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_DRIVING;

            // update torque with last saved value
            this->DVL_heartbeat.torqueRequest = this->last_torque;

            // if (this->RES_status.bt_k3) {
            //     // transition to finished state when RES R2D button is pressed
            //     this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
            // }

            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_SHUTDOWN) {
                // transition to E-Stop state when EBS VCU reports shutdown
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
            }
        }

        // EBS Activated state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_ACTIVATE_EBS) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_FINISH;

            // this state activates the EBS without tripping shutdown
            // used at end of missions

            // if the vcu says its braking, we go to finished
            if (this->EBS_VCU_heartbeat.stateID == VCU_STATES::VCU_STATE_EBS_BRAKING) {
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
            }
        }

        // Finished state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_FINISHED) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_FINISH;

            this->DVL_heartbeat.torqueRequest = 0;
            if (!this->RES_status.sw_k2) {
                // transition to start when RES swtiched back
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;
            }
        }

        // Emergency state
        if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_EMERGENCY) {
            this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_EBRAKE;

            this->DVL_heartbeat.torqueRequest = 0;
            if (!this->RES_status.estop && !this->RES_status.sw_k2) {
                // transition to start when RES swtiched back
                this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;
            }
        }

        // E-stop overrides - these are checked no matter what state we are in
        if (this->RES_status.estop) {
            // transition to E-Stop state when RES reports E-Stop or loss of signal
            this->DVL_heartbeat.torqueRequest = 0;
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
        }
    }

    void reset_dataLogger() {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;
        this->DVL_systemStatus._fields.EBS_state = DVL_EBS_State::DVL_EBS_STATE_UNAVAILABLE;
        this->DVL_systemStatus._fields.AMI_state = 0;
        this->DVL_systemStatus._fields.steering_state = 0;
        this->DVL_systemStatus._fields.service_brake_state = DVL_SERVICE_BRAKE_State::DVL_SERVICE_BRAKE_STATE_AVAILABLE;
        this->DVL_systemStatus._fields.lap_counter = 0;
        this->DVL_systemStatus._fields.cones_count_actual = 0;
        this->DVL_systemStatus._fields.cones_count_all = 0;

        this->DVL_drivingDynamics1._fields.speed_actual = 0;
        this->DVL_drivingDynamics1._fields.speed_target = 0;
        this->DVL_drivingDynamics1._fields.steering_angle_actual = 0;
        this->DVL_drivingDynamics1._fields.steering_angle_target = 0;
        this->DVL_drivingDynamics1._fields.brake_hydr_actual = 0;
        this->DVL_drivingDynamics1._fields.brake_hydr_target = 0;
        this->DVL_drivingDynamics1._fields.motor_moment_actual = 0;
        this->DVL_drivingDynamics1._fields.motor_moment_target = 0;
    }

   public:
    ASSupervisor() : Node("vehicle_supervisor_node") {
        // Setup inital states
        this->ros_state.state = driverless_msgs::msg::State::START;
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;

        this->reset_dataLogger();

        // CAN
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", 10);
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canbus_rosbound", 10, std::bind(&ASSupervisor::canbus_callback, this, _1));

        // State
        this->state_pub = this->create_publisher<driverless_msgs::msg::State>("/system/as_status", 10);

        // Reset
        this->reset_pub = this->create_publisher<driverless_msgs::msg::Reset>("/system/reset", 10);

        // RES
        this->res_pub = this->create_publisher<driverless_msgs::msg::RES>("/vehicle/res_status", 10);

        // Motor RPM
        this->motorRPM_pub = this->create_publisher<driverless_msgs::msg::MotorRPM>("/vehicle/motor_rpm", 10);
        this->wss_vel_pub = this->create_publisher<driverless_msgs::msg::WSSVelocity>("/vehicle/wheel_speed", 10);

        // Steering
        this->steering_reading_pub =
            this->create_publisher<driverless_msgs::msg::SteeringReading>("/vehicle/steering_reading", 10);

        this->steering_angle_pub = this->create_publisher<std_msgs::msg::Int32>("/vehicle/steering_angle", 10);

        // Ackermann -> sub to acceleration command
        this->ackermann_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/accel_command", 10, std::bind(&ASSupervisor::ackermann_callback, this, _1));

        // Heartbeat
        this->heartbeat_timer =
            this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ASSupervisor::heartbeat_callback, this));

        // RES Alive
        this->res_alive_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                                        std::bind(&ASSupervisor::res_alive_callback, this));

        // Data Logger
        this->dataLogger_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                         std::bind(&ASSupervisor::dataLogger_callback, this));

        this->logging_drivingDynamics1_pub =
            this->create_publisher<driverless_msgs::msg::DrivingDynamics1>("/data_logger/drivingDynamics1", 10);
        this->logging_systemStatus_pub =
            this->create_publisher<driverless_msgs::msg::SystemStatus>("/data_logger/systemStatus", 10);

        // Shutdown emergency
        this->shutdown_sub = this->create_subscription<driverless_msgs::msg::Shutdown>(
            "/system/shutdown", 10, std::bind(&ASSupervisor::shutdown_callback, this, _1));
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ASSupervisor>());
    rclcpp::shutdown();
    return 0;
}
