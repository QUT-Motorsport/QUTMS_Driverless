#include "component_supervisor.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace vehicle_supervisor {

void ASSupervisor::canopen_callback(const driverless_msgs::msg::Can::SharedPtr msg) {
    // RES boot up
    if (msg->id == (RES_BOOT_UP_ID)) {
        /*
        RES has reported in, request state change to enable it

        Doing so will result in the RES reporting PDOs
        2000 - 20007 every 30ms with the ID 0x180 + RES_NODE_ID

        Byte 0 = state command (start up)
        Byte 1 = Node ID (0x00 = All Nodes)
        */
        RCLCPP_INFO(this->get_logger(), "RES Booted");
        uint8_t p[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        this->can_pub_->publish(this->_d_2_f(0x00, false, p, sizeof(p)));
        this->res_alive = 1;
    }
    // RES heartbeat
    else if (msg->id == (RES_HEARTBEAT_ID)) {
        // RES Reciever Status Packet
        Parse_RES_Heartbeat((uint8_t *)&msg->data[0], &this->RES_status);
        // Log RES state
        RCLCPP_DEBUG(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
                     this->RES_status.sw_k2, this->RES_status.bt_k3, this->RES_status.estop,
                     this->RES_status.radio_quality);
        // // publish RES status
        // driverless_msgs::msg::RES res_msg;
        // res_msg.sw_k2 = this->RES_status.sw_k2;
        // res_msg.bt_k3 = this->RES_status.bt_k3;
        // res_msg.estop = this->RES_status.estop;
        // res_msg.radio_quality = this->RES_status.radio_quality;
        // res_msg.loss_of_signal_shutdown_notice = this->RES_status.loss_of_signal_shutdown_notice;
        // this->res_status_pub_->publish(res_msg);
        this->res_alive = 1;
    }
}

void ASSupervisor::can_callback(const driverless_msgs::msg::Can::SharedPtr msg) {
    uint32_t qutms_masked_id = msg->id & ~0xF;

    // CAN hearbeats
    if (qutms_masked_id == VCU_Heartbeat_ID) {
        uint8_t VCU_ID = msg->id & 0xF;
        RCLCPP_DEBUG(this->get_logger(), "VCU ID: %u STATE: %02x", VCU_ID, msg->data[0]);

        // data vector to uint8_t array
        uint8_t data[8];
        copy_data(msg->data, data, 8);
    } else if (msg->id == EBS_CTRL_Heartbeat_ID) {
        uint8_t data[8];
        copy_data(msg->data, data, 8);

        Parse_EBS_CTRL_Heartbeat(data, &this->EBS_heartbeat);

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE_DRIVE) {
            this->DVL_systemStatus._fields.EBS_state = DVL_EBS_STATE_ARMED;
        } else {
            this->DVL_systemStatus._fields.EBS_state = DVL_EBS_STATE_ACTIVATED;
        }
    } else if (qutms_masked_id == SW_Heartbeat_ID) {
        // data vector to uint8_t array
        uint8_t data[4];
        copy_data(msg->data, data, 4);

        Parse_SW_Heartbeat(data, &this->SW_heartbeat);
        RCLCPP_DEBUG(this->get_logger(), "SW State: %02x Mission Id: %d", this->SW_heartbeat.stateID,
                     this->SW_heartbeat.missionID);
    }
}

void ASSupervisor::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->last_velocity = msg->data;
    this->DVL_drivingDynamics1._fields.speed_actual = (int8_t)msg->data;
}

void ASSupervisor::steering_angle_callback(const driverless_msgs::msg::Float32Stamped::SharedPtr msg) {
    this->last_steering_angle = msg->data;
    this->DVL_drivingDynamics1._fields.steering_angle_actual = (int8_t)msg->data;
}

void ASSupervisor::control_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
    // input range: -1 to 1, convert to -100 to 100
    this->last_torque = 100 * msg->drive.acceleration;
    // convert requested accel to estimated motor torque
    float torqueValue = msg->drive.acceleration * 9 * 4.5 * 4;

    this->DVL_drivingDynamics1._fields.steering_angle_target = (int8_t)msg->drive.steering_angle;
    this->DVL_drivingDynamics1._fields.speed_target = (int8_t)msg->drive.speed;
    this->DVL_drivingDynamics1._fields.motor_moment_target = (int8_t)torqueValue;
    this->DVL_drivingDynamics1._fields.motor_moment_actual = (int8_t)torqueValue;
}

void ASSupervisor::lap_counter_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    this->ros_state.lap_count = msg->data;
    this->DVL_systemStatus._fields.lap_counter = msg->data;
}

void ASSupervisor::steering_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->ros_state.navigation_ready = msg->data;
}

void ASSupervisor::shutdown_callback(const driverless_msgs::msg::Shutdown::SharedPtr msg) {
    if (msg->emergency_shutdown) {
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
    } else if (msg->finished_engage_ebs) {
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_ACTIVATE_EBS;
    } else if (msg->finished) {
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_FINISHED;
    }
}

void ASSupervisor::publish_heartbeart() {
    if (this->get_parameter("manual_override").as_bool()) {
        // override straight to driving trackdrive
        DVL_HeartbeatState_t or_DVL_heartbeat;
        or_DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_DRIVING;
        or_DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_SELECTED;
        auto heartbeat = Compose_DVL_Heartbeat(&or_DVL_heartbeat);
        this->can_pub_->publish(std::move(this->_d_2_f(heartbeat.id, true, heartbeat.data, sizeof(heartbeat.data))));

        driverless_msgs::msg::State::UniquePtr or_ros_state(new driverless_msgs::msg::State());
        or_ros_state->state = or_DVL_heartbeat.stateID;
        or_ros_state->mission = DRIVERLESS_MISSIONS::MISSION_TRACK;
        or_ros_state->lap_count = this->ros_state.lap_count;
        or_ros_state->navigation_ready = this->ros_state.navigation_ready;
        or_ros_state->header.stamp = this->now();
        this->state_pub_->publish(std::move(or_ros_state));

        return;
    }
    // CAN publisher
    auto heartbeat = Compose_DVL_Heartbeat(&this->DVL_heartbeat);
    this->can_pub_->publish(std::move(this->_d_2_f(heartbeat.id, true, heartbeat.data, sizeof(heartbeat.data))));

    // ROS publisher
    driverless_msgs::msg::State::UniquePtr state_msg(new driverless_msgs::msg::State());
    state_msg->header.stamp = this->now();
    state_msg->state = this->DVL_heartbeat.stateID;
    state_msg->mission = this->ros_state.mission;
    state_msg->lap_count = this->ros_state.lap_count;
    state_msg->navigation_ready = this->ros_state.navigation_ready;
    this->state_pub_->publish(std::move(state_msg));
}

void ASSupervisor::dvl_heartbeat_timer_callback() {
    run_fsm();
    publish_heartbeart();
}

void ASSupervisor::res_alive_timer_callback() {
    if (!this->res_alive) {
        this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
        RCLCPP_WARN(this->get_logger(), "RES TIMEOUT: Attemping to start RES");
        uint8_t p[8] = {0x80, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        this->can_pub_->publish(std::move(this->_d_2_f(0x00, false, p, sizeof(p))));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        uint8_t p2[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        this->can_pub_->publish(std::move(this->_d_2_f(0x00, false, p2, sizeof(p2))));
    }
    this->res_alive = 0;
}

void ASSupervisor::dataLogger_timer_callback() {
    // system status
    auto systemStatusMsg = Compose_DVL_SystemStatus(&this->DVL_systemStatus);
    this->can_pub_->publish(
        std::move(this->_d_2_f(systemStatusMsg.id, false, systemStatusMsg.data, sizeof(systemStatusMsg.data))));

    driverless_msgs::msg::SystemStatus systemStatus_ROSmsg;
    systemStatus_ROSmsg.as_state = this->DVL_systemStatus._fields.AS_state;
    systemStatus_ROSmsg.ebs_state = this->DVL_systemStatus._fields.EBS_state;
    systemStatus_ROSmsg.ami_state = this->DVL_systemStatus._fields.AMI_state;
    systemStatus_ROSmsg.steering_state = this->DVL_systemStatus._fields.steering_state;
    systemStatus_ROSmsg.service_brake_state = this->DVL_systemStatus._fields.service_brake_state;
    systemStatus_ROSmsg.lap_counter = this->DVL_systemStatus._fields.lap_counter;
    systemStatus_ROSmsg.cones_count_actual = this->DVL_systemStatus._fields.cones_count_actual;
    systemStatus_ROSmsg.cones_count_all = this->DVL_systemStatus._fields.cones_count_all;

    this->logging_systemStatus_pub_->publish(systemStatus_ROSmsg);

    // very small delay between messages
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // driving dynamics 1
    auto drivingDynamics1Msg = Compose_DVL_DrivingDynamics1(&this->DVL_drivingDynamics1);
    this->can_pub_->publish(std::move(
        this->_d_2_f(drivingDynamics1Msg.id, false, drivingDynamics1Msg.data, sizeof(drivingDynamics1Msg.data))));

    driverless_msgs::msg::DrivingDynamics1 drivingDynamics1_ROSmsg;
    drivingDynamics1_ROSmsg.speed_actual = this->DVL_drivingDynamics1._fields.speed_actual;
    drivingDynamics1_ROSmsg.speed_target = this->DVL_drivingDynamics1._fields.speed_target;
    drivingDynamics1_ROSmsg.steering_angle_actual = this->DVL_drivingDynamics1._fields.steering_angle_actual;
    drivingDynamics1_ROSmsg.steering_angle_target = this->DVL_drivingDynamics1._fields.steering_angle_target;
    drivingDynamics1_ROSmsg.brake_hydr_actual = this->DVL_drivingDynamics1._fields.brake_hydr_actual;
    drivingDynamics1_ROSmsg.brake_hydr_target = this->DVL_drivingDynamics1._fields.brake_hydr_target;
    drivingDynamics1_ROSmsg.motor_moment_actual = this->DVL_drivingDynamics1._fields.motor_moment_actual;
    drivingDynamics1_ROSmsg.motor_moment_target = this->DVL_drivingDynamics1._fields.motor_moment_target;

    this->logging_drivingDynamics1_pub_->publish(drivingDynamics1_ROSmsg);
}

void ASSupervisor::run_fsm() {
    // by default, no torque
    this->DVL_heartbeat.torqueRequest = 0.0;

    // Starting state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_START) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;

        // reset mission selections
        this->ros_state.mission = DVL_MISSION::DVL_MISSION_NONE;
        this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_NONE;

        // transition to select mission when res switch is backwards
        if (!this->RES_status.sw_k2) {
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_SELECT_MISSION;
        }
    }

    // Select mission state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_SELECT_MISSION) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;
        if (this->SW_heartbeat.stateID == sw_state_t::SW_STATE_SELECT_MISSION ||
            this->SW_heartbeat.stateID == sw_state_t::SW_STATE_MISSION_ACK) {
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_WAIT_FOR_MISSION;
        }
    }

    // Wait for mission state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_WAIT_FOR_MISSION) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;
        if (this->SW_heartbeat.stateID == sw_state_t::SW_STATE_MISSION_ACK) {
            this->ros_state.mission = this->SW_heartbeat.missionID;
        }

        if (this->ros_state.mission != DVL_MISSION::DVL_MISSION_NONE) {
            // set mission for logging
            if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_INSPECTION) {
                this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_INSPECTION;
            } else if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_EBS) {
                this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_BRAKETEST;
            } else if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_TRACK) {
                this->DVL_systemStatus._fields.AMI_state = DVL_AMI_State::DVL_AMI_STATE_TRACKDRIVE;
            } else {
                this->DVL_systemStatus._fields.AMI_state = 0;
            }

            // transition to Check EBS state when mission is selected
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_CHECK_EBS;

            // set the DVL mission IDs according to selection
            if (this->ros_state.mission == DVL_MISSION::DVL_MISSION_MANUAL) {
                this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_MANUAL;
            } else {
                this->DVL_heartbeat.missionID = DVL_MISSION::DVL_MISSION_SELECTED;
            }
        }
    }

    // Check EBS state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_CHECK_EBS) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_OFF;

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_READY && this->RES_status.sw_k2) {
            // transition to Ready state when EBS reports EBS checks complete and res switch is forward
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

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_SHUTDOWN) {
            // transition to E-Stop state when EBS reports shutdown
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
        }
    }

    // Release brake state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_RELEASE_EBS) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_DRIVING;

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_DRIVE) {
            // transition to Driving state when brakes r good
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_DRIVING;
        }

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_SHUTDOWN) {
            // transition to E-Stop state when EBS reports shutdown
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

        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_SHUTDOWN) {
            // transition to E-Stop state when EBS reports shutdown
            this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_EMERGENCY;
        }
    }

    // EBS Activated state
    if (this->DVL_heartbeat.stateID == DVL_STATES::DVL_STATE_ACTIVATE_EBS) {
        this->DVL_systemStatus._fields.AS_state = DVL_AS_State::DVL_AS_STATE_FINISH;

        // this state activates the EBS without tripping shutdown
        // used at end of missions

        // if the EBS says its braking, we go to finished
        if (this->EBS_heartbeat.stateID == EBS_CTRL_STATE::EBS_CTRL_STATE_BRAKE_ACTIVATE) {
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

void ASSupervisor::reset_dataLogger() {
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

ASSupervisor::ASSupervisor(const rclcpp::NodeOptions &options, std::string name) : Node(name, options) {
    // Setup inital states
    this->ros_state.state = DVL_STATES::DVL_STATE_START;
    this->ros_state.mission = DVL_MISSION::DVL_MISSION_NONE;
    this->ros_state.lap_count = 0;
    this->ros_state.navigation_ready = false;
    this->DVL_heartbeat.stateID = DVL_STATES::DVL_STATE_START;

    this->declare_parameter("manual_override", false);

    this->reset_dataLogger();

    sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ctrl_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sensor_opt = rclcpp::SubscriptionOptions();
    sensor_opt.callback_group = sensor_cb_group_;
    auto ctrl_opt = rclcpp::SubscriptionOptions();
    ctrl_opt.callback_group = ctrl_cb_group_;
    auto timer_opt = rclcpp::SubscriptionOptions();
    timer_opt.callback_group = timer_cb_group_;

    // CAN
    this->can_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "/can/canbus_rosbound", QOS_ALL, std::bind(&ASSupervisor::can_callback, this, _1), sensor_opt);
    this->canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "/can/canopen_rosbound", QOS_ALL, std::bind(&ASSupervisor::canopen_callback, this, _1), sensor_opt);

    // Steering sub
    this->steering_angle_sub_ = this->create_subscription<driverless_msgs::msg::Float32Stamped>(
        "/vehicle/steering_angle", QOS_LATEST, std::bind(&ASSupervisor::steering_angle_callback, this, _1), sensor_opt);

    // Velocity sub
    this->velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/vehicle/velocity", QOS_LATEST, std::bind(&ASSupervisor::velocity_callback, this, _1), sensor_opt);

    // Control -> sub to acceleration command
    this->control_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/control/accel_command", QOS_LATEST, std::bind(&ASSupervisor::control_callback, this, _1), ctrl_opt);

    // Lap counter sub
    this->lap_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/system/laps_completed", QOS_LATEST, std::bind(&ASSupervisor::lap_counter_callback, this, _1), ctrl_opt);

    // steering ready sub
    this->steering_ready_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/system/steering_ready", QOS_LATEST, std::bind(&ASSupervisor::steering_state_callback, this, _1), ctrl_opt);

    // Shutdown emergency
    this->shutdown_sub_ = this->create_subscription<driverless_msgs::msg::Shutdown>(
        "/system/shutdown", 10, std::bind(&ASSupervisor::shutdown_callback, this, _1), timer_opt);

    // AS Heartbeat
    this->heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), std::bind(&ASSupervisor::dvl_heartbeat_timer_callback, this), timer_cb_group_);

    // RES Alive
    this->res_alive_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4000), std::bind(&ASSupervisor::res_alive_timer_callback, this), timer_cb_group_);

    // Data Logger
    this->dataLogger_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ASSupervisor::dataLogger_timer_callback, this), timer_cb_group_);

    // Publishers
    this->logging_drivingDynamics1_pub_ =
        this->create_publisher<driverless_msgs::msg::DrivingDynamics1>("/data_logger/drivingDynamics1", 10);
    this->logging_systemStatus_pub_ =
        this->create_publisher<driverless_msgs::msg::SystemStatus>("/data_logger/systemStatus", 10);

    // Outgoing CAN
    this->can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", 10);

    // State pub
    this->state_pub_ = this->create_publisher<driverless_msgs::msg::State>("/system/as_status", 10);

    // RES status pub
    // this->res_status_pub_ = this->create_publisher<driverless_msgs::msg::RES>("/system/res_status", 10);

    RCLCPP_INFO(this->get_logger(), "---Vehicle Supervisor Node Initialised---");
}

}  // namespace vehicle_supervisor

RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_supervisor::ASSupervisor);
