#include "component_supervisor_launch.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vehicle_supervisor {

void ASSupervisorLaunch::launch_mission() {
    // run the mission program based on the mission selected
    // command: ros2 run mission_controller MISSION_handler_node
    std::string command = "ros2 run mission_controller ";
    if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_INSPECTION) {
        RCLCPP_INFO(this->get_logger(), "Launching Inspection Mission");
        command += "inspection";
    } else if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_EBS) {
        RCLCPP_INFO(this->get_logger(), "Launching EBS Mission");
        command += "ebs";
    } else if (this->ros_state.mission == DRIVERLESS_MISSIONS::MISSION_TRACK) {
        RCLCPP_INFO(this->get_logger(), "Launching Trackdrive Mission");
        command += "trackdrive";
    } else {
        RCLCPP_INFO(this->get_logger(), "Manual driving, no action required");
    }
    command += "_handler_node &";
    // run command without blocking (ampersand at end)
    system(command.c_str());

    RCLCPP_INFO(this->get_logger(), "Mission Launched");
}

void ASSupervisorLaunch::run_fsm() {
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
            ASSupervisorLaunch::launch_mission();
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

void ASSupervisorLaunch::dvl_heartbeat_timer_callback() {
    run_fsm();
    publish_heartbeart();
}

ASSupervisorLaunch::ASSupervisorLaunch(const rclcpp::NodeOptions & options) : ASSupervisor(options, "vehicle_supervisor_launch_node") {
    // override the heartbeat timer
    this->heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&ASSupervisorLaunch::dvl_heartbeat_timer_callback, this));
}

}  // namespace vehicle_supervisor

RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_supervisor::ASSupervisorLaunch)
