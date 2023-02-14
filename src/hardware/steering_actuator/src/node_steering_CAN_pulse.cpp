#include <chrono>  // Timer library
#include <map>     // Container library

#include "CAN_VCU.h"
#include "can_interface.hpp"  // CAN interface library to convert data array into a canbus frame (data_2_frame)
#include "canopen.hpp"        // CAN library to communicate systems via sdo_read and sdo_write
#include "driverless_msgs/msg/can.hpp"  // ROS Messages
#include "driverless_msgs/msg/state.hpp"
#include "driverless_msgs/msg/steering_reading.hpp"
#include "rclcpp/rclcpp.hpp"  // C++ Required Libraries
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

const int C5_E_ID = 0x70;

/* State ID Definitions
 * NRTSO   = 0   Not ready to switch on
 * SOD     = 64  Switch on disabled
 * RTSO    = 33  Ready to switch on
 * SO      = 35  Switched on
 * OE      = 39  Operation enabled
 * QSA     = 7   Quick stop active
 * FRA     = 15  Fault reaction active
 * F       = 72  Fault
 */

typedef enum c5e_state_id {
    NRTSO = 0b0000000000000000,
    SOD = 0b0000000001000000,
    RTSO = 0b0000000000100001,
    SO = 0b0000000000100011,
    OE = 0b0000000000100111,
    QSA = 0b0000000000000111,
    FRA = 0b0000000000001111,
    F = 0b0000000001001000
} c5e_state_id_t;

/* Object ID Definitions
 * HOME_OFFSET               = 24700
 * MOTION_PROFILE_TYPE       = 24710
 * PROFILE_VELOCITY          = 24705
 * END_VELOCTITY             = 24706
 * PROFILE_ACCELERATION      = 24707
 * PROFILE_DECELERATION      = 24708
 * QUICK_STOP_DECELERATION   = 24709
 * MAX_ACCELERATION          = 24773
 * MAX_DECELERATION          = 24774
 * MODE_OF_OPERATION         = 24672
 * TARGET_POSITION           = 24698
 * CONTROL_WORD              = 24640
 * STATUS_WORD               = 24641
 * REVOLUTION_POS            = 24676
 */

typedef enum c5e_object_id {
    HOME_OFFSET = 0x607C,
    MOTION_PROFILE_TYPE = 0x6086,
    PROFILE_VELOCITY = 0x6081,
    END_VELOCTITY = 0x6082,
    PROFILE_ACCELERATION = 0x6083,
    PROFILE_DECELERATION = 0x6084,
    QUICK_STOP_DECELERATION = 0x6085,
    MAX_ACCELERATION = 0x60C5,
    MAX_DECELERATION = 0x60C6,
    MODE_OF_OPERATION = 0x6060,
    TARGET_POSITION = 0x607A,
    CONTROL_WORD = 0x6040,
    STATUS_WORD = 0x6041,
    REVOLUTION_POS = 0x6064,
} c5e_object_id_t;

/* State Structure
name - The state which the steering wheel is switched on, enabled, or at fault
mask - ?
state_id - State ID Number
control_word - ?
*/

struct c5e_state {
    std::string name;
    uint16_t mask;
    uint16_t state_id;
    uint16_t control_word;

    bool operator==(const c5e_state &rhs) {
        return (this->name == rhs.name && this->mask == rhs.mask && this->state_id == rhs.state_id);
    }

    bool operator!=(const c5e_state &rhs) { return !(*this == rhs); }
};

// State Map Definitions
std::map<uint16_t, c5e_state> states = {
    {NRTSO, {"Not ready to switch on", 0b0000000001001111, NRTSO, 0}},
    {SOD, {"Switch on disabled", 0b0000000001001111, SOD, 0}},
    {RTSO, {"Ready to switch on", 0b0000000001101111, RTSO, 6}},
    {SO, {"Switched on", 0b0000000001101111, SO, 7}},
    {OE, {"Operation enabled", 0b0000000001101111, OE, 15}},
    {QSA, {"Quick stop active", 0b0000000001101111, QSA, 0}},
    {FRA, {"Fault reaction active", 0b0000000001001111, FRA, 0}},
    {F, {"Fault", 0b0000000001001111, F, 0}},
};

void copy_data(const std::vector<uint8_t> &vec, uint8_t *dest, size_t n) {
    for (size_t i = 0; i < n; i++) {
        dest[i] = vec[i];
    }
}

// Steering Actuation Class
class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    driverless_msgs::msg::State state;       // State message
    c5e_state desired_state = states[RTSO];  // Desired State
    c5e_state current_state = states[RTSO];  // Current State
    bool motor_enabled = false;              // Enable motors logic
    bool centred = false;
    bool steering_ang_received = false;
    int32_t offset = 0;
    int settled_count = 0;
    int32_t pre_offset_target = 0;
    bool initial_enc_saved = false;
    int32_t initial_enc;
    bool shutdown_requested = false;         // Shutdown logic

    // Request callback for configuration (via ROS2)
    // These publishings are required to send a 'trigger' to the c5e controller
    // with the specific object ID (seen in struct above).
    // Then the c5e controller will send a CAN message back with that ID
    void c5e_state_request_callback() {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_read(C5_E_ID, STATUS_WORD, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, REVOLUTION_POS, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    // Receive message from CAN
    void can_callback(const driverless_msgs::msg::Can msg) {
        // Message ID from the steering actuator
        if (msg.id == 0x5F0) {
            uint32_t id;     // Packet id out
            uint8_t out[8];  // Data out

            if (msg.data[0] == 0x60 || msg.data[0] == 0x80) {
                // 0x60 -> success ack
                // 0x80 -> error ack
                return;
            }

            uint16_t object_id = (msg.data[2] & 0xFF) << 8 | (msg.data[1] & 0xFF);
            uint32_t data = 0;
            size_t size = can_open_size_map[msg.data[0]];
            for (size_t i = 0; i < size; i++) {
                data |= (msg.data[4 + i] & 0xFF) << i * 8;
            }

            uint32_t param_velocity = this->get_parameter(PARAM_VELOCITY).as_int();
            uint32_t param_acceleration = this->get_parameter(PARAM_ACCELERATION).as_int();

            // To set the controller to a usable state, we must set the:
            // Home Offset = zero - start
            // Motion Profile Type = trapezoidal ramp (0)
            // Profile Velocity = PARAM_VELOCITY
            // End Velocity = 0
            // Profile Acceleration = PARAM_ACCELERATION
            // Profile Deceleration = PARAM_ACCELERATION
            // Quick Stop Deceleration = PARAM_ACCELERATION
            // Max Acceleration = PARAM_ACCELERATION
            // Max Deceleration = PARAM_ACCELERATION
            // Mode of Operation = 1 (Profile Position)
            if (object_id == STATUS_WORD) {
                uint16_t status_word = (msg.data[3] << 8 | msg.data[4]);
                this->current_state = this->parse_state(status_word);

                if (this->motor_enabled) {
                    if (this->current_state == this->desired_state) {
                        // enabled transitions
                        if (this->current_state == states[RTSO]) {
                            this->desired_state = states[SO];
                        } else if (this->current_state == states[SO]) {
                            this->desired_state = states[OE];
                        } else if (this->current_state == states[OE]) {
                            // stay here -> no transition
                        } else {
                            this->desired_state == states[RTSO];
                        }
                    }
                } else {
                    // disabled transitions
                    this->desired_state = states[RTSO];
                }

                if (shutdown_requested) {
                    this->desired_state = states[RTSO];
                    if (this->current_state == this->desired_state) {
                        rclcpp::shutdown();
                    }
                }

                // Print current and desired states
                RCLCPP_DEBUG(this->get_logger(), "Parsed state: %s", this->current_state.name.c_str());
                RCLCPP_DEBUG(this->get_logger(), "Desired state: %s", this->desired_state.name.c_str());

                // State transition stage via state map definitions (seriously figure out what the hell sdo write
                // is)
                if (this->current_state != this->desired_state) {
                    RCLCPP_INFO(this->get_logger(), "Sending state: %s", this->desired_state.name.c_str());
                    sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&this->desired_state.control_word, 2, &id, out);
                    this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                }
            }
            // message received for position revolution
            else if (object_id == REVOLUTION_POS) {
                int32_t val = (int32_t)data;
                this->current_enc_revolutions = val;
                if (!this->initial_enc_saved) {
                    this->initial_enc = val;
                    this->initial_enc_saved = true;
                }
                std_msgs::msg::Int32 enc_msg;
                enc_msg.data = this->current_enc_revolutions;
                this->encoder_pub->publish(enc_msg);
            }
        }
    }

    // Figure out what control_worrd is and what it does
    void target_position(int32_t target) {
        if (this->current_state != states[OE]) {
            return;
        }

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        uint16_t control_word;

        if (this->centred) {
            control_word = 0b0101111;
        } else {
            control_word = 0b1101111;
        }
        sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_write(C5_E_ID, TARGET_POSITION, 0x00, (uint8_t *)&target, 4, &id, out);  // Target
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        if (this->centred) {
            control_word = 0b0111111;
        } else {
            control_word = 0b1111111;
        }
        sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    // Print state function
    c5e_state parse_state(uint16_t status_word) {
        for (const auto &[key, actuator_state] : states) {
            if ((status_word & actuator_state.mask) == actuator_state.state_id) {
                return actuator_state;
            }
        }
        return states[F];
    }

   public:
    SteeringActuator() : Node("steering_controller_node") {

        // Create publisher to topic "canbus_carbound"
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", 10);

        // Create publisher to topic "encoder_reading"
        this->encoder_pub = this->create_publisher<std_msgs::msg::Int32>("/vehicle/encoder_reading", 10);

        // Create subscriber to topic "canbus_rosbound"
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canbus_rosbound", 10, std::bind(&SteeringActuator::can_callback, this, _1));

        // Create state request and config timers
        this->c5e_state_request_timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SteeringActuator::c5e_state_request_callback, this));

        this->c5e_config_request_timer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&SteeringActuator::c5e_config_request_callback, this));
    }

    // Shutdown system
    void shutdown() { this->shutdown_requested = true; }
};

// Prototype functions initialisations?
std::function<void(int)> handler;
void signal_handler(int signal) { handler(signal); }

// Main loop
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);  // Initialise ROS2

    auto node = std::make_shared<SteeringActuator>();  // Constructs an empty SteeringActuator class

    // Hack
    signal(SIGINT, signal_handler);
    handler = [node](int signal) {
        RCLCPP_INFO(node->get_logger(), "Shutting down motor.");
        node->shutdown();
        return signal;
    };

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}