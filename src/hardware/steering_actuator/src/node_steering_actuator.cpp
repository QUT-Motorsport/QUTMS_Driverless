#include <map>

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can_interface.hpp"
#include "canopen.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "driverless_msgs/msg/steering_reading.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

const int C5_E_ID = 0x70;

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
} c5e_object_id_t;

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

const std::string PARAM_ACCELERATION = "acceleration";
const std::string PARAM_VELOCITY = "velocity";

class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    rclcpp::TimerBase::SharedPtr c5e_state_request_timer;
    rclcpp::TimerBase::SharedPtr c5e_config_request_timer;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<driverless_msgs::msg::SteeringReading>::SharedPtr steering_reading_sub;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;

    driverless_msgs::msg::State state;
    c5e_state desired_state = states[RTSO];
    c5e_state current_state = states[RTSO];
    bool motor_enabled = false;
    double current_steering_angle = 0;
    double requested_steering_angle = 0;
    bool shutdown_requested = false;

    void c5e_state_request_callback() {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_read(C5_E_ID, STATUS_WORD, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    void c5e_config_request_callback() {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_read(C5_E_ID, HOME_OFFSET, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, MOTION_PROFILE_TYPE, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, PROFILE_VELOCITY, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, END_VELOCTITY, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, PROFILE_ACCELERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, PROFILE_DECELERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, QUICK_STOP_DECELERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, MAX_ACCELERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, MAX_DECELERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_read(C5_E_ID, MODE_OF_OPERATION, 0x00, &id, (uint8_t *)&out);
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    void can_callback(const driverless_msgs::msg::Can msg) {
        if (msg.id == 0x5F0) {
            // CAN message from the steering actuator

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

            switch (object_id) {
                case STATUS_WORD: {
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

                    RCLCPP_DEBUG(this->get_logger(), "Parsed state: %s", this->current_state.name.c_str());
                    RCLCPP_DEBUG(this->get_logger(), "Desired state: %s", this->desired_state.name.c_str());

                    if (this->current_state != this->desired_state) {
                        RCLCPP_DEBUG(this->get_logger(), "Sending state: %s", this->desired_state.name.c_str());
                        sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&this->desired_state.control_word, 2, &id,
                                  out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }

                    break;
                }

                    // To set the controller to a usable state, we must set the:
                    // Home Offset = 0
                    // Motion Profile Type = trapezoidal ramp (0)
                    // Profile Velocity = PARAM_VELOCITY
                    // End Velocity = 0
                    // Profile Acceleration = PARAM_ACCELERATION
                    // Profile Deceleration = PARAM_ACCELERATION
                    // Quick Stop Deceleration = PARAM_ACCELERATION
                    // Max Acceleration = PARAM_ACCELERATION
                    // Max Deceleration = PARAM_ACCELERATION
                    // Mode of Operation = 1 (Profile Position)

                case HOME_OFFSET: {
                    int32_t val = (int32_t)data;
                    RCLCPP_INFO(this->get_logger(), "HOME_OFFSET: %i", val);

                    int32_t desired_val = 0;
                    if (val != desired_val) {
                        sdo_write(C5_E_ID, HOME_OFFSET, 0x00, (uint8_t *)&desired_val, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case MOTION_PROFILE_TYPE: {
                    int16_t val = (int16_t)data;
                    RCLCPP_INFO(this->get_logger(), "MOTION_PROFILE_TYPE: %i", val);

                    int32_t desired_val = 0;
                    if (val != desired_val) {
                        sdo_write(C5_E_ID, MOTION_PROFILE_TYPE, 0x00, (uint8_t *)&desired_val, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case PROFILE_VELOCITY: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "PROFILE_VELOCITY: %u", val);

                    if (val != param_velocity) {
                        sdo_write(C5_E_ID, PROFILE_VELOCITY, 0x00, (uint8_t *)&param_velocity, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case END_VELOCTITY: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "END_VELOCTITY: %u", val);

                    uint32_t desired_val = 0;
                    if (val != desired_val) {
                        sdo_write(C5_E_ID, END_VELOCTITY, 0x00, (uint8_t *)&desired_val, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case PROFILE_ACCELERATION: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "PROFILE_ACCELERATION: %u", val);

                    if (val != param_acceleration) {
                        sdo_write(C5_E_ID, PROFILE_ACCELERATION, 0x00, (uint8_t *)&param_acceleration, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case PROFILE_DECELERATION: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "PROFILE_DECELERATION: %u", val);

                    if (val != param_acceleration) {
                        sdo_write(C5_E_ID, PROFILE_DECELERATION, 0x00, (uint8_t *)&param_acceleration, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case QUICK_STOP_DECELERATION: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "QUICK_STOP_DECELERATION: %u", val);

                    if (val != param_acceleration) {
                        sdo_write(C5_E_ID, QUICK_STOP_DECELERATION, 0x00, (uint8_t *)&param_acceleration, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case MAX_ACCELERATION: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "MAX_ACCELERATION: %u", val);

                    if (val != param_acceleration) {
                        sdo_write(C5_E_ID, MAX_ACCELERATION, 0x00, (uint8_t *)&param_acceleration, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case MAX_DECELERATION: {
                    uint32_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "MAX_DECELERATION: %u", val);

                    if (val != param_acceleration) {
                        sdo_write(C5_E_ID, MAX_DECELERATION, 0x00, (uint8_t *)&param_acceleration, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
                case MODE_OF_OPERATION: {
                    int8_t val = (uint32_t)data;
                    RCLCPP_INFO(this->get_logger(), "MODE_OF_OPERATION: %i", val);

                    int32_t desired_val = 1;
                    if (val != desired_val) {
                        sdo_write(C5_E_ID, MODE_OF_OPERATION, 0x00, (uint8_t *)&desired_val, 4, &id, out);
                        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
                    }
                    break;
                }
            }
        }
    }

    void as_state_callback(const driverless_msgs::msg::State msg) {
        this->state = msg;
        if (msg.state == driverless_msgs::msg::State::DRIVING ||
            msg.state == driverless_msgs::msg::State::ACTIVATE_EBS ||
            msg.state == driverless_msgs::msg::State::EMERGENCY) {
            // Enable motor
            this->motor_enabled = true;
        } else {
            this->motor_enabled = false;
        }
    }

    void steering_reading_callback(const driverless_msgs::msg::SteeringReading msg) {
        this->current_steering_angle = msg.steering_angle;
        this->update_steering();
    }

    void driving_command_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        float cappedAngle = std::fmax(std::fmin(msg->steering_angle, 90), -90);
        this->requested_steering_angle = cappedAngle;
        this->update_steering();
    }

    void update_steering() {
        // left hand down is +, rhd is -
        double steering_angle_difference = this->requested_steering_angle - this->current_steering_angle;

        double enc_to_des_angle = ((steering_angle_difference * 0.2443f) / 10.0f) * 3600.f;
        RCLCPP_INFO(this->get_logger(), "Diff: %lf, Enc: %lf", steering_angle_difference, -enc_to_des_angle);

        this->target_position(-enc_to_des_angle);
    }

    void target_position(int32_t target) {
        if (this->current_state != states[OE]) {
            return;
        }

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        uint16_t control_word;

        control_word = 111;
        sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        sdo_write(C5_E_ID, TARGET_POSITION, 0x00, (uint8_t *)&target, 4, &id, out);  // Target
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));

        // Set Control Word
        control_word = 127;
        sdo_write(C5_E_ID, CONTROL_WORD, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    c5e_state parse_state(uint16_t status_word) {
        for (const auto &[key, state] : states) {
            if ((status_word & state.mask) == state.state_id) {
                return state;
            }
        }
        return states[F];
    }

   public:
    SteeringActuator() : Node("steering") {
        // Defaults
        this->declare_parameter<int>(PARAM_ACCELERATION, 0);
        this->declare_parameter<int>(PARAM_VELOCITY, 0);

        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);

        this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
            "as_status", 10, std::bind(&SteeringActuator::as_state_callback, this, _1));

        this->steering_reading_sub = this->create_subscription<driverless_msgs::msg::SteeringReading>(
            "steering_reading", 10, std::bind(&SteeringActuator::steering_reading_callback, this, _1));

        this->ackermann = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "driving_command", 10, std::bind(&SteeringActuator::driving_command_callback, this, _1));

        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&SteeringActuator::can_callback, this, _1));

        this->c5e_state_request_timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SteeringActuator::c5e_state_request_callback, this));

        this->c5e_config_request_timer = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&SteeringActuator::c5e_config_request_callback, this));
    }

    void shutdown() { this->shutdown_requested = true; }
};

std::function<void(int)> handler;
void signal_handler(int signal) { handler(signal); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SteeringActuator>();

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
