#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "can_interface.hpp"
#include "canopen.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/state.hpp"
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
} c53_state_id_t;

std::tuple<std::string, int, int> states[8] = {
    // Name, mask, stateid
    {"Not ready to switch on", 0b0000000001001111, NRTSO}, {"Switch on disabled", 0b0000000001001111, SOD},
    {"Ready to switch on", 0b0000000001101111, RTSO},      {"Switched on", 0b0000000001101111, SO},
    {"Operation enabled", 0b0000000001101111, OE},         {"Quick stop active", 0b0000000001101111, QSA},
    {"Fault reaction active", 0b0000000001001111, FRA},    {"Fault", 0b0000000001001111, F}};

typedef struct c5e_config {
    int32_t default_velocity;
    uint32_t default_accelerations;
    uint32_t default_limits;
    uint32_t default_current;

    std::string to_string() {
        std::stringstream ss;
        ss << "vel: " << this->default_velocity << " acc: " << this->default_accelerations
           << " lims: " << this->default_limits << " curr: " << this->default_current << std::endl;
        return ss.str();
    }
} c5e_config_t;

class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    int32_t target;
    int32_t velocity;
    int32_t current;
    std::pair<uint32_t, uint32_t> accelerations;
    std::pair<int32_t, int32_t> limits;
    c5e_config_t defaults;

    std::tuple<std::string, int, int> c5e_state;
    rclcpp::TimerBase::SharedPtr c5e_state_timer;

    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;

    driverless_msgs::msg::State state;

    void state_callback(const driverless_msgs::msg::State msg) {
        this->state = msg;
        if (msg.state == driverless_msgs::msg::State::READY || msg.state == driverless_msgs::msg::State::DRIVING ||
            msg.state == driverless_msgs::msg::State::ACTIVATE_EBS ||
            msg.state == driverless_msgs::msg::State::EMERGENCY) {
            // Enable motor
            this->enable();
        } else {
            this->shutdown();
        }
    }

    void steering_callback(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        // Validate driving state
        float cappedAngle = std::fmax(std::fmin(msg->steering_angle, 1), -1);
        int32_t steeringDemandStepper = cappedAngle * this->limits.first;

        this->target_position(steeringDemandStepper);
    }

    void c5e_state_callback() {
        // Send a request for a statusword packet
        uint32_t statusword_id;
        uint8_t statusword_data[8];
        sdo_read(C5_E_ID, 0x6041, 0x00, &statusword_id, (uint8_t *)&statusword_data);
        this->can_pub->publish(_d_2_f(statusword_id, 0, statusword_data));
    }

    void can_callback(const driverless_msgs::msg::Can msg) {
        switch (msg.id) {
            case (0x5F0):
                // Can message from the steering actuator
                {
                    uint16_t statusword_id = 0x6041;
                    if (msg.data[0] == 0x4B && msg.data[1] == (statusword_id & 0xFF) &&
                        msg.data[2] == ((statusword_id >> 8) & 0xFF)) {
                        // Status Word
                        uint16_t state = (msg.data[3] << 8 | msg.data[4]);
                        auto parsed_state = this->parse_state(state);
                    }
                }
                break;
            default:
                break;
        }
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters) {
            if (param.get_name() == "d_acceleration") {
                RCLCPP_INFO(this->get_logger(), "Setting acceleration to %li.", param.as_int());
                this->set_acceleration(std::make_pair<uint32_t, uint32_t>(param.as_int(), param.as_int()));

            } else if (param.get_name() == "d_velocity") {
                RCLCPP_INFO(this->get_logger(), "Setting velocity to %li.", param.as_int());
                this->set_velocity(param.as_int());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Do not set current and limits on the fly. Request ignored.");
                result.successful = false;
                result.reason = "Do not set current and limits on the fly.";
            }
        }

        return result;
    }

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle;

   public:
    SteeringActuator() : Node("steering") {
        // Defaults
        this->declare_parameter<int>("d_acceleration", 0);
        this->declare_parameter<int>("d_current", 0);
        this->declare_parameter<int>("d_limits", 0);
        this->declare_parameter<int>("d_velocity", 0);

        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("canbus_carbound", 10);

        this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
            "as_status", 10, std::bind(&SteeringActuator::state_callback, this, _1));

        this->ackermann = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "driving_command", 10, std::bind(&SteeringActuator::steering_callback, this, _1));

        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&SteeringActuator::can_callback, this, _1));

        this->c5e_state_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                                        std::bind(&SteeringActuator::c5e_state_callback, this));

        this->parameter_callback_handle =
            this->add_on_set_parameters_callback(std::bind(&SteeringActuator::parameter_callback, this, _1));

        uint32_t _d_acceleration = 0;
        uint32_t _d_current = 0;
        uint32_t _d_limits = 0;
        int32_t _d_velocity = 0;
        this->get_parameter("d_acceleration", _d_acceleration);
        this->get_parameter("d_current", _d_current);
        this->get_parameter("d_limits", _d_limits);
        this->get_parameter("d_velocity", _d_velocity);

        c5e_config_t config;
        config.default_accelerations = _d_acceleration;
        config.default_current = _d_current;
        config.default_limits = _d_limits;
        config.default_velocity = _d_velocity;

        RCLCPP_INFO(this->get_logger(), "Using c5e_config: %s", config.to_string().c_str());

        this->velocity = config.default_velocity;
        this->accelerations =
            std::make_pair<int32_t, int32_t>(config.default_accelerations, config.default_accelerations);
        this->limits = std::make_pair<int32_t, int32_t>(config.default_limits, -config.default_limits);

        this->current = config.default_current;
        this->target = 0;

        this->setup();
    }

    void setup() {
        /* To initialise the controller to a usable state, we must set the:
        Target Position = 0
        Min Position Limit = -DEFAULT_LIMITS
        Max Position Limit = DEFAULT_LIMITS
        Home Offset = 0
        Motion Profile Type = trapezoidal ramp
        Profile Velocity = DEFAULT_VELOCITY
        End Velocity = 0
        Profile Acceleration = DEFAULT_ACCELERATIONS
        Profile Deceleration = DEFAULT_ACCELERATIONS
        Quick Stop Deceleration = DEFAUL_ACCELERATIONS
        Max Acceleration = DEFAULT_ACCELERATIONS
        MAX Deceleration = DEFAULT_ACCELERATIONS
        Mode of Operation = 1 (Profile Position)
    */

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        std::cout << "Performing C5-E Setup... ";

        sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x607D, 0x01, (uint8_t *)&this->limits.second, 4, &id, out);  // Min Limit
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x607D, 0x02, (uint8_t *)&this->limits.first, 4, &id, out);  // Max Limit
        this->can_pub->publish(_d_2_f(id, 0, out));

        int32_t ho = 0;
        sdo_write(C5_E_ID, 0x607C, 0x00, (uint8_t *)&ho, 4, &id, out);  // Home Offset
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6086, 0x00, (uint8_t *)&ho, 2, &id, out);  // Motion Profile Type (trap)
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);  // Profile Velocity
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6082, 0x00, (uint8_t *)&ho, 4, &id, out);  // End Velocity
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Profile Accelerataion
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);  // Profile Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
                  out);  // Quick Stop Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Max Acceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);  // Max Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        int8_t ppm = 1;
        sdo_write(C5_E_ID, 0x6060, 0x00, (uint8_t *)&ppm, 1, &id, out);  // Modes of Operation
        this->can_pub->publish(_d_2_f(id, 0, out));

        std::cout << "Done (Motor Configured)" << std::endl;
    }

    void target_position(int32_t target) {
        // Set Target
        this->target = target;

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        uint16_t control_word = 47;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out));

        // sus
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
        this->can_pub->publish(_d_2_f(id, 0, out));

        // Set Control Word
        control_word = 63;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out));
    }

    void target_position(int32_t target, int32_t velocity) {
        this->target = target;

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_write(C5_E_ID, 0x607A, 0x00, (uint8_t *)&this->target, 4, &id, out);  // Target
        this->can_pub->publish(_d_2_f(id, 0, out));

        this->velocity = velocity;

        sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);  // Profile Velocity
        this->can_pub->publish(_d_2_f(id, 0, out));

        uint16_t control_word = 47;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Set Control Word
        control_word = 63;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out));
    }

    void set_velocity(int32_t velocity) {
        this->velocity = velocity;

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_write(C5_E_ID, 0x6081, 0x00, (uint8_t *)&this->velocity, 4, &id, out);  // Profile Velocity
        this->can_pub->publish(_d_2_f(id, 0, out));
    }

    void set_acceleration(std::pair<uint32_t, uint32_t> accelerations) {
        this->accelerations = accelerations;

        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out

        sdo_write(C5_E_ID, 0x6083, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
                  out);  // Profile Accelerataion
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6084, 0x00, (uint8_t *)&this->accelerations.second, 4, &id,
                  out);  // Profile Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x6085, 0x00, (uint8_t *)&this->accelerations.first, 4, &id,
                  out);  // Quick Stop Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x60C5, 0x00, (uint8_t *)&this->accelerations.first, 4, &id, out);  // Max Acceleration
        this->can_pub->publish(_d_2_f(id, 0, out));

        sdo_write(C5_E_ID, 0x60C6, 0x00, (uint8_t *)&this->accelerations.second, 4, &id, out);  // Max Deceleration
        this->can_pub->publish(_d_2_f(id, 0, out));
    }

    void shutdown() {
        uint32_t id;
        uint8_t out[8];
        if (std::get<2>(this->c5e_state) == SO) return;
        uint16_t control_word = 6;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Shutdown
        this->can_pub->publish(_d_2_f(id, 0, out));
        while (std::get<2>(this->c5e_state) != SO)
            ;
    }

    void enable() {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        if (std::get<2>(this->c5e_state) == OE) return;
        uint16_t control_word = 6;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Shutdown
        this->can_pub->publish(_d_2_f(id, 0, out));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        control_word = 7;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Switched On
        this->can_pub->publish(_d_2_f(id, 0, out));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        control_word = 15;
        sdo_write(C5_E_ID, 0x6040, 0x00, (uint8_t *)&control_word, 2, &id, out);  // Op Enabled
        this->can_pub->publish(_d_2_f(id, 0, out));

        while (std::get<2>(this->c5e_state) != OE)
            ;
    }

    void set_c5e_config(c5e_config_t config) {
        this->defaults.default_accelerations = config.default_accelerations;
        this->defaults.default_current = config.default_current;
        this->defaults.default_limits = config.default_limits;
        this->defaults.default_velocity = config.default_velocity;
    }

    std::tuple<std::string, int, int> parse_state(uint16_t msg) {
        for (int i = 0; i < 8; i++) {
            if ((msg & std::get<1>(states[i])) == std::get<2>(states[i])) {
                if (this->c5e_state != states[i]) {
                    RCLCPP_INFO(this->get_logger(), "%s >>>> %s", std::get<0>(this->c5e_state).c_str(),
                                std::get<0>(states[i]).c_str());
                }
                this->c5e_state = states[i];
                return states[i];
            }
        }
        return states[7];
    }

    c5e_config_t get_c5e_config() { return this->defaults; }
};

std::function<void(int)> handler;
void signal_handler(int signal) { handler(signal); }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Hack
    auto x = std::make_shared<SteeringActuator>();
    signal(SIGINT, signal_handler);
    handler = [x](int signal) {
        x->shutdown();
        RCLCPP_INFO(x->get_logger(), "Motor shutdown, exiting node.");
        rclcpp::shutdown();
        return signal;
    };

    rclcpp::spin(x);
    rclcpp::shutdown();
    return 0;
}
