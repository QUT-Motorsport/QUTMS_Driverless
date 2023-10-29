#include <chrono>  // Timer library
#include <map>     // Container library
#include <bitset>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"  // ROS Messages
#include "can_interface.hpp"  // CAN interface library to convert data array into a canbus frame (data_2_frame)
#include "canopen.hpp"        // CAN library to communicate systems via sdo_read and sdo_write
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"  // ROS Messages
#include "driverless_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"  // C++ Required Libraries
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "steering_actuator/steering_common.hpp"

using std::placeholders::_1;

// Steering Actuation Class
class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    // Creates
    rclcpp::TimerBase::SharedPtr steering_update_timer;
    rclcpp::TimerBase::SharedPtr state_request_timer;

    // Creates publishers and subscribers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_ang_sub;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr canopen_sub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;

    // params
    double Kp, Ki, Kd, centre_range;

    bool shutdown_requested = false;
    driverless_msgs::msg::State state;
    c5e_state desired_state = states[RTSO];
    c5e_state current_state = states[NRTSO];
    uint16_t control_method = MODE_RELATIVE;
    bool motor_enabled = true;
    bool centred = false;
    int32_t offset = 0;
    int settled_count = 0;
    bool initial_enc_saved = false;
    int32_t initial_enc = 0;
    bool steering_ang_received = false;
    double current_steering_angle = 0;
    double center_angle = -2.0;  // angle sensor has a -2 deg offset
    int32_t current_enc_revolutions = 0;                // Current Encoder Revolutions (Stepper encoder)
    uint32_t current_velocity;
    uint32_t current_acceleration;

    // time to reset node if no state received
    std::chrono::time_point<std::chrono::system_clock> last_state_time = std::chrono::system_clock::now();

    void configure_c5e() {
        this->send_steering_data(PROFILE_VELOCITY, (uint8_t *)&this->current_velocity, 4);
        this->send_steering_data(PROFILE_ACCELERATION, (uint8_t *)&this->current_acceleration, 4);
        this->send_steering_data(PROFILE_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
        this->send_steering_data(QUICK_STOP_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
        this->send_steering_data(MAX_ACCELERATION, (uint8_t *)&this->current_acceleration, 4);
        this->send_steering_data(MAX_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
        // this->send_steering_data(HOME_OFFSET, (uint8_t *)&this->offset, 4);

        RCLCPP_INFO(this->get_logger(), "Configured C5E");
    }

    void c5e_state_request_callback() {
        this->read_steering_data(STATUS_WORD);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Requesting state");
    }

    // Check State to enable or disable motor
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

    // Get steering angle reading
    void steering_angle_callback(const std_msgs::msg::Float32 msg) {
        this->current_steering_angle = msg.data;
        if (!this->steering_ang_received) this->steering_ang_received = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Steering angle received");
        RCLCPP_DEBUG(this->get_logger(), "Current angle: %f", msg.data);
    }

    // Get desired steering angle to update steering
    void driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) {
        if (!this->motor_enabled || !this->centred) return;

        double requested_steering_angle = msg.drive.steering_angle;
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Requested angle: %f", msg.drive.steering_angle);
        // turning left eqn: -83.95x - 398.92
        // turning right eqn: -96.19x - 83.79
        int32_t target;
        if (requested_steering_angle > this->center_angle) {
            target = int32_t(-86.45 * requested_steering_angle - 398.92) - this->offset;
        } else {
            target = int32_t(-94.58 * requested_steering_angle - 83.79) - this->offset;
        }
        target = std::max(std::min(target, 7500 - this->offset), -7500 - this->offset);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Target: %f = %d", requested_steering_angle, target);
        this->target_position(target);
    }

    // Receive message from CAN
    void can_callback(const driverless_msgs::msg::Can msg) {
        // RCLCPP_INFO(this->get_logger(), "CAN ID: %x", msg.id);
        // Message ID from the steering actuator
        if (msg.id == C5E_EMCY_ID) {
            // first two bytes are the error code
            uint16_t error_code = (msg.data[1] << 8 | msg.data[0]);
            RCLCPP_ERROR(this->get_logger(), "Emergency error code: %x", error_code);
        } else if (msg.id == C5E_BOOT_UP_ID) {
            RCLCPP_INFO(this->get_logger(), "C5E Booted Up");
        } else if (msg.id == C5E_POS_ID) {
            // first 4 bytes are the position (int32)
            uint32_t data = 0;
            // iterate to get the 4 bytes
            for (int i = 0; i < 4; i++) {
                data |= msg.data[i] << (8 * i);
            }

            // message received for position revolution
            int32_t val = (int32_t)data;
            this->current_enc_revolutions = val;
            if (!this->initial_enc_saved) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Position received");
                this->initial_enc = val;
                this->initial_enc_saved = true;
            }
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Encoder: %d", val);
            
            std_msgs::msg::Int32 enc_msg;
            enc_msg.data = this->current_enc_revolutions;
            this->encoder_pub->publish(enc_msg);
        } else if (msg.id == C5E_SRV_ID) {
            // objects returned from sdo read
            uint16_t object_id = (msg.data[2] & 0xFF) << 8 | (msg.data[1] & 0xFF);

            // if (msg.data[0] == 0x60 || msg.data[0] == 0x80) {
            if (msg.data[0] == 0x80) {
                // acknowledgements of sdo write
                // 0x60 -> success ack
                // 0x80 -> error ack
                RCLCPP_INFO(this->get_logger(), "ACK object: %x, %x", object_id, msg.data[0]);
                return;
            }
            // RCLCPP_INFO(this->get_logger(), "Object: %x, %x", object_id, msg.data[0]);

            if (object_id == STATUS_WORD) {
                uint16_t status_word = (msg.data[5] << 8 | msg.data[4]);
                this->current_state = parse_state(status_word);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Status word: %s", std::bitset<16>(status_word).to_string().c_str());
                RCLCPP_DEBUG(this->get_logger(), "Current state: %s", this->current_state.name.c_str());

            if (this->motor_enabled) {
                if (this->current_state == states[RTSO]) {
                    this->desired_state = states[SO];
                } else if (this->current_state == states[SO]) {
                    this->desired_state = states[OE];
                    // default params
                    this->configure_c5e();
                } else if (this->current_state == states[OE]) {
                    // if (!this->centred && this->steering_ang_received) {
                    //     this->pre_op_centering();
                    // }
                    // stay here -> no transition
                    } else {
                        this->desired_state = states[RTSO];
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
            RCLCPP_DEBUG(this->get_logger(), "Desired state: %s", this->desired_state.name.c_str());

                // State transition stage via state map definitions
            if (this->current_state != this->desired_state) {
                RCLCPP_INFO(this->get_logger(), "Current state: %s", this->current_state.name.c_str());
                RCLCPP_INFO(this->get_logger(), "Sending state: %s", this->desired_state.name.c_str());
                    this->send_steering_data(CONTROL_WORD, (uint8_t *)&this->desired_state.control_word, 2);
                }
            }
        }
    }

    // Update Steering
    void pre_op_centering() {
        if (!this->motor_enabled || !this->steering_ang_received || this->centred || this->current_state != states[OE]) return;  // if multithread doesn't work

        RCLCPP_INFO_ONCE(this->get_logger(), "Centering");
        // center steering
        // splitting into threads means steering angle will be updated separately to this function
        // while (this->settled_count < 200) {
        if (this->settled_count < 200) {  // if multithread doesn't work
            double error = this->current_steering_angle - this->center_angle;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error: %f", error);
            if (abs(error) < this->centre_range) {
                // motor has settled enough
                this->settled_count += 1;
            } else {
                // reset settled count to debounce initial overshoots
                this->settled_count = 0;
            }
            // left hand down is +, rhd is -
            double target = this->Kp * error;  // P commands to send to plant
            this->target_position(target);
            return;  // if multithread doesn't work
        }
        this->offset = this->initial_enc - this->current_enc_revolutions;
        this->centred = true;
        // save offset and configure c5e for different motion profile
        this->current_velocity = this->get_parameter("velocity").as_int();
        this->current_acceleration = this->get_parameter("acceleration").as_int();
        this->control_method = MODE_ABSOLUTE;
        RCLCPP_INFO_ONCE(this->get_logger(), "Centered, offset: %d", this->offset);
        this->configure_c5e();
        return;
    }

    void target_position(int32_t target) {
        if (this->current_state != states[OE]) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Not enabled");
            return;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Enabled, Targeting");

        this->send_steering_data(CONTROL_WORD, (uint8_t *)&this->control_method, 2);
        // print bitset control word
        // RCLCPP_INFO(this->get_logger(), "Control word: %s", std::bitset<16>(this->control_method).to_string().c_str());
        this->send_steering_data(TARGET_POSITION, (uint8_t *)&target, 4);
        uint16_t trigger_control_method = this->control_method | TRIGGER_MOTION;
        this->send_steering_data(CONTROL_WORD, (uint8_t *)&trigger_control_method, 2);
        // RCLCPP_INFO(this->get_logger(),"Trigger control word: %s", std::bitset<16>(trigger_control_method).to_string().c_str());
    }

    void send_steering_data(uint16_t obj_index, uint8_t* data, size_t data_size) {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        sdo_write(C5E_NODE_ID, obj_index, 0x00, (uint8_t *)data, data_size, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    void read_steering_data(uint16_t obj_index) {
        uint32_t id;     // Packet id out
        uint8_t out[8];  // Data out
        sdo_read(C5E_NODE_ID, obj_index, 0x00, &id, out);  // Control Word
        this->can_pub->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

   public:
    SteeringActuator() : Node("steering_actuator_node") {
        // Steering parameters
        this->declare_parameter<int>("velocity", 10000);
        this->declare_parameter<int>("velocity_centering", 100);
        this->declare_parameter<int>("acceleration", 2000);
        this->declare_parameter<int>("acceleration_centering", 100);
        this->declare_parameter<float>("centre_range", 5);
        // PID controller parameters
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<float>("Ki", 0.0);
        this->declare_parameter<float>("Kd", 0.0);

        this->centre_range = this->get_parameter("centre_range").as_double();
        this->Kp = this->get_parameter("Kp").as_double();
        this->Ki = this->get_parameter("Ki").as_double();
        this->Kd = this->get_parameter("Kd").as_double();
        this->current_velocity = this->get_parameter("velocity").as_int();
        this->current_acceleration = this->get_parameter("acceleration").as_int();

        callback_group_subscriber1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        // Create subscriber to topic "canbus_rosbound"
        this->canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canopen_rosbound", QOS_ALL, std::bind(&SteeringActuator::can_callback, this, _1), sub1_opt);

        // Create subscriber to topic "steering_angle"
        this->steer_ang_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/steering_angle", QOS_ALL, std::bind(&SteeringActuator::steering_angle_callback, this, _1), sub2_opt);

        // Create subscriber to topic "driving_command"
        this->ackermann_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1), sub2_opt);

        // Create subscriber to topic AS status
        // this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
        //     "/system/as_status", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1));

        this->steering_update_timer =
            this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SteeringActuator::pre_op_centering, this));
        
        // Create state request and config timers
        this->state_request_timer = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&SteeringActuator::c5e_state_request_callback, this), callback_group_subscriber1_);

        // Create publisher to topic "canbus_carbound"
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", QOS_ALL);

        // Create publisher to topic "encoder_reading"
        this->encoder_pub = this->create_publisher<std_msgs::msg::Int32>("/vehicle/encoder_reading", QOS_ALL);

        RCLCPP_INFO(this->get_logger(), "---Steering Actuator Node Initialised---");
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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    // rclcpp::spin(node);
    // rclcpp::shutdown();    
    return 0;
}
