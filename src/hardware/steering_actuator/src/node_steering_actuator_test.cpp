#include <bitset>
#include <chrono>  // Timer library
#include <map>     // Container library

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"  // ROS Messages
#include "can_interface.hpp"  // CAN interface library to convert data array into a canbus frame (data_2_frame)
#include "canopen.hpp"        // CAN library to communicate systems via sdo_read and sdo_write
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"  // ROS Messages
#include "driverless_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"  // C++ Required Libraries
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "steering_actuator/steering_common.hpp"

using std::placeholders::_1;

// Steering Actuation Class
class SteeringActuator : public rclcpp::Node, public CanInterface {
   private:
    // Creates
    rclcpp::TimerBase::SharedPtr steering_update_timer_;
    rclcpp::TimerBase::SharedPtr state_request_timer_;

    // Creates publishers and subscribers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steering_ready_pub_;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_ang_sub_;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr canopen_sub_;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber2_;

    // params
    double Kp_, Ki_, Kd_, centre_range_, centre_angle_;
    int settling_iter_, max_position_;

    bool shutdown_requested_ = false;
    driverless_msgs::msg::State state_;
    c5e_state desired_state_ = states[RTSO];
    c5e_state current_state_ = states[NRTSO];
    uint16_t control_method_ = MODE_RELATIVE;
    bool motor_enabled_ = false;
    bool centred_ = false;
    int32_t offset_ = 0;
    int settled_count_ = 0;
    bool initial_enc_saved_ = false;
    int32_t initial_enc_ = 0;
    bool steering_ang_received_ = false;
    double current_steering_angle_ = 0;
    int32_t current_enc_revolutions_ = 0;  // Current Encoder Revolutions (Stepper encoder)
    uint32_t current_velocity_;
    uint32_t current_acceleration_;

    // time to reset node if no state received
    std::chrono::time_point<std::chrono::system_clock> last_state_time = std::chrono::system_clock::now();

    void configure_c5e() {
        this->send_steering_data(PROFILE_VELOCITY, (uint8_t *)&current_velocity_, 4);
        this->send_steering_data(PROFILE_ACCELERATION, (uint8_t *)&current_acceleration_, 4);
        this->send_steering_data(PROFILE_DECELERATION, (uint8_t *)&current_acceleration_, 4);
        this->send_steering_data(QUICK_STOP_DECELERATION, (uint8_t *)&current_acceleration_, 4);
        this->send_steering_data(MAX_ACCELERATION, (uint8_t *)&current_acceleration_, 4);
        this->send_steering_data(MAX_DECELERATION, (uint8_t *)&current_acceleration_, 4);
        // this->send_steering_data(HOME_OFFSET, (uint8_t *)&offset_, 4);

        RCLCPP_INFO(this->get_logger(), "Configured C5E");
    }

    void c5e_state_request_callback() {
        this->read_steering_data(STATUS_WORD);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Requesting state");
        std_msgs::msg::Bool msg;
        msg.data = centred_;
        steering_ready_pub_->publish(msg);
    }

    // Check State to enable or disable motor
    void as_state_callback(const driverless_msgs::msg::State msg) {
        state_ = msg;
        if (msg.state == driverless_msgs::msg::State::DRIVING ||
            msg.state == driverless_msgs::msg::State::ACTIVATE_EBS) {
            // Enable motor
            motor_enabled_ = true;
        } else {
            // Disable motor
            motor_enabled_ = false;
            centred_ = false;
            settled_count_ = 0;
            initial_enc_saved_ = false;
            steering_ang_received_ = false;
        }
    }

    // Get steering angle reading
    void steering_angle_callback(const std_msgs::msg::Float32 msg) {
        current_steering_angle_ = msg.data;
        if (!steering_ang_received_) steering_ang_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Steering angle received");
        RCLCPP_DEBUG(this->get_logger(), "Current angle: %f", msg.data);
    }

    // Get desired steering angle to update steering
    void driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) {
        if (!motor_enabled_ || !centred_) return;

        double requested_steering_angle = msg.drive.steering_angle;
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Requested angle: %f",
        // msg.drive.steering_angle); turning left eqn: -83.95x - 398.92 turning right eqn: -96.19x - 83.79
        int32_t target;
        if (requested_steering_angle > centre_angle_) {
            target = int32_t(-86.45 * requested_steering_angle - 398.92) - offset_;
        } else {
            target = int32_t(-94.58 * requested_steering_angle - 83.79) - offset_;
        }
        target = std::max(std::min(target, max_position_ - offset_), -max_position_ - offset_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250, "Target: %f = %d", requested_steering_angle,
                             target);
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
            current_enc_revolutions_ = val;
            if (!initial_enc_saved_) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Position received");
                // initial_enc_ = val;
                initial_enc_saved_ = true;
            }

            std_msgs::msg::Int32 enc_msg;
            enc_msg.data = current_enc_revolutions_;
            encoder_pub_->publish(enc_msg);
        } else if (msg.id == C5E_SRV_ID) {
            // objects returned from sdo read
            uint16_t object_id = (msg.data[2] & 0xFF) << 8 | (msg.data[1] & 0xFF);

            // if (msg.data[0] == 0x60 || msg.data[0] == 0x80) {
            if (msg.data[0] == 0x80) {
                // acknowledgements of sdo write: 0x60 -> success ack, 0x80 -> error ack
                RCLCPP_INFO(this->get_logger(), "ACK object: %x, %x", object_id, msg.data[0]);
                return;
            }

            if (object_id == STATUS_WORD) {
                uint16_t status_word = (msg.data[5] << 8 | msg.data[4]);
                current_state_ = parse_state(status_word);
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Status word: %s",
                                      std::bitset<16>(status_word).to_string().c_str());
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Current state: %s",
                                      current_state_.name.c_str());

                if (motor_enabled_) {
                    if (current_state_ == states[RTSO]) {
                        desired_state_ = states[SO];
                    } else if (current_state_ == states[SO]) {
                        desired_state_ = states[OE];
                        // default params
                        this->configure_c5e();
                    } else if (current_state_ == states[OE]) {
                        // stay here -> no transition
                    } else {
                        desired_state_ = states[RTSO];
                    }
                } else {
                    // disabled transitions
                    desired_state_ = states[RTSO];
                }

                if (shutdown_requested_) {
                    desired_state_ = states[RTSO];
                    if (current_state_ == desired_state_) {
                        rclcpp::shutdown();
                    }
                }

                // Print current and desired states
                RCLCPP_DEBUG(this->get_logger(), "Desired state: %s", desired_state_.name.c_str());

                // State transition stage via state map definitions
                if (current_state_ != desired_state_) {
                    RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.name.c_str());
                    RCLCPP_INFO(this->get_logger(), "Sending state: %s", desired_state_.name.c_str());
                    this->send_steering_data(CONTROL_WORD, (uint8_t *)&desired_state_.control_word, 2);
                }
            }
        }
    }

    // Update Steering
    void pre_op_centering() {
        if (!motor_enabled_ || !steering_ang_received_ || centred_ || current_state_ != states[OE])
            return;  // if multithread doesn't work

        RCLCPP_INFO_ONCE(this->get_logger(), "Centering");
        // center steering
        // splitting into threads means steering angle will be updated separately to this function
        if (settled_count_ < settling_iter_) {  // if multithread doesn't work
            double error = current_steering_angle_ - centre_angle_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error: %f", error);
            if (abs(error) < centre_range_) {
                // motor has settled enough
                settled_count_ += 1;
            } else {
                // reset settled count to debounce initial overshoots
                settled_count_ = 0;
            }
            // left hand down is +, rhd is -
            double target = Kp_ * error;  // P commands to send to plant
            this->target_position(target);
            return;  // if multithread doesn't work
        }
        offset_ = initial_enc_ - current_enc_revolutions_;
        centred_ = true;
        // save offset and configure c5e for different motion profile
        current_velocity_ = this->get_parameter("velocity").as_int();
        current_acceleration_ = this->get_parameter("acceleration").as_int();
        control_method_ = MODE_ABSOLUTE;
        RCLCPP_INFO_ONCE(this->get_logger(), "Centered, offset: %d", offset_);
        this->configure_c5e();
        return;
    }

    void target_position(int32_t target) {
        if (current_state_ != states[OE]) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Not enabled");
            return;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Enabled, Targeting");

        this->send_steering_data(CONTROL_WORD, (uint8_t *)&control_method_, 2);
        // print bitset control word
        // RCLCPP_INFO(this->get_logger(), "Control word: %s",
        // std::bitset<16>(control_method_).to_string().c_str());
        this->send_steering_data(TARGET_POSITION, (uint8_t *)&target, 4);
        uint16_t trigger_control_method = control_method_ | TRIGGER_MOTION;
        this->send_steering_data(CONTROL_WORD, (uint8_t *)&trigger_control_method, 2);
        // RCLCPP_INFO(this->get_logger(),"Trigger control word: %s",
        // std::bitset<16>(trigger_control_method).to_string().c_str());
    }

    void send_steering_data(uint16_t obj_index, uint8_t *data, size_t data_size) {
        uint32_t id;                                                                    // Packet id out
        uint8_t out[8];                                                                 // Data out
        sdo_write(C5E_NODE_ID, obj_index, 0x00, (uint8_t *)data, data_size, &id, out);  // Control Word
        can_pub_->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

    void read_steering_data(uint16_t obj_index) {
        uint32_t id;                                       // Packet id out
        uint8_t out[8];                                    // Data out
        sdo_read(C5E_NODE_ID, obj_index, 0x00, &id, out);  // Control Word
        can_pub_->publish(_d_2_f(id, 0, out, sizeof(out)));
    }

   public:
    SteeringActuator() : Node("steering_actuator_node") {
        // Steering parameters
        this->declare_parameter<int>("velocity", 10000);
        this->declare_parameter<int>("velocity_centering", 100);
        this->declare_parameter<int>("acceleration", 2000);
        this->declare_parameter<int>("acceleration_centering", 100);
        this->declare_parameter<float>("centre_range", 5.0);
        this->declare_parameter<float>("centre_angle", -2.0);
        this->declare_parameter<int>("settling_iter", 20);
        this->declare_parameter<int>("max_position", 7500);
        // PID controller parameters
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<float>("Ki", 0.0);
        this->declare_parameter<float>("Kd", 0.0);

        centre_range_ = this->get_parameter("centre_range").as_double();
        centre_angle_ = this->get_parameter("centre_angle").as_double();
        settling_iter_ = this->get_parameter("settling_iter").as_int();
        max_position_ = this->get_parameter("max_position").as_int();
        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        current_velocity_ = this->get_parameter("velocity").as_int();
        current_acceleration_ = this->get_parameter("acceleration").as_int();

        callback_group_subscriber1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_subscriber2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub1_opt = rclcpp::SubscriptionOptions();
        sub1_opt.callback_group = callback_group_subscriber1_;
        auto sub2_opt = rclcpp::SubscriptionOptions();
        sub2_opt.callback_group = callback_group_subscriber2_;

        // Create subscriber to topic "canbus_rosbound"
        canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canopen_rosbound", QOS_ALL, std::bind(&SteeringActuator::can_callback, this, _1), sub1_opt);

        // Create subscriber to topic "steering_angle"
        steer_ang_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/steering_angle", QOS_ALL, std::bind(&SteeringActuator::steering_angle_callback, this, _1),
            sub2_opt);

        // Create subscriber to topic "driving_command"
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1),
            sub2_opt);

        // Create subscriber to topic AS status
        state_sub_ = this->create_subscription<driverless_msgs::msg::State>(
            "/system/as_status", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1));

        steering_update_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                                         std::bind(&SteeringActuator::pre_op_centering, this));

        // Create state request and config timers
        state_request_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
                                                       std::bind(&SteeringActuator::c5e_state_request_callback, this),
                                                       callback_group_subscriber1_);

        // Create publisher to topic "canbus_carbound"
        can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", QOS_ALL);

        // Create publisher to topic "encoder_reading"
        encoder_pub_ = this->create_publisher<std_msgs::msg::Int32>("/vehicle/encoder_reading", QOS_ALL);

        // Create publisher to topic "steering_state"
        steering_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system/steering_ready", QOS_ALL);

        RCLCPP_INFO(this->get_logger(), "---Steering Actuator Node Initialised---");
    }

    // Shutdown system
    void shutdown() { shutdown_requested_ = true; }
};

// Prototype functions initialisations?
std::function<void(int)> handler;
void signal_handler(int signal) { handler(signal); }

// Main loop
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);                          // Initialise ROS2
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
