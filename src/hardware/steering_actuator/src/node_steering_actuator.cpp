#include <chrono>  // Timer library
#include <map>     // Container library

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
    rclcpp::TimerBase::SharedPtr c5e_state_request_timer;
    rclcpp::TimerBase::SharedPtr c5e_config_request_timer;
    rclcpp::TimerBase::SharedPtr steering_update_timer;

    // Creates publishers and subscribers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_pub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steer_ang_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr canopen_sub_;

    // params
    double Kp, Ki, Kd, centre_range;

    bool shutdown_requested = false;
    driverless_msgs::msg::State state;
    c5e_state desired_state = states[RTSO];
    c5e_state current_state = states[RTSO];
    uint16_t control_method = MODE_ABSOLUTE;
    bool motor_enabled = true;
    bool centred = false;
    bool steering_ang_received = false;
    int32_t offset = 0;
    int settled_count = 0;
    bool initial_enc_saved = false;
    int32_t initial_enc;
    double current_steering_angle = 0;
    double center_steering = -2.0;
    double requested_steering_angle = center_steering;  // Desired Steering Angle (Guidance Logic)
    int32_t current_enc_revolutions = 0;                // Current Encoder Revolutions (Stepper encoder)
    float integral_error = 0;  // Integral Error
    float prev_error = 0;      // Derivative Error
    uint32_t current_velocity;
    uint32_t current_acceleration;
    std::chrono::high_resolution_clock::time_point last_update = std::chrono::high_resolution_clock::now();   // Timer
    std::chrono::high_resolution_clock::time_point last_reading = std::chrono::high_resolution_clock::now();  // Timer

    // These publishings are required to send a service 'trigger' to the c5e controller
    // with the specific object ID (seen in struct above).
    // Then the c5e controller will send a CAN message back with that ID
    void c5e_state_request_callback() {
        this->read_steering_data(STATUS_WORD);
        this->read_steering_data(POSITION_ACTUAL_VAL);
    }

    void c5e_config_request_callback() {
        this->read_steering_data(HOME_OFFSET);
        this->read_steering_data(MOTION_PROFILE_TYPE);
        this->read_steering_data(PROFILE_VELOCITY);
        this->read_steering_data(END_VELOCITY);
        this->read_steering_data(PROFILE_ACCELERATION);
        this->read_steering_data(PROFILE_DECELERATION);
        this->read_steering_data(QUICK_STOP_DECELERATION);
        this->read_steering_data(MAX_ACCELERATION);
        this->read_steering_data(MAX_DECELERATION);
        this->read_steering_data(MODE_OF_OPERATION);
    }

    // Receive message from CAN
    void can_callback(const driverless_msgs::msg::Can msg) {
        if (msg.id == C5E_BOOT_UP_ID) {
            RCLCPP_INFO(this->get_logger(), "C5E Booted Up");
        } else if (msg.id == C5E_SRV_ID) {
            uint16_t object_id = (msg.data[2] & 0xFF) << 8 | (msg.data[1] & 0xFF);
            if (msg.data[0] == 0x60 || msg.data[0] == 0x80) {
                // 0x60 -> success ack
                // 0x80 -> error ack
                RCLCPP_DEBUG(this->get_logger(), "ACK object: %x, %x", object_id, msg.data[0]);
                return;
            }

            uint32_t data = 0;
            size_t size = can_open_size_map[msg.data[0]];
            for (size_t i = 0; i < size; i++) {
                data |= (msg.data[4 + i] & 0xFF) << i * 8;
            }

            RCLCPP_INFO(this->get_logger(), "Object ID: %x", object_id);

            if (object_id == STATUS_WORD) {
                uint16_t status_word = (msg.data[3] << 8 | msg.data[4]);
                this->current_state = parse_state(status_word);

                if (this->motor_enabled) {
                    if (this->current_state == this->desired_state) {
                        // enabled transitions
                        if (this->current_state == states[NRTSO]) {
                            this->desired_state = states[SOD];
                        } else if (this->current_state == states[SOD]) {
                            this->desired_state = states[RTSO];
                        } else if (this->current_state == states[RTSO]) {
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
                    this->send_steering_data(CONTROL_WORD, (uint8_t *)&this->desired_state.control_word, 2);
                }
            }
            // message received for position revolution
            else if (object_id == POSITION_ACTUAL_VAL) {
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
            // message received for offset position
            else if (object_id == HOME_OFFSET) {
                int32_t val = (int32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "HOME_OFFSET: %i", val);

                int32_t desired_val = 0;
                if (val != desired_val) {
                    this->send_steering_data(HOME_OFFSET, (uint8_t *)&this->offset, 4);
                }
            } else if (object_id == MOTION_PROFILE_TYPE) {
                int16_t val = (int16_t)data;
                RCLCPP_DEBUG(this->get_logger(), "MOTION_PROFILE_TYPE: %i", val);

                int32_t desired_val = 0;
                if (val != desired_val) {
                    this->send_steering_data(MOTION_PROFILE_TYPE, (uint8_t *)&desired_val, 4);
                }
            } else if (object_id == PROFILE_VELOCITY) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "PROFILE_VELOCITY: %u", val);

                if (val != this->current_velocity) {
                    this->send_steering_data(PROFILE_VELOCITY, (uint8_t *)&this->current_velocity, 4);
                }
            } else if (object_id == END_VELOCITY) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "END_VELOCITY: %u", val);

                uint32_t desired_val = 0;
                if (val != desired_val) {
                    this->send_steering_data(END_VELOCITY, (uint8_t *)&desired_val, 4);
                }
            } else if (object_id == PROFILE_ACCELERATION) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "PROFILE_ACCELERATION: %u", val);

                if (val != this->current_acceleration) {
                    this->send_steering_data(PROFILE_ACCELERATION, (uint8_t *)&this->current_acceleration, 4);
                }
            } else if (object_id == PROFILE_DECELERATION) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "PROFILE_DECELERATION: %u", val);

                if (val != this->current_acceleration) {
                    this->send_steering_data(PROFILE_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
                }
            } else if (object_id == QUICK_STOP_DECELERATION) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "QUICK_STOP_DECELERATION: %u", val);

                if (val != this->current_acceleration) {
                    this->send_steering_data(QUICK_STOP_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
                }
            } else if (object_id == MAX_ACCELERATION) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "MAX_ACCELERATION: %u", val);

                if (val != this->current_acceleration) {
                    this->send_steering_data(MAX_ACCELERATION, (uint8_t *)&this->current_acceleration, 4);
                }
            } else if (object_id == MAX_DECELERATION) {
                uint32_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "MAX_DECELERATION: %u", val);

                if (val != this->current_acceleration) {
                    this->send_steering_data(MAX_DECELERATION, (uint8_t *)&this->current_acceleration, 4);
                }
            } else if (object_id == MODE_OF_OPERATION) {
                int8_t val = (uint32_t)data;
                RCLCPP_DEBUG(this->get_logger(), "MODE_OF_OPERATION: %i", val);

                int32_t desired_val = 1;
                if (val != desired_val) {
                    this->send_steering_data(MODE_OF_OPERATION, (uint8_t *)&desired_val, 4);
                }
            }
        }
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
        auto current_reading = std::chrono::high_resolution_clock::now();  // Update clock
        double elapsed_time_seconds =
            std::chrono::duration<double, std::milli>(current_reading - last_reading).count() /
            1000;                              // Calculate time elapsed
        this->last_reading = current_reading;  // Set previous time to current time
        if (!this->steering_ang_received) this->steering_ang_received = true;
        RCLCPP_DEBUG(this->get_logger(), "Current angle: %f", msg.data);
        RCLCPP_DEBUG(this->get_logger(), "Time: %f", elapsed_time_seconds);
    }

    // Get desired steering angle to update steering
    void driving_command_callback(const ackermann_msgs::msg::AckermannDriveStamped msg) {
        this->requested_steering_angle = msg.drive.steering_angle;
        RCLCPP_INFO(this->get_logger(), "Requested angle: %f", msg.drive.steering_angle);
    }

    // Update Steering
    void update_steering() {
        // all state transitions passed
        if (this->motor_enabled && this->steering_ang_received) {
            // motor has not centred itself with steering ang sensor
            if (!this->centred) {
                auto current_update = std::chrono::high_resolution_clock::now();  // Update clock
                double elapsed_time_seconds =
                    std::chrono::duration<double, std::milli>(current_update - last_update).count() /
                    1000;                            // Calculate time elapsed
                this->last_update = current_update;  // Set previous time to current time

                // Grab error between steering angle and "zero"
                double error = -(this->current_steering_angle - this->center_steering);
                auto& clk = *this->get_clock();
                RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 1000, "Error: %f", error);

                if (abs(error) < this->centre_range) {
                    // motor has settled enough
                    this->settled_count += 1;

                    if (this->settled_count > 200) {
                        this->offset = this->initial_enc - this->current_enc_revolutions;
                        this->centred = true;
                        this->current_velocity = this->get_parameter("velocity").as_int();
                        this->current_acceleration = this->get_parameter("acceleration").as_int();
                        this->control_method = MODE_ABSOLUTE;
                        RCLCPP_INFO(this->get_logger(), "CENTRED STEERING, %i", this->offset);
                        return;
                    }
                } else {
                    // reset settled count to debounce initial overshoots
                    this->settled_count = 0;
                }

                this->integral_error += error * elapsed_time_seconds;                         // Grab integral error
                double derivative_error = (error - this->prev_error) / elapsed_time_seconds;  // Grab derivative error

                // left hand down is +, rhd is -
                double target = -(this->Kp * error + this->Ki * this->integral_error +
                                  this->Kd * derivative_error);  // PID commands to send to plant
                // RCLCPP_INFO(this->get_logger(), "Kp: %f err: %f Ki: %f i: %f Kd: %f d: %f target: %f", Kp, error, Ki,
                //             integral_error, Kd, derivative_error, target);  // Prirnt PID commands

                this->prev_error = error;

                this->target_position(target);
            }
            // we have set a home offset
            else if (this->centred) {
                // turning left eqn: -83.95x - 398.92
                // turning right eqn: -96.19x - 83.79
                int32_t target;
                if (this->requested_steering_angle > this->center_steering) {
                    target = int32_t(-86.45 * this->requested_steering_angle - 398.92) - this->offset;
                } else {
                    target = int32_t(-94.58 * this->requested_steering_angle - 83.79) - this->offset;
                }
                target = std::max(std::min(target, 7500 - this->offset), -7500 - this->offset);
                this->target_position(target);
            }
        }
    }

    void target_position(int32_t target) {
        if (this->current_state != states[OE]) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Not enabled, can't target");
            return;
        }
        RCLCPP_INFO_ONCE(this->get_logger(), "Enabled, Targeting");

        this->send_steering_data(CONTROL_WORD, (uint8_t *)&this->control_method, 2);
        this->send_steering_data(TARGET_POSITION, (uint8_t *)&target, 4);
        uint16_t trigger_control_method = this->control_method | TRIGGER_MOTION;
        this->send_steering_data(CONTROL_WORD, (uint8_t *)&trigger_control_method, 2);
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
        this->declare_parameter<int>("velocity", 100);
        this->declare_parameter<int>("velocity_centering", 200);
        this->declare_parameter<int>("acceleration", 100);
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
        this->current_velocity = this->get_parameter("velocity_centering").as_int();
        this->current_acceleration = this->get_parameter("acceleration_centering").as_int();
        this->control_method = MODE_RELATIVE;

        // Create publisher to topic "canbus_carbound"
        this->can_pub = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_carbound", QOS_ALL);

        // Create publisher to topic "encoder_reading"
        this->encoder_pub = this->create_publisher<std_msgs::msg::Int32>("/vehicle/encoder_reading", QOS_ALL);

        // Create subscriber to topic AS status
        // this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
        //     "/system/as_status", QOS_ALL, std::bind(&SteeringActuator::as_state_callback, this, _1));

        // Create subscriber to topic "steering_angle"
        this->steer_ang_sub = this->create_subscription<std_msgs::msg::Float32>(
            "/vehicle/steering_angle", QOS_ALL, std::bind(&SteeringActuator::steering_angle_callback, this, _1));

        // Create subscriber to topic "driving_command"
        this->ackermann_sub = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/control/driving_command", QOS_ALL, std::bind(&SteeringActuator::driving_command_callback, this, _1));

        // Create subscriber to topic "canbus_rosbound"
        this->canopen_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canopen_rosbound", QOS_ALL, std::bind(&SteeringActuator::can_callback, this, _1));

        // Create state request and config timers
        this->steering_update_timer =
            this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SteeringActuator::update_steering, this));

        // Create state request and config timers
        this->c5e_state_request_timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&SteeringActuator::c5e_state_request_callback, this));

        // this->c5e_config_request_timer = this->create_wall_timer(
        //     std::chrono::seconds(1), std::bind(&SteeringActuator::c5e_config_request_callback, this));

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
    // signal(SIGINT, signal_handler);
    // handler = [node](int signal) {
    //     RCLCPP_INFO(node->get_logger(), "Shutting down motor.");
    //     node->shutdown();
    //     return signal;
    // };

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
