#include "component_canbus_translator.hpp"
#include "rclcpp_components/register_node_macro.hpp"

void CANTranslator::update_twist() {
    // use last velocity and steering angle to update twist
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = ros_base_frame_;  // PARAMETERISE
    twist_msg.twist.twist.linear.x = last_velocity;
    twist_msg.twist.twist.linear.y = 0.0;
    twist_msg.twist.twist.angular.z = last_velocity * tan(last_steering_angle) / AXLE_WIDTH;
    twist_pub_->publish(twist_msg);
}

void CANTranslator::canmsg_timer() {
    auto res = this->can_interface_->rx(this->get_logger(), this->get_clock());

    for (auto &msg : *res) {
        // only publish can messages with IDs we care about to not flood memory
        uint32_t canopen_index = std::find(canopen_ids.begin(), canopen_ids.end(), msg.id) - canopen_ids.begin();
        if (canopen_index < canopen_ids.size()) {
            // int age_ms = (this->now().nanoseconds() - this->last_canopen_times[canopen_index].nanoseconds()) / 1e6;
            // RCLCPP_INFO(this->get_logger(), "Received %s, age: %d", canopen_names[canopen_index].c_str(), age_ms);
            // this->last_canopen_times[canopen_index] = this->now();
            this->canopen_pub_->publish(msg);
            return;
        }
        uint32_t can_index = std::find(can_ids.begin(), can_ids.end(), msg.id) - can_ids.begin();
        if (can_index < can_ids.size()) {
            // int age_ms = (this->now().nanoseconds() - this->last_can_times[can_index].nanoseconds()) / 1e6;
            // RCLCPP_INFO(this->get_logger(), "Received %s, age: %d", can_names[can_index].c_str(), age_ms);
            // this->last_can_times[can_index] = this->now();
            this->can_pub_->publish(msg);
            return;
        }

        // CAN TRANSLATION OPTIONS
        // Wheel speed velocity
        uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
        uint8_t vesc_id = msg.id & 0xFF;
        if (vesc_id < 4) {
            if (vesc_masked_id == VESC_CAN_PACKET_STATUS) {
                uint8_t data[8];
                this->copy_data(msg.data, data, 8);
                // extract and publish RPM
                int32_t rpm;
                float current;
                float duty;
                Parse_VESC_CANPacketStatus(data, &rpm, &current, &duty);

                wheel_speeds[vesc_id] = (rpm / (21.0 * 4.50)) * M_PI * WHEEL_DIAMETER / 60;
                float av_velocity = 0;
                for (int i = 0; i < 4; i++) {
                    av_velocity += this->wheel_speeds[i];
                }
                av_velocity = av_velocity / 4;

                // publish velocity
                std_msgs::msg::Float32 vel_msg;
                vel_msg.data = av_velocity;
                last_velocity = av_velocity;
                this->velocity_pub_->publish(vel_msg);

                // update twist msg with new velocity
                update_twist();
            }
        }
        // Steering Angle
        else if (msg.id == VCU_TransmitSteering_ID) {
            // int age_ms = (this->now().nanoseconds() - this->last_can_times[2].nanoseconds()) / 1e6;
            // RCLCPP_INFO(this->get_logger(), "Received %s, age: %d", can_names[2].c_str(), age_ms);
            // this->last_can_times[2] = this->now();

            // data vector to uint8_t array
            uint8_t data[8];
            copy_data(msg.data, data, 8);

            int16_t steering_0_raw;
            int16_t steering_1_raw;
            uint16_t adc_0;
            uint16_t adc_1;

            Parse_VCU_TransmitSteering(data, &steering_0_raw, &steering_1_raw, &adc_0, &adc_1);
            // Log steering angle
            RCLCPP_DEBUG(this->get_logger(), "Steering Angle 0: %i  Steering Angle 1: %i ADC 0: %i ADC 1: %i",
                            steering_0_raw, steering_1_raw, adc_0, adc_1);
            double steering_0 = steering_0_raw / 10.0;
            double steering_1 = steering_1_raw / 10.0;

            std_msgs::msg::Float32 angle_msg;
            if (abs(steering_0 - steering_1) < 10) {
                angle_msg.data = steering_0;
                last_steering_angle = steering_0;
                this->steering_angle_pub_->publish(angle_msg);
                update_twist();
            } else {
                RCLCPP_FATAL(this->get_logger(),
                                "MISMATCH: Steering Angle 0: %i  Steering Angle 1: %i ADC 0: %i ADC 1: %i",
                                steering_0_raw, steering_1_raw, adc_0, adc_1);
            }
        }
    }
}

// ROS can msgs
void CANTranslator::canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const {
    this->can_interface_->tx(msg.get(), this->get_logger());
}

bool CANTranslator::set_interface() {
    // can_interface_ parameters
    std::string interface = this->declare_parameter<std::string>("interface", "");
    std::string interface_name = this->declare_parameter<std::string>("interface_name", "");
    this->get_parameter("interface", interface);
    this->get_parameter("base_frame", ros_base_frame_);

    RCLCPP_INFO(this->get_logger(), "Creating Connection on %s...", interface.c_str());

    if (interface_name == "socketcan") {
        this->can_interface_ = std::make_shared<SocketCAN>();
    } else if (interface_name == "tritium") {
        this->can_interface_ = std::make_shared<TritiumCAN>();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid interface_name: %s", interface_name.c_str());
        return false;
    }

    while (!this->can_interface_->setup(interface, this->get_logger())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create connection on %s. Retrying...", interface.c_str());
        // check for keyboard interrupt
        if (!rclcpp::ok()) {
            return false;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
    return true;
}

CANTranslator::CANTranslator(const rclcpp::NodeOptions & options) : Node("canbus_translator_node", options) {
    ros_base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    // set can interface
    if (!set_interface()) { return; }

    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = sub_cb_group_;

    // retrieve can messages from queue
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CANTranslator::canmsg_timer, this), timer_cb_group_);
    // subscribe to can messages from ROS system
    this->can_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
        "/can/canbus_carbound", QOS_ALL, std::bind(&CANTranslator::canmsg_callback, this, _1), sub_opt);
    // publish can messages to ROS system
    this->can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_rosbound", QOS_ALL);
    this->canopen_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canopen_rosbound", QOS_ALL);

    // ADD PUBS FOR CAN TOPICS HERE
    // Steering ang
    this->steering_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vehicle/steering_angle", 10);
    // Vehicle velocity
    this->velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vehicle/velocity", 10);
    // Twist
    this->twist_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/wheel_twist", 10);

    RCLCPP_INFO(this->get_logger(), "---CANBus Translator Node Initialised---");
}

// deconstruct
CANTranslator::~CANTranslator() { this->can_interface_->deconstruct(); }

RCLCPP_COMPONENTS_REGISTER_NODE(CANTranslator)

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CanBus>();

//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node);
//     executor.spin();

//     // rclcpp::spin(node);
//     // rclcpp::shutdown();
//     return 0;
// }
