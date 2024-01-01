#include <algorithm>
#include <bitset>
#include <iostream>

#include "CAN_BMU.h"
#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "CAN_VESC.h"
#include "QUTMS_can.h"
#include "TritiumCAN.hpp"
#include "can_interface.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "driverless_msgs/msg/car_status.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

using std::placeholders::_1;

const int NUM_CMUS = 8;
const int NUM_VOLTAGES = 14;
const int NUM_TEMPERATURES = 16;
const float WHEEL_DIAMETER = 0.4064;
const float AXLE_WIDTH = 1.4;

void copy_data(const std::vector<uint8_t> &vec, uint8_t *dest, size_t n) {
    for (size_t i = 0; i < n; i++) {
        dest[i] = vec[i];
    }
}

// create array of CAN IDs we care about
std::vector<uint32_t> canopen_ids = {RES_BOOT_UP_ID, RES_HEARTBEAT_ID, C5E_BOOT_UP_ID, C5E_POS_ID,
                                     C5E_EMCY_ID,    C5E_STATUS_ID,    C5E_SRV_ID};

class CanBus : public rclcpp::Node {
   private:
    // can connection queue retrieval timer
    rclcpp::TimerBase::SharedPtr timer_;

    // subscriber
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub_;

    // publishers
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr canopen_pub_;
    // ADD PUBS FOR CAN TOPICS HERE
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::Publisher<driverless_msgs::msg::CarStatus>::SharedPtr bmu_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

    std::string ros_base_frame_;

    // can connection
    std::shared_ptr<TritiumCAN> tritiumCAN;

    // class variables for sensor data
    float wheel_speeds[4];
    driverless_msgs::msg::CarStatus bmu_status;
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    float last_velocity;
    float last_steering_angle;

    void update_twist() {
        // use last velocity and steering angle to update twist
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = ros_base_frame_;  // PARAMETERISE
        twist_msg.twist.twist.linear.x = last_velocity;
        twist_msg.twist.twist.linear.y = 0.0;
        twist_msg.twist.twist.angular.z = last_velocity * tan(last_steering_angle) / AXLE_WIDTH;
        twist_pub_->publish(twist_msg);
    }

    void canmsg_timer() {
        auto res = this->tritiumCAN->rx();
        // std::cout << "RX - Num CAN: " << res->size() << std::endl;

        for (auto &msg : *res) {
            // cout all msgs and timestamp and data
            uint32_t qutms_masked_id = msg.id & ~0xF;
            // only publish can messages with IDs we care about to not flood memory
            if (std::find(canopen_ids.begin(), canopen_ids.end(), msg.id) != canopen_ids.end()) {
                this->canopen_pub_->publish(msg);
            } else if (qutms_masked_id == VCU_Heartbeat_ID || qutms_masked_id == SW_Heartbeat_ID) {
                this->can_pub_->publish(msg);
            }

            // CAN TRANSLATION OPTIONS
            // Wheel speed velocity
            uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
            uint8_t vesc_id = msg.id & 0xFF;
            if (vesc_id < 4) {
                if (vesc_masked_id == VESC_CAN_PACKET_STATUS) {
                    uint8_t data[8];
                    copy_data(msg.data, data, 8);
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
            else if (qutms_masked_id == VCU_TransmitSteering_ID) {
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
            // BMU
            else if (msg.id == BMU_TransmitVoltage_ID) {
                uint8_t cmu_id;
                uint8_t packet_id;
                uint16_t voltages[3];
                Parse_BMU_TransmitVoltage((uint8_t *)&msg.data[0], &cmu_id, &packet_id, voltages);
                for (int i = 0; i < 3 && (packet_id * 3 + i) < NUM_VOLTAGES; i++) {
                    this->bmu_status.brick_data[cmu_id].voltages[packet_id * 3 + i] = voltages[i];
                }
                this->bmu_status_pub_->publish(this->bmu_status);

            } else if (msg.id == BMU_TransmitTemperature_ID) {
                uint8_t cmu_id;
                uint8_t packet_id;
                uint8_t temps[6];
                Parse_BMU_TransmitTemperatures((uint8_t *)&msg.data[0], &cmu_id, &packet_id, temps);
                for (int i = 0; i < 6 && (packet_id * 6 + i) < NUM_TEMPERATURES; i++) {
                    bmu_status.brick_data[cmu_id].temperatures[packet_id * 6 + i] = temps[i];
                }
                this->bmu_status_pub_->publish(this->bmu_status);
            }
        }
    }

    // ROS can msgs
    void canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const { this->tritiumCAN->tx(msg.get()); }

   public:
    CanBus() : Node("canbus_translator_node") {
        // Can2Ethernet parameters
        // IP configured in TritiumCAN driver (x.x.2. subnet separate from car x.x.3. subnet for data passthru)
        // https://tritiumcharging.com/wp-content/uploads/2020/11/TritiumCAN%E2%80%93Ethernet-Bridge_Manual.pdf
        std::string ip = this->declare_parameter<std::string>("ip", "192.168.2.126");
        int port = this->declare_parameter<int>("port", 20005);
        ros_base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        this->get_parameter("ip", ip);
        this->get_parameter("port", port);
        this->get_parameter("base_frame", ros_base_frame_);

        RCLCPP_INFO(this->get_logger(), "Creating Connection on %s:%i...", ip.c_str(), port);
        this->tritiumCAN = std::make_shared<TritiumCAN>();
        this->tritiumCAN->setup(ip);
        RCLCPP_INFO(this->get_logger(), "done!");

        // retrieve can messages from queue
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CanBus::canmsg_timer, this));
        // subscribe to can messages from ROS system
        this->can_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canbus_carbound", QOS_ALL, std::bind(&CanBus::canmsg_callback, this, _1));
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
        // BMU
        this->bmu_status_pub_ = this->create_publisher<driverless_msgs::msg::CarStatus>("/vehicle/bmu_status", 10);
        this->bmu_status.brick_data = std::vector<driverless_msgs::msg::BrickData>(NUM_CMUS);
        for (int i = 0; i < NUM_CMUS; i++) {
            this->bmu_status.brick_data[i].id = i + 1;
            this->bmu_status.brick_data[i].voltages = std::vector<uint16_t>(NUM_VOLTAGES);
            this->bmu_status.brick_data[i].temperatures = std::vector<uint8_t>(NUM_TEMPERATURES);
        }

        RCLCPP_INFO(this->get_logger(), "---CANBus Translator Node Initialised---");
    }

    ~CanBus() { this->tritiumCAN->~TritiumCAN(); }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanBus>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
