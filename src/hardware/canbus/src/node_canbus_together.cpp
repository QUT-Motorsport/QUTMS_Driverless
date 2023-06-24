#include <algorithm>
#include <bitset>
#include <iostream>

#include "TritiumCAN.hpp"
#include "CAN_DVL.h"
#include "CAN_RES.h"
#include "CAN_SW.h"
#include "CAN_VCU.h"
#include "CAN_VESC.h"
#include "CAN_BMU.h"
#include "QUTMS_can.h"
#include "can_interface.hpp"

#include "rclcpp/rclcpp.hpp"

#include "driverless_msgs/msg/can.hpp"
#include "std_msgs/msg/float32.hpp"
#include "driverless_msgs/msg/wss_velocity.hpp"
#include "driverless_msgs/msg/res.hpp"
#include "driverless_msgs/msg/car_status.hpp"

using std::placeholders::_1;

const int NUM_CMUS = 8;
const int NUM_VOLTAGES = 14;
const int NUM_TEMPERATURES = 16;
const float WHEEL_DIAMETER = 0.4064;

void copy_data(const std::vector<uint8_t> &vec, uint8_t *dest, size_t n) {
    for (size_t i = 0; i < n; i++) {
        dest[i] = vec[i];
    }
}

class CanBus : public rclcpp::Node, public CanInterface {
   private:
    // std::shared_ptr<Can2Ethernet> c;
    std::shared_ptr<TritiumCAN> tritiumCAN;
    // can connection queue retrieval timer
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr ros_subscription_;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr publisher_;
    
    // ADD PUBS FOR CAN TOPICS HERE
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;
    rclcpp::Publisher<driverless_msgs::msg::WSSVelocity>::SharedPtr wss_vel_pub_;
    rclcpp::Publisher<driverless_msgs::msg::RES>::SharedPtr res_status_pub_;
    rclcpp::Publisher<driverless_msgs::msg::CarStatus>::SharedPtr bmu_status_pub_;

    // class variables for each datastream
    float wheel_speeds[4];
    RES_Status_t RES_status;
    driverless_msgs::msg::CarStatus bmu_status;

    void canmsg_timer_callback() {
        auto res = this->tritiumCAN->rx();

        for (auto& msg : *res) {
            this->publisher_->publish(msg);

            // CAN TOPIC OPTIONS

            // Wheel speeds
            uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
            uint8_t vesc_id = msg.id & 0xFF;
            if (vesc_id < 4) {
                if (vesc_id == VESC_CAN_PACKET_STATUS) {
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

                    driverless_msgs::msg::WSSVelocity vel_msg;
                    vel_msg.velocity = av_velocity;
                    vel_msg.header.stamp = this->now();
                    this->wss_vel_pub_->publish(vel_msg);
                }
            }

            // Steering Angle
            uint32_t qutms_masked_id = msg.id & ~0xF;
            if (qutms_masked_id == VCU_TransmitSteering_ID){
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
                } else {
                    angle_msg.data = 1111.1; // error identifier (impossible value)
                }
                this->steering_angle_pub_->publish(angle_msg);
            }
            
            // RES
            else if (msg.id == (0x700 + RES_NODE_ID)) {
                /*
                RES has reported in, request state change to enable it

                Doing so will result in the RES reporting PDOs
                2000 - 20007 every 30ms with the ID 0x180 + RES_NODE_ID

                Byte 0 = state command (start up)
                Byte 1 = Node ID (0x00 = All Nodes)
                */
                uint8_t p[8] = {0x01, RES_NODE_ID, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                driverless_msgs::msg::Can res_can_msg = this->_d_2_f(0x00, false, p, sizeof(p));
                this->tritiumCAN->tx(&res_can_msg);

            } else if (msg.id == (0x180 + RES_NODE_ID)) {
                // RES Reciever Status Packet
                Parse_RES_Heartbeat((uint8_t *)&msg.data[0], &this->RES_status);

                driverless_msgs::msg::RES res_msg;
                res_msg.sw_k2 = this->RES_status.sw_k2;
                res_msg.bt_k3 = this->RES_status.bt_k3;
                res_msg.estop = this->RES_status.estop;
                res_msg.radio_quality = this->RES_status.radio_quality;
                res_msg.loss_of_signal_shutdown_notice = this->RES_status.loss_of_signal_shutdown_notice;
                this->res_status_pub_->publish(res_msg);
                // Log RES state
                RCLCPP_DEBUG(this->get_logger(), "RES Status: [SW, BT]: %i, %i -- [EST]: %i, -- [RAD_QUAL]: %i",
                             this->RES_status.sw_k2, this->RES_status.bt_k3, this->RES_status.estop,
                             this->RES_status.radio_quality);
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
    void canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const {
        this->tritiumCAN->tx(msg.get());
    }

   public:
    CanBus() : Node("canbus_translator_node") {
        // Can2Ethernet parameters
        std::string _ip = this->declare_parameter<std::string>("ip", "192.168.2.125");
        int _port = this->declare_parameter<int>("port", 20005);
        this->get_parameter("ip", _ip);
        this->get_parameter("port", _port);

        RCLCPP_INFO(this->get_logger(), "Creating Connection on %s:%i...", _ip.c_str(), _port);
        this->tritiumCAN = std::make_shared<TritiumCAN>();
        this->tritiumCAN->setup(_ip);
        RCLCPP_INFO(this->get_logger(), "done!");

        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CanBus::canmsg_timer_callback, this));

        this->ros_subscription_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canbus_carbound", 10, std::bind(&CanBus::canmsg_callback, this, _1));

        this->publisher_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_rosbound", 10);

        // ADD PUBS FOR CAN TOPICS HERE
        // Steering ang
        this->steering_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vehicle/steering_angle", 10);
        // Motor RPM
        this->wss_vel_pub_ = this->create_publisher<driverless_msgs::msg::WSSVelocity>("/vehicle/wheel_speed", 10);
        // RES
        this->res_status_pub_ = this->create_publisher<driverless_msgs::msg::RES>("/vehicle/res_status", 10);
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

    ~CanBus() {
        // this->c->~Can2Ethernet();
        this->tritiumCAN->~TritiumCAN();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanBus>());
    rclcpp::shutdown();
    return 0;
}
