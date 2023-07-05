#include <algorithm>
#include <bitset>
#include <iostream>

#include "CAN_VESC.h"
#include "QUTMS_can.h"
#include "TritiumCAN.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/can.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

// create array of CAN IDs we care about
std::vector<uint32_t> can_ids = {
    0x5F0,  // C5_E stepper ID
    BMU_TransmitVoltage_ID,
    BMU_TransmitTemperature_ID,
    (0x700 + RES_NODE_ID),  // boot up message
    RES_Heartbeat_ID,
};

class CanBus : public rclcpp::Node {
   private:
    // std::shared_ptr<Can2Ethernet> c;
    std::shared_ptr<TritiumCAN> tritiumCAN;

    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub_;
    rclcpp::Publisher<driverless_msgs::msg::Can>::SharedPtr can_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void canmsg_callback(const driverless_msgs::msg::Can::SharedPtr msg) const { this->tritiumCAN->tx(msg.get()); }

    void canmsg_timer() {
        auto res = this->tritiumCAN->rx();

        for (auto& msg : *res) {
            uint32_t vesc_masked_id = (msg.id & ~0xFF) >> 8;
            uint8_t vesc_id = msg.id & 0xFF;
            uint32_t qutms_masked_id = msg.id & ~0xF;
            // only publish messages with IDs we care about to not flood memory
            if (std::find(can_ids.begin(), can_ids.end(), msg.id) != can_ids.end()) {
                this->can_pub_->publish(msg);
            }
            // motor RPM CAN
            else if (vesc_id < 4 && vesc_masked_id == VESC_CAN_PACKET_STATUS) {
                this->can_pub_->publish(msg);
            }
            // VCU CAN
            else if (qutms_masked_id == VCU_Heartbeat_ID || qutms_masked_id == VCU_TransmitSteering_ID ||
                     qutms_masked_id == SW_Heartbeat_ID) {
                this->can_pub_->publish(msg);
            }
        }
    }

   public:
    CanBus() : Node("canbus_translator_node") {
        // Can2Ethernet parameters
        std::string _ip = this->declare_parameter<std::string>("ip", "192.168.2.125");
        int _port = this->declare_parameter<int>("port", 20005);
        this->get_parameter("ip", _ip);
        this->get_parameter("port", _port);

        RCLCPP_INFO(this->get_logger(), "Creating Connection on %s:%i...", _ip.c_str(), _port);
        // this->c = std::make_shared<Can2Ethernet>(_ip, _port);
        this->tritiumCAN = std::make_shared<TritiumCAN>();
        this->tritiumCAN->setup(_ip);
        RCLCPP_INFO(this->get_logger(), "done!");

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CanBus::canmsg_timer, this));

        this->can_sub_ = this->create_subscription<driverless_msgs::msg::Can>(
            "/can/canbus_carbound", QOS_ALL, std::bind(&CanBus::canmsg_callback, this, _1));

        this->can_pub_ = this->create_publisher<driverless_msgs::msg::Can>("/can/canbus_rosbound", 10);

        RCLCPP_INFO(this->get_logger(), "---CANBus Translator Node Initialised---");
    }

    ~CanBus() { this->tritiumCAN->~TritiumCAN(); }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanBus>());
    rclcpp::shutdown();
    return 0;
}
