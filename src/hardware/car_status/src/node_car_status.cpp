#include <iostream>

#include "CAN_BMU.h"
#include "can_interface.hpp"
#include "driverless_msgs/msg/car_status.hpp"
#include "rclcpp/rclcpp.hpp"

const int NUM_CMUS = 8;
const int NUM_VOLTAGES = 14;
const int NUM_TEMPERATURES = 16;

using std::placeholders::_1;

class CarStatusNode : public rclcpp::Node, public CanInterface {
   private:
    rclcpp::Subscription<driverless_msgs::msg::Can>::SharedPtr can_sub;
    rclcpp::Publisher<driverless_msgs::msg::CarStatus>::SharedPtr car_status_pub;

    driverless_msgs::msg::CarStatus car_status;

    void canbus_callback(const driverless_msgs::msg::Can msg) {
        uint8_t cmu_id;
        uint8_t packet_id;

        switch (msg.id) {
            case BMU_TransmitVoltage_ID:
                uint16_t voltages[3];
                // https://stackoverflow.com/a/2923290
                Parse_BMU_TransmitVoltage((uint8_t *)&msg.data[0], &cmu_id, &packet_id, voltages);
                for (int i = 0; i < 3 && (packet_id * 3 + i) < NUM_VOLTAGES; i++) {
                    car_status.brick_data[cmu_id].voltages[packet_id * 3 + i] = voltages[i];
                }
                break;
            case BMU_TransmitTemperature_ID:
                uint8_t temps[6];
                // https://stackoverflow.com/a/2923290
                Parse_BMU_TransmitTemperatures((uint8_t *)&msg.data[0], &cmu_id, &packet_id, temps);
                for (int i = 0; i < 6 && (packet_id * 6 + i) < NUM_TEMPERATURES; i++) {
                    car_status.brick_data[cmu_id].temperatures[packet_id * 6 + i] = temps[i];
                }
                break;
            default:
                break;
        }

        this->car_status_pub->publish(this->car_status);
    }

   public:
    CarStatusNode() : Node("car_status") {
        this->can_sub = this->create_subscription<driverless_msgs::msg::Can>(
            "canbus_rosbound", 10, std::bind(&CarStatusNode::canbus_callback, this, _1));
        this->car_status_pub = this->create_publisher<driverless_msgs::msg::CarStatus>("car_status", 10);

        this->car_status.brick_data = std::vector<driverless_msgs::msg::BrickData>(NUM_CMUS);
        for (int i = 0; i < NUM_CMUS; i++) {
            this->car_status.brick_data[i].id = i + 1;
            this->car_status.brick_data[i].voltages = std::vector<uint16_t>(NUM_VOLTAGES);
            this->car_status.brick_data[i].temperatures = std::vector<uint8_t>(NUM_TEMPERATURES);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarStatusNode>());
    rclcpp::shutdown();
    return 0;
}
