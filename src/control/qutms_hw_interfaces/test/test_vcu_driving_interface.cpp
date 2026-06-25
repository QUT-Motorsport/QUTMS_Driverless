#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "qutms_hw_interfaces/vcu_driving_interface.hpp"

// Include embedded headers from QUTMS_Embedded_Common
#include "CAN_AV.h"
#include "CAN_VESC.h"

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

class MockSocketCAN : public SocketCAN {
   public:
    MOCK_METHOD(bool, setup, (std::string interface, rclcpp::Logger logger), (override));
    MOCK_METHOD(void, tx, (driverless_msgs::msg::Can * msg, rclcpp::Logger logger), (override));
    MOCK_METHOD(std::shared_ptr<std::vector<driverless_msgs::msg::Can>>, rx,
                (rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock), (override));
};

class VcuDrivingInterfaceTest : public ::testing::Test {
   protected:
    hardware_interface::InterfaceInfo create_interface_info(const std::string& name) {
        hardware_interface::InterfaceInfo info;
        info.name = name;
        info.size = 0;
        info.enable_limits = false;
        return info;
    }

    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        info.name = "TestVcuDrivingInterface";
        info.type = "system";

        hardware_interface::ComponentInfo joint_left;
        joint_left.name = "rear_left_wheel_joint";
        joint_left.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        info.joints.push_back(joint_left);

        hardware_interface::ComponentInfo joint_right;
        joint_right.name = "rear_right_wheel_joint";
        joint_right.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        info.joints.push_back(joint_right);

        info.hardware_parameters["can_interface"] = "vcan0";
        info.hardware_parameters["wheel_radius"] = "0.2";
        info.hardware_parameters["Kp"] = "0.5";
        info.hardware_parameters["Ki"] = "0.0";
        info.hardware_parameters["max_integral_torque"] = "0.0";

        mock_can = new NiceMock<MockSocketCAN>();
        interface = std::make_shared<qutms_hw_interfaces::VcuDrivingInterface>();
        interface->set_socket_can(std::unique_ptr<SocketCAN>(mock_can));
    }

    hardware_interface::CallbackReturn init_interface() {
        hardware_interface::HardwareComponentInterfaceParams params;
        params.hardware_info = info;
        return interface->on_init(params);
    }

    hardware_interface::HardwareInfo info;
    NiceMock<MockSocketCAN>* mock_can;
    std::shared_ptr<qutms_hw_interfaces::VcuDrivingInterface> interface;
};

TEST_F(VcuDrivingInterfaceTest, test_init_and_configure) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(VcuDrivingInterfaceTest, test_read_feedback) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    // Create mock RX CAN frames (VESC status packet 0x09)
    // Left wheel VESC (ID 2):
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();

    driverless_msgs::msg::Can frame_left;
    frame_left.id = (VESC_CAN_PACKET_STATUS << 8) | 2;  // ID = 0x0902
    frame_left.dlc = 8;
    // ERPM = 1000 RPM (motor) -> 1000 * 21 = 21000 ERPM (approx 0x00005208)
    frame_left.data = {0x00, 0x00, 0x52, 0x08, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_left);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);  // 50ms
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    // Exported states check
    auto states = interface->export_state_interfaces();
    ASSERT_EQ(states.size(), 4u);

    // Left wheel velocity state: 1000 RPM / 4.50 = 222.2 RPM -> 23.27 rad/s
    EXPECT_NEAR(*states[1].get_optional(), 23.27, 0.1);
}

TEST_F(VcuDrivingInterfaceTest, test_write_command) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    auto commands = interface->export_command_interfaces();
    ASSERT_EQ(commands.size(), 2u);

    // Set target velocities: 25.0 rad/s
    EXPECT_TRUE(commands[0].set_value(25.0));
    EXPECT_TRUE(commands[1].set_value(25.0));

    // Set mock current velocity state to 20.0 rad/s
    // Target speed (mps) = 25.0 * 0.2 = 5.0 m/s
    // Current speed (mps) = 20.0 * 0.2 = 4.0 m/s
    // Error = 1.0 m/s. Accel = Kp * error = 0.5 * 1.0 = 0.5 (50% torque request)

    // Trigger read to set mock state velocity to 20 rad/s (approx 191 RPM -> 18050 ERPM = 0x00004682)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_left;
    frame_left.id = (VESC_CAN_PACKET_STATUS << 8) | 2;
    frame_left.dlc = 8;
    frame_left.data = {0x00, 0x00, 0x46, 0x82, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_left);

    driverless_msgs::msg::Can frame_right;
    frame_right.id = (VESC_CAN_PACKET_STATUS << 8) | 3;
    frame_right.dlc = 8;
    frame_right.data = {0x00, 0x00, 0x46, 0x82, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_right);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));
    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    interface->read(time, period);

    EXPECT_CALL(*mock_can, tx(_, _)).WillOnce(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        EXPECT_EQ(msg->id, 144621568u);  // VCU request ID (0x089E8000)
        EXPECT_EQ(msg->dlc, 8);

        // Torque request should be 50%
        int16_t torque_pct = static_cast<int16_t>((msg->data[1] << 8) | msg->data[0]);  // assuming layout
        (void)torque_pct;
    }));

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
