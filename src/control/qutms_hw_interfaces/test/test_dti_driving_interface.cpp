#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "qutms_hw_interfaces/dti_driving_interface.hpp"

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

class DtiDrivingInterfaceTest : public ::testing::Test {
   protected:
    hardware_interface::InterfaceInfo create_interface_info(const std::string& name) {
        hardware_interface::InterfaceInfo info;
        info.name = name;
        info.size = 0;
        info.enable_limits = false;
        return info;
    }

    void SetUp() override {
        info.name = "TestDtiDrivingInterface";
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
        info.hardware_parameters["left_motor_id"] = "34";
        info.hardware_parameters["right_motor_id"] = "35";
        info.hardware_parameters["use_extended_id"] = "false";
        info.hardware_parameters["gear_ratio"] = "4.50";
        info.hardware_parameters["wheel_radius"] = "0.2";
        info.hardware_parameters["pole_pairs"] = "21";

        mock_can = new NiceMock<MockSocketCAN>();
        interface = std::make_shared<qutms_hw_interfaces::DtiDrivingInterface>();
        interface->set_socket_can(std::unique_ptr<SocketCAN>(mock_can));
    }

    hardware_interface::CallbackReturn init_interface() {
        hardware_interface::HardwareComponentInterfaceParams params;
        params.hardware_info = info;
        return interface->on_init(params);
    }

    hardware_interface::HardwareInfo info;
    NiceMock<MockSocketCAN>* mock_can;
    std::shared_ptr<qutms_hw_interfaces::DtiDrivingInterface> interface;
};

TEST_F(DtiDrivingInterfaceTest, test_init_and_configure) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(DtiDrivingInterfaceTest, test_read_feedback) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    // Create mock Packet 0x20 RX frame (Standard ID format: ID = 0x20 << 5 | motor_id)
    // Left motor (ID 34 = 0x22). Since 0x22 is > 30, wait, standard ID Node ID is 5 bits (1-30).
    // Let's test standard ID with motor ID = 8 (0x08)
    info.hardware_parameters["left_motor_id"] = "8";
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame;
    frame.id = (0x20 << 5) | 8;  // ID = 0x0408
    frame.dlc = 8;
    // ERPM = 94500 (approx 0x00017124) Big Endian
    frame.data = {0x00, 0x01, 0x71, 0x24, 0x00, 0x00, 0x0F, 0xA0};  // cap voltage 4000 (400.0V since scale 10)
    rx_frames->push_back(frame);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    auto states = interface->export_state_interfaces();
    // erpm = 94500 -> motor RPM = 94500/21 = 4500 RPM -> wheel velocity = (4500/4.5) * 2pi/60 = 104.72 rad/s
    EXPECT_NEAR(*states[1].get_optional(), 104.72, 0.1);
    EXPECT_DOUBLE_EQ(*states[7].get_optional(), 400.0);  // dc_voltage
}

TEST_F(DtiDrivingInterfaceTest, test_write_command) {
    info.hardware_parameters["left_motor_id"] = "8";
    info.hardware_parameters["right_motor_id"] = "9";
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    auto commands = interface->export_command_interfaces();
    ASSERT_EQ(commands.size(), 2u);
    EXPECT_TRUE(commands[0].set_value(20.0));  // rad/s
    EXPECT_TRUE(commands[1].set_value(20.0));  // rad/s

    // Target ERPM = (20.0 * 4.5 * 60 / 2pi) * 21 = 18050 ERPM (approx 0x00004682)

    EXPECT_CALL(*mock_can, tx(_, _)).Times(4).WillRepeatedly(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        uint8_t packet_id = (msg->id >> 5) & 0x7F;
        if (packet_id == 0x03) {  // Set ERPM
            // Verify Big Endian signed 32-bit value
            int32_t val = (msg->data[0] << 24) | (msg->data[1] << 16) | (msg->data[2] << 8) | msg->data[3];
            EXPECT_NEAR(val, 18050, 10);
        } else if (packet_id == 0x0C) {  // Drive Enable
            EXPECT_EQ(msg->data[0], 1);
        }
    }));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
