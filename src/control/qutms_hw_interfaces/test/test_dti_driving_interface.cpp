#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

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
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        info.name = "TestDtiDrivingInterface";
        info.type = "system";

        hardware_interface::ComponentInfo joint_left;
        joint_left.name = "rear_left_wheel_joint";
        joint_left.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        joint_left.state_interfaces.push_back(create_interface_info("motor_temp"));
        joint_left.state_interfaces.push_back(create_interface_info("inverter_temp"));
        joint_left.state_interfaces.push_back(create_interface_info("fault_code"));
        joint_left.state_interfaces.push_back(create_interface_info("dc_voltage"));
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
        hardware_interface::HardwareComponentParams params;
        params.hardware_info = info;
        params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        params.logger = rclcpp::get_logger("TestLogger");
        return interface->init(params);
    }

    hardware_interface::CallbackReturn configure_and_activate() {
        (void)interface->on_export_state_interfaces();
        (void)interface->on_export_command_interfaces();

        EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
        rclcpp_lifecycle::State state;
        if (interface->on_configure(state) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (interface->on_activate(state) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::HardwareInfo info;
    NiceMock<MockSocketCAN>* mock_can;
    std::shared_ptr<qutms_hw_interfaces::DtiDrivingInterface> interface;
};

TEST_F(DtiDrivingInterfaceTest, test_init_and_configure) {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    params.logger = rclcpp::get_logger("TestLogger");
    EXPECT_EQ(interface->init(params), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(DtiDrivingInterfaceTest, test_read_feedback) {
    info.hardware_parameters["left_motor_id"] = "8";
    info.hardware_parameters["right_motor_id"] = "9";
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_left;
    frame_left.id = (0x20 << 5) | 8;  // Left Motor (ID 8)
    frame_left.dlc = 8;
    frame_left.data = {0x00, 0x01, 0x71, 0x24, 0x00, 0x00, 0x0F, 0xA0};  // ERPM = 94500, cap voltage = 4000
    rx_frames->push_back(frame_left);

    driverless_msgs::msg::Can frame_right;
    frame_right.id = (0x20 << 5) | 9;  // Right Motor (ID 9)
    frame_right.dlc = 8;
    frame_right.data = {0x00, 0x01, 0x71, 0x24, 0x00, 0x00, 0x0F, 0xA0};  // ERPM = 94500, cap voltage = 4000
    rx_frames->push_back(frame_right);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    auto left_vel_handle = interface->get_state_interface_handle("rear_left_wheel_joint/velocity");
    auto left_dc_volt_handle = interface->get_state_interface_handle("rear_left_wheel_joint/dc_voltage");

    // erpm = 94500 -> motor RPM = 94500/21 = 4500 RPM -> wheel velocity = (4500/4.5) * 2pi/60 = 104.72 rad/s
    EXPECT_NEAR(*left_vel_handle->get_optional(), 104.72, 0.1);
    EXPECT_DOUBLE_EQ(*left_dc_volt_handle->get_optional(), 400.0);  // dc_voltage
}

TEST_F(DtiDrivingInterfaceTest, test_write_command) {
    info.hardware_parameters["left_motor_id"] = "8";
    info.hardware_parameters["right_motor_id"] = "9";
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto left_vel_cmd_handle = interface->get_command_interface_handle("rear_left_wheel_joint/velocity");
    auto right_vel_cmd_handle = interface->get_command_interface_handle("rear_right_wheel_joint/velocity");

    EXPECT_TRUE(left_vel_cmd_handle->set_value(20.0));   // rad/s
    EXPECT_TRUE(right_vel_cmd_handle->set_value(20.0));  // rad/s

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
