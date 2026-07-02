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
        joint_left.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_EFFORT));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        info.joints.push_back(joint_left);

        hardware_interface::ComponentInfo joint_right;
        joint_right.name = "rear_right_wheel_joint";
        joint_right.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_EFFORT));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        info.joints.push_back(joint_right);

        info.hardware_parameters["can_interface"] = "vcan0";
        info.hardware_parameters["wheel_radius"] = "0.2";

        mock_can = new NiceMock<MockSocketCAN>();
        interface = std::make_shared<qutms_hw_interfaces::VcuDrivingInterface>();
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
    std::shared_ptr<qutms_hw_interfaces::VcuDrivingInterface> interface;
};

TEST_F(VcuDrivingInterfaceTest, test_init_and_configure) {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    params.logger = rclcpp::get_logger("TestLogger");
    EXPECT_EQ(interface->init(params), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(VcuDrivingInterfaceTest, test_read_feedback) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    // Create mock RX CAN frames (VESC status packet 0x09)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();

    driverless_msgs::msg::Can frame_left;
    frame_left.id = (VESC_CAN_PACKET_STATUS << 8) | 2;  // ID = 0x0902
    frame_left.dlc = 8;
    frame_left.data = {0x00, 0x00, 0x52, 0x08, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_left);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);  // 50ms
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    auto left_vel_handle = interface->get_state_interface_handle("rear_left_wheel_joint/velocity");

    // Left wheel velocity state: 1000 RPM / 4.50 = 222.2 RPM -> 23.27 rad/s
    EXPECT_NEAR(*left_vel_handle->get_optional(), 23.27, 0.1);
}

TEST_F(VcuDrivingInterfaceTest, test_write_command) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto left_eff_cmd_handle = interface->get_command_interface_handle("rear_left_wheel_joint/effort");
    auto right_eff_cmd_handle = interface->get_command_interface_handle("rear_right_wheel_joint/effort");

    // Set effort commands (0.75 = 75% torque/effort demand)
    EXPECT_TRUE(left_eff_cmd_handle->set_value(0.75));
    EXPECT_TRUE(right_eff_cmd_handle->set_value(0.75));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);

    // Expecting Compose_Request_Heartbeat to be called with 75% torque request
    EXPECT_CALL(*mock_can, tx(_, _)).WillOnce(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        EXPECT_EQ(msg->id, 144621568u);  // VCU request ID
        EXPECT_EQ(msg->dlc, 8);

        int16_t torque_pct = static_cast<int16_t>((msg->data[1] << 8) | msg->data[0]);
        EXPECT_EQ(torque_pct, 24575);  // 75% of INT16_MAX
    }));

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
