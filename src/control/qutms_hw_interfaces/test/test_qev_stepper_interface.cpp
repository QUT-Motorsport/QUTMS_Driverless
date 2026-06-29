#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include "CAN_VCU.h"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "qutms_hw_interfaces/qev_stepper_interface.hpp"

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

class QevStepperInterfaceTest : public ::testing::Test {
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
        info.name = "TestQevStepperInterface";
        info.type = "system";

        hardware_interface::ComponentInfo joint;
        joint.name = "virtual_front_wheel_joint";
        joint.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint.state_interfaces.push_back(create_interface_info("motor_temp"));
        joint.state_interfaces.push_back(create_interface_info("inverter_temp"));
        joint.state_interfaces.push_back(create_interface_info("fault_code"));
        joint.state_interfaces.push_back(create_interface_info("dc_voltage"));
        info.joints.push_back(joint);

        info.hardware_parameters["can_interface"] = "vcan0";
        info.hardware_parameters["node_id"] = "112";  // 0x70
        info.hardware_parameters["max_position"] = "7500";
        info.hardware_parameters["velocity"] = "10000";
        info.hardware_parameters["acceleration"] = "2000";

        mock_can = new NiceMock<MockSocketCAN>();
        interface = std::make_shared<qutms_hw_interfaces::QevStepperInterface>();
        interface->set_socket_can(std::unique_ptr<SocketCAN>(mock_can));
    }

    hardware_interface::CallbackReturn init_interface() {
        hardware_interface::HardwareComponentInterfaceParams params;
        params.hardware_info = info;
        return interface->on_init(params);
    }

    hardware_interface::HardwareInfo info;
    NiceMock<MockSocketCAN>* mock_can;
    std::shared_ptr<qutms_hw_interfaces::QevStepperInterface> interface;
};

TEST_F(QevStepperInterfaceTest, test_init_and_configure) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(QevStepperInterfaceTest, test_read_and_state_transitions) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    // Mock status word response representing "Ready to switch on" (0x0021)
    // SDO read status word response from Node 0x70. CAN ID = 0x580 + 0x70 = 0x5F0
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_status;
    frame_status.id = 0x5F0;
    frame_status.dlc = 8;
    // Object status word is 0x6041 (msg.data[1]=0x41, data[2]=0x60)
    // Status value: 0x0021 (data[4]=0x21, data[5]=0x00)
    frame_status.data = {0x4B, 0x41, 0x60, 0x00, 0x21, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status);

    // Also include a position actual value frame (Node ID 0x70, ID = 0x280 + 0x70 = 0x2F0)
    driverless_msgs::msg::Can frame_pos;
    frame_pos.id = 0x2F0;
    frame_pos.dlc = 8;
    // Position = 500 ticks (bytes 0-3: 0x000001F4)
    frame_pos.data = {0xF4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_pos);

    // Also include a VCU Transmit Steering calibration frame (reports 8.0 deg steering angle)
    driverless_msgs::msg::Can frame_vcu_steer;
    frame_vcu_steer.id = VCU_TransmitSteering_ID;
    frame_vcu_steer.dlc = 8;
    VCU_TransmitSteering_t vcu_steer_data = Compose_VCU_TransmitSteering(80, 80, 0, 0);
    frame_vcu_steer.data.assign(vcu_steer_data.data, vcu_steer_data.data + 8);
    rx_frames->push_back(frame_vcu_steer);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    // When desired state is Operation Enabled (0x0027) upon activation
    rclcpp_lifecycle::State active_state;
    interface->on_activate(active_state);

    // Check transitions: since state is RTSO (0x21) and we want OE (0x27), state transitions should trigger write of SO
    // (0x0023) control word
    EXPECT_CALL(*mock_can, tx(_, _)).Times(2);  // 1 request for status, 1 for transition command

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    // Check position mapping (reports 8.0 deg steering angle, so position state is 8.0 deg in rad)
    auto states = interface->export_state_interfaces();
    EXPECT_DOUBLE_EQ(*states[0].get_optional(), 8.0 * (M_PI / 180.0));
}

TEST_F(QevStepperInterfaceTest, test_write_position) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);

    auto commands = interface->export_command_interfaces();
    ASSERT_EQ(commands.size(), 1u);

    // Set position command: 500.0 ticks (since always chained and we removed target ticks algorithm)
    EXPECT_TRUE(commands[0].set_value(500.0));

    // Trigger read to change current state to Operation Enabled (0x0027)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_status;
    frame_status.id = 0x5F0;
    frame_status.dlc = 8;
    frame_status.data = {0x4B, 0x41, 0x60, 0x00, 0x27, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status);

    driverless_msgs::msg::Can frame_pos;
    frame_pos.id = 0x2F0;
    frame_pos.dlc = 8;
    frame_pos.data = {0xF4, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_pos);

    driverless_msgs::msg::Can frame_vcu_steer;
    frame_vcu_steer.id = VCU_TransmitSteering_ID;
    frame_vcu_steer.dlc = 8;
    VCU_TransmitSteering_t vcu_steer_data = Compose_VCU_TransmitSteering(80, 80, 0, 0);
    frame_vcu_steer.data.assign(vcu_steer_data.data, vcu_steer_data.data + 8);
    rx_frames->push_back(frame_vcu_steer);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));
    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);

    // Activate to enable transitions
    rclcpp_lifecycle::State active_state;
    interface->on_activate(active_state);

    // Read to register state is OE
    interface->read(time, period);

    // Write should trigger SDO write to Target Position object (0x607A) with value 500 ticks
    EXPECT_CALL(*mock_can, tx(_, _)).Times(3);  // 1 for absolute mode, 1 for target value, 1 for trigger bit

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
