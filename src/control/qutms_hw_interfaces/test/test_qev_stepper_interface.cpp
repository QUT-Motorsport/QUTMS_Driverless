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
    std::shared_ptr<qutms_hw_interfaces::QevStepperInterface> interface;
};

TEST_F(QevStepperInterfaceTest, test_init_and_configure) {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    params.logger = rclcpp::get_logger("TestLogger");
    EXPECT_EQ(interface->init(params), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(QevStepperInterfaceTest, test_read_and_state_transitions) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    // Mock status word response representing "Ready to switch on" (0x0021)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_status;
    frame_status.id = 0x5F0;
    frame_status.dlc = 8;
    frame_status.data = {0x4B, 0x41, 0x60, 0x00, 0x21, 0x00, 0x00, 0x00};
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

    // Check transitions
    EXPECT_CALL(*mock_can, tx(_, _)).Times(2);

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    // Check position mapping
    auto pos_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/position");
    EXPECT_DOUBLE_EQ(*pos_handle->get_optional(), 8.0 * (M_PI / 180.0));
}

TEST_F(QevStepperInterfaceTest, test_write_position) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto command_handle = interface->get_command_interface_handle("virtual_front_wheel_joint/position");
    EXPECT_TRUE(command_handle->set_value(500.0));

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

    // Read to register state is OE
    interface->read(time, period);

    // Write should trigger SDO write to Target Position object (0x607A) with value 500 ticks
    EXPECT_CALL(*mock_can, tx(_, _)).Times(3);

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
