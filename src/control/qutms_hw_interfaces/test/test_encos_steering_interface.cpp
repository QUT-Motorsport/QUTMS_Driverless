#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "qutms_hw_interfaces/encos_steering_interface.hpp"

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

class EncosSteeringInterfaceTest : public ::testing::Test {
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
        // Build mock HardwareInfo
        info.name = "TestEncosSteeringActuator";
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
        info.hardware_parameters["motor_id"] = "1";
        info.hardware_parameters["target_speed_rpm"] = "50.0";
        info.hardware_parameters["current_limit_a"] = "10.0";

        mock_can = new NiceMock<MockSocketCAN>();

        // Initialize interface
        interface = std::make_shared<qutms_hw_interfaces::EncosSteeringInterface>();
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
    std::shared_ptr<qutms_hw_interfaces::EncosSteeringInterface> interface;
};

TEST_F(EncosSteeringInterfaceTest, test_init_and_configure) {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    params.logger = rclcpp::get_logger("TestLogger");
    EXPECT_EQ(interface->init(params), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(EncosSteeringInterfaceTest, test_read_feedback) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    // Create a mock RX CAN frame (Type 1 feedback)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame;
    frame.id = 1;  // Motor ID
    frame.dlc = 8;
    frame.data = {0x25, 0x7F, 0xFF, 0x00, 0x00, 0x00, 150, 170};
    rx_frames->push_back(frame);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 0);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    // Exported state interfaces check
    auto pos_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/position");
    auto motor_temp_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/motor_temp");
    auto inverter_temp_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/inverter_temp");
    auto fault_code_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/fault_code");
    auto dc_voltage_handle = interface->get_state_interface_handle("virtual_front_wheel_joint/dc_voltage");

    // Check decrypted values
    EXPECT_NEAR(*pos_handle->get_optional(), 0.0, 0.01);
    EXPECT_DOUBLE_EQ(*motor_temp_handle->get_optional(), 50.0);     // motor_temp
    EXPECT_DOUBLE_EQ(*inverter_temp_handle->get_optional(), 60.0);  // inverter_temp
    EXPECT_DOUBLE_EQ(*fault_code_handle->get_optional(), 5.0);      // fault_code (error_code_ is 0x05)
    EXPECT_DOUBLE_EQ(*dc_voltage_handle->get_optional(), 0.0);      // dc_voltage
}

TEST_F(EncosSteeringInterfaceTest, test_write_command) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto command_handle = interface->get_command_interface_handle("virtual_front_wheel_joint/position");

    // Set position command: 0.5 rad (approx 28.65 degrees)
    EXPECT_TRUE(command_handle->set_value(0.5));

    EXPECT_CALL(*mock_can, tx(_, _)).WillOnce(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        EXPECT_EQ(msg->id, 1u);
        EXPECT_EQ(msg->dlc, 8);

        // Mode (first 3 bits of Big Endian 64-bit uint is type 1: 0x01 << 5 = 0x20 in msg.data[0])
        EXPECT_EQ((msg->data[0] >> 5) & 0x7, 0x01);

        // Check float packing
        float target_pos_deg;
        uint32_t pos_bits = 0;
        uint64_t packet = 0;
        for (int i = 0; i < 8; i++) {
            packet |= (static_cast<uint64_t>(msg->data[i]) << (8 * (7 - i)));
        }
        pos_bits = (packet >> 29) & 0xFFFFFFFF;
        std::memcpy(&target_pos_deg, &pos_bits, 4);

        EXPECT_NEAR(target_pos_deg, 28.65, 0.1);
    }));

    rclcpp::Time time;
    rclcpp::Duration period(0, 0);
    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
