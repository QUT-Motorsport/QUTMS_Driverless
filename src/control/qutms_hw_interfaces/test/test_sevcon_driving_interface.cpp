#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "qutms_hw_interfaces/sevcon_driving_interface.hpp"

using ::testing::_;
using ::testing::Invoke;
using ::testing::NiceMock;
using ::testing::Return;

static constexpr uint8_t PF_HC1 = 0x10;

class MockSocketCAN : public SocketCAN {
   public:
    MOCK_METHOD(bool, setup, (std::string interface, rclcpp::Logger logger), (override));
    MOCK_METHOD(void, tx, (driverless_msgs::msg::Can * msg, rclcpp::Logger logger), (override));
    MOCK_METHOD(std::shared_ptr<std::vector<driverless_msgs::msg::Can>>, rx,
                (rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock), (override));
};

class SevconDrivingInterfaceTest : public ::testing::Test {
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
        info.name = "TestSevconDrivingInterface";
        info.type = "system";

        hardware_interface::ComponentInfo joint_left;
        joint_left.name = "rear_left_wheel_joint";
        joint_left.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_EFFORT));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_left.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        joint_left.state_interfaces.push_back(create_interface_info("motor_temp"));
        joint_left.state_interfaces.push_back(create_interface_info("inverter_temp"));
        joint_left.state_interfaces.push_back(create_interface_info("fault_code"));
        joint_left.state_interfaces.push_back(create_interface_info("dc_voltage"));
        info.joints.push_back(joint_left);

        hardware_interface::ComponentInfo joint_right;
        joint_right.name = "rear_right_wheel_joint";
        joint_right.command_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_EFFORT));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_POSITION));
        joint_right.state_interfaces.push_back(create_interface_info(hardware_interface::HW_IF_VELOCITY));
        info.joints.push_back(joint_right);

        info.hardware_parameters["can_interface"] = "vcan0";
        info.hardware_parameters["left_motor_id"] = "34";
        info.hardware_parameters["right_motor_id"] = "35";
        info.hardware_parameters["vcu_sa"] = "0";
        info.hardware_parameters["gear_ratio"] = "4.50";
        info.hardware_parameters["wheel_radius"] = "0.2";
        info.hardware_parameters["control_mode"] = "internal_pid";
        info.hardware_parameters["torque_limit_nm"] = "100.0";
        info.hardware_parameters["regen_limit_nm"] = "-100.0";

        mock_can = new NiceMock<MockSocketCAN>();
        interface = std::make_shared<qutms_hw_interfaces::SevconDrivingInterface>();
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
    std::shared_ptr<qutms_hw_interfaces::SevconDrivingInterface> interface;
};

TEST_F(SevconDrivingInterfaceTest, test_init_and_configure) {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    params.logger = rclcpp::get_logger("TestLogger");
    EXPECT_EQ(interface->init(params), hardware_interface::CallbackReturn::SUCCESS);

    EXPECT_CALL(*mock_can, setup("vcan0", _)).WillOnce(Return(true));
    rclcpp_lifecycle::State state;
    EXPECT_EQ(interface->on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(SevconDrivingInterfaceTest, test_read_feedback) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    // Create mock J1939 HS1 (speed feedback) frame from Left Motor (ID 34 = 0x22)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();

    driverless_msgs::msg::Can frame_speed;
    frame_speed.id = (6 << 26) | (0x18 << 16) | (0xFF << 8) | 0x22;
    frame_speed.dlc = 8;
    frame_speed.data = {0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_speed);

    // Create HS2 status word frame showing state = Shutdown (0x02)
    driverless_msgs::msg::Can frame_status;
    frame_status.id = (6 << 26) | (0x19 << 16) | (0xFF << 8) | 0x22;
    frame_status.dlc = 8;
    frame_status.data = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));

    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    EXPECT_EQ(interface->read(time, period), hardware_interface::return_type::OK);

    auto left_vel_handle = interface->get_state_interface_handle("rear_left_wheel_joint/velocity");
    // 1000 RPM / 4.5 = 222.2 RPM -> 23.27 rad/s
    EXPECT_NEAR(*left_vel_handle->get_optional(), 23.27, 0.1);
}

TEST_F(SevconDrivingInterfaceTest, test_write_and_state_machine) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto left_eff_cmd_handle = interface->get_command_interface_handle("rear_left_wheel_joint/effort");
    auto right_eff_cmd_handle = interface->get_command_interface_handle("rear_right_wheel_joint/effort");

    EXPECT_TRUE(left_eff_cmd_handle->set_value(0.5));
    EXPECT_TRUE(right_eff_cmd_handle->set_value(0.5));

    // Transition test: Setup mock status representing Shutdown (0x02)
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_status_left;
    frame_status_left.id = (6 << 26) | (0x19 << 16) | (0xFF << 8) | 0x22;
    frame_status_left.dlc = 8;
    frame_status_left.data = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status_left);

    driverless_msgs::msg::Can frame_status_right;
    frame_status_right.id = (6 << 26) | (0x19 << 16) | (0xFF << 8) | 0x23;
    frame_status_right.dlc = 8;
    frame_status_right.data = {0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status_right);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));
    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    interface->read(time, period);

    // During write, since current state is Shutdown (0x02), state machine should issue ENERGISE (0x0003) command
    EXPECT_CALL(*mock_can, tx(_, _)).Times(6).WillRepeatedly(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        uint8_t pf = (msg->id >> 16) & 0xFF;
        if (pf == PF_HC1) {
            // Control word (bytes 2-3) should be ENERGISE (0x0003)
            uint16_t cw = static_cast<uint16_t>((msg->data[3] << 8) | msg->data[2]);
            EXPECT_EQ(cw, 0x0003);

            // Checksum (byte 7) should be correct modulo 256 sum
            uint8_t cs = 0;
            for (int i = 0; i < 7; i++) {
                cs += msg->data[i];
            }
            EXPECT_EQ(msg->data[7], cs);
        }
    }));

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}

TEST_F(SevconDrivingInterfaceTest, test_write_command) {
    EXPECT_EQ(init_interface(), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(configure_and_activate(), hardware_interface::CallbackReturn::SUCCESS);

    auto left_eff_cmd_handle = interface->get_command_interface_handle("rear_left_wheel_joint/effort");
    auto right_eff_cmd_handle = interface->get_command_interface_handle("rear_right_wheel_joint/effort");

    // Set effort commands: 0.5 (50% torque limit = 50.0 Nm)
    EXPECT_TRUE(left_eff_cmd_handle->set_value(0.5));
    EXPECT_TRUE(right_eff_cmd_handle->set_value(0.5));

    // Mock status representing Operation Enabled (0x07) to prevent state machine transitions
    auto rx_frames = std::make_shared<std::vector<driverless_msgs::msg::Can>>();
    driverless_msgs::msg::Can frame_status_left;
    frame_status_left.id = (6 << 26) | (0x19 << 16) | (0xFF << 8) | 0x22;
    frame_status_left.dlc = 8;
    frame_status_left.data = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status_left);

    driverless_msgs::msg::Can frame_status_right;
    frame_status_right.id = (6 << 26) | (0x19 << 16) | (0xFF << 8) | 0x23;
    frame_status_right.dlc = 8;
    frame_status_right.data = {0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00};
    rx_frames->push_back(frame_status_right);

    EXPECT_CALL(*mock_can, rx(_, _)).WillOnce(Return(rx_frames));
    rclcpp::Time time;
    rclcpp::Duration period(0, 50000000);
    interface->read(time, period);

    // During write, since current state is OE (0x07) and command is effort 0.5:
    // Torque demand should be 0.5 * 100.0 Nm = 50.0 Nm
    // Raw torque = 50.0 * 16 = 800 (0x0320)
    EXPECT_CALL(*mock_can, tx(_, _)).Times(6).WillRepeatedly(Invoke([](driverless_msgs::msg::Can* msg, rclcpp::Logger) {
        uint8_t pf = (msg->id >> 16) & 0xFF;
        if (pf == PF_HC1) {
            // Raw torque is in msg->data[0] and [1]
            int16_t torque_raw = static_cast<int16_t>((msg->data[1] << 8) | msg->data[0]);
            EXPECT_EQ(torque_raw, 800);
        }
    }));

    EXPECT_EQ(interface->write(time, period), hardware_interface::return_type::OK);
}
