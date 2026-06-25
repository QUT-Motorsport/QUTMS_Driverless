# QUTMS ros2_control Hardware Interfaces Integration Plan

This document outlines the implementation plan for integrating several `ros2_control` hardware interfaces within the `qutms_driverless` repository.

## 1. Directory & Package Structure

As decided, we will recreate the split package structure directly under `src/control/` and remove the existing placeholder `qev_controllers` package.

```
src/control/
├── qutms_hw_interfaces/              # [NEW] Custom hardware interfaces package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── qutms_hw_interfaces.xml       # pluginlib description
│   ├── include/qutms_hw_interfaces/
│   │   ├── qev_stepper_interface.hpp  # Stepper/CANopen steering actuator
│   │   ├── encos_steering_interface.hpp # ENCOS CAN steering actuator
│   │   ├── vcu_driving_interface.hpp  # VCU heartbeat driving interface
│   │   ├── sevcon_driving_interface.hpp # Sevcon H-protocol driving interface
│   │   └── dti_driving_interface.hpp  # DTI inverter driving interface
│   └── src/
│       ├── qev_stepper_interface.cpp
│       ├── encos_steering_interface.cpp
│       ├── vcu_driving_interface.cpp
│       ├── sevcon_driving_interface.cpp
│       └── dti_driving_interface.cpp
└── ros2_control_bringup/             # [NEW] Launch and controller configuration package
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── qev-3d_controllers.yaml
    └── launch/
        └── qev-3d.launch.py
```

---

## 2. Hardware Interfaces Specifications

All hardware interfaces will be implemented as separate plugins inheriting from `hardware_interface::SystemInterface`. They will use **Direct CAN-based communication** via a dedicated `SocketCAN` instance binding directly to the configured CAN interface (e.g. `can0`), bypassing intermediate driver nodes.

### Steering Interfaces

#### A. QEV Stepper Interface (`qev_stepper_interface`)
* **Base Reference**: Existing setup in `hardware/steering_actuator` and `hardware/canbus`.
* **Joints**: `virtual_steering_hinge_joint` (Position Command & State).
* **Protocol**: CANopen SDO Read/Write commands to a specific `C5E_NODE_ID`.
  * **Configuration**: Set profile velocity, acceleration, deceleration, quick stop deceleration, max acceleration, max deceleration.
  * **Transitions**: State machine sequencing (Not ready to switch on -> Switch on disabled -> Ready to switch on -> Switched on -> Operation enabled).
  * **Control**: Write `TARGET_POSITION` and trigger motion via `CONTROL_WORD`.
  * **State**: Read `POSITION_ACTUAL_VAL` via SDO.

#### B. ENCOS Steering Interface (`encos_steering_interface`)
* **Base Reference**:南京 Encos Intelligent Technology Motor Debugging Manual.
* **Joints**: `virtual_steering_hinge_joint` (Position Command & State).
* **Protocol**: Encos CAN Protocol (Standard 11-bit CAN, 1Mbps baud rate).
  * **Control Mode**: Servo Position Control Mode (Mode `0x01`).
  * **TX Frame**: Sent to `Motor ID`. Data length 8 bytes:
    * Mode (3 bits): `0x01`
    * Target Position (32-bit float): Target angle in degrees.
    * Target Speed (15-bit uint): `0 ~ 32767` corresponds to `0 ~ 3276.7 rpm` (scale 10).
    * Current Limit (12-bit uint): `0 ~ 4095` corresponds to `0 ~ 409.5 A` (scale 10).
    * Msg Return Status (2 bits): `1` (Type 1 feedback).
  * **Heartbeat**: 500ms timeout protection. Commands must be sent periodically.
  * **RX Frame (Type 1 feedback)**: Received from `Motor ID`. Data length 8 bytes:
    * Msg Type (3 bits): `0x01`
    * Error Info (5 bits): Over-temp, over-current, voltage faults, encoder error, etc.
    * Position (16-bit uint): Output shaft position in radians (`0 ~ 65535` corresponds to `-12.5 ~ 12.5 rad`).
    * Speed (12-bit uint): Output shaft speed in rad/s (`0 ~ 4095` corresponds to `-18.0 ~ 18.0 rad/s`).
    * Phase Current (12-bit uint): Motor current.
    * Temps (2x 8-bit): Motor and MOS temperatures.

---

### Driving Interfaces

All driving interfaces will adopt a **Dual-motor drive** setup managing `rear_left_wheel_joint` and `rear_right_wheel_joint` (Velocity State & Velocity/Effort Command).

#### A. VCU Heartbeat Driving Interface (`vcu_driving_interface`)
* **Base Reference**: Existing setup in `control/velocity_controller`.
* **Protocol**: Standard QUTMS VCU Heartbeat CAN frames.
  * **TX Frame**: `Request_Heartbeat` sent containing torque command percentage (`accel * 100`), target steering angle, and target speed.

#### B. Sevcon H-Protocol Driving Interface (`sevcon_driving_interface`)
* **Base Reference**: BorgWarner H-Protocol Application Note (29-bit CAN, J1939-based).
* **Protocol**: Configurable PDU1 J1939 peer-to-peer format (PF < 240, PS = Destination Address).
  * **Velocity Control**: The H-protocol is fundamentally a torque-control protocol and does **not** support a direct target velocity command. To control velocity, the hardware interface must either:
    1. Receive a torque/effort command from a ROS2 velocity controller (such as a PID controller running in ROS2).
    2. Set a constant torque demand and dynamically update the **Forward/Reverse Speed Limits** in **HC2** to act as the velocity targets.
  * **Gearbox and Conversions**: Since the motor is connected via a gearbox, we will read the `gear_ratio` parameter from the URDF joint configuration to convert wheel joint velocity (rad/s) to motor speed (RPM):
    $$\text{Motor RPM} = \left(\text{Joint Velocity} \times \frac{60}{2\pi}\right) \times \text{gear\_ratio}$$
  * **Diagnostics & Faults**: The interface will read `HS2` and `HS3`/`HS4` to monitor available torques, status flags (Precharge, Energised, Enabled, Fault off), temperatures (heat sink, cutback limits), and fault codes. These will be published to a ROS2 diagnostics topic and exported as custom state interfaces (e.g., `motor_temp`, `inverter_temp`, `fault_code`).
  * **State Machine**: The startup state machine does not have to be handled by ROS2 (e.g., pre-charge sequencing is managed externally). The interface only needs to read the current state (from HS2) and transmit control commands.
  * **TX Frames**:
    * **HC1 (PF 0x10)**: Torque demand (Signed 16-bit, 0.0625 Nm/bit), Control Word, Drive Torque Limit (Signed 16-bit), SEQ, CS.
    * **HC2 (PF 0x11)**: Regen Torque Limit, Forward Speed Limit (Signed 16-bit, 1 RPM/bit), Reverse Speed Limit (Signed 16-bit, 1 RPM/bit), SEQ, CS.
    * **HC3 (PF 0x12)**: Battery Current Limits, Target Cap Voltage, SEQ, CS.
  * **RX Frames**:
    * **HS1 (PF 0x18)**: Measured output torque, Measured motor speed, Battery current, SEQ, CS.
    * **HS2 (PF 0x19)**: Available forward/reverse torque, Status Word (lower 4 bits is state), SEQ, CS.
  * **CS (Checksum)**: Sum of bytes 0 to 6 modulo 256.
  * **Heartbeat/Timeout**: 5ms base period (HC3 is 50ms).

#### C. DTI Inverter Driving Interface (`dti_driving_interface`)
* **Base Reference**: DTI Inverter CAN Manual V2.5.
* **Protocol**: DTI CAN2 Protocol (Big-Endian/Motorola, Standard 11-bit or Extended 29-bit CAN, 500kbps or 1Mbps).
  * **Velocity Control**: **Supported directly**. The interface can write directly to **Packet 0x03 (Set ERPM)** to control target velocity, or **Packet 0x01 (Set AC Current)** for torque control.
  * **Gearbox and Conversions**: We will read `gear_ratio` and `pole_pairs` parameters from the URDF joint configuration to convert wheel joint velocity (rad/s) to motor electrical RPM (ERPM):
    $$\text{Motor ERPM} = \left(\text{Joint Velocity} \times \frac{60}{2\pi}\right) \times \text{gear\_ratio} \times \text{pole\_pairs}$$
  * **Diagnostics & Faults**: The interface will read `Packet 0x22` to monitor controller and motor temperatures, and decode fault codes (e.g., Overvoltage, Undervoltage, DRV fault, Overcurrent, Overtemp, Sensor wire fault). These will be published to ROS2 diagnostics and exposed as custom joint state interfaces.
  * **ID Format**:
    * Standard: `(Packet ID << 5) | Node ID`
    * Extended: `(Packet ID << 8) | Node ID`
  * **TX Frames**:
    * **Packet 0x03 (Set Speed/ERPM)**: Target ERPM (Signed 32-bit, Big Endian). Automatically enables inverter speed control.
    * **Packet 0x01 (Set AC Current)**: AC current (Signed 16-bit, scale 10). Automatically switches inverter to current control.
    * **Packet 0x0C (Drive Enable)**: Byte 0 set to `1` (Enable) or `0` (Disable).
  * **RX Frames**:
    * **Packet 0x20**: ERPM (Signed 32-bit), Duty Cycle (Signed 16-bit, scale 10), Input DC Voltage (16-bit).
    * **Packet 0x21**: AC Current (Signed 16-bit, scale 10), DC Current (Signed 16-bit, scale 10).
    * **Packet 0x22**: Controller Temp, Motor Temp, Fault Code.

---

## 3. Diagnostics & Fault Reporting System

To integrate raw CAN device status into the wider ROS2 ecosystem, the hardware interfaces will implement a unified diagnostics reporting mechanism:

1. **Custom State Interfaces**:
   In addition to standard `position` and `velocity` state interfaces, each plugin will export custom state interfaces in `export_state_interfaces()`:
   * `<joint_name>/motor_temp` (double value in °C)
   * `<joint_name>/inverter_temp` (double value in °C)
   * `<joint_name>/fault_code` (double raw integer representation of the active fault code)
   * `<joint_name>/dc_voltage` (double value in V)

2. **ROS2 Diagnostics Publisher**:
   During `on_init()`, each hardware interface will create a ROS2 diagnostics publisher (publishing to `/diagnostics` using `diagnostic_msgs/msg/DiagnosticArray`). This publisher will broadcast human-readable diagnostics updates on a regular interval (e.g., 10Hz) containing:
   * Inverter/Controller temperature warnings.
   * Specific decoded fault names (e.g., "Overvoltage", "Sensor Wire Fault").
   * Connection status and heartbeat timeout warnings.

---

## 4. Verification Plan

### Automated Tests
* Unit test targets inside `qutms_hw_interfaces/test` (e.g. `test_qev_stepper_interface.cpp` using GMock to load the plugins and verify state conversions).
* Verify successful compilation:
  ```bash
  colcon build --packages-select qutms_hw_interfaces ros2_control_bringup
  ```

### Manual Verification
* Run launch file locally in mock/sim mode:
  ```bash
  ros2 launch ros2_control_bringup qev-3d.launch.py use_mock_hardware:=true
  ```
* Echo state topics and publish command velocities to verify end-to-end controller loop integration.
