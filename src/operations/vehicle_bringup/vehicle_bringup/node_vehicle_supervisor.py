import time

from ament_index_python.packages import get_package_share_path
import can
import cantools

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray
from driverless_msgs.msg import AVStateStamped, ConeDetectionStamped, ROSStateStamped, Shutdown
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8

from std_srvs.srv import SetBool

from driverless_common.status_constants import INT_MISSION_TYPE

can_bus = can.interface.Bus("can0", bustype="socketcan")
# can_bus = can.Bus(interface="virtual", channel="can0", receive_own_messages=True)
dbc_path = get_package_share_path("QUTMS_Embedded_Common") / "QUTMS_Embedded_Common" / "QUTMS.dbc"
db = cantools.database.load_file(dbc_path)


class VehicleSupervisor(Node):
    system_started: bool = False
    mission_launched: bool = False
    finished: bool = False

    ros_state = ROSStateStamped()
    av_state = AVStateStamped()

    timeout = 1.0  # seconds
    lidar_update_time = time.time()
    planning_update_time = time.time()
    sbg_update_time = time.time()

    def __init__(self):
        super().__init__("vehicle_supervisor_node")

        # subscribers
        self.create_subscription(Odometry, "imu/odometry", self.odom_callback, 1)
        self.create_subscription(ConeDetectionStamped, "lidar/cone_detection", self.lidar_callback, 1)
        self.create_subscription(ConeDetectionStamped, "slam/cone_detection", self.map_callback, 1)
        self.create_subscription(Shutdown, "system/shutdown", self.shutdown_callback, 1)
        self.create_subscription(UInt8, "system/laps_completed", self.laps_callback, 1)
        self.create_subscription(DiagnosticArray, "diagnostics", self.diagnostics_callback, 1)

        self.srv_list = [
            None,
            None,
            self.create_client(SetBool, "launch/inspection"),
            self.create_client(SetBool, "launch/ebs"),
            self.create_client(SetBool, "launch/trackdrive"),
        ]

        self.system_launch_cli = self.create_client(SetBool, "launch/system")

        self.create_timer(0.05, self.timer_callback)

        # publishers
        self.av_state_pub = self.create_publisher(AVStateStamped, "system/av_state", 1)
        self.ros_state_pub = self.create_publisher(ROSStateStamped, "system/ros_state", 1)

        self.reader = can.BufferedReader()
        self.notifier = can.Notifier(can_bus, [self.reader], 0.1)

        self.get_logger().info("---Mission control node initialised---")

    def send_mission_request(self, mission: int, request_val: bool):
        assert type(mission) == int
        request = SetBool.Request()
        request.data = request_val
        self.future = self.srv_list[mission].call_async(request)
        # rclpy.spin_until_future_complete(self, self.future)
        time.sleep(0.5)
        self.system_launch_cli.remove_pending_request(self.future)
        return True

    def send_system_request(self, request_val: bool):
        request = SetBool.Request()
        request.data = request_val
        self.future = self.system_launch_cli.call_async(request)
        # rclpy.spin_until_future_complete(self, self.future)
        time.sleep(0.5)
        self.system_launch_cli.remove_pending_request(self.future)
        return True

    def timer_callback(self):
        # consolidate current ROS state
        self.ros_state.header.stamp = self.get_clock().now().to_msg()

        # check for timeouts
        if time.time() - self.planning_update_time > self.timeout:
            self.ros_state.planning = False
        if time.time() - self.lidar_update_time > self.timeout:
            self.ros_state.lidar_operational = False
        if time.time() - self.sbg_update_time > self.timeout:
            self.ros_state.sbg_operational = False

        if (
            self.ros_state.lidar_operational
            and self.ros_state.sbg_operational
            # and self.ros_state.planning
            and not self.ros_state.finished
        ):
            self.ros_state.good_to_go = True
        elif self.av_state.mission == AVStateStamped.INSPECTION and not self.ros_state.finished:
            self.ros_state.good_to_go = True
        else:
            self.ros_state.good_to_go = False
        self.ros_state_pub.publish(self.ros_state)

        # publish ROS state to CAN
        ros_state_message = db.get_message_by_name("ROS_State")
        data = ros_state_message.encode(
            {
                "ROS_State_Steering": self.ros_state.steering_ctrl,
                "ROS_State_SBG": self.ros_state.sbg_operational,
                "ROS_State_LiDAR": self.ros_state.lidar_operational,
                "ROS_State_Planning": self.ros_state.planning,
                "ROS_State_Good_to_go": self.ros_state.good_to_go,
                "ROS_State_Finished": self.ros_state.finished,
                "ROS_State_Laps": self.ros_state.lap_count,
                "ROS_State_Cones_Identified": self.ros_state.identified_cones,
                "ROS_State_Cones_Mapped": self.ros_state.mapped_cones,
            }
        )
        try:
            message = can.Message(arbitration_id=ros_state_message.frame_id, data=data)
            can_bus.send(message)
        except can.CanOperationError:
            self.get_logger().error("Waiting for CAN bus to be available", throttle_duration_sec=1)

        # incoming messages for AV state
        while not self.reader.buffer.empty():
            message = self.reader.get_message()
            if message.arbitration_id != db.get_message_by_name("AV_State").frame_id:
                continue  # ignore messages that are not AV state

            msg_signals = db.decode_message(message.arbitration_id, message.data)

            # publish AV state to ROS
            self.av_state.header.stamp = self.get_clock().now().to_msg()
            self.av_state.mode = msg_signals["AV_State_Mode"]
            self.av_state.mission = msg_signals["AV_State_Mission"]
            self.av_state.state = msg_signals["AV_State_Status"]
            self.av_state_pub.publish(self.av_state)

            # start system if in autonomous mode
            if self.av_state.mode == AVStateStamped.AUTONOMOUS and not self.system_started:
                self.system_started = self.send_system_request(True)

            # start mission if in autonomous mode and mission is not none
            if (
                self.system_started
                and self.av_state.mission != AVStateStamped.MISSION_NONE
                and not self.mission_launched
            ):
                target_mission = INT_MISSION_TYPE[self.av_state.mission].value
                self.get_logger().info("Mission started: " + target_mission)
                self.mission_launched = self.send_mission_request(self.av_state.mission, True)

            if self.av_state.state == AVStateStamped.DRIVING:
                self.ros_state.steering_ctrl = True

            # close mission if mission is finished
            if self.av_state.state == AVStateStamped.END and self.mission_launched and not self.finished:
                self.get_logger().warn("Closing mission")
                self.finished = True

    def diagnostics_callback(self, msg: DiagnosticArray):
        if "velodyne_driver_node" in msg.status[0].name and int.from_bytes(msg.status[0].level, "big") == 0:
            self.ros_state.lidar_operational = True
            self.lidar_update_time = time.time()

        if "ft_planner_node" in msg.status[0].name and int.from_bytes(msg.status[0].level, "big") == 0:
            self.ros_state.planning = True
            self.planning_update_time = time.time()

    def odom_callback(self, msg: Odometry):
        """Ensure the SBG EKF has settled and we get odom msgs before starting the mission"""
        ## THIS SHOULD BE LOGIC BASED ON SBG EKF STATUS VALUE
        self.ros_state.sbg_operational = True
        self.sbg_update_time = time.time()

    def lidar_callback(self, msg: ConeDetectionStamped):
        self.ros_state.identified_cones = len(msg.cones)

    def map_callback(self, msg: ConeDetectionStamped):
        self.ros_state.mapped_cones = len(msg.cones)

    def laps_callback(self, msg: UInt8):
        self.ros_state.lap_count = msg.data

    def shutdown_callback(self, msg: Shutdown):
        self.ros_state.finished = msg.finished_engage_ebs


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSupervisor()
    rclpy.spin(node)
    node.can_bus.shutdown()
    node.destroy_node()
    rclpy.shutdown()
