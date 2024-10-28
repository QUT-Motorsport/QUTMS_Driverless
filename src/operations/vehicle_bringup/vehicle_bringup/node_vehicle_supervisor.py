import signal
from subprocess import Popen
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
    system_process = None
    mission_launched: bool = False
    mission_process = None
    finished: bool = False

    ros_state = ROSStateStamped()
    av_state = AVStateStamped()

    def __init__(self):
        super().__init__("vehicle_supervisor_node")

        # subscribers
        self.create_subscription(Odometry, "imu/odometry", self.odom_callback, 1)
        self.create_subscription(ConeDetectionStamped, "lidar/cone_detection", self.lidar_callback, 1)
        self.create_subscription(ConeDetectionStamped, "slam/global_map", self.map_callback, 1)
        self.create_subscription(Bool, "system/steering_ready", self.steering_callback, 1)
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

        self.create_timer(0.001, self.timer_callback)

        # publishers
        self.av_state_pub = self.create_publisher(AVStateStamped, "system/av_state", 1)
        self.ros_state_pub = self.create_publisher(ROSStateStamped, "system/ros_state", 1)

        self.reader = can.BufferedReader()
        self.notifier = can.Notifier(can_bus, [self.reader], 0.1)

        self.get_logger().info("---Mission control node initialised---")

    def send_request(self, mission: int, request: bool):
        request = SetBool.Request()
        request.data = request
        self.future = self.srv_list[mission].call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def timer_callback(self):
        # consolidate current ROS state
        self.ros_state.header.stamp = self.get_clock().now().to_msg()
        if (
            self.ros_state.sbg_operational
            and self.ros_state.lidar_operational
            and self.ros_state.planning
            # and self.ros_state.steering_ctrl
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
                command = ["stdbuf", "-o", "L", "ros2", "launch", "vehicle_bringup", "system.launch.py"]
                self.get_logger().info(f"Command: {' '.join(command)}")
                self.system_process = Popen(command)
                self.get_logger().info("Autnomous System started")
                self.system_started = True

            # start mission if in autonomous mode and mission is not none
            if (
                self.system_started
                and self.av_state.mission != AVStateStamped.MISSION_NONE
                and not self.mission_launched
                and self.ros_state.steering_ctrl
            ):
                # target_mission = INT_MISSION_TYPE[self.av_state.mission].value
                # node = target_mission + "_handler_node"
                # command = ["stdbuf", "-o", "L", "ros2", "run", "vehicle_bringup", node]

                # self.get_logger().info(f"Command: {' '.join(command)}")
                # self.mission_process = Popen(command)
                # self.get_logger().info("Mission started: " + target_mission)
                result = self.send_request(2, True)
                if result.success:
                    self.mission_launched = True

            # close mission if mission is finished
            if self.av_state.state == AVStateStamped.END and self.mission_launched and not self.finished:
                self.get_logger().warn("Closing mission")
                self.mission_process.send_signal(signal.SIGINT)
                self.finished = True

    def diagnostics_callback(self, msg: DiagnosticArray):
        if "velodyne_driver_node" in msg.status[0].name and msg.status[0].level == 0:
            self.ros_state.lidar_operational = True
        # if "sbg_driver_node" in msg.status[0].name and msg.status[0].level == 0:
        #     self.ros_state.sbg_operational = True
        if "ft_planner_node" in msg.status[0].name and msg.status[0].level == 0:
            self.ros_state.planning = True

    def steering_callback(self, msg: Bool):
        self.ros_state.steering_ctrl = msg.data

    def odom_callback(self, msg: Odometry):
        """Ensure the SBG EKF has settled and we get odom msgs before starting the mission"""
        ## THIS SHOULD BE LOGIC BASED ON SBG EKF STATUS VALUE
        self.ros_state.sbg_operational = True

    def lidar_callback(self, msg: ConeDetectionStamped):
        self.ros_state.identified_cones = len(msg.cones_with_cov)

    def map_callback(self, msg: ConeDetectionStamped):
        self.ros_state.mapped_cones = len(msg.cones_with_cov)

    def laps_callback(self, msg: UInt8):
        self.ros_state.lap_count = msg.data

    def shutdown_callback(self, msg: Shutdown):
        self.ros_state.finished = msg.finished_engage_ebs


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSupervisor()
    rclpy.spin(node)
    # shut down processes
    # node.system_process.terminate()
    # node.mission_process.terminate()
    node.can_bus.shutdown()
    node.destroy_node()
    rclpy.shutdown()
