import os
from pathlib import Path
from pprint import pprint
import shlex
import signal
import subprocess
import time
import traceback

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from experiment_helpers import get_run_name


def start_sim(track_name: str, camera_range_noise: float, camera_gaussian_range_noise: bool):
    env_vars = os.environ.copy()
    env_vars["EUFS_MASTER"] = "/home/alistair/dev/repos/eufs_sim"
    cmd = f"ros2 launch eufs_launcher simulation2.launch.py track:={track_name} camera_range_noise:={camera_range_noise} camera_gaussian_range_noise:={camera_gaussian_range_noise}"
    return subprocess.Popen(shlex.split(cmd), stdin=subprocess.PIPE, shell=False, env=env_vars)


def manual_mode():
    cmd = 'ros2 service call /ros_can/set_mission eufs_msgs/srv/SetCanState "{ami_state: 21}"'
    subprocess.run(shlex.split(cmd))


def start_slam(known_association: bool, slam_range_var: float):
    cmd = f"ros2 launch slam sim_vision_launch.py range_variance:={slam_range_var} use_known_association:={known_association}"
    return subprocess.Popen(shlex.split(cmd), stdin=subprocess.PIPE, shell=False)


def start_controls(track_name: str):
    cmd = f"ros2 bag play /home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/controls"
    return subprocess.Popen(shlex.split(cmd), stdin=subprocess.PIPE, shell=False)


def record_bag(path: Path):
    cmd = f"ros2 bag record /slam/pose /slam/track /ground_truth/odom /ground_truth/global_map -o {path.absolute()}"
    return subprocess.Popen(shlex.split(cmd), stdin=subprocess.PIPE, shell=False)


class DataWatcherNode(Node):
    start_box_x = 9
    start_box_y = 4
    start_box_exit_count = 0
    in_start_box = True

    processes: list

    def __init__(
        self,
        track_name: str,
        camera_range_noise: float,
        camera_gaussian_range_noise: bool,
        known_association: bool,
        slam_range_var: float,
        processes: list,
    ):
        super().__init__("data_grabber")

        self.create_subscription(Odometry, "/ground_truth/odom", self.callback, 10)

        self.processes = processes

    def callback(self, msg: Odometry):
        now_in_start_box = (
            abs(msg.pose.pose.position.x) < self.start_box_x and abs(msg.pose.pose.position.y) < self.start_box_y
        )
        if not now_in_start_box and self.in_start_box:
            self.start_box_exit_count += 1
            if self.start_box_exit_count > 1:
                raise SystemExit

        self.in_start_box = now_in_start_box


def exit_processes(processes):
    for p in processes:
        p.send_signal(signal.SIGINT)

    while True:
        try:
            all_finished = True
            for p in processes:
                ret = p.wait()
                if ret is None:
                    all_finished = False
            if all_finished:
                break

        except KeyboardInterrupt:
            for p in processes:
                p.send_signal(signal.SIGINT)


def do_one_run(
    track_name: str,
    camera_gaussian_range_noise: bool,
    known_association: bool,
    camera_range_noise: float,
    slam_range_var: float,
    run_num: int,
):
    print(
        track_name,
        camera_gaussian_range_noise,
        known_association,
        camera_range_noise,
        slam_range_var,
        run_num,
    )
    bag_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")
    bag_folder.mkdir(parents=True, exist_ok=True)
    bag_name = get_run_name(
        track_name,
        camera_gaussian_range_noise,
        known_association,
        camera_range_noise,
        slam_range_var,
        run_num,
    )

    processes = []
    try:
        sim = start_sim(track_name, camera_range_noise, camera_gaussian_range_noise)
        processes.append(sim)

        time.sleep(5)

        manual_mode()

        slam = start_slam(known_association, slam_range_var)
        processes.append(slam)

        bag = record_bag(bag_folder / bag_name)
        processes.append(bag)

        time.sleep(2)

        controls = start_controls(track_name)
        processes.append(controls)

        rclpy.init()
        node = DataWatcherNode(
            track_name,
            camera_range_noise,
            camera_gaussian_range_noise,
            known_association,
            slam_range_var,
            processes,
        )
        try:
            rclpy.spin(node)
        except SystemExit:
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(traceback.format_exc())
    finally:
        exit_processes(processes)


if __name__ == "__main__":
    # do_one_run(
    #     track_name="small_track",
    #     camera_gaussian_range_noise=True,
    #     known_association=True,
    #     camera_range_noise=0.8,
    #     slam_range_var=0.8,
    #     run_num=1,
    # )

    # for track_name in ["small_track", "B_shape_02_03_2023", "QR_Nov_2022"]:
    for track_name in ["small_track"]:

        for camera_gaussian_range_noise in [True, False]:

            for known_association in [False]:

                # for camera_range_noise in [0.01, 0.05] + [round(0.1 * i, 1) for i in range(1, 5)]:
                for camera_range_noise in [round(0.1 * i, 1) for i in range(1, 5)]:

                    if camera_range_noise < 0.1:
                        range_var_options = [0.05, 0.08, 0.1]
                    else:
                        range_var_options = [
                            round(0.1 * i, 1)
                            for i in range(int(10 * camera_range_noise), int(10 * camera_range_noise) + 4)
                        ]

                    for slam_range_var in range_var_options:

                        for run_num in range(1, 4):

                            do_one_run(
                                track_name,
                                camera_gaussian_range_noise,
                                known_association,
                                camera_range_noise,
                                slam_range_var,
                                run_num,
                            )
