from collections import OrderedDict
import csv
from dataclasses import dataclass
from itertools import product
from math import degrees, sqrt
from pathlib import Path
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np
from transforms3d.euler import quat2euler

SLAM_TRACK = "_slam_track_"
SLAM_POSE = "_slam_pose_"
GT_TRACK = "_ground_truth_global_map_"
GT_ODOM = "_ground_truth_odom_"
CSV_PATH = "/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/small_track/sim_lidar_test/csv_data/"

TIMESTAMP = "2023-05-08T15:41:19"

slam_track_path = Path(CSV_PATH + SLAM_TRACK + TIMESTAMP + ".csv")
slam_pose_path = Path(CSV_PATH + SLAM_POSE + TIMESTAMP + ".csv")
gt_track_path = Path(CSV_PATH + GT_TRACK + TIMESTAMP + ".csv")
gt_odom_path = Path(CSV_PATH + GT_ODOM + TIMESTAMP + ".csv")


@dataclass
class Cone:
    x: float
    y: float
    colour: int
    id_: int


@dataclass
class Pose:
    x: float
    y: float
    theta: float


gt_cones: list[Cone] = []


def dict_to_cone(d: dict, id_: int) -> Cone:
    return Cone(
        x=d["location"]["x"],
        y=d["location"]["y"],
        colour=d["color"],
        id_=id_,
    )


def cone_dist(c1: Cone, c2: Cone) -> float:
    return sqrt((c1.x - c2.x) ** 2 + (c1.y - c2.y) ** 2)


def pose_euc_dist(p1: Pose, p2: Pose) -> float:
    return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def smallest_angle_dist(target_a: float, source_a: float):
    a = target_a - source_a
    return (a + 180) % 360 - 180


with open(gt_track_path) as f:
    reader = csv.DictReader(f)
    first_row = next(reader)
    for i, cone in enumerate(eval(first_row["cones_with_cov"])):
        gt_cones.append(dict_to_cone(cone["cone"], i))


cone_err = []
cone_err_stamp = []
unmatched_cones = []
unmatched_cones_stamp = []

with open(slam_track_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        stamp = float(row["header.stamp.sec"]) + (float(row["header.stamp.nanosec"]) * 1e-9)
        slam_cones: list[Cone] = []
        for i, cone in enumerate(eval(row["cones_with_cov"])):
            slam_cones.append(dict_to_cone(cone["cone"], i))

        unprocessed_gt = list(range(len(gt_cones)))
        unprocessed_slam = list(range(len(slam_cones)))

        final_pairs = []
        total_err = 0

        for gt_c, slam_c in sorted(
            product(gt_cones, slam_cones),
            key=lambda e: cone_dist(e[0], e[1])
            + (float("inf") if e[1].colour != 1 and e[0].colour != e[1].colour else 0),
        ):
            if len(unprocessed_gt) == 0 or len(unprocessed_slam) == 0:
                break

            if slam_c.colour != 1 and gt_c.colour != slam_c.colour:
                continue

            if gt_c.id_ in unprocessed_gt and slam_c.id_ in unprocessed_slam:
                unprocessed_gt.remove(gt_c.id_)
                unprocessed_slam.remove(slam_c.id_)
                final_pairs.append((gt_c, slam_c))
                total_err += cone_dist(gt_c, slam_c)

        adj_total_err = total_err / (len(slam_cones) - len(unprocessed_slam))
        print(f"Cones: {stamp} ({adj_total_err})")
        cone_err.append(adj_total_err)
        cone_err_stamp.append(stamp)
        unmatched_cones.append(len(unprocessed_slam))
        unmatched_cones_stamp.append(stamp)

slam_poses = []
slam_pose_uncertanties = []
slam_pose_stamps = []

with open(slam_pose_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        stamp = float(row["header.stamp.sec"]) + (float(row["header.stamp.nanosec"]) * 1e-9)

        # i, j, k angles in rad
        ai, aj, ak = quat2euler(
            [
                float(row["pose.pose.orientation.w"]),
                float(row["pose.pose.orientation.x"]),
                float(row["pose.pose.orientation.y"]),
                float(row["pose.pose.orientation.z"]),
            ]
        )

        slam_poses.append(
            Pose(
                x=float(row["pose.pose.position.x"]),
                y=float(row["pose.pose.position.y"]),
                theta=degrees(ak),
            ),
        )

        covariance = eval(row["pose.covariance"])

        slam_pose_uncertanties.append(Pose(x=covariance[0], y=covariance[1], theta=degrees(covariance[-1])))

        slam_pose_stamps.append(stamp)

x_err = []
x_err_stamp = []
y_err = []
y_err_stamp = []
euc_err = []
euc_err_stamp = []
theta_err = []
theta_err_uncertanty = []
theta_err_stamp = []

with open(gt_odom_path) as f:
    reader = csv.DictReader(f)
    for row in reader:
        stamp = float(row["header.stamp.sec"]) + (float(row["header.stamp.nanosec"]) * 1e-9)
        # i, j, k angles in rad
        ai, aj, ak = quat2euler(
            [
                float(row["pose.pose.orientation.w"]),
                float(row["pose.pose.orientation.x"]),
                float(row["pose.pose.orientation.y"]),
                float(row["pose.pose.orientation.z"]),
            ]
        )
        pose = Pose(
            x=float(row["pose.pose.position.x"]),
            y=float(row["pose.pose.position.y"]),
            theta=degrees(ak),
        )
        slam_id = np.argmin([abs(s - stamp) for s in slam_pose_stamps])
        stamp_diff = slam_pose_stamps[slam_id] - stamp

        if abs(stamp_diff) > 0.1:
            print(f"Pose out of sync: {stamp} ({stamp_diff})")
            continue

        print(f"Pose: {stamp} ({stamp_diff})")

        slam_pose = slam_poses[slam_id]
        x_err.append(slam_pose.x - pose.x)
        y_err.append(slam_pose.y - pose.y)
        euc_err.append(pose_euc_dist(pose, slam_pose))
        theta_err.append(smallest_angle_dist(slam_pose.theta, pose.theta))
        theta_err_uncertanty.append(slam_pose_uncertanties[slam_id].theta)
        x_err_stamp.append(stamp)
        y_err_stamp.append(stamp)
        euc_err_stamp.append(stamp)
        theta_err_stamp.append(stamp)

# plot
fig, axs = plt.subplots(7, 1, sharex="col")

axs[0].plot(cone_err_stamp, cone_err, linewidth=2.0)
axs[1].plot(unmatched_cones_stamp, unmatched_cones, linewidth=2.0)

axs[2].plot(x_err_stamp, x_err, linewidth=2.0)
axs[3].plot(y_err_stamp, y_err, linewidth=2.0)
axs[4].plot(euc_err_stamp, euc_err, linewidth=2.0)
axs[5].plot(theta_err_stamp, theta_err, linewidth=2.0)
axs[6].plot(theta_err_stamp, theta_err_uncertanty, linewidth=2.0)

plt.show()
