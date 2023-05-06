from collections import OrderedDict
import csv
from dataclasses import dataclass
from itertools import product
from math import sqrt
from pathlib import Path
from pprint import pprint

import matplotlib.pyplot as plt

SLAM_TRACK = "_slam_track_"
SLAM_POSE = "_slam_pose_"
GT_TRACK = "_ground_truth_global_map_"
GT_ODOM = "_ground_truth_odom_"
CSV_PATH = "/home/alistair/dev/repos/QUTMS_Driverless/csv_data/"

TIMESTAMP = "2023-05-06T09:58:56"

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
            key=lambda e: cone_dist(e[0], e[1]) + (float("inf") if e[1].colour != 1 and e[0] != e[1] else 0),
        ):
            if len(unprocessed_gt) == 0 or len(unprocessed_slam) == 0:
                break

            if gt_c.id_ in unprocessed_gt and slam_c.id_ in unprocessed_slam:
                unprocessed_gt.remove(gt_c.id_)
                unprocessed_slam.remove(slam_c.id_)
                final_pairs.append((gt_c, slam_c))
                total_err += cone_dist(gt_c, slam_c)

        adj_total_err = total_err / (len(slam_cones) - len(unprocessed_slam))
        print(f"{stamp} ({adj_total_err})")
        cone_err.append(adj_total_err)
        cone_err_stamp.append(stamp)
        unmatched_cones.append(len(unprocessed_slam))
        unmatched_cones_stamp.append(stamp)


# plot
fig, ax = plt.subplots()

ax.plot(cone_err_stamp, cone_err, linewidth=2.0)
ax.plot(unmatched_cones_stamp, unmatched_cones, linewidth=2.0)

plt.show()
