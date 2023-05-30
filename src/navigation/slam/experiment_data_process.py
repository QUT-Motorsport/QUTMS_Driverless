import csv
from dataclasses import dataclass
from itertools import product
from math import degrees, sqrt
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.highlevel.anyreader import AnyReaderError
from rosbags.typesys import get_types_from_msg, register_types
from transforms3d.euler import quat2euler

from builtin_interfaces.msg import Time

from experiment_helpers import get_run_details_from_name, search_query

track_name = "QR_Nov_2022"
# track_name = "B_shape_02_03_2023"
# track_name = "small_track"

SEARCH_QUERY = search_query(
    track_name=track_name,
    # camera_gaussian_range_noise=True,
    # known_association=False,
    # include_csv=False,
)

print(SEARCH_QUERY)

add_types = {}
msgs_folder = Path("/home/alistair/dev/repos/QUTMS_Driverless/src/common/driverless_msgs/msg")
add_types.update(get_types_from_msg((msgs_folder / "Cone.msg").read_text(), "driverless_msgs/msg/Cone"))
add_types.update(
    get_types_from_msg((msgs_folder / "ConeWithCovariance.msg").read_text(), "driverless_msgs/msg/ConeWithCovariance")
)
add_types.update(
    get_types_from_msg(
        (msgs_folder / "ConeDetectionStamped.msg").read_text(), "driverless_msgs/msg/ConeDetectionStamped"
    )
)
register_types(add_types)


bag_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")


def time_to_float(time: Time) -> float:
    return time.sec + (time.nanosec * 1e-9)


@dataclass
class Cone:
    x: float
    y: float
    colour: int
    id_: int


def cone_dist(c1: Cone, c2: Cone) -> float:
    return sqrt((c1.x - c2.x) ** 2 + (c1.y - c2.y) ** 2)


@dataclass
class Pose:
    x: float
    y: float
    theta: float


def pose_euc_dist(p1: Pose, p2: Pose) -> float:
    return sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def smallest_angle_dist(target_a: float, source_a: float):
    a = target_a - source_a
    return (a + 180) % 360 - 180


for p in sorted(bag_folder.glob(SEARCH_QUERY)):
    if "csv_data" in p.name:
        continue

    csv_folder = Path(str(p.absolute()) + "_csv_data")
    csv_folder.mkdir(exist_ok=True)

    (
        _,
        _,
        _,
        sim_rv,
        slam_rv,
        _,
    ) = get_run_details_from_name(p.name)

    if (csv_folder / "cone_error.csv").exists():
        continue

    print(p.name)

    try:
        with AnyReader([Path(p)]) as reader:
            gt_track_conn = next((c for c in reader.connections if c.topic == "/ground_truth/global_map"))
            gt_odom_conn = next((c for c in reader.connections if c.topic == "/ground_truth/odom"))
            slam_track_conn = next((c for c in reader.connections if c.topic == "/slam/track"))
            slam_pose_conn = next((c for c in reader.connections if c.topic == "/slam/pose"))

            print("Cones...")

            gt_cones: list[Cone] = []
            connection, _, rawdata = next(reader.messages(connections=[gt_track_conn]))
            msg = reader.deserialize(rawdata, connection.msgtype)
            for cone in msg.cones_with_cov:
                gt_cones.append(
                    Cone(cone.cone.location.x, cone.cone.location.y, cone.cone.color, cone.cone.sim_cone_index)
                )

            with open(csv_folder / "cone_error.csv", "w") as f:
                cone_err_writer = csv.DictWriter(
                    f,
                    fieldnames=[
                        "stamp",
                        "total_cone_error",
                        "average_cone_error",
                        "matched_cones",
                        "unmatched_slam_cones",
                        "unmatched_gt_cones",
                    ],
                )
                cone_err_writer.writeheader()

                for connection, _, rawdata in reader.messages(connections=[slam_track_conn]):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    stamp = time_to_float(msg.header.stamp)
                    slam_cones: list[Cone] = []
                    for i, cone in enumerate(msg.cones_with_cov):
                        slam_cones.append(Cone(cone.cone.location.x, cone.cone.location.y, cone.cone.color, i))

                    unprocessed_gt = [c.id_ for c in gt_cones]
                    unprocessed_slam = [c.id_ for c in slam_cones]

                    final_pairs = []
                    total_err = 0

                    cone_pairs = sorted(
                        product(gt_cones, slam_cones),
                        key=lambda e: cone_dist(e[0], e[1])
                        + (float("inf") if e[1].colour != 1 and e[0].colour != e[1].colour else 0),
                    )

                    for gt_c, slam_c in cone_pairs:
                        if len(unprocessed_gt) == 0 or len(unprocessed_slam) == 0:
                            break

                        if slam_c.colour != 1 and gt_c.colour != slam_c.colour:
                            continue

                        if cone_dist(gt_c, slam_c) > 1.5:
                            continue

                        if gt_c.id_ in unprocessed_gt and slam_c.id_ in unprocessed_slam:
                            unprocessed_gt.remove(gt_c.id_)
                            unprocessed_slam.remove(slam_c.id_)
                            final_pairs.append((gt_c, slam_c))
                            total_err += cone_dist(gt_c, slam_c)

                    if len(final_pairs) < 1:
                        avg_total_err = 0
                    else:
                        avg_total_err = total_err / len(final_pairs)

                    cone_err_writer.writerow(
                        {
                            "stamp": stamp,
                            "total_cone_error": total_err,
                            "average_cone_error": avg_total_err,
                            "matched_cones": len(final_pairs),
                            "unmatched_slam_cones": len(unprocessed_slam),
                            "unmatched_gt_cones": len(unprocessed_gt),
                        }
                    )

            # print(unprocessed_gt)
            # print([next(c for c in gt_cones if c.id_ == i) for i in unprocessed_gt])

            # print("Poses...")

            # slam_poses: list[Pose] = []
            # slam_pose_uncertanties: list[Pose] = []
            # slam_pose_stamps: list[float] = []

            # for connection, _, rawdata in reader.messages(connections=[slam_pose_conn]):
            #     msg = reader.deserialize(rawdata, connection.msgtype)
            #     stamp = time_to_float(msg.header.stamp)

            #     # i, j, k angles in rad
            #     ai, aj, ak = quat2euler(
            #         [
            #             msg.pose.pose.orientation.w,
            #             msg.pose.pose.orientation.x,
            #             msg.pose.pose.orientation.y,
            #             msg.pose.pose.orientation.z,
            #         ]
            #     )

            #     slam_poses.append(
            #         Pose(
            #             x=msg.pose.pose.position.x,
            #             y=msg.pose.pose.position.y,
            #             theta=degrees(ak),
            #         ),
            #     )

            #     slam_pose_uncertanties.append(
            #         Pose(
            #             x=msg.pose.covariance[0],
            #             y=msg.pose.covariance[7],
            #             theta=degrees(msg.pose.covariance[-1]),
            #         )
            #     )

            #     slam_pose_stamps.append(stamp)

            # with open(csv_folder / "pose_erorr.csv", "w") as f:
            #     pose_err_writer = csv.DictWriter(
            #         f,
            #         fieldnames=[
            #             "stamp",
            #             "x_err",
            #             "y_err",
            #             "euc_err",
            #             "theta_err",
            #             "x_uncertanty",
            #             "y_uncertanty",
            #             "theta_uncertanty",
            #         ],
            #     )
            #     pose_err_writer.writeheader()

            #     for connection, _, rawdata in reader.messages(connections=[gt_odom_conn]):
            #         msg = reader.deserialize(rawdata, connection.msgtype)
            #         stamp = time_to_float(msg.header.stamp)

            #         # i, j, k angles in rad
            #         ai, aj, ak = quat2euler(
            #             [
            #                 msg.pose.pose.orientation.w,
            #                 msg.pose.pose.orientation.x,
            #                 msg.pose.pose.orientation.y,
            #                 msg.pose.pose.orientation.z,
            #             ]
            #         )
            #         pose = Pose(
            #             x=msg.pose.pose.position.x,
            #             y=msg.pose.pose.position.y,
            #             theta=degrees(ak),
            #         )
            #         slam_id = np.argmin([abs(s - stamp) for s in slam_pose_stamps])
            #         stamp_diff = slam_pose_stamps[slam_id] - stamp

            #         if abs(stamp_diff) > 0.1:
            #             print(f"Pose out of sync: {stamp} ({stamp_diff})")
            #             continue

            #         slam_pose = slam_poses[slam_id]
            #         pose_err_writer.writerow(
            #             {
            #                 "stamp": stamp,
            #                 "x_err": slam_pose.x - pose.x,
            #                 "y_err": slam_pose.y - pose.y,
            #                 "euc_err": pose_euc_dist(pose, slam_pose),
            #                 "theta_err": smallest_angle_dist(slam_pose.theta, pose.theta),
            #                 "x_uncertanty": slam_pose_uncertanties[slam_id].x,
            #                 "y_uncertanty": slam_pose_uncertanties[slam_id].y,
            #                 "theta_uncertanty": slam_pose_uncertanties[slam_id].theta,
            #             }
            #        )

    except AnyReaderError:
        continue
