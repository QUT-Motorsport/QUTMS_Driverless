import csv
from math import ceil
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from experiment_helpers import (
    get_run_details_from_name,
    range_slam_testing_lists,
    search_query,
    unique_ranges_for_query,
)


def window_smooth(data: list, window_size: int) -> list:
    window_half_width = ceil(window_size / 2) - 1

    averaged_err = data.copy()
    for i in range(window_half_width, len(data) - window_half_width):
        averaged_err[i] = np.mean(data[i - window_half_width : i + window_half_width])

    return averaged_err


camera_gaussian_range_noise = True
known_association = True

track_name = "small_track"
# track_name = "B_shape_02_03_2023"
# track_name = "QR_Nov_2022"

# error_metric = "total_cone_error"
error_metric = "average_cone_error"

# -4 for all gt unmapped for the 4 orange cones
# -1 on QR_Nov_2022 for a double up in ground truth
gt_cone_count_offset = {
    "small_track": 4,
    "B_shape_02_03_2023": 4,
    "QR_Nov_2022": 5,
}

data_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")

top_level_query = search_query(
    track_name=track_name,
    camera_gaussian_range_noise=camera_gaussian_range_noise,
    known_association=known_association,
    include_csv=True,
)

cone_axs = [None, None, None]
fig, cone_axs[0] = plt.subplots(1, 1, sharex="col", sharey="row")
fig, cone_axs[1] = plt.subplots(1, 1, sharex="col", sharey="row")
fig, cone_axs[2] = plt.subplots(1, 1, sharex="col", sharey="row")

# fig, pose_axs = plt.subplots(2, 1, sharex="col", sharey="row")

sim_ranges, _ = unique_ranges_for_query(data_folder, top_level_query)

for sim_range in sim_ranges:
    print(sim_range)

    lowest_cone_err_slam_range = None
    lowest_cone_err_cone_data = {}
    lowest_cone_err_pose_data = {}
    lowest_cone_err_name = ""
    lowest_cone_err = float("inf")

    sim_range_search_query = search_query(
        track_name=track_name,
        camera_gaussian_range_noise=camera_gaussian_range_noise,
        known_association=known_association,
        camera_range_noise=sim_range,
        include_csv=True,
    )

    for p in sorted(data_folder.glob(sim_range_search_query)):
        try:
            with open(p / "cone_error.csv") as f:
                reader = csv.DictReader(f)
                rows = list(reader)

                if len(rows) > 0:

                    if float(rows[-1][error_metric]) == 0:
                        print(p.name, "failed error metric", rows[-1][error_metric])
                        continue

                    if float(rows[-1]["stamp"]) > 300 or float(rows[-1]["stamp"]) < 100:
                        print(p.name, "failed stamp check", rows[-1]["stamp"])
                        continue

                    err = (
                        float(rows[-1][error_metric])
                        * (max(float(rows[-1]["unmatched_slam_cones"]), 0) + 1)
                        * (max(float(rows[-1]["unmatched_gt_cones"]), 0) + 1)
                    )

                    if err < lowest_cone_err:
                        lowest_cone_err = err
                        lowest_cone_err_name = p.name

                        (
                            _,
                            _,
                            _,
                            _,
                            slam_rv,
                            _,
                        ) = get_run_details_from_name(p.name)

                        lowest_cone_err_slam_range = slam_rv

                        lowest_cone_err_cone_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

                        with open(p / "pose_erorr.csv") as f:
                            reader = csv.DictReader(f)
                            rows = list(reader)
                            lowest_cone_err_pose_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

        except FileNotFoundError:
            continue

    if len(lowest_cone_err_cone_data) < 1:
        print(f"No data for range {sim_range}.")
        continue

    lowest_cone_err_cone_data["unmatched_gt_cones"] = [
        c - gt_cone_count_offset[track_name] for c in lowest_cone_err_cone_data["unmatched_gt_cones"]
    ]

    cone_axs[0].plot(
        "stamp",
        error_metric,
        data=lowest_cone_err_cone_data,
        label=lowest_cone_err_name,
        linewidth=2.0,
    )
    cone_axs[1].plot(
        "stamp",
        "unmatched_slam_cones",
        data=lowest_cone_err_cone_data,
        label=lowest_cone_err_name + " slam",
        linewidth=2.0,
    )
    cone_axs[2].plot(
        "stamp",
        "unmatched_gt_cones",
        data=lowest_cone_err_cone_data,
        label=lowest_cone_err_name + " gt",
        linewidth=2.0,
    )

    print("Best SLAM range: ", lowest_cone_err_slam_range)

    _, slam_ranges = unique_ranges_for_query(data_folder, sim_range_search_query)

    idx = slam_ranges.index(lowest_cone_err_slam_range)
    test_list = sorted(
        set(
            (
                slam_ranges[max(0, idx - 1)],
                slam_ranges[idx],
                slam_ranges[min(len(slam_ranges) - 1, idx + 1)],
            )
        )
    )

    print("Getting stats for: ", test_list)

    total_runs = 0
    total_loops = 0
    for slam_range in test_list:
        slam_range_query = search_query(
            track_name=track_name,
            camera_gaussian_range_noise=camera_gaussian_range_noise,
            known_association=known_association,
            camera_range_noise=sim_range,
            slam_range_var=slam_range,
            include_csv=True,
        )

        individual_runs = 0
        individual_loops = 0
        for p in sorted(data_folder.glob(slam_range_query)):
            with open(p / "cone_error.csv") as f:
                reader = csv.DictReader(f)
                rows = list(reader)

            if len(rows) > 0:
                if float(rows[-1]["stamp"]) > 300 or float(rows[-1]["stamp"]) < 100:
                    print(p.name, "failed stamp check", rows[-1]["stamp"])
                    continue

                individual_runs += 1
                individual_loops += float(rows[-1]["unmatched_gt_cones"]) < (gt_cone_count_offset[track_name] + 2)

        total_runs += individual_runs
        total_loops += individual_loops
        if individual_runs > 0:
            print(
                f"SLAM Range {slam_range}: runs {individual_runs} loops {individual_loops} ({round((individual_loops / individual_runs)*100, 2)}%)"
            )

    if total_runs > 0:
        print(f"Total: runs {total_runs} loops {total_loops} ({round((total_loops / total_runs)*100, 2)}%)")

    # lowest_cone_err_pose_data["euc_err"] = window_smooth(lowest_cone_err_pose_data["euc_err"], 101)
    # lowest_cone_err_pose_data["theta_err"] = window_smooth(lowest_cone_err_pose_data["theta_err"], 101)

    # pose_axs[0].plot(
    #     "stamp",
    #     "euc_err",
    #     data=lowest_cone_err_pose_data,
    #     label=lowest_cone_err_name,
    #     linewidth=2.0,
    # )

    # pose_axs[1].plot(
    #     "stamp",
    #     "theta_err",
    #     data=lowest_cone_err_pose_data,
    #     label=lowest_cone_err_name,
    #     linewidth=2.0,
    # )

    # pose_axs[2].plot(
    #     "stamp",
    #     "x_uncertanty",
    #     data=lowest_cone_err_pose_data,
    #     label=lowest_cone_err_name + " x",
    #     linewidth=2.0,
    # )

    # pose_axs[2].plot(
    #     "stamp",
    #     "y_uncertanty",
    #     data=lowest_cone_err_pose_data,
    #     label=lowest_cone_err_name + " y",
    #     linewidth=2.0,
    # )

    # pose_axs[3].plot(
    #     "stamp",
    #     "theta_uncertanty",
    #     data=lowest_cone_err_pose_data,
    #     label=lowest_cone_err_name,
    #     linewidth=2.0,
    # )

    # cone_axs[0].title.set_text(f"range variance: {r}")
    # cone_axs[1].title.set_text(f"range variance: {r}")

    # cone_axs[0].set_ylim(0, 1.5)
    # cone_axs[1].set_ylim(0, 1)

# for ax in pose_axs:
#     ax.legend()

# pose_axs[0].title.set_text("Euclidian Pose Error")
# pose_axs[1].title.set_text("Heading Pose Error")
# pose_axs[2].title.set_text("x, y uncertanty Error")
# pose_axs[3].title.set_text("Heading uncertanty Error")

for ax in cone_axs:
    ax.legend()

plt.show()


# print(plot_sim_variance)
# print(final_error)
# print(unmached_cones)

# hindex = camera_gaussian_range_noise + known_association*2

# cone_axs[0][hindex].scatter(plot_sim_variance, final_error, linewidth=2.0, label=track_name)
# cone_axs[1][hindex].scatter(plot_sim_variance, unmached_cones, linewidth=2.0, label=track_name)

# title = f"{'Gaussian' if camera_gaussian_range_noise else 'Uniform'} Noise, {'Known' if known_association else 'Unknown'} Association"
# cone_axs[0][hindex].title.set_text(title)
# cone_axs[1][hindex].title.set_text(title)

# cone_axs[2].plot("stamp", "x_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[3].plot("stamp", "y_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[4].plot("stamp", "euc_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[5].plot("stamp", "theta_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[6].plot("stamp", "x_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[7].plot("stamp", "y_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# cone_axs[8].plot("stamp", "theta_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
