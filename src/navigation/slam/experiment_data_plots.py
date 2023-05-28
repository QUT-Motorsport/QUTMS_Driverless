from collections import defaultdict
import csv
from pathlib import Path
from pprint import pprint

import matplotlib.pyplot as plt
import numpy as np
from tabulate import tabulate

from experiment_helpers import (
    get_run_details_from_name,
    range_slam_testing_lists,
    search_query,
    unique_ranges_for_query,
)

gaus = True
ka = False
# error_metric = "total_cone_error"
error_metric = "average_cone_error"

# -4 for all gt unmapped for the 4 orange cones
# -1 on QR_Nov_2022 for a double up in ground truth
gt_cone_count_offset = {
    "small_track": 4,
    "B_shape_02_03_2023": 4,
    "QR_Nov_2022": 5,
}

tracks = ["small_track", "B_shape_02_03_2023", "QR_Nov_2022"]
track_aliases = {
    "small_track": "Track A",
    "B_shape_02_03_2023": "Track B",
    "QR_Nov_2022": "Track C",
}

gt_mapped_fig, gt_mapped_axs = plt.subplots(1, len(tracks), sharex="row", sharey="row")
slam_uncorrelated_fig, slam_uncorrelated_axs = plt.subplots(1, len(tracks), sharex="row", sharey="row")
slam_accuracy_fig, slam_accuracy_axs = plt.subplots(1, len(tracks), sharex="row", sharey="row")

success_percentage_table = defaultdict(lambda: defaultdict(dict))

for track_idx, track_name in enumerate(tracks):
    data_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")
    top_level_query = search_query(
        track_name=track_name,
        camera_gaussian_range_noise=gaus,
        known_association=ka,
        include_csv=True,
    )

    sim_ranges, _ = unique_ranges_for_query(data_folder, top_level_query)

    for sim_range in sim_ranges:
        print(sim_range)

        lowest_cone_err_slam_range = None
        lowest_cone_err_cone_data = {}
        lowest_cone_err_pose_data = {}
        lowest_cone_err_label = ""
        lowest_cone_err = float("inf")

        sim_range_search_query = search_query(
            track_name=track_name,
            camera_gaussian_range_noise=gaus,
            known_association=ka,
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
                            (
                                _,
                                _,
                                _,
                                sim_rv,
                                slam_rv,
                                _,
                            ) = get_run_details_from_name(p.name)

                            lowest_cone_err_slam_range = slam_rv
                            lowest_cone_err_label = f"r={sim_rv}, s={slam_rv}"

                            lowest_cone_err_cone_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

                            # with open(p / "pose_erorr.csv") as f:
                            #     reader = csv.DictReader(f)
                            #     rows = list(reader)
                            #     lowest_cone_err_pose_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

            except FileNotFoundError:
                continue

        if len(lowest_cone_err_cone_data) < 1:
            print(f"No data for range {sim_range}.")
            continue

        lowest_cone_err_cone_data["unmatched_gt_cones"] = [
            c - gt_cone_count_offset[track_name] for c in lowest_cone_err_cone_data["unmatched_gt_cones"]
        ]

        slam_accuracy_axs[track_idx].plot(
            "stamp",
            error_metric,
            data=lowest_cone_err_cone_data,
            label=lowest_cone_err_label,
            linewidth=2.0,
        )
        slam_uncorrelated_axs[track_idx].plot(
            "stamp",
            "unmatched_slam_cones",
            data=lowest_cone_err_cone_data,
            label=lowest_cone_err_label,
            linewidth=2.0,
        )
        gt_mapped_axs[track_idx].plot(
            "stamp",
            "unmatched_gt_cones",
            data=lowest_cone_err_cone_data,
            label=lowest_cone_err_label,
            linewidth=2.0,
        )

        slam_accuracy_axs[track_idx].title.set_text(track_aliases[track_name])
        slam_uncorrelated_axs[track_idx].title.set_text(track_aliases[track_name])
        gt_mapped_axs[track_idx].title.set_text(track_aliases[track_name])

        slam_accuracy_axs[track_idx].set_xlabel("Time (s)")
        slam_uncorrelated_axs[track_idx].set_xlabel("Time (s)")
        gt_mapped_axs[track_idx].set_xlabel("Time (s)")

        # print("Best SLAM range: ", lowest_cone_err_slam_range)

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

        # print("Getting stats for: ", test_list)

        total_runs = 0
        total_loops = 0
        for slam_range in test_list:
            slam_range_query = search_query(
                track_name=track_name,
                camera_gaussian_range_noise=gaus,
                known_association=ka,
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
                individual_percentage = round((individual_loops / individual_runs) * 100, 2)
                success_percentage_table[sim_range][track_aliases[track_name]][slam_range] = individual_percentage
                # print(
                #     f"SLAM Range {slam_range}: runs {individual_runs} loops {individual_loops} ({individual_percentage}%)"
                # )

        if total_runs > 0:
            total_percentage = round((total_loops / total_runs) * 100, 2)
            success_percentage_table[sim_range][track_aliases[track_name]]["Total"] = total_percentage
            # print(f"Total: runs {total_runs} loops {total_loops} ({total_percentage}%)")


table = []
for r in success_percentage_table:
    first_r = True
    for j, track in enumerate(success_percentage_table[r]):
        first_track = True
        for i, name in enumerate(success_percentage_table[r][track]):
            table.append(
                [
                    r if first_r else None,
                    track if first_track else None,
                    name,
                    success_percentage_table[r][track][name],
                ]
            )
            first_r = False
            first_track = False

totals_table = []
for r in success_percentage_table:
    row = [r]
    for track in success_percentage_table[r]:
        row.append(success_percentage_table[r][track]["Total"])
    totals_table.append(row)


print()
print(tabulate(table, headers=["r", "Track", "s", "Success (%)"], tablefmt="latex"))
print()

print(tabulate(totals_table, headers=["r"] + [f"{track_aliases[t]} Success (%)" for t in tracks], tablefmt="latex"))

slam_accuracy_axs[0].set_ylabel("Average Euclidian Accuracy (m)")
for ax in slam_accuracy_axs:
    ax.legend()

slam_uncorrelated_axs[0].set_ylabel("Uncorrelated Mapped Cones")
for ax in slam_uncorrelated_axs:
    ax.legend()

gt_mapped_axs[0].set_ylabel("Unmapped Ground Truth Cones")
for ax in gt_mapped_axs:
    ax.legend()


def save_fig(fig, name: str):
    fig.set_size_inches(16, 9)
    fig.tight_layout()
    fig.savefig(
        f"/home/alistair/Documents/Thesis/thesis-paperwork/3_final_report/figures/{name}_{'gaus' if gaus else 'uni'}_{'ka' if ka else 'uka'}.png",
        dpi=200,
    )


save_fig(gt_mapped_fig, "gt_mapped")
save_fig(slam_uncorrelated_fig, "slam_uncorrelated")
save_fig(slam_accuracy_fig, "slam_accuracy")

plt.show()
