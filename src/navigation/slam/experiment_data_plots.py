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

# gaus = True
# ka = False
# err_plotting_r_cutoff = 0.2

# gaus = True
# ka = True
# err_plotting_r_cutoff = 0.6

# gaus = False
# ka = False
# err_plotting_r_cutoff = 0.6

gaus = False
ka = True
err_plotting_r_cutoff = 1.6

# error_metric = "total_cone_error"
error_metric = "average_cone_error"
success_threshold = 2

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

gt_mapped_fig, gt_mapped_axs = plt.subplots(1, len(tracks), sharey="row")
slam_uncorrelated_fig, slam_uncorrelated_axs = plt.subplots(1, len(tracks), sharey="row")
slam_accuracy_fig, slam_accuracy_axs = plt.subplots(1, len(tracks), sharey="row")
slam_avg_accuracy_fig, slam_avg_accuracy_axs = plt.subplots(1, len(tracks), sharey="row")

success_percentage_table = defaultdict(lambda: defaultdict(dict))

for track_idx, track_name in enumerate(tracks):
    slam_accuracy_axs[track_idx].title.set_text(track_aliases[track_name])
    slam_uncorrelated_axs[track_idx].title.set_text(track_aliases[track_name])
    gt_mapped_axs[track_idx].title.set_text(track_aliases[track_name])
    slam_avg_accuracy_axs[track_idx].title.set_text(track_aliases[track_name])

    slam_accuracy_axs[track_idx].set_xlabel("Time (s)")
    slam_uncorrelated_axs[track_idx].set_xlabel("Time (s)")
    gt_mapped_axs[track_idx].set_xlabel("Time (s)")
    slam_avg_accuracy_axs[track_idx].set_xlabel("Simulation Range Variance, r (m)")

    avg_accuracy_data = {
        "r": [],
        "avg_accuracy": [],
    }

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
        lowest_cone_err_slam_range_datas = []
        lowest_cone_err_cone_data = {}
        lowest_cone_err_label = ""
        lowest_cone_err = float("inf")
        highest_suceeded_run_count = 0

        sim_range_search_query = search_query(
            track_name=track_name,
            camera_gaussian_range_noise=gaus,
            known_association=ka,
            camera_range_noise=sim_range,
            include_csv=True,
        )

        _, slam_ranges = unique_ranges_for_query(data_folder, sim_range_search_query)

        for slam_range in slam_ranges:
            slam_range_search_query = search_query(
                track_name=track_name,
                camera_gaussian_range_noise=gaus,
                known_association=ka,
                camera_range_noise=sim_range,
                slam_range_var=slam_range,
                include_csv=True,
            )

            slam_range_cone_datas = []
            slam_range_lowest_cone_err_data = {}
            slam_range_lowest_cone_err = float("inf")
            slam_range_suceeded_run_count = 0

            for p in sorted(data_folder.glob(slam_range_search_query)):
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
                                np.mean([float(r[error_metric]) for r in rows])
                                * (max(float(rows[-1]["unmatched_slam_cones"]), 0) + 1)
                                * (max(float(rows[-1]["unmatched_gt_cones"]), 0) + 1)
                            )

                            cone_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

                            if err < slam_range_lowest_cone_err:
                                slam_range_lowest_cone_err = err
                                slam_range_lowest_cone_err_data = cone_data

                            slam_range_cone_datas.append(cone_data)
                            success = float(rows[-1]["unmatched_gt_cones"]) < (
                                gt_cone_count_offset[track_name] + success_threshold
                            )
                            slam_range_suceeded_run_count += success

                except FileNotFoundError:
                    continue

            if (
                slam_range_suceeded_run_count >= highest_suceeded_run_count
                and slam_range_lowest_cone_err < lowest_cone_err
            ):
                highest_suceeded_run_count = slam_range_suceeded_run_count
                lowest_cone_err = slam_range_lowest_cone_err
                lowest_cone_err_slam_range = slam_range
                lowest_cone_err_label = f"r={sim_range}, s={slam_range}"
                lowest_cone_err_slam_range_datas = slam_range_cone_datas
                lowest_cone_err_cone_data = slam_range_lowest_cone_err_data

        if len(lowest_cone_err_cone_data) < 1:
            print(f"No data for range {sim_range}")
            continue

        if sim_range <= err_plotting_r_cutoff:
            line = slam_accuracy_axs[track_idx].plot(
                "stamp",
                error_metric,
                data=lowest_cone_err_slam_range_datas[0],
                label=lowest_cone_err_label,
                linewidth=2.0,
            )
            for data in lowest_cone_err_slam_range_datas[1:]:
                slam_accuracy_axs[track_idx].plot(
                    "stamp",
                    error_metric,
                    data=data,
                    linewidth=2.0,
                    label="_Hidden",
                    color=line[0].get_color(),
                )

            avg_accuracy = np.mean([d[error_metric][-1] for d in lowest_cone_err_slam_range_datas])
            avg_accuracy_data["r"].append(sim_range)
            avg_accuracy_data["avg_accuracy"].append(avg_accuracy)

        lowest_cone_err_cone_data["unmatched_gt_cones"] = [
            c - gt_cone_count_offset[track_name] for c in lowest_cone_err_cone_data["unmatched_gt_cones"]
        ]

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

        _, slam_ranges = unique_ranges_for_query(data_folder, sim_range_search_query)

        idx = slam_ranges.index(lowest_cone_err_slam_range)
        # test_list = sorted(
        #     set(
        #         (
        #             slam_ranges[max(0, idx - 1)],
        #             slam_ranges[idx],
        #             slam_ranges[min(len(slam_ranges) - 1, idx + 1)],
        #         )
        #     )
        # )
        test_list = [lowest_cone_err_slam_range]

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
                    individual_loops += float(rows[-1]["unmatched_gt_cones"]) < (
                        gt_cone_count_offset[track_name] + success_threshold
                    )

            total_runs += individual_runs
            total_loops += individual_loops

            if individual_runs > 0:
                # individual_percentage = round((individual_loops / individual_runs) * 100, 2)
                success_percentage_table[sim_range][track_aliases[track_name]][slam_range] = (
                    individual_loops,
                    individual_runs,
                )

        if total_runs > 0:
            # total_percentage = round((total_loops / total_runs) * 100, 2)
            success_percentage_table[sim_range][track_aliases[track_name]]["Total"] = (total_loops, total_runs)

    slam_avg_accuracy_axs[track_idx].scatter("r", "avg_accuracy", data=avg_accuracy_data)
    # slam_avg_accuracy_axs[track_idx].set_ylim(0, 0.8)


def str_ratio(ratio: tuple[float, float]) -> str:
    return f"{round((ratio[0] / ratio[1]) * 100, 2)}% ({ratio[0]}/{ratio[1]})"


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
                    str_ratio(success_percentage_table[r][track][name]),
                ]
            )
            first_r = False
            first_track = False

totals_table = []
for r in success_percentage_table:
    row = [r]
    for track in success_percentage_table[r]:
        row.append(str_ratio(success_percentage_table[r][track]["Total"]))
    totals_table.append(row)

print()
print(tabulate(table, headers=["r", "Track", "s", "Success"], tablefmt="latex"))
print()

print(tabulate(totals_table, headers=["r"] + [f"{track_aliases[t]} Success" for t in tracks], tablefmt="latex"))

slam_accuracy_axs[0].set_ylabel("Euclidean RMSE (m)")
for ax in slam_accuracy_axs:
    ax.legend()

slam_uncorrelated_axs[0].set_ylabel("Uncorrelated Mapped Cones")
for ax in slam_uncorrelated_axs:
    ax.legend()

gt_mapped_axs[0].set_ylabel("Unmapped Ground Truth Cones")
for ax in gt_mapped_axs:
    ax.legend()

slam_avg_accuracy_axs[0].set_ylabel("Average Final Euclidean RMSE")


def save_fig(fig, name: str, size_x: int, size_y: int):
    fig.set_size_inches(size_x, size_y)
    fig.tight_layout()
    fig.savefig(
        f"/home/alistair/Documents/Thesis/thesis-paperwork/3_final_report/figures/{name}_{'gaus' if gaus else 'uni'}_{'ka' if ka else 'uka'}.png",
        dpi=200,
    )


save_fig(gt_mapped_fig, "gt_mapped", 20, 6)
save_fig(slam_uncorrelated_fig, "slam_uncorrelated", 20, 6)
save_fig(slam_accuracy_fig, "slam_accuracy", 20, 6)
save_fig(slam_avg_accuracy_fig, "slam_avg_accuracy", 10, 3)

# plt.show()
