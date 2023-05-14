import csv
from pathlib import Path

import matplotlib.pyplot as plt

from experiment_helpers import get_run_details_from_name, search_query

track_name = "small_track"
camera_gaussian_range_noise = True
known_association = False

data_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")


for r in [0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5]:
    print(r)
    fig, axs = plt.subplots(2, 1, sharex="col")

    range_query = search_query(
        track_name=track_name,
        camera_gaussian_range_noise=camera_gaussian_range_noise,
        known_association=known_association,
        camera_range_noise=r,
    )

    range_noises = set()
    for p in sorted(data_folder.glob(range_query)):
        (
            _,
            _,
            _,
            _,
            slam_rv,
            _,
        ) = get_run_details_from_name(p.name)
        range_noises.add(slam_rv)

    range_noises = sorted(list(range_noises))
    print(range_noises)

    for range_noise in range_noises:
        lowest_cone_err_cone_data = {}
        lowest_cone_err_pose_data = {}
        lowest_cone_err_name = {}
        lowest_cone_err = float("inf")

        range_search_query = search_query(
            track_name=track_name,
            camera_gaussian_range_noise=camera_gaussian_range_noise,
            known_association=known_association,
            camera_range_noise=r,
            slam_range_var=range_noise,
        )

        for p in sorted(data_folder.glob(range_search_query)):
            with open(p / "cone_error.csv") as f:
                reader = csv.DictReader(f)
                rows = list(reader)
                if len(rows) > 0:
                    err = float(rows[-1]["cone_error"]) * (float(rows[-1]["unmatched_cones"]) + 1)
                    if err < lowest_cone_err:
                        lowest_cone_err = err
                        lowest_cone_err_cone_data = {k: [float(row[k]) for row in rows] for k in rows[0]}
                        lowest_cone_err_name = p.name

                        with open(p / "pose_erorr.csv") as f:
                            reader = csv.DictReader(f)
                            rows = list(reader)
                            if len(rows) > 0:
                                lowest_cone_err_pose_data = {k: [float(row[k]) for row in rows] for k in rows[0]}

        if len(lowest_cone_err_cone_data) < 1:
            print(f"No data for range {range_noise}.")
            continue

        axs[0].plot("stamp", "cone_error", data=lowest_cone_err_cone_data, label=lowest_cone_err_name, linewidth=2.0)
        axs[1].plot(
            "stamp", "unmatched_cones", data=lowest_cone_err_cone_data, label=lowest_cone_err_name, linewidth=2.0
        )

        # axs[2].plot("stamp", "x_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[3].plot("stamp", "y_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[4].plot("stamp", "euc_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[5].plot("stamp", "theta_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[6].plot("stamp", "x_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[7].plot("stamp", "y_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
        # axs[8].plot("stamp", "theta_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)

    for ax in axs:
        ax.legend()
        ax.set_ylim(0, 1)

    plt.show()
