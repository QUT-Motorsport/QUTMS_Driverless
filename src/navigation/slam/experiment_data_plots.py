from collections import defaultdict
import csv
from pathlib import Path

import matplotlib.pyplot as plt

from typing import Optional, Union


def search_query(
    track_name: str = "*",
    camera_gaussian_range_noise: Optional[bool] = None,
    known_association: Optional[bool] = None,
    camera_range_noise: Union[float, str] = "*",
    slam_range_var: Union[float, str] = "*",
    trial_num: Union[float, str] = "*",
) -> str:
    if camera_gaussian_range_noise is not None:
        gaus = "gaus" if camera_gaussian_range_noise else "uni"
    else:
        gaus = "*"

    if known_association is not None:
        ka = "ka" if known_association else "uka"
    else:
        ka = "*"

    return (
        f"{track_name}_"
        + f"{gaus}_"
        + f"{ka}_"
        + f"sim_rv_{camera_range_noise}_"
        + f"slam_rv_{slam_range_var}_"
        + f"{trial_num}_"
        + f"csv_data"
    )


track_name = "small_track"
camera_gaussian_range_noise = True
known_association = False
camera_range_noise = 0.1
slam_range_var = 0.2
# SEARCH_QUERY = f"{track_name}_{'gaus' if camera_gaussian_range_noise else 'uni'}_{'ka' if known_association else 'uka'}_*"
SEARCH_QUERY = f"{track_name}_{'gaus' if camera_gaussian_range_noise else 'uni'}_{'ka' if known_association else 'uka'}_sim_rv_{camera_range_noise}_slam_rv_{slam_range_var}_*_csv_data"
# SEARCH_QUERY = SEARCH_QUERY = f"{track_name}_{'gaus' if camera_gaussian_range_noise else 'uni'}_{'ka' if known_association else 'uka'}_sim_rv_{camera_range_noise}_slam_rv_*_csv_data"


def get_name_parts(name: str) -> tuple:
    name_parts = list(reversed(name.split("_")))

    track_name = "_".join(name_parts[11:])
    gaussian = name_parts[10] == "gaus"
    known_association = name_parts[9] == "ka"
    sim_rv = name_parts[6]
    slam_rv = name_parts[3]
    run_num = name_parts[2]

    return (
        track_name,
        gaussian,
        known_association,
        sim_rv,
        slam_rv,
        run_num,
    )


data_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")


for r in [0.01, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5]:
    print(r)
    fig, axs = plt.subplots(2, 1, sharex="col")

    range_query = search_query(
        track_name="small_track",
        camera_gaussian_range_noise=True,
        known_association=True,
        camera_range_noise=r,
    )

    range_noises = set()
    for p in sorted(data_folder.glob(range_query)):
        (
            track_name,
            gaussian,
            known_association,
            sim_rv,
            slam_rv,
            run_num,
        ) = get_name_parts(p.name)
        range_noises.add(slam_rv)

    range_noises = sorted(list(range_noises))
    print(range_noises)

    for range_noise in range_noises:
        lowest_cone_err_cone_data = {}
        lowest_cone_err_pose_data = {}
        lowest_cone_err_name = {}
        lowest_cone_err = float("inf")

        range_search_query = search_query(
            track_name="small_track",
            camera_gaussian_range_noise=True,
            known_association=True,
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
