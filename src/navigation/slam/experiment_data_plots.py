import csv
from pathlib import Path

import matplotlib.pyplot as plt

from experiment_helpers import get_run_details_from_name, search_query

# fig, axs = plt.subplots(2, 4, sharex="col", sharey="row")

camera_gaussian_range_noise = True
known_association = False
# track_name = "small_track"
# track_name = "B_shape_02_03_2023"
track_name = "QR_Nov_2022"

data_folder = Path(f"/home/alistair/dev/repos/QUTMS_Driverless/datasets/final_sim/{track_name}/range_testing")

plot_sim_variance = []
final_error = []
unmached_cones = []

sim_var_query = search_query(
    track_name=track_name,
    camera_gaussian_range_noise=camera_gaussian_range_noise,
    known_association=known_association,
    include_csv=True,
)

sim_variance = set()
for p in sorted(data_folder.glob(sim_var_query)):
    (
        _,
        _,
        _,
        sim_rv,
        _,
        _,
    ) = get_run_details_from_name(p.name)
    sim_variance.add(sim_rv)

sim_variance = sorted(list(sim_variance))


for r in sim_variance:
    fig, axs = plt.subplots(2, 1, sharex="col")
    print(r)

    range_query = search_query(
        track_name=track_name,
        camera_gaussian_range_noise=camera_gaussian_range_noise,
        known_association=known_association,
        camera_range_noise=r,
        include_csv=True,
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
            include_csv=True,
        )

        for p in sorted(data_folder.glob(range_search_query)):
            try:
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
            except FileNotFoundError:
                continue

        if len(lowest_cone_err_cone_data) < 1:
            # print(f"No data for range {range_noise}.")
            print(f"No data for range {r}.")
            continue

        final_error.append(lowest_cone_err_cone_data["cone_error"][-1])
        unmached_cones.append(lowest_cone_err_cone_data["unmatched_cones"][-1])
        plot_sim_variance.append(r)

        axs[0].plot("stamp", "cone_error", data=lowest_cone_err_cone_data, label=lowest_cone_err_name, linewidth=2.0)
        axs[1].plot(
            "stamp", "unmatched_cones", data=lowest_cone_err_cone_data, label=lowest_cone_err_name, linewidth=2.0
        )

        for ax in axs:
            ax.legend()

        # axs[0].set_ylim(0, 1.5)
        # axs[1].set_ylim(0, 1)

    plt.show()


# print(plot_sim_variance)
# print(final_error)
# print(unmached_cones)

# hindex = camera_gaussian_range_noise + known_association*2

# axs[0][hindex].scatter(plot_sim_variance, final_error, linewidth=2.0, label=track_name)
# axs[1][hindex].scatter(plot_sim_variance, unmached_cones, linewidth=2.0, label=track_name)

# title = f"{'Gaussian' if camera_gaussian_range_noise else 'Uniform'} Noise, {'Known' if known_association else 'Unknown'} Association"
# axs[0][hindex].title.set_text(title)
# axs[1][hindex].title.set_text(title)

# axs[2].plot("stamp", "x_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[3].plot("stamp", "y_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[4].plot("stamp", "euc_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[5].plot("stamp", "theta_err", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[6].plot("stamp", "x_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[7].plot("stamp", "y_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)
# axs[8].plot("stamp", "theta_uncertanty", data=lowest_cone_err_pose_data, label=lowest_cone_err_name, linewidth=2.0)

plt.show()
