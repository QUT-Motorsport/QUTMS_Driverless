def window_smooth(data: list, window_size: int) -> list:
    window_half_width = ceil(window_size / 2) - 1

    averaged_err = data.copy()
    for i in range(window_half_width, len(data) - window_half_width):
        averaged_err[i] = np.mean(data[i - window_half_width : i + window_half_width])

    return averaged_err


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
