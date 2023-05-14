from typing import Optional, Union


def get_run_name(
    track_name: str,
    camera_gaussian_range_noise: bool,
    known_association: bool,
    camera_range_noise: float,
    slam_range_var: float,
    run_num: float,
) -> str:
    return f"{track_name}_{'gaus' if camera_gaussian_range_noise else 'uni'}_{'ka' if known_association else 'uka'}_sim_rv_{camera_range_noise}_slam_rv_{slam_range_var}_{run_num}"


def search_query(
    track_name: str = "*",
    camera_gaussian_range_noise: Optional[bool] = None,
    known_association: Optional[bool] = None,
    camera_range_noise: Union[float, str] = "*",
    slam_range_var: Union[float, str] = "*",
    run_num: Union[float, str] = "*",
    include_csv: bool = False,
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
        + f"{run_num}"
        + (f"_csv_data" if include_csv else "")
    )


def get_run_details_from_name(name: str) -> tuple:
    name_parts = list(reversed(name.split("_")))

    if name.endswith("csv_data"):
        name_parts = name_parts[2:]

    track_name = "_".join(reversed(name_parts[9:]))
    gaussian = name_parts[8] == "gaus"
    known_association = name_parts[7] == "ka"
    sim_rv = name_parts[4]
    slam_rv = name_parts[1]
    run_num = name_parts[0]

    return (
        track_name,
        gaussian,
        known_association,
        sim_rv,
        slam_rv,
        run_num,
    )
