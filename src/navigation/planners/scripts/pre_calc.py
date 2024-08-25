from fsd_path_planning import ConeTypes, MissionTypes, PathPlanner
import numpy as np


def get_planner_cfg():
    cone_sorting_kwargs = {
        "max_n_neighbors": 5,
        "max_dist": 6.5,
        "max_dist_to_first": 6.0,
        "max_length": 12,
        "threshold_directional_angle": np.deg2rad(30),
        "threshold_absolute_angle": np.deg2rad(55),
        "use_unknown_cones": True,
    }

    cone_fitting_kwargs = {
        "smoothing": 0.2,
        "predict_every": 0.1,
        "max_deg": 3,
    }

    path_calculation_kwargs = {
        "maximal_distance_for_valid_path": 5,
        "mpc_path_length": 20,
        "mpc_prediction_horizon": 40,
    }

    cone_matching_kwargs = {
        "min_track_width": 4.0,
        "max_search_range": 5,
        "max_search_angle": np.deg2rad(50),
        "matches_should_be_monotonic": True,
    }

    return {
        "mission": MissionTypes.trackdrive,
        "cone_sorting_kwargs": cone_sorting_kwargs,
        "cone_fitting_kwargs": cone_fitting_kwargs,
        "path_calculation_kwargs": path_calculation_kwargs,
        "cone_matching_kwargs": cone_matching_kwargs,
    }


pre_track = [
    np.array([]),
    np.array(
        [
            [2.0613708496093754, 12.062088012695312],
            [2.248870849609375, 15.124588012695314],
            [1.6238708496093752, 3.812088012695313],
            [2.811370849609375, 18.499588012695312],
            [1.9988708496093752, 9.624588012695312],
        ]
    ),
    np.array(
        [
            [-0.688629150390625, 12.499588012695314],
            [-0.626129150390625, 14.999588012695314],
            [-0.313629150390625, 19.124588012695316],
            [-1.5011291503906252, 4.187088012695313],
            [-0.9386291503906252, 9.374588012695312],
        ]
    ),
    np.array([]),
    np.array(
        [
            [1.873870849609375, 6.187088012695313],
            [-1.188629150390625, 6.3745880126953125],
            [1.7488708496093752, 5.6245880126953125],
            [-1.251129150390625, 5.9370880126953125],
        ]
    ),
]

path_planner = PathPlanner(**get_planner_cfg())

init_plan_calcs = path_planner.calculate_path_in_global_frame(
    pre_track, np.array([0.0, 0.0]), 0.0, return_intermediate_results=True
)
print("Initialised planner calcs")
