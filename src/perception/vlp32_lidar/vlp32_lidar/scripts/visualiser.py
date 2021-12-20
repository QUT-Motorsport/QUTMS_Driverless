# Modules
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from typing import List

def init_plot_2D(title, xlabel, ylabel):
    plt.figure()

    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

def init_plot_3D(title, xlabel, ylabel, zlabel, azim, elev):
    plt.figure()
    ax = plt.axes(projection='3d')

    ax.view_init(azim=azim, elev=elev)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    ax.set_zlim3d(-1, 1)

    return ax

def get_colour(idx):
    return COLOR_CODES[idx % len(COLOR_CODES)]

def get_cmap(idx):
    return CMAPS[idx % len(CMAPS)]

def get_angles(idx):
    return np.linspace(idx * DELTA_ALPHA, (idx + 1) * DELTA_ALPHA, ANGLE_POINTS)

def plot_lidar_radius(line_colour):
    plt.plot(LIDAR_RANGE * np.cos(ANGLE_RESOLUTION), LIDAR_RANGE * np.sin(ANGLE_RESOLUTION), color=line_colour)

def plot_partial_angle(angles, line_colour):
    plt.plot(LIDAR_RANGE * np.cos(angles), LIDAR_RANGE * np.sin(angles), color=line_colour)

def plot_segment_line(idx, colour, _alpha, _linestyle):
    x = [0, LIDAR_RANGE * math.cos(idx * DELTA_ALPHA)]
    y = [0, LIDAR_RANGE * math.sin(idx * DELTA_ALPHA)]

    plt.plot(x, y, color=colour, alpha=_alpha, linestyle=_linestyle)

def plot_seg_bin_prototype(ax, segments_bins_prototype, plot_bins):
    for i in range(len(segments_bins_prototype)):

        color_1 = get_colour(i)
        angles = get_angles(i)
        for j in range(len(segments_bins_prototype[i])):
            if len(segments_bins_prototype[i][j]) > 0:
                norm = segments_bins_prototype[i][j][0]
                z = segments_bins_prototype[i][j][1]

                new_x = norm * math.cos((i + 0.5) * DELTA_ALPHA)
                new_y = norm * math.sin((i + 0.5) * DELTA_ALPHA)
                
                ax.scatter3D(new_x, new_y, z, c=z, cmap='viridis')

            if plot_bins:
                ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color_1)

def plot_ground_lines(ax, ground_lines, colour):
    for i in range(len(ground_lines)):
        for j in range(len(ground_lines[i])):
            start = ground_lines[i][j][2]
            end = ground_lines[i][j][3]

            r = np.linspace(start[0], end[0], 50)
            z = ground_lines[i][j][0] * r + ground_lines[i][j][1]

            x = r * math.cos((i + 0.5) * DELTA_ALPHA)
            y = r * math.sin((i + 0.5) * DELTA_ALPHA)

            ax.plot3D(x, y, z, color=colour)

def plot_data_2D():
    init_plot_2D("Point Cloud (2D)", "x", "y")

    plt.plot(X_RAW, Y_RAW, '.', color='green')
    plt.plot(0, 0, 'o', color='black')
    plot_lidar_radius('black')

    if SAVE_FIGURES: 
        plt.savefig(FIGURES_DIR + "1_Point-Cloud-2D")

def plot_data_3D():
    ax = init_plot_3D("Point Cloud", "x", "y", "Height", 45, 45)

    ax.scatter3D(X_RAW, Y_RAW, Z_RAW, c=Z_RAW, cmap='viridis')
    ax.plot3D(LIDAR_RANGE * np.cos(ANGLE_RESOLUTION), LIDAR_RANGE * np.sin(ANGLE_RESOLUTION), color='black')

    if SAVE_FIGURES: 
        plt.savefig(FIGURES_DIR + "2_Point-Cloud-3D")

def plot_segments(segments):
    init_plot_2D("Points assigned to Segments", "x", "y")

    for i in range(len(segments)):
        color = get_colour(i)
        x = [coords[0] for coords in segments[i]]
        y = [coords[1] for coords in segments[i]]

        plt.plot(x, y, '.', color=color)

        plot_segment_line(i, color, 1, '-')

        angles = get_angles(i)
        plot_partial_angle(angles, color)

    if SAVE_FIGURES:
        plt.savefig(FIGURES_DIR + "3_Segments")

def plot_segments_bins(segments_bins: List[List[List]], plot_bins: bool):
    init_plot_2D("Points assigned to Bins within Segments", "x", "y")

    for i in range(len(segments_bins)):

        color_1 = get_colour(i)
        angles = get_angles(i)
        for j in range(len(segments_bins[i])):
            color_2 = get_colour(j)

            x = [coords[0] for coords in segments_bins[i][j]]
            y = [coords[1] for coords in segments_bins[i][j]]

            plt.plot(x, y, '.', color=color_2)

            if plot_bins: 
                plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color_1)

        plot_segment_line(i, color_1, 1, '-')
        plot_partial_angle(angles, color_1)

    #if SAVE_FIGURES:
    #    plt.savefig(FIGURES_DIR + "4_Bins-Segments")

def plot_segments_bins_2D(segments_bins_2D, plot_bins):
    init_plot_2D("2D Approximation of Point Cloud", "x", "y")

    for i in range(len(segments_bins_2D)):

        color_1 = get_colour(i)
        angles = get_angles(i)
        for j in range(len(segments_bins_2D[i])):
            color_2 = get_colour(j)

            norm = [coords[0] for coords in segments_bins_2D[i][j]]

            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))

            plt.plot(new_x, new_y, '.', color=color_2)

            if plot_bins: 
                plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color_1)

        plot_segment_line(i, color_1, 1, '-')
        plot_segment_line(i+0.5, 'black', 0.5, '--')
        plot_partial_angle(angles, color_1)

    if SAVE_FIGURES: 
        plt.savefig(FIGURES_DIR + "5_2D-Approx-Point-Cloud-2D")

def plot_segments_bins_2D_3D(segments_bins_2D, plot_bins):
    ax = init_plot_3D("2D Approximation of Point Cloud", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_2D)):

        color_1 = get_colour(i)
        cmap_1 = get_cmap(i)
        angles = get_angles(i)
        for j in range(len(segments_bins_2D[i])):
            norm = [coords[0] for coords in segments_bins_2D[i][j]]
            z = [coords[1] for coords in segments_bins_2D[i][j]]

            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))

            ax.scatter3D(new_x, new_y, z, c=z, cmap=cmap_1)

            if plot_bins:
                 ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color_1)

    if SAVE_FIGURES: 
        plt.savefig(FIGURES_DIR + "6_2D-Approx-Point-Cloud-3D")

def plot_segments_bins_prototype_3D(segments_bins_prototype, plot_bins):
    ax = init_plot_3D("Prototype Points", "x", "y", "Height", 45, 45)

    plot_seg_bin_prototype(ax, segments_bins_prototype, plot_bins)

    if SAVE_FIGURES:
        plt.savefig(FIGURES_DIR + "7_Prototype-Points")

def plot_ground_lines_3D(segments_bins_prototype: List[List[List]], ground_lines: List[List[List]], plot_bins: bool):
    ax = init_plot_3D("Ground Plane Estimation", "x", "y", "Height", 45, 45)

    plot_seg_bin_prototype(ax, segments_bins_prototype, plot_bins)
    plot_ground_lines(ax, ground_lines, 'black')

    if SAVE_FIGURES:
        plt.savefig(FIGURES_DIR + "8_Ground-Plane-Estimation")

def plot_segments_fitted(segments_bins_prototype: List[List[List]], ground_lines: List[List[List]]):
    for i in range(len(segments_bins_prototype)):
        if len(ground_lines[i]) > 0:
            color_1 = get_colour(i)
            init_plot_2D("Segment " + str(i + 1) + " | Degrees: " + str(round((i * DELTA_ALPHA) * 180/math.pi, 2)) + " to " + str(round((i + 1) * DELTA_ALPHA * 180/math.pi, 2)) + " | Points: " + str(ground_lines[i][0][4]), "Distance from origin", "Height")
            plt.ylim(-2, 2)
            
            x = []
            y = []
            for j in range(len(segments_bins_prototype[i])):
                if len(segments_bins_prototype[i][j]) != 0:
                    x.append(segments_bins_prototype[i][j][0])
                    y.append(segments_bins_prototype[i][j][1])
                    
                    for k in range(len(ground_lines[i])):
                        origin = ground_lines[i][k][2][0]
                        end = ground_lines[i][k][3]

                        x_gp = np.linspace(origin, end[0], 50)
                        y_gp = ground_lines[i][k][0] * x_gp + ground_lines[i][k][1]

                        plt.plot(x_gp, y_gp, '--', color=color_1)

            plt.plot(x, y, 'o', color='black')

            if SAVE_FIGURES:
                 plt.savefig(FIGURES_DIR + "9_Segment-" + str(i+1) + "-Fitted_Line")

def plot_labelled_points(labelled_points: List[List[List]], ground_lines: List[List[List]]):
    ax = init_plot_3D("Point Cloud Labelled", "x", "y", "Height", 45, 45)
    
    plot_ground_lines(ax, ground_lines, 'yellow')
    
    # Flatten parent array (remove segements):
    labelled_points = [points for sublist in labelled_points for points in sublist]

    ground_points = []
    object_points = []
    for i in range(len(labelled_points)):
        for j in range(len(labelled_points[i])):
            point = labelled_points[i][j]
            if point[3] == True:
                ground_points.append(point)
            else:
                object_points.append(point)

    x_ground = [coords[0] for coords in ground_points]
    y_ground = [coords[1] for coords in ground_points]
    z_ground = [coords[2] for coords in ground_points]

    ax.scatter3D(x_ground, y_ground, z_ground, color='green')
    
    x_non_ground = [coords[0] for coords in object_points]
    y_non_ground = [coords[1] for coords in object_points]
    z_non_ground = [coords[2] for coords in object_points]

    ax.scatter3D(x_non_ground, y_non_ground, z_non_ground, color='red')

    #if SAVE_FIGURES:
    #     plt.savefig(FIGURES_DIR + "10_Point_Cloud_Labelled")

def plot_grid_2D(object_points: List[List[List]]):
    init_plot_2D("Non-ground Points", "x", "y")

    x = [coords[0] for coords in object_points]
    y = [coords[1] for coords in object_points]

    plt.xlim([-1, LIDAR_RANGE])
    plt.ylim([-LIDAR_RANGE, LIDAR_RANGE])
    plt.plot(0, 0, '>', color="blue")
    plt.plot(x, y, 'x', color='red')

    #if SAVE_FIGURES: 
    #    plt.savefig(FIGURES_DIR + "11_Non-Ground_Points")

def plot_reconstruction(reconstructed_clusters: List[List]):
    init_plot_2D("Reconstructed Clusters", "x", "y")

    colours = ['g', 'grey', 'm', 'orange']
    for i in range(len(reconstructed_clusters)):
        if len(reconstructed_clusters[i]) > 0:
            x_cluster = [coords[0] for coords in reconstructed_clusters[i]]
            y_cluster = [coords[1] for coords in reconstructed_clusters[i]]

            plt.plot(x_cluster, y_cluster, '.', color=colours[i % len(colours)])

            x_mean  = sum(x_cluster) / len(x_cluster)
            y_mean  = sum(y_cluster) / len(y_cluster)

            plt.plot(x_mean, y_mean, 'x', color='blue')

    plt.plot(0, 0, '>', color="blue")
    plt.xlim([-1, LIDAR_RANGE])
    plt.ylim([-LIDAR_RANGE, LIDAR_RANGE])

    #if SAVE_FIGURES:
    #     plt.savefig(FIGURES_DIR + "11_Reconstructed_Clusters")

def plot_clusters(clusters, noise):
    init_plot_2D("Clustered Objects", "x", "y")

    x_noise = [coords[0] for coords in noise]
    y_noise = [coords[1] for coords in noise]

    plt.plot(x_noise, y_noise, '.', color='red')

    colours = ['g', 'grey', 'm', 'orange']
    for i in range(len(clusters)):
        x_cluster = [coords[0] for coords in clusters[i]]
        y_cluster = [coords[1] for coords in clusters[i]]

        plt.plot(x_cluster, y_cluster, '.', color=colours[i % len(colours)])

        x_mean  = sum(x_cluster) / len(x_cluster)
        y_mean  = sum(y_cluster) / len(y_cluster)

        plt.plot(x_mean, y_mean, 'x', color='blue')

    plt.xlim([-1, LIDAR_RANGE])
    plt.ylim([-LIDAR_RANGE, LIDAR_RANGE])

    #if SAVE_FIGURES: 
    #    plt.savefig(FIGURES_DIR + "12_Clustered_Objects")

def plot_cones(cones: List[List]):
    init_plot_2D("All Points with Identified Cones (Red)", "x", "y")

    plt.plot(X_RAW, Y_RAW, '.', color='black')

    x_centers = [coords[0] for coords in cones]
    y_centers = [coords[1] for coords in cones]

    plt.plot(x_centers, y_centers, 'o', color='red')
    plt.xlim([-1, LIDAR_RANGE])
    plt.ylim([-LIDAR_RANGE, LIDAR_RANGE])

    #if SAVE_FIGURES: 
    #    plt.savefig(FIGURES_DIR + "13_Identified_Cones")

def plot_bad_boys(cluster, bad_boys, good_boys, segs_to_count):
    ax = init_plot_3D(str(cluster[0]) + " " + str(cluster[1]) + " stc: " + str(segs_to_count), "x", "y", "Height", 45, 45)

    x_bad = [coords[0] for coords in bad_boys]
    y_boys = [coords[1] for coords in bad_boys]
    z = [coords[2] for coords in bad_boys]

    ax.scatter3D(x_bad, y_boys, z, color='red')

    x_good = [coords[0] for coords in good_boys]
    y_boys = [coords[1] for coords in good_boys]
    z = [coords[2] for coords in good_boys]

    ax.scatter3D(x_good, y_boys, z, color='green')

    plt.xlim([-1, LIDAR_RANGE])
    plt.ylim([-LIDAR_RANGE, LIDAR_RANGE])

    ax.scatter3D(cluster[0], cluster[1], 0, color='blue')

    #if SAVE_FIGURES: 
    #    plt.savefig(FIGURES_DIR + "14_bad_boys")


def init_constants(point_cloud, _delta_alpha, _lidar_range, _bin_size, _save_figures, _figures_dir):
    global X_RAW
    global Y_RAW
    global Z_RAW

    global DELTA_ALPHA
    global LIDAR_RANGE
    global BIN_SIZE
    global ANGLE_RESOLUTION
    global ANGLE_POINTS     
    global COLOR_CODES
    global CMAPS
    global SAVE_FIGURES
    global FIGURES_DIR
    
    for i in range(len(point_cloud) - 1, -1, -1):
        point = point_cloud[i]
        if math.sqrt(point[0]**2 + point[1]**2 + point[2]**2) >= _lidar_range:
            point_cloud.pop(i)
            

    X_RAW = [coords[0] for coords in point_cloud]
    Y_RAW = [coords[1] for coords in point_cloud]
    Z_RAW = [coords[2] for coords in point_cloud]

    DELTA_ALPHA = _delta_alpha
    LIDAR_RANGE = _lidar_range
    BIN_SIZE = _bin_size
    ANGLE_RESOLUTION = np.linspace(0, 2*math.pi, 100)
    ANGLE_POINTS = 25
    COLOR_CODES = ['b', 'g', 'r', 'grey', 'm', 'orange']
    CMAPS = ["Blues", "Greens", "Reds", "Greys", "Purples", "Oranges"]
    SAVE_FIGURES = _save_figures
    FIGURES_DIR = _figures_dir
    
