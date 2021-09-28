def init_plot_2D(title, xlabel, ylabel):
    fig = plt.figure()
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)

def init_plot_3D(title, xlabel, ylabel, zlabel, azim, elev):
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.view_init(azim=azim, elev=elev)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    ax.set_zlim3d(-1, 1)
    return ax

def plot_data_2D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    
    init_plot_2D("Point Cloud (2D)", "x", "y")
    plt.plot(x, y, '.', color='green')
    plt.plot(0, 0, 'o', color='black')
    angles = np.linspace(0, 2*math.pi, 100)
    plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color='black')
    plt.savefig(FIGURES_DIR + "1_Point-Cloud-2D")

def plot_data_3D(points):
    x = [coords[0] for coords in points]
    y = [coords[1] for coords in points]
    z = [coords[2] for coords in points]

    ax = init_plot_3D("Point Cloud", "x", "y", "Height", 45, 45)
    ax.scatter3D(x, y, z, c=z, cmap='Greens');
    
    angles = np.linspace(0, 2*math.pi, 100)
    ax.plot3D(LIDAR_RANGE * np.cos(angles), LIDAR_RANGE * np.sin(angles), color='black')

    plt.savefig(FIGURES_DIR + "2_Point-Cloud-3D")

plot_data_2D(test_data)
plot_data_3D(test_data)

def plot_segments(segments, color_codes, angle_points):
    init_plot_2D("Points assigned to Segments", "x", "y")
    for i in range(len(segments)):
        color = color_codes[i % len(color_codes)]
        x = [coords[0] for coords in segments[i]]
        y = [coords[1] for coords in segments[i]]
        plt.plot(x, y, '.', color=color)
        xpoints = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints, ypoints, color=color)
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color)
    plt.savefig(FIGURES_DIR + "3_Segments")

def plot_segments_bins(segments_bins, color_codes, angle_points):
    init_plot_2D("Points assigned to Bins within Segments", "x", "y")

    for i in range(len(segments_bins)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins[i])):
            color2 = color_codes[j % len(color_codes)]
            x = [coords[0] for coords in segments_bins[i][j]]
            y = [coords[1] for coords in segments_bins[i][j]]
            plt.plot(x, y, '.', color=color2)
            plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
        xpoints = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints, ypoints, color=color1)
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "4_Bins-Segments")

def plot_segments_bins_2D(segments_bins_2D, color_codes, angle_points):
    init_plot_2D("2D Approximation of Point Cloud", "x", "y")

    for i in range(len(segments_bins_2D)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_2D[i])):
            color2 = color_codes[j % len(color_codes)]
            norm = [coords[0] for coords in segments_bins_2D[i][j]]
            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))
            plt.plot(new_x, new_y, '.', color=color2)
            plt.plot((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
        xpoints1 = [0, LIDAR_RANGE * math.cos(i * DELTA_ALPHA)]
        ypoints1 = [0, LIDAR_RANGE * math.sin(i * DELTA_ALPHA)]
        plt.plot(xpoints1, ypoints1, color=color1)
        xpoints2 = [0, LIDAR_RANGE * math.cos((i + 0.5) * DELTA_ALPHA)]
        ypoints2 = [0, LIDAR_RANGE * math.sin((i + 0.5) * DELTA_ALPHA)]
        plt.plot(xpoints2, ypoints2, color='black', alpha=0.5, linestyle='--')
        plt.plot(LIDAR_RANGE*np.cos(angles), LIDAR_RANGE*np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "5_2D-Approx-Point-Cloud-2D")

def plot_segments_bins_2D_3D(segments_bins_2D, color_codes, angle_points, cmaps):
    ax = init_plot_3D("2D Approximation of Point Cloud", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_2D)):
        color1 = color_codes[i % len(color_codes)]
        cmap1 = cmaps[i % len(cmaps)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_2D[i])):
            norm = [coords[0] for coords in segments_bins_2D[i][j]]
            z = [coords[1] for coords in segments_bins_2D[i][j]]
            new_x = []
            new_y = []
            for k in range(len(norm)):
                new_x.append(norm[k] * math.cos((i + 0.5) * DELTA_ALPHA))
                new_y.append(norm[k] * math.sin((i + 0.5) * DELTA_ALPHA))
            ax.scatter3D(new_x, new_y, z, c=z, cmap=cmap1);
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "6_2D-Approx-Point-Cloud-3D")

def plot_segments_bins_prototype_3D(segments_bins_prototype, color_codes, angle_points, cmaps):
    ax = init_plot_3D("Prototype Points", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_prototype)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_prototype[i])):
            print(segments_bins_prototype[i])
            if len(segments_bins_prototype[i][j]) > 0:
                norm = segments_bins_prototype[i][j][0]
                z = segments_bins_prototype[i][j][1]
                new_x = norm * math.cos((i + 0.5) * DELTA_ALPHA)
                new_y = norm * math.sin((i + 0.5) * DELTA_ALPHA)
                ax.scatter3D(new_x, new_y, z, c=z, cmap='viridis');
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)
    plt.savefig(FIGURES_DIR + "7_Prototype-Points")

def plot_ground_lines_3D(segments_bins_prototype, color_codes, angle_points, ground_lines):
    ax = init_plot_3D("Ground Plane Estimation", "x", "y", "Height", 45, 45)

    for i in range(len(segments_bins_prototype)):
        color1 = color_codes[i % len(color_codes)]
        angles = np.linspace(i * DELTA_ALPHA, (i + 1) * DELTA_ALPHA, angle_points)
        for j in range(len(segments_bins_prototype[i])):
            print(segments_bins_prototype[i])
            if len(segments_bins_prototype[i][j]) > 0:
                norm = segments_bins_prototype[i][j][0]
                z = segments_bins_prototype[i][j][1]
                new_x = norm * math.cos((i + 0.5) * DELTA_ALPHA)
                new_y = norm * math.sin((i + 0.5) * DELTA_ALPHA)
                ax.scatter3D(new_x, new_y, z, c=z, cmap='viridis');
            ax.plot3D((j * BIN_SIZE) * np.cos(angles), (j * BIN_SIZE) * np.sin(angles), color=color1)

    print("hey", ground_lines)

    for i in range(len(ground_lines)):
        for j in range(len(ground_lines[i])):
            start = ground_lines[i][j][2]
            end = ground_lines[i][j][3]
            r = np.linspace(start[0], end[0], 50)
            z = ground_lines[i][j][0] * r + ground_lines[i][j][1]
            x = r * math.cos((i + 0.5) * DELTA_ALPHA)
            y = r * math.sin((i + 0.5) * DELTA_ALPHA)
            ax.plot3D(x, y, z, color='black')

    plt.savefig(FIGURES_DIR + "8_Ground-Plane-Estimation")

def plot_segments_fitted(segments_bins_prototype, ground_lines, color_codes):
    print("SP", segments_bins_prototype)
    print("--- Prototype Points ---")
    for i in range(len(segments_bins_prototype)):
        # This if statement is hacky. Without it, this visualisation function crashes
        if len(ground_lines[i]) > 0:
            color1 = color_codes[i % len(color_codes)]
            print("Segment:", i + 1)
            print(str(ground_lines[i][0][4]))
            init_plot_2D("Segment " + str(i + 1) + " | Degrees: " + str(round((i * DELTA_ALPHA) * 180/math.pi, 2)) + " to " + str(round((i + 1) * DELTA_ALPHA * 180/math.pi, 2)) + " | Points: " + str(ground_lines[i][0][4]), "Distance from origin", "Height")
            plt.ylim(-2, 2)
            x = []
            y = []
            for j in range(len(segments_bins_prototype[i])):
                if len(segments_bins_prototype[i][j]) != 0:
                    x.append(segments_bins_prototype[i][j][0])
                    y.append(segments_bins_prototype[i][j][1])
                    print("     Bin:", j, "Point:", segments_bins_prototype[i][j][0], segments_bins_prototype[i][j][1])
                    for k in range(len(ground_lines[i])):
                        origin = ground_lines[i][k][2][0]
                        end = ground_lines[i][k][3]
                        x_gp = np.linspace(origin, end[0], 50)
                        y_gp = ground_lines[i][k][0] * x_gp + ground_lines[i][k][1]
                        plt.plot(x_gp, y_gp, '--', color=color1)
            plt.plot(x, y, 'o', color='black')
            plt.savefig(FIGURES_DIR + "9_Segment-" + str(i+1) + "-Fitted_Line")

def plot_labelled_points(labelled_points, color_codes, angle_points, ground_lines):
    ax = init_plot_3D("Point Cloud Labelled", "x", "y", "Height", 45, 45)

    for i in range(len(ground_lines)):
        for j in range(len(ground_lines[i])):
            start = ground_lines[i][j][2]
            end = ground_lines[i][j][3]
            r = np.linspace(start[0], end[0], 50)
            z = ground_lines[i][j][0] * r + ground_lines[i][j][1]
            x = r * math.cos((i + 0.5) * DELTA_ALPHA)
            y = r * math.sin((i + 0.5) * DELTA_ALPHA)
            ax.plot3D(x, y, z, color='yellow')

    # Flatten parent array (remove segements)
    labelled_points = [points for sublist in labelled_points for points in sublist]

    ground_points = []
    object_points = []
    for i in range(len(labelled_points)):
        point = labelled_points[i]
        if point[3] == True:
            ground_points.append(point)
        else:
            object_points.append(point)
    
    x_ground = [coords[0] for coords in ground_points]
    y_ground = [coords[1] for coords in ground_points]
    z_ground = [coords[2] for coords in ground_points]
    ax.scatter3D(x_ground, y_ground, z_ground, color='green');

    x_non_ground = [coords[0] for coords in object_points]
    y_non_ground = [coords[1] for coords in object_points]
    z_non_ground = [coords[2] for coords in object_points]
    ax.scatter3D(x_non_ground, y_non_ground, z_non_ground, color='red');

    plt.savefig(FIGURES_DIR + "10_Point_Cloud_Labelled")

def plot_grid_2D(object_points):
    init_plot_2D("Clustering Grid", "x", "y")

    x = [coords[0] for coords in object_points]
    y = [coords[1] for coords in object_points]

    plt.plot(x, y, '.', color='red')

def plot_reconstruction(reconstructed_clusters):
    init_plot_2D("Reconstructed Clusters", "x", "y")
    colours = ['g', 'grey', 'm', 'orange']
    for i in range(len(reconstructed_clusters)):
        # I put this if statement in to avoid program crashing when cone_width was 0.1
        # this should be investigated!!!
        if len(reconstructed_clusters[i]) > 0:
            x_cluster = [coords[0] for coords in reconstructed_clusters[i]]
            y_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            plt.plot(x_cluster, y_cluster, '.', color=colours[i % len(colours)])
            x_mean  = sum(x_cluster) / len(x_cluster)
            y_mean  = sum(y_cluster) / len(y_cluster)
            plt.plot(x_mean, y_mean, 'x', color='blue')

def plot_clusters(clusters, noise):
    init_plot_2D("Object Segmentation", "x", "y")

    x_noise = [coords[0] for coords in noise]
    y_noise = [coords[1] for coords in noise]

    plt.plot(x_noise, y_noise, '.', color='red')

    colours = ['g', 'grey', 'm', 'orange']
    print(clusters[0]==clusters[1])
    for i in range(len(clusters)):
        x_cluster = [coords[0] for coords in clusters[i]]
        y_cluster = [coords[1] for coords in clusters[i]]
        plt.plot(x_cluster, y_cluster, '.', color=colours[i % len(colours)])
        x_mean  = sum(x_cluster) / len(x_cluster)
        y_mean  = sum(y_cluster) / len(y_cluster)
        plt.plot(x_mean, y_mean, 'x', color='blue')