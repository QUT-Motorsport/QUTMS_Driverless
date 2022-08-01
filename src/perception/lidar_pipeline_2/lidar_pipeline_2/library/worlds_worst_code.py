import math as meth

# I NEED TO COMPUTE THE CENTER OF A CLUSTER ONLY ONCE
# AND KEEP THIS VALUE. Instead of calculating it multiple times.
# HORIZONTAL_RES = 0.192 * (math.pi / 180) # 0.384 degrees in between each point
HORIZONTAL_RES = 0.05 * (meth.pi / 180)  # 0.384 degrees in between each point
VERTICAL_RES = 1.25 * (meth.pi / 180)  # 1.25 degrees in between each point

CONE_HEIGHT = 0.3  # m
CONE_WIDTH = 0.15  # m


def F_3(x):
    return 1.05 * x - 7.3


def newtons_method(x, x1, y1):
    x_2 = x * x
    x_5 = x_2 * x_2 * x
    x_6 = x_5 * x
    return (x * (12 * A_2 - 8 * A * x_2 * y1 + x_5 * x1)) / (10 * A_2 - 6 * A * x_2 * y1 + x_6)


def cone_filter(distance: float) -> float:
    # if distance < BIN_SIZE:
    #    distance = BIN_SIZE
    return (
        (1 / 2)
        * (CONE_HEIGHT / (2 * distance * meth.tan(VERTICAL_RES / 2)))
        * (CONE_WIDTH / (2 * distance * meth.tan(HORIZONTAL_RES / 2)))
    )


# A = 307.7506
A = 1181.7633
A_2 = A * A


def new_cone_filter(distance: float, point_count: int) -> bool:
    ERROR_MARGIN = 0.95

    if point_count >= F_3(distance):
        x_0 = meth.sqrt(A / point_count)
    else:
        x_0 = distance

    x_n = x_0
    iterations = 10
    for i in range(iterations):
        x_n = newtons_method(x_n, distance, point_count)
        # NEED TO DO CHECK HERE. IF LAST ONE EQUALS NEW ONE THEN BREAK

    closest_point = cone_filter(x_n)
    dist = meth.sqrt((x_n - distance) ** 2 + (closest_point - point_count) ** 2)
    # print(distance, point_count, x_n, closest_point, dist)
    # and x_n >= distance * ERROR_MARGIN and closest_point * ERROR_MARGIN <= point_count
    # and closest_point <= point_count * 1.5
    if dist <= 3 and x_n >= distance * 0.65 and x_n <= distance * 1.4:
        return True
    else:
        print(round(dist, 2), round(x_n, 2), round(distance * 0.85))
        print(dist <= 2.5, x_n >= distance * 0.85)
        return False


FAR_X = 60  # m


def get_cones(reconstructed_clusters):
    cones = []
    ERROR_MARGIN = 0.20  # Constant
    for i in range(len(reconstructed_clusters)):
        point_count = len(reconstructed_clusters[i])
        if point_count >= 1:
            x_cluster = [coords[0] for coords in reconstructed_clusters[i]]
            y_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            # Univserity of melbourne used z as well
            # z_cluster = [coords[1] for coords in reconstructed_clusters[i]]
            x_mean = sum(x_cluster) / len(x_cluster)
            y_mean = sum(y_cluster) / len(y_cluster)
            # z_mean  = sum(y_cluster) / len(y_cluster)
            distance = meth.sqrt(x_mean**2 + y_mean**2)

            # only checks centre of scan for cones - noise filter (delete if needed)
            if abs(x_mean) < FAR_X:
                # print("    ", x_mean, y_mean, point_count, distance)
                if new_cone_filter(distance, point_count):
                    cones.append([x_mean, y_mean])
    return cones
