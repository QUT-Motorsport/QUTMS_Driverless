from sklearn.cluster import DBSCAN

def group_points(object_points):
    EPSILON = 0.4  # Neighbourhood Scan Size
    MIN_POINTS = 3 # Number of points required to form a neighbourhood

    clustering = DBSCAN(eps=EPSILON, min_samples=MIN_POINTS).fit(object_points)
    print(clustering.labels_)
    pass