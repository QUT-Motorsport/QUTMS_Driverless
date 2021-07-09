"""
This is a tiny autonomous system that should be able to finish a lap in an empty map with only cones. 
Use the following settings.json:

{
  "SettingsVersion": 1.2,
  "Vehicles": {
    "FSCar": {
      "DefaultVehicleState": "",
      "EnableCollisionPassthrogh": false,
      "EnableCollisions": true,
      "AllowAPIAlways": true,
      "RC":{
          "RemoteControlID": -1
      },
      "Sensors": {
        "Gps" : {
          "SensorType": 3,
          "Enabled": true
        },
        "Lidar": {
          "SensorType": 6,
          "Enabled": true,
          "X": 1.3, "Y": 0, "Z": 0.1,
          "Roll": 0, "Pitch": 0, "Yaw" : 0,
          "NumberOfLasers": 1,
          "PointsPerScan": 500,
          "VerticalFOVUpper": 0,
          "VerticalFOVLower": 0,
          "HorizontalFOVStart": -90,
          "HorizontalFOVEnd": 90,
          "RotationsPerSecond": 10,
          "DrawDebugPoints": true
        }
      },
      "Cameras": {},
      "X": 0, "Y": 0, "Z": 0,
      "Pitch": 0, "Roll": 0, "Yaw": 0
    }
  }
}
"""

import numpy
import math

cones_range_cutoff = 7 # meters

def pointgroup_to_cone(group):
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point[0]
        average_y += point[1]
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return [average_x, average_y]

def distance(x1, y1, x2, y2):
    return math.sqrt(math.pow(abs(x1-x2), 2) + math.pow(abs(y1-y2), 2))

def find_points(pcl):
    # no points
    if len(pcl) < 3:
        return []

    # Convert the list of floats into a list of xyz coordinates
    points = numpy.array(pcl, dtype=numpy.dtype('i4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))

    return points

def find_cones(points):
    # Go through all the points and find nearby groups of points that are close together as those will probably be cones.
    current_group = []
    cones = []
    for i in range(1, len(points)):

        # Get the distance from current to previous point
        distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

        if distance_to_last_point < 0.1:
            # Points closer together then 10 cm are part of the same group
            current_group.append([points[i][0], points[i][1]])
        else:
            # points further away indiate a split between groups
            if len(current_group) > 0:
                cone = pointgroup_to_cone(current_group)
                # calculate distance between lidar and cone
                if distance(0, 0, cone[0], cone[1]) < cones_range_cutoff:
                    cones.append(cone)
                current_group = []
    return cones

def find_avg(cones):
    if len(cones) != 0:
        average_y = 0
        for cone in cones:
            average_y += cone[1]
        average_y = average_y / len(cones)

        return average_y
    
    else:
        return 0
