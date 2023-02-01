from pathlib import Path

import pandas as pd

import rclpy
from rclpy.node import Node

from driverless_msgs.msg import TrackDetectionStamped


class NodeTopicToCSV(Node):
    csv_folder = Path("./csv_data")

    def __init__(self) -> None:
        super().__init__("track_to_csv_node")

        # subscribe to topic
        self.subscription = self.create_subscription(TrackDetectionStamped, "/slam/track", self.callback, 10)

        self.get_logger().info("---Track Writer Initalised---")

    def callback(self, msg: TrackDetectionStamped):
        df = pd.DataFrame(columns=["colour", "x", "y"])
        # iterate through detections, extract colour and position
        # add each detection
        for cone in msg.cones:
            # create a new row
            if cone.cone.color == 0:
                colour = "blue"
            elif cone.cone.color == 1:
                colour = "unknown"
            elif cone.cone.color == 2:
                colour = "orange"
            elif cone.cone.color == 3:
                colour = "orange"
            elif cone.cone.color == 4:
                colour = "yellow"

            row = {
                "colour": colour,
                "x": cone.cone.location.x,
                "y": cone.cone.location.y,
            }

            # append the new row to the csv file
            df = df.append(row, ignore_index=True)

        # write the csv file
        df.to_csv(self.csv_folder / "track.csv", index=False)


def main():
    # begin ros node
    rclpy.init()
    node = NodeTopicToCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
