from collections import OrderedDict
import csv
import datetime as dt
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict

from driverless_msgs.msg import ConeDetectionStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from driverless_common.common import QOS_ALL

from typing import Any, Dict, List, Tuple

# List of (type, topic)
SUBSCRIPTIONS: List[Tuple[Any, str]] = [
    (PoseWithCovarianceStamped, "/slam/pose"),
    (ConeDetectionStamped, "/slam/track"),
    (Odometry, "/ground_truth/odom"),
    (ConeDetectionStamped, "/ground_truth/global_map"),
]


def flatten_msg_dict(msg_dict: OrderedDict, parent_key: str = "", sep: str = ".") -> OrderedDict:
    items = []
    for key, value in msg_dict.items():
        new_key = parent_key + sep + key if parent_key else key
        if isinstance(value, OrderedDict):
            items.extend(flatten_msg_dict(value, new_key, sep=sep).items())
        else:
            items.append((new_key, value))
    items.append(("Time", time.time()))
    return OrderedDict(items)


class NodeTopicToCSV(Node):
    csv_writers: Dict[str, csv.DictWriter]
    records: int = 0
    start_time: float = 0
    run_begin: dt.datetime

    def __init__(self) -> None:
        super().__init__("topic_to_csv")

        self.csv_writers = {}

        for type_, topic in SUBSCRIPTIONS:
            self.get_logger().info(f"Subscribing to {topic}")
            callback = lambda x, y=topic: self.msg_callback(x, y)
            self.create_subscription(type_, topic, callback, QOS_ALL)

        self.run_begin = dt.datetime.now()
        self.get_logger().info("---Topic to CSV node initalised---")

    def msg_callback(self, msg: Any, topic: str):
        msg_dict = flatten_msg_dict(message_to_ordereddict(msg))

        topic_readable = topic.replace("/", "_")
        if topic_readable not in self.csv_writers:

            csv_folder = Path("./csv_data")
            csv_folder.mkdir(exist_ok=True)
            f = open(csv_folder / f"{topic_readable}_{self.run_begin.isoformat(timespec='seconds')}.csv", "w")
            self.csv_writers[topic_readable] = csv.DictWriter(f, fieldnames=msg_dict.keys(), extrasaction="ignore")
            self.csv_writers[topic_readable].writeheader()

            # self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        self.csv_writers[topic_readable].writerow(msg_dict)

        # get frequency
        # self.records += 1
        # if self.records % 100 == 0:
        #     now = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        #     self.get_logger().info(f"Frequency: {self.records / (now - self.start_time)}")


def main():
    # begin ros node
    rclpy.init()
    node = NodeTopicToCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
