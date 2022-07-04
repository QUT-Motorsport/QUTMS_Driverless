from collections import OrderedDict
import csv
import datetime as dt
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu

from typing import Any, Dict, List, Tuple

# List of (type, topic)
SUBSCRIPTIONS: List[Tuple[str, Any]] = [
    (Imu, "imu"),
]


def flatten_msg_dict(msg_dict: OrderedDict, parent_key: str = "", sep: str = ".") -> OrderedDict:
    items = []
    for key, value in msg_dict.items():
        new_key = parent_key + sep + key if parent_key else key
        if isinstance(value, OrderedDict):
            items.extend(flatten_msg_dict(value, new_key, sep=sep).items())
        else:
            items.append((new_key, value))
    return OrderedDict(items)


class NodeTopicToCSV(Node):
    csv_writers: Dict[str, csv.DictWriter]

    def __init__(self) -> None:
        super().__init__("topic_to_csv")

        self.csv_writers = {}

        for type_, topic in SUBSCRIPTIONS:
            callback = lambda x: self.msg_callback(x, topic)
            self.create_subscription(type_, topic, callback, 10)

        self.get_logger().info("Node topic_to_csv initalised")

    def msg_callback(self, msg: Any, topic: str):
        msg_dict = flatten_msg_dict(message_to_ordereddict(msg))
        print(msg_dict)

        if topic not in self.csv_writers:
            csv_folder = Path("./csv_data")
            csv_folder.mkdir(exist_ok=True)
            f = open(csv_folder / f"{topic}_{dt.datetime.now().isoformat(timespec='seconds')}.csv", "w")
            self.csv_writers[topic] = csv.DictWriter(f, fieldnames=msg_dict.keys())
            self.csv_writers[topic].writeheader()

        self.csv_writers[topic].writerow(msg_dict)


def main():
    # begin ros node
    rclpy.init()
    node = NodeTopicToCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
