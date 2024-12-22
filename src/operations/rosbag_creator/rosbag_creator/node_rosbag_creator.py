import os.path as path
import threading

from rosbag2_py import Recorder, RecordOptions, StorageOptions

import rclpy
import rclpy.logging
from rclpy.node import Node

from driverless_msgs.srv import TriggerBagRecord
from std_srvs.srv import Trigger


class RosbagCreator(Node):
    def __init__(self):
        super().__init__("rosbag_creator")
        self.recording = False
        self.record_srv = self.create_service(TriggerBagRecord, "bag/start", self.bag_record_callback)
        self.stop_record_srv = self.create_service(Trigger, "bag/stop", self.stop_record_callback)
        self.recorder = None
        self.get_logger().info("Rosbag creator node started")

    def bag_record_callback(self, request, response):
        # Check if already recording, return success false if so
        if self.recording:
            self.get_logger().info("Already recording")
            response.success = False
            response.message = "Already recording"
        # Check if the output folder exists, if not start recording
        elif not path.isdir(request.filename):
            self.recorder = Recorder()
            self.record_options = RecordOptions()
            self.storage_options = StorageOptions(uri=request.filename, storage_id="mcap")
            self.record_options.all_topics = True
            self.record_options.exclude_topics = ["/velodyne_points", "/velodyne_packets"]
            self.record_thread = threading.Thread(
                target=self.recorder.record, args=(self.storage_options, self.record_options)
            )
            self.record_thread.start()
            self.recording = True
            self.get_logger().info(f"Recording '{request.filename}' started")
            response.success = True
        # If the output folder already exists, return success false
        else:
            self.get_logger().info(f"Output folder '{request.filename}' already exists")
            response.success = False
            response.message = f"Output folder '{request.filename}' already exists"
        return response

    def stop_record_callback(self, request, response):
        # Check if recording, return success false if not (Nothing to stop)
        if not self.recording:
            self.get_logger().info("Not recording")
            response.success = False
            response.message = "Not recording"
        # Stop recording if recording
        else:
            self.recorder.cancel()
            self.recording = False
            self.recorder = None
            self.record_thread.join()
            response.success = True
            self.get_logger().info(f"Recording stopped. Saved to {path.abspath(self.storage_options.uri)}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RosbagCreator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
