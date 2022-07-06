# Publisher Node that calls methods from curses_key_control.py
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from typing import Tuple, List, Optional
from fs_msgs.msg import ControlCommand
from std_msgs.msg import String, Float32
from driverless_msgs.msg import GuiKeyboard   # custom ros msg type



class Remotecontrol(Node):
    def __init__(self):
        super().__init__("remote_control")
        self.get_logger().info("Key inputs are now available")
        self.p_sim = self.create_publisher(GuiKeyboard, "gui", 1)

    

    def send_control_command_callback(self) -> None: #, brake):
        (throttle, steering, brake) = 1,0.5,0
        print('Hi from remote_control.')
        control_msg = ControlCommand()
        control_msg.throttle = float(throttle)
        control_msg.steering = float(steering)
        control_msg.brake = float(brake) #0.0
        GuiKeyboard = [control_msg.throttle,control_msg.steering,control_msg.brake] # dereference the control_msg object?
        self.p_gui.publish(GuiKeyboard)
        self.p_sim.publish(control_msg)




def main():
    # begin ros node
    rclpy.init()
    node = Remotecontrol.send_control_command()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    pass

if __name__ == '__main__':
    main()
