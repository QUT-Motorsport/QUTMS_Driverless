from rclpy.node import Node

from driverless_msgs.msg import State


class ShutdownNode(Node):
    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        self.reset_sub = self.create_subscription(State, "/system/as_status", self.shutdown_callback, 10)

    def shutdown_callback(self, msg: State):
        if msg.state in [State.START, State.SELECT_MISSION, State.ACTIVATE_EBS, State.FINISHED, State.EMERGENCY]:
            self.destroy_node()
            raise Exception("Node Shutdown")
