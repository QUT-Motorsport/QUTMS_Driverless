import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup


class ServiceClient:
    def __init__(self, service_name, node, srv_type):
        self.node = node
        self.service_name = service_name
        self.client = node.create_client(srv_type, service_name, callback_group=ReentrantCallbackGroup())

    def invoke(self, request, timeout_sec=3):
        while not self.client.wait_for_service(timeout_sec):
            if not rclpy.ok():
                raise Exception(self.service_name + " service client: interrupted while waiting for service")
            self.node.get_logger().info(self.service_name + " service client: waiting for service to appear...")

        future = self.client.call_async(request)

        try:
            self.node.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        except ValueError:
            # Something weird is happening with the executor, will come back to this later.
            pass

        if not future.done():
            future.cancel()
            return None
        return future.result()  # Unhandled throw

    def wait_for_service(self, timeout_sec=3):
        return self.client.wait_for_service(timeout_sec)
