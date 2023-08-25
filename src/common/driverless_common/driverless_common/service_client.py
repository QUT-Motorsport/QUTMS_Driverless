import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup


class LostService(Exception):
    pass


class ServiceClient:
    def __init__(self, service_name, node, srv_type):
        self.node = node
        self.service_name = service_name
        self.client = node.create_client(srv_type, service_name, callback_group=ReentrantCallbackGroup())

    def invoke(self, request, timeout_sec=3):
        # If we lose contact with the service, we throw a LostService exception, which the caller is expected to catch.
        # This gives the caller the opportunity to restore the client to the expected state from whichever state it is
        # now in. See lifecycle_service_client.py for an example.
        if not self.client.service_is_ready():
            raise LostService()

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
