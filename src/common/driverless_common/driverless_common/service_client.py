import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup


class ServiceClient:
    def __init__(self, service_name, node, srv_type):
        self.node = node
        self.service_name = service_name
        self.client = node.create_client(srv_type, service_name, callback_group=ReentrantCallbackGroup())

    def invoke(self, request, timeout_sec=-1):
        while not self.client.wait_for_service(timeout_sec):
            if not rclpy.ok():
                raise Exception(self.service_name + " service client: interrupted while waiting for service")
            self.node.get_logger().info(self.service_name + " service client: waiting for service to apprear...")

        self.node.get_logger().debug(self.service_name + " service client: send async request")
        future = self.client.call_async(request)

        self.node.executor.spin_until_future_complete(future, timeout_sec)

        try:
            return future.result()
        except Exception as e:
            self.client.remove_pending_request(future)
            raise e

    def wait_for_service(self, timeout_sec=5):
        return self.client.wait_for_service(timeout_sec)

    def get_service_name(self):
        return self.service_name
