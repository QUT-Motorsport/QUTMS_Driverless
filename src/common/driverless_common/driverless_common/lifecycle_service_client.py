import rclpy
from driverless_common.service_client import ServiceClient
from lifecycle_msgs.msg import State as LifecycleState, Transition
from lifecycle_msgs.srv import GetState, ChangeState


class LifecycleServiceClient:
    def __init__(self, client_service_name, node):
        self.client_get_state = ServiceClient(client_service_name + '/get_state', node, GetState)
        self.client_change_state = ServiceClient(client_service_name + '/change_state', node, ChangeState)

    def get_state(self):
        request = GetState.Request()
        result = self.client_get_state.invoke(request)
        return result.current_state.id

    def change_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        result = self.client_change_state.invoke(request)
        return result.success
