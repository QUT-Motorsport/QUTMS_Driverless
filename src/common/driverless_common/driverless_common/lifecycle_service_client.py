from lifecycle_msgs.msg import State, Transition

from lifecycle_msgs.srv import ChangeState, GetState

from driverless_common.service_client import ServiceClient


class LifecycleServiceClient:
    def __init__(self, client_service_name, node):
        self.node = node
        self.client_get_state = ServiceClient(client_service_name + "/get_state", node, GetState)
        self.client_change_state = ServiceClient(client_service_name + "/change_state", node, ChangeState)

    def activate(self):
        if self.get_state() == State.PRIMARY_STATE_UNCONFIGURED:
            self.change_state(Transition.TRANSITION_CONFIGURE)
        if self.get_state() == State.PRIMARY_STATE_INACTIVE:
            self.change_state(Transition.TRANSITION_ACTIVATE)

    def deactivate(self):
        if self.get_state() == State.PRIMARY_STATE_ACTIVE:
            self.change_state(Transition.TRANSITION_DEACTIVATE)

    def shutdown(self):
        if self.get_state() == State.PRIMARY_STATE_INACTIVE:
            self.change_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)

    def get_state(self):
        request = GetState.Request()
        result = self.client_get_state.invoke(request)
        return result.current_state.id if result is not None else State.PRIMARY_STATE_UNKNOWN

    def change_state(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        result = self.client_change_state.invoke(request)
        return result.success if result is not None else False
