from driverless_common.service_client import LostService, ServiceClient

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


class LifecycleServiceClient:
    expected_state: State

    def __init__(self, client_service_name, node):
        self.node = node
        self.client_get_state = ServiceClient(client_service_name + "/get_state", node, GetState)
        self.client_change_state = ServiceClient(client_service_name + "/change_state", node, ChangeState)
        self.expected_state = State.PRIMARY_STATE_UNKNOWN

    def is_active(self):
        return self.get_state() == State.PRIMARY_STATE_ACTIVE

    def is_alive(self):
        return self.client_get_state.client.service_is_ready()

    def is_in_expected_state(self):
        return self.get_state() == self.expected_state

    def change_to_expected_state(self):
        if self.expected_state == State.PRIMARY_STATE_ACTIVE:
            self.activate()
        elif self.expected_state == State.PRIMARY_STATE_INACTIVE:
            self.deactivate()
        else:
            self.shutdown()

    def activate(self):
        try:
            if self.get_state() == State.PRIMARY_STATE_UNCONFIGURED:
                self.change_state(Transition.TRANSITION_CONFIGURE)
            if self.get_state() == State.PRIMARY_STATE_INACTIVE:
                self.change_state(Transition.TRANSITION_ACTIVATE)
        except LostService:
            # Lost service while changing state, this exception is thrown once connection is reestablished, we recall
            # this method so that we can ensure the node is activated correctly from whatever state it is in when
            # it reconnects.
            self.activate()

    def deactivate(self):
        try:
            if self.get_state() == State.PRIMARY_STATE_ACTIVE:
                self.change_state(Transition.TRANSITION_DEACTIVATE)
        except LostService:
            # Probably not necessary, but just in case.
            self.deactivate()

    def shutdown(self):
        try:
            if self.get_state() == State.PRIMARY_STATE_INACTIVE:
                self.change_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)
        except LostService:
            # Probably not necessary, but just in case.
            self.shutdown()

    def get_state(self):
        request = GetState.Request()
        result = self.client_get_state.invoke(request)
        return result.current_state.id if result is not None else State.PRIMARY_STATE_UNKNOWN

    def change_state(self, transition_id):
        if transition_id == Transition.TRANSITION_ACTIVATE:
            self.expected_state = State.PRIMARY_STATE_ACTIVE
        elif transition_id == Transition.TRANSITION_DEACTIVATE or transition_id == Transition.TRANSITION_CONFIGURE:
            self.expected_state = State.PRIMARY_STATE_INACTIVE
        elif (transition_id == Transition.TRANSITION_INACTIVE_SHUTDOWN
              or transition_id == Transition.TRANSITION_ACTIVE_SHUTDOWN
              or transition_id == Transition.TRANSITION_UNCONFIGURED_SHUTDOWN):
            self.expected_state = State.PRIMARY_STATE_UNCONFIGURED

        request = ChangeState.Request()
        request.transition.id = transition_id
        result = self.client_change_state.invoke(request)
        return result.success if result is not None else False
