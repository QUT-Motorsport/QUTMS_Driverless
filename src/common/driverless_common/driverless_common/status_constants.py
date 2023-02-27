from enum import Enum

from driverless_msgs.msg import State


class MissionType(Enum):
    MISSION_NONE = "None"
    MANUAL_DRIVING = "Manual driving"
    EBS_TEST = "EBS test"
    INSPECTION = "Inspection"
    TRACKDRIVE = "Trackdrive"


class StateType(Enum):
    START = "Start"
    SELECT_MISSION = "Select mission"
    CHECK_EBS = "Check EBS"
    READY = "Ready"
    DRIVING = "Driving"
    ACTIVATE_EBS = "Activate EBS"
    FINISHED = "Finished"
    EMERGENCY = "Emergency"


INT_MISSION_TYPE = {
    State.MISSION_NONE: MissionType.MISSION_NONE,
    State.MANUAL_DRIVING: MissionType.MANUAL_DRIVING,
    State.INSPECTION: MissionType.INSPECTION,
    State.EBS_TEST: MissionType.EBS_TEST,
    State.TRACKDRIVE: MissionType.TRACKDRIVE,
}

INT_STATE_TYPE = {
    State.START: StateType.START,
    State.SELECT_MISSION: StateType.SELECT_MISSION,
    State.CHECK_EBS: StateType.CHECK_EBS,
    State.READY: StateType.READY,
    State.DRIVING: StateType.DRIVING,
    State.ACTIVATE_EBS: StateType.ACTIVATE_EBS,
    State.FINISHED: StateType.FINISHED,
    State.EMERGENCY: StateType.EMERGENCY,
}
