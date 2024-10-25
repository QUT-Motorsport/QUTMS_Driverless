from enum import Enum

from driverless_msgs.msg import AVStateStamped


class MissionType(Enum):
    MISSION_NONE = "none"
    MANUAL_DRIVING = "manual"
    INSPECTION = "inspection"
    EBS_TEST = "ebs_test"
    TRACKDRIVE = "trackdrive"


class StateType(Enum):
    NOT_READY = "Not ready"
    START_MISSION = "Start mission"
    DRIVING = "Driving"
    END = "End"


INT_MISSION_TYPE = {
    AVStateStamped.MISSION_NONE: MissionType.MISSION_NONE,
    AVStateStamped.MANUAL_DRIVING: MissionType.MANUAL_DRIVING,
    AVStateStamped.INSPECTION: MissionType.INSPECTION,
    AVStateStamped.EBS_TEST: MissionType.EBS_TEST,
    AVStateStamped.TRACKDRIVE: MissionType.TRACKDRIVE,
}

INT_STATE_TYPE = {
    AVStateStamped.NOT_READY: StateType.NOT_READY,
    AVStateStamped.START_MISSION: StateType.START_MISSION,
    AVStateStamped.DRIVING: StateType.DRIVING,
    AVStateStamped.END: StateType.END,
}
