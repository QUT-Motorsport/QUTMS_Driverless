from enum import Enum


class MissionType(Enum):
    MANUAL_DRIVING = "manual_driving"
    EBS_TEST = "ebs_test"
    INSPECTION = "inspection"
    TRACKDRIVE = "trackdrive"


INT_MISSION_TYPE = {
    1: MissionType.MANUAL_DRIVING,
    2: MissionType.EBS_TEST,
    3: MissionType.INSPECTION,
    4: MissionType.TRACKDRIVE,
}
