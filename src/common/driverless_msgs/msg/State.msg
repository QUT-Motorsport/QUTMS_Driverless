std_msgs/Header header

uint8 state
# use enum below
uint8 START=0
uint8 SELECT_MISSION=1
uint8 WAIT_FOR_MISSION=105
uint8 CHECK_EBS=2
uint8 READY=3
uint8 RELEASE_EBS=11
uint8 DRIVING=4
uint8 ACTIVATE_EBS=10
uint8 FINISHED=15
uint8 EMERGENCY=255

uint8 mission
# use enum below
uint8 MISSION_NONE=0
uint8 MANUAL_DRIVING=1
uint8 INSPECTION=2
uint8 EBS_TEST=3
uint8 TRACKDRIVE=4
