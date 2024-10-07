std_msgs/Header header

uint8 mode
# use enum below
uint8 MANUAL=0
uint8 AUTONOMOUS=1

uint8 state
# use enum below
uint8 NOT_READY=0
uint8 START_MISSION=1
uint8 DRIVING=2
uint8 END=3

uint8 mission
# use enum below
uint8 MISSION_NONE=0
uint8 INSPECTION=1
uint8 EBS_TEST=2
uint8 TRACKDRIVE=3
