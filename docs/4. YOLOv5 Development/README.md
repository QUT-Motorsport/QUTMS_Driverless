# QUTMS yoloV5 cone detector - WIP
This branch details the yoloController package 

__WORK IN PROGRESS__ 

## Requirements for use (not complete yet) 
Ros2 Foxy<br><br>
Python3.8<br><br>
Torch3.8<br><br>
Yolov5 (use pip)<br><br> 
OpenCV<br><br>
cv_bridge<br><br>

## How to use 
Assuming all requirements work and are build correctly<br><br>
Open two terminals and cd into your workspace
```
cd dev_ws
```
Install the workspace on each
```
. install/setup.bash
```
Run the Subsriber node
```
ros2 run yoloController detectCones 
```
Run the Publisher node
```
ros2 run yoloController testConesTopic 
```
## Update 11/07/2021
There are two nodes that are being used 'detectCones.py' and 'yoloTester.py'.
Where 'detectCones.py' is the subscriber node and 'yoloTester.py' is the publisher. 
Currently the two nodes work with the publisher node sending up a test image which included in the 'yoloController' folder as 'testImage' and its directory is hard-coded. The two nodes are also talking on the topic 'Image' as the simulator topics or functionality hasnt been done yet. The yoloV5 detection model is also included in the file a 'bestOne.pt' and its path is also hard-coded. 

