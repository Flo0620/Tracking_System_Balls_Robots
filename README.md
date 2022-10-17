# Tracking_System_Balls_Robots
This repository contains a tracking system for the RoboCup environment and was created as a bachelor thesis at the University of Hamburg. The tracking system is capable of tracking the pose of robots which have a mounted AprilTag on their head and uses YOLOv3 to detect the position of the balls on the field. The results are published as ROS2 transforms.

More information about the system can be found in the associated bachelor thesis:
Schleid, Florian Capturing Pose Data for RoboCup Humanoid Soccer using AprilTags and YOLO. University of Hamburg, 2022.

## Installing and running the Tracking System
1. First clone this repository:
```
git clone https://github.com/Flo0620/Tracking_System_Balls_Robots.git
```
2. Download the yolov3-weights at https://data.bit-bots.de/FlorianSchleid_BA/ and paste the file in the folder src/ball_detection/ball_detection/config_and_weights
3. Now open a terminal and navigate to the cloned repository.
4. Source your terminal:
```
source /opt/ros/rolling/setup.zsh
```
5. Build the system:
```
colcon build --symlink-install
```
6. Open a new terminal and navigate to the repository and source the terminal:
```
source install/setup.zsh
```
7. Now you can run the system:
```
ros2 launch launch_tracking_system.xml
```
