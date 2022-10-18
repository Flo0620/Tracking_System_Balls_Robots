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

## Configure the system
There are some parameters that can be changed. For the Ball Detection System the radius of the balls that should be tracked is needed and the confidence threshold, the nms threshold and the number of balls that are expected in the image, if this is known, can be changed. To do that the parameters can be set in src/ball_detection/ball_detection/config_and_weights/ball_detector_config.py.

To add new AprilTags, for instance with a new size or id, but also to use a different tag family at src/apriltag_ros/apriltag_ros/cfg/ a new yaml file for a new tag family have to be created or the existing yaml-files have to be adjusted for the new AprilTags that should be used.
In src/apriltag_ros/apriltag_ros/launch/tag_realsense.launch.py the adjusted yaml-file has to be selected. 

The camera calibration matrix can be changed in src/bitbots_ceiling_cam/config/camera_calibration_ceiling_cam.yaml.

## Sources
AprilTag Detection: https://github.com/AprilRobotics

Camera: https://github.com/bit-bots/bitbots_meta

YOLO: https://github.com/eriklindernoren/PyTorch-YOLOv3
