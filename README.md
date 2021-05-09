# gesture-recognition-project
### A CSCI 5551 Project
#### by Luke Friede, Jason Vivit, Varun Kamath

Gesture recognition and robot control project using ROS 2 and OpenCV (maybe Gazebo too lol)

## Files
This project implements the convex hull method for gesture recognition in OpenCV and uses elementary robot control from `geometry_msg/cmd_vel` in Python 3. Relevant files apart from those provided are `/scripts/recognize.py` and `/launch/recognize.launch` 


## Usage
With `gesture-recognition-project` (this repo) in `~/catkin_ws/src`:
1. Run `catkin_make`  in `~/catkin_ws`
2. Run `source ~/catkin_ws/devel/setup.bash` or `source ~/catkin_ws/devel/setup.zsh`
3. Run `roslaunch gesture-recognition-project recognize.launch`
