# gesture-recognition-project
### A CSCI 5551 Project
#### by Luke Friede, Jason Vivit, Varun Kamath

Gesture recognition and robot control project using ROS 2, OpenCV, and Gazebo.

## Files
This project implements the convex hull method for gesture recognition in OpenCV and uses elementary robot control from `geometry_msg/cmd_vel` in Python 3. Relevant files apart from those provided are `scripts/recognize.py` and `launch/recognize.launch` 

## Dependencies
This project requires:
1. ROS Noetic
2. OpenCV 4.2.0
3. Python >= 3.6
4. Python 3: `numpy`
5. Python 3: `sklearn`
6. Python 3: `imutils`
7. Gazebo 11.3

## Usage
Place `grp/`, `controller/`, `turtlebot3`, `turtlebot3_msgs`, and `turtlebot3_simulations` in `~/catkin_ws/src`. Then:
1. Run `./configure.sh` to install dependencies and OpenCV 4.2.0
2. Source `bash` again to initialize turtlebot model (run `source ~/.bashrc`)
3. Run `catkin_make`  in `~/catkin_ws`
4. Run `source ~/catkin_ws/devel/setup.bash` or `source ~/catkin_ws/devel/setup.zsh`
5. Run `roslaunch grp recognize.launch`
