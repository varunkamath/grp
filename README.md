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
1. Run `./config/configure.sh` to install dependencies and OpenCV 4.2.0
2. Source `bash` again to initialize turtlebot model (run `source ~/.bashrc`)
3. Run `catkin_make`  in `~/catkin_ws`
4. Run `source ~/catkin_ws/devel/setup.bash` or `source ~/catkin_ws/devel/setup.zsh`
5. Run `roslaunch grp recognize.launch`
6. Allow the camera to calibrate for a few seconds without anything moving in the background before putting your hands up.

## Controlling the Robot
#### Note: For best results, establish strong contrast between hands and (unimodal) background 

Steering:
Moving your hands up and down opposite of eachother within the AOI boxes will result in the steering text at the bottom of the screen to change.  It will feel similar to a steering wheel motion.

Velocity:
1. Holding up two fists (should) stop the bot from moving.
2. Holding up one finger on each hand will put the robot in reverse.
3. Holding up more than 2 fingers total will incrementally increase your speed.
4. If you find a way to reach more than 10 fingers, you will unlock super speed! 