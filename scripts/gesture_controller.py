#!/usr/bin/env python3

from genpy import message
import sys
import rospy
from geometry_msgs.msg import Point, Twist
from controller.msg import gestures
#msg = gestures()

def callback(gestures):
    print(gestures.fingers)
# Define velocity variable
    v = Twist()


# Steering
    v.angular.z = gestures.dir
    v.angular.z = -  v.angular.z
 #   if gestures.dir == "s":
  #      v.angular.z = 0
   # elif gestures.dir == "l":
    #    v.angular.z = 0.2
    #elif gestures.dir == "r":
     #   v.angular.z = -0.2
    #else:
     #   print("unknown input")


            
# Velocity 
    if  gestures.fingers >= 10:
        v.linear.x = 0
    elif gestures.fingers < 10:
        v.linear.x = 0.8
    elif gestures.fingers < 7:
        v.linear.x = 0.4
    elif gestures.fingers < 5:
        v.linear.x = 0
    elif gestures.fingers < 3:
        v.linear.x = -0.4
    else:
        v.linear.x = 0 

# Push velocity to /cmd_vel 
    pub.publish(v)
    


# Initialize subscriptions
if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber("gestures", gestures, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    print("init")
    rospy.spin()

# We will create this message in the gesture recognition package
# $mkdir msg
# $echo -e "int64 fingers\nstring dir" > msg/gestures.msg
# fingers will be number for velocity, dir will be a character for left right straight

# uncomment in package.xml
#   <build_depend>message_generation</build_depend>
#  <exec_depend>message_runtime</exec_depend>

# # Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis
#find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   message_generation

#uncomment add_message_files and add gesture.msg

#add add_dependencies(your_program ${catkin_EXPORTED_TARGETS}) if using multiple packages

#http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
