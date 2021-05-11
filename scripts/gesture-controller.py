#!/usr/bin/env python3

# Controller for gesture recognition program

from genpy import message
import sys
import rospy
from geometry_msgs.msg import Point, Twist
from controller.msg import gestures

def callback(gestures):

# Define velocity variable
    v = Twist()

# Steering
    v.angular.z = -(gestures.dir)

# Velocity
    fing = gestures.fingers
    if fing == 0:
        v.linear.x = 0
    elif  fing < 3:
        v.linear.x = -0.3
        #print("less than 3")
    elif fing < 5:
        v.linear.x = 0.1
        #print("less than 5")
    elif fing < 7:
        v.linear.x = 0.3
        #print("less than 7")
    elif fing <= 10:
        v.linear.x = 0.5
        #print("less than 10")
    elif fing > 10:
        v.linear.x = 2
        #print("more than 10")
    else:
        print("finger count error") 

# Push velocity to /cmd_vel 
    pub.publish(v)
    


# Initialize subscriptions
if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber("gestures", gestures, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    print("Listening...")
    rospy.spin()