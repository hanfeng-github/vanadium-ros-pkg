#!/usr/bin/env python

""" Example code of how to move a servo. """

# We always import roslib, and load the manifest to handle dependencies
import roslib; roslib.load_manifest('mini_max_tutorials')
import rospy

# Servo commands are sent in radians as a Float64
from std_msgs.msg import Float64

if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('move_servo')

    # create a publisher to move the head tilt servo
    tilt = rospy.Publisher('head_tilt_joint/command', Float64)
    
    # from now until the end of time:
    while not rospy.is_shutdown():
        # move head around
        tilt.publish(Float64(0.1))
        rospy.sleep(1.0)
        tilt.publish(Float64(0.0))
        rospy.sleep(1.0)
        tilt.publish(Float64(-0.1))
        rospy.sleep(1.0)
        tilt.publish(Float64(0.0))
        rospy.sleep(1.0)


