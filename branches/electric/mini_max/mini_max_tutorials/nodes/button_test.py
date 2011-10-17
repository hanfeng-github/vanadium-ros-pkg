#!/usr/bin/env python

""" Example code of how to access button state. """

# We always import roslib, and load the manifest to handle dependencies
import roslib; roslib.load_manifest('mini_max_tutorials')
import rospy

# arbotix_msgs defines the Digital message, which tells us the state
#  of the button (a digital pin)
from arbotix_msgs.msg import *

class ButtonTest:
    
    def buttonCb(self, msg):
        # msg is arbotix_msgs.Digital, value == state of the button
        if msg.value == 0:
            print "Button pressed!"

    def __init__(self):
        # we have to initialize the node, with a name in the ROS graph
        rospy.init_node('button_test')

        # subscribe to the green_button value. 
        rospy.Subscriber('/arbotix/green_button',Digital,self.buttonCb)

        # everything will be handled in the callback, just spin
        rospy.spin()

# this quick check means that the following code runs ONLY if this is 
#  in the main file -- if we "import button_test" in another file, 
#  this code will not execute
if __name__ == "__main__":
    bt = ButtonTest()
