#!/usr/bin/env python

""" Example code of how to relax a servo. """

# We always import roslib, and load the manifest to handle dependencies
import roslib; roslib.load_manifest('mini_max_tutorials')
import rospy

# The "relax" service is defined in arbotix_msgs
from arbotix_msgs.srv import Relax

if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('relax_servo')

    # wait for service to exist
    # this prevents an exception being thrown if this node starts before the ArbotiX is running.
    rospy.wait_for_service('/head_tilt_joint/relax')

    # create a proxy to relax the head tilt servo
    relax_proxy = rospy.ServiceProxy('/head_tilt_joint/relax', Relax)

    # use the proxy to make a service call
    # Relax has no parameters
    relax_proxy()


