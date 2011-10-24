
# this server offers 2 actions:

# pick action:
#  inputs:  PickupAction: id of object
#                         array of potential grasp poses
#                         gripper closure/force/?
#
#  outputs: 1) pre-grasp call to arm_navigation
#           2) interpolated IK towards grasp
#           3) gripper closure
#           4) move gripper out of field of view

# place action:
#  inputs:  PlaceAction: id of object
#                        array of potential goal poses
#                        gripper opening
#
#  outputs: 1) pre-grasp call to arm_navigation
#           2) interpolated IK towards grasp
#           3) gripper closure
#           4) move gripper out of field of view


###
#   GripperCommand.msg
#       std_msgs/String   frame_id      # gripper may have multiple attachment points
#       std_msgs/Float64  opening       # opening, in meters, >= 0. -1 means ignore.
#       std_msgs/Float64  force         # closure force, in N, >= 0. -1 means ignore.


class GraspServer():
    
    def PickCallback(self, goal):
        pass

    def PlaceCallback(self, goal):
        pass


if __name__=="__main__":
    rospy.init_node("simple_grasp_server")
    server = GraspServer()
    rospy.spin()

