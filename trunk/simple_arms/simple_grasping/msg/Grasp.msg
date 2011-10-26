# A gripper may have multiple attachement points.
string gripper_frame_id

# Position of the end effector frame relative to the object
geometry_msgs/Pose grasp_pose

# The gripper opening, >= 0. -1 means ignore.
float64 opening

# Gripper closure force, >= 0. -1 means ignore.
float64 force
