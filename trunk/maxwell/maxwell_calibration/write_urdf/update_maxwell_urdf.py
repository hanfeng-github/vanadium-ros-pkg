#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('maxwell_calibration')
import rospy

import pdb
import sys
import rosbag

try:
    import yaml
except ImportError, e:
    print >> sys.stderr, "Cannot import yaml. Please make sure the pyyaml system dependency is installed"
    raise e

import pr2_calibration_propagation.update_joint as update_joint
import pr2_calibration_propagation.process_changelist as process_changelist
import tf.transformations as transformations
import math

def update_urdf(initial_system, calibrated_system, xml_in):
    dh_offsets = {"arm_chain":[],
                  "head_chain":[]}

    dh_joint_names = {"arm_chain"       : ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'arm_wrist_roll_joint'],
                      "head_chain"      : ['head_pan_joint', 'head_tilt_joint'] }

    print "Computing All dh chain offsets"
    for chain_name in dh_offsets.keys():
        dh_offsets[chain_name] = find_dh_param_offsets(chain_name, initial_system, calibrated_system)
        print "%s offsets:" % chain_name, pplist(dh_offsets[chain_name])

    # Need to be able to lookup the joint offset for each joint
    joint_offsets_list = []
    for chain_name in dh_offsets.keys():
        joint_offsets_list.extend(zip(dh_joint_names[chain_name], dh_offsets[chain_name]))
    joint_offsets = dict(joint_offsets_list)

    #convert transforms to rpy
    transformdict = dict()
    for(name, rotvect) in calibrated_system['transforms'].iteritems():
        floatvect = [mixed_to_float(x) for x in rotvect]
        transformdict[name] = [floatvect[0:3], angle_axis_to_RPY(floatvect[3:6])]

    # Combine the transforms and joint offsets into a single dict
    joints_dict = dict([(joint_name, [None, None, None]) for joint_name in transformdict.keys() + joint_offsets.keys()])
    for joint_name, val in transformdict.items():
        joints_dict[joint_name][0] = val[0]
        joints_dict[joint_name][1] = val[1]

    for joint_name, offset in joint_offsets.items():
        joints_dict[joint_name][2] = offset

    not_found = joints_dict.keys()
    changelist = []

    for joint_name, val in joints_dict.items():
        print joint_name
        cur_cl = update_joint.update_joint(xml_in, joint_name, xyz=val[0], rpy=val[1], ref_shift=val[2])
        if cur_cl is not None:
            for span, result in cur_cl:
                print "\"%s\" -> \"%s\"" % (xml_in[span[0]:span[1]], result)
            not_found.remove(joint_name)
            changelist.extend(cur_cl)

    print "jointnames never found: ", not_found

    xml_out = process_changelist.process_changelist(changelist, xml_in)
    return xml_out


#pretty-print list to string
def pplist(list):
    zeroed = []
    for x in list:
        if x is None:
            zeroed.append(0.)
        else:
            zeroed.append(x)
    return ' '.join(['%2.3f'%x for x in zeroed])


#return 1 if value1 and value2 are within eps of each other, 0 otherwise
def epsEq(value1, value2, eps = 1e-10):
    if math.fabs(value1-value2) <= eps:
        return 1
    return 0


#convert a float/int/string containing 'pi' to just float
def mixed_to_float(mixed):
    pi = math.pi
    if type(mixed) == str:
        try:
            val = eval(mixed)
        except:
            print >> sys.stderr, "bad value:", mixed, "substituting zero!!\n\n"
            val = 0.
    else:
        val = float(mixed)
    return val


#calculate calibration offsets (whicharm = 'left' or 'right')
def find_dh_param_offsets(chain_name, system_default, system_calibrated):
    offsets = []
    for (default_params, calib_params) in zip(system_default['dh_chains'][chain_name]['dh'], system_calibrated['dh_chains'][chain_name]['dh']):
        #print "default_params[0]:", default_params[0], "calib_params[0]:", calib_params[0]
        diff = mixed_to_float(calib_params[0]) - mixed_to_float(default_params[0])
        if epsEq(diff, 0):
            diff = None
        offsets.append(diff)
    return offsets


#convert from rotation-axis-with-angle-as-magnitude representation to Euler RPY
def angle_axis_to_RPY(vec):
    angle = math.sqrt(sum([vec[i]**2 for i in range(3)]))
    hsa = math.sin(angle/2.)
    if epsEq(angle, 0):
        return (0.,0.,0.)
    quat = [vec[0]/angle*hsa, vec[1]/angle*hsa, vec[2]/angle*hsa, math.cos(angle/2.)]
    rpy = quat_to_rpy(quat)
    return rpy

def rpy_to_quat(rpy):
    return transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2], 'sxyz')

def quat_to_rpy(q):
    rpy = transformations.euler_from_quaternion(q, 'sxyz')
    return rpy

def parse_rpy(line):
    return [float(x) for x in line.split("rpy=\"")[1].split("\"")[0].split()]

def parse_xyz(line):
    return [float(x) for x in line.split("xyz=\"")[1].split("\"")[0].split()]


if __name__ == '__main__':

    if len(rospy.myargv()) < 5:
        print "Usage: ./propagate_config [initial.yaml] [calibrated.yaml] [cal_measurements.bag] [cal_output.xml]"
        sys.exit(0)

    #filenames
    initial_yaml_filename    = rospy.myargv()[1]
    calibrated_yaml_filename = rospy.myargv()[2]
    measurement_filename     = rospy.myargv()[3]
    output_filename          = rospy.myargv()[4]

    bag = rosbag.Bag(measurement_filename)
    xml_in = None
    for topic, msg, t in bag.read_messages():
        if topic == "robot_description" or topic == "/robot_description":
            xml_in = msg.data
    if xml_in is None:
        print "Error: Could not find URDF in bagfile. Make sure topic 'robot_description' exists"
        sys.exit(-1)
    bag.close()
    
    #read in files
    system_default = yaml.load(file(initial_yaml_filename, 'r'))
    system_calibrated = yaml.load(file(calibrated_yaml_filename, 'r'))

    xml_out = update_urdf(system_default, system_calibrated, xml_in)

    outfile = open(output_filename, 'w')
    outfile.write(xml_out)
    outfile.close()

