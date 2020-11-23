#!/usr/bin/env python

import rospy
import math
import moveit_commander
rospy.init_node('print_joints')

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_left = moveit_commander.MoveGroupCommander("left_arm")

group_right = moveit_commander.MoveGroupCommander("right_arm")

def normalize(x):
    if x > math.pi:
        return normalize(x - math.pi)
    elif x < (0 - math.pi):
        return normalize(x + math.pi)
    else:
        return x

while not rospy.is_shutdown():
    print("left:")
    joints = group_left.get_current_joint_values()
    joints[4] = normalize(joints[4])
    joints[6] = normalize(joints[6])


    print(joints)
    print("right:")
    joints = group_right.get_current_joint_values()
    joints[4] = normalize(joints[4])
    joints[6] = normalize(joints[6])
    print(joints)
    rospy.sleep(1)