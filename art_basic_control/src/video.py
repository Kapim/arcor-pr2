#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupAction
import sys

import actionlib
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_mechanism_msgs.srv import SwitchController, SwitchControllerRequest
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

#from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Process
from multiprocessing import Pool

import time

class Video:

    def __init__(self):

        self.tfl = tf.TransformListener()

        self.ns = "/art/robot/"  # the node cannot be started in namespace - MoveGroupCommander does not work like that

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        self.group_left = moveit_commander.MoveGroupCommander("left_arm")

        self.group_right = moveit_commander.MoveGroupCommander("right_arm")

        self.task1_srv = rospy.Service("/arms/task1", Trigger,
                                       self.task1_cb)
        self.task1_init_srv = rospy.Service("/arms/task1_init", Trigger,
                                            self.task1_init)
        self.task1 = False

        self.task2_srv = rospy.Service("/arms/task2", Trigger,
                                       self.task2_cb)
        self.task2_init_srv = rospy.Service("/arms/task2_init", Trigger,
                                            self.task2_init)
        self.task2 = False

        self.look_at_me = rospy.ServiceProxy('/head/look_at_me', Trigger)

        self.task3_srv = rospy.Service("/arms/task3", Trigger,
                                       self.task3_cb)
        self.task3_init_srv = rospy.Service("/arms/task3_init", Trigger,
                                            self.task3_init)
        self.task3 = False

    def task1_init(self, req):
        self.left_base_position()
        self.right_base_position()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task1_cb(self, req):
        self.task1 = True
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task2_init(self, req):
        self.left_up_position()
        self.right_down_position()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task2_cb(self, req):
        self.task2 = True
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task3_init(self, req):
        self.left_diploma_init_position()
        self.right_pedel_position()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task3_cb(self, req):
        self.task3 = True
        resp = TriggerResponse()
        resp.success = True
        return resp

    def move(self, group, pose=None):

        assert pose is not None

        # group.set_workspace([minX, minY, minZ, maxX, maxY, maxZ])


        group.set_pose_target(pose)

        group.allow_looking(False)
        group.allow_replanning(False)
        group.set_num_planning_attempts(1)

        group.go(wait=True)

    def move_joints(self, group, joints):
        group.go(joints, wait=True)

    def left_point_front(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.8
        pose.pose.position.y = 0.15
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_left, pose=pose)

    def right_point_front(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.8
        pose.pose.position.y = -0.15
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_right, pose=pose)

    def left_base_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.411
        pose.pose.position.y = -0.112
        pose.pose.position.z = -0.029
        pose.pose.orientation.x = -0.028
        pose.pose.orientation.y = 0.7
        pose.pose.orientation.z = 0.057
        pose.pose.orientation.w = 0.711
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_left, pose=pose)

    def right_base_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.353
        pose.pose.position.y = 0.163
        pose.pose.position.z = -0.374
        pose.pose.orientation.x = -0.022
        pose.pose.orientation.y = 0.771
        pose.pose.orientation.z = -0.636
        pose.pose.orientation.w = -0.021
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_right, pose=pose)


    def left_down_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.409
        pose.pose.position.y = 0.382
        pose.pose.position.z = -0.469
        pose.pose.orientation.x = 0.704
        pose.pose.orientation.y = -0.498
        pose.pose.orientation.z = -0.361
        pose.pose.orientation.w = -0.354
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_left, pose=pose)

    def right_down_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.307
        pose.pose.position.y = -0.355
        pose.pose.position.z = -0.600
        pose.pose.orientation.x = -0.630
        pose.pose.orientation.y = 0.122
        pose.pose.orientation.z = 0.402
        pose.pose.orientation.w = 0.653
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_right, pose=pose)


    '''def left_up_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.230
        pose.pose.position.y = 0.649
        pose.pose.position.z = 0.181
        pose.pose.orientation.x = 0.777
        pose.pose.orientation.y = -0.351
        pose.pose.orientation.z = 0.522
        pose.pose.orientation.w = -0.031
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_right, pose=pose)

    def right_up_position(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.133
        pose.pose.position.y = -0.690
        pose.pose.position.z = 0.074
        pose.pose.orientation.x = 0.713
        pose.pose.orientation.y = 0.174
        pose.pose.orientation.z = 0.579
        pose.pose.orientation.w = -0.356
        pose.header.frame_id = "torso_lift_link"
        self.move(self.group_right, pose=pose)'''

    def left_up_position(self):
        self.move_joints(self.group_left, [0.7609417881996512, 0.4618117318628601, 0.4128562948128305, -1.5029827324220433, -0.24106517705087427, -0.603932873918261, 0.062428466957282325])

    def right_up_position(self):
        self.move_joints(self.group_right, [-1.0126058533253033, 0.43326521261073264, -0.6409457447611411, -1.7018976898630103, 0.5745774996758257, -0.4815071534029076, 0.25594430935577517])

    def left_diploma_position(self):
        self.move_joints(self.group_left,
                         [0.51122705672041, 0.26140635902840703, 1.1585062374328015, -1.4397179315474415,
                          2.5068780498438485, -0.8878720864882139, -2.2947729439555005])

    def left_diploma_init_position(self):
        self.move_joints(self.group_left,
                         [1.3053066377656197, 0.47357042617437933, 0.8199971989659718, -1.6783000639074954,
                          0.8875562809662512, -1.6227812558904644, -0.8181485669384045])

    def right_pedel_position(self):
        self.move_joints(self.group_right, [-0.7269925565304873, 0.6990624610049292, -0.6723752907296386, -1.3249147574203042, -1.433245305161737, -1.521848941904309, -1.481746019380405])
try:
    rospy.init_node('art_video')
    node = Video()

    while not rospy.is_shutdown():
        if node.task1:

            #node.left_base_position()
            #node.right_base_position()
            node.left_up_position()
            node.right_up_position()
            node.look_at_me()
            print("lookatme")
            node.left_point_front()
            node.task1 = False
        if node.task2:
            node.left_down_position()
            #node.right_down_position()
            node.task2 = False
        if node.task3:
            node.left_diploma_position()
            #node.right_down_position()
            node.task3 = False
        rospy.sleep(0.1)

except rospy.ROSInterruptException:
    pass
