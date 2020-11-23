#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
import actionlib
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

class Head:

    def __init__(self):
        self.head_action_client = actionlib.SimpleActionClient("/head_traj_controller/point_head_action",
                                                               PointHeadAction)
        self.head_action_client.wait_for_server()

        self.task1_srv = rospy.Service("/head/task1", Trigger,
                                       self.task1_cb)

        self.task1_init_srv = rospy.Service("/head/task1_init", Trigger,
                                            self.task1_init)

        self.look_at_me_srv = rospy.Service("/head/look_at_me", Trigger,
                                            self.look_at_me_cb)

        self.task1 = False

        self.task2_init_srv = rospy.Service("/head/task2_init", Trigger,
                                            self.task2_init)

        self.task2_srv = rospy.Service("/head/task2", Trigger,
                                       self.task2_cb)
        self.task2 = False

        self.task3_init_srv = rospy.Service("/head/task3_init", Trigger,
                                            self.task3_init)

    def task1_init(self, req):
        self.look_down()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task1_cb(self, req):
        self.task1 = True
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task2_init(self, req):
        self.look_at_me()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task2_cb(self, req):
        self.task2 = True
        resp = TriggerResponse()
        resp.success = True
        return resp

    def task3_init(self, req):
        self.look_at_me()
        resp = TriggerResponse()
        resp.success = True
        return resp

    def look_at_default(self, speed=0.7):
        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.point.x = 0.4
        pt.point.y = -0.15
        pt.point.z = 1.2
        self.look_at(pt, speed, send_and_wait=True)

    def look_at_me_cb(self, req):
        self.look_at_me(False, 0.3)

        self.task1 = False
        resp = TriggerResponse()
        resp.success = True
        return resp

    def look_at_me(self, wait=True, speed=0.7):
        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.point.x = 0.4
        pt.point.y = -0.15
        pt.point.z = 1.4
        self.look_at(pt, speed, send_and_wait=wait)

    def look_down(self, speed=0.7):

        pt = PointStamped()
        pt.header.frame_id = "torso_lift_link"
        pt.point.x = 0.5
        pt.point.y = 0
        pt.point.z = -0.8
        self.look_at(pt, speed, send_and_wait=True)

    def look_left(self, speed=0.7):

        pt = PointStamped()
        pt.header.frame_id = "torso_lift_link"
        pt.point.x = 0.4
        pt.point.y = 1
        pt.point.z = 0.3
        self.look_at(pt, speed, send_and_wait=True)

    def look_right(self, speed=0.7):

        pt = PointStamped()
        pt.header.frame_id = "torso_lift_link"
        pt.point.x = 0.4
        pt.point.y = -1
        pt.point.z = 0.3
        self.look_at(pt, speed, send_and_wait=True)

    def look_down_left(self, speed=0.7):
        print("left")
        pt = PointStamped()
        pt.header.frame_id = "torso_lift_link"
        pt.point.x = 0.4
        pt.point.y = 0.05
        pt.point.z = -0.8
        self.look_at(pt, speed, send_and_wait=True)
        print("left_done")

    def look_down_right(self, speed=0.7):
        print("right")
        pt = PointStamped()
        pt.header.frame_id = "torso_lift_link"
        pt.point.x = 0.4
        pt.point.y = -0.05
        pt.point.z = -0.8
        self.look_at(pt, speed, send_and_wait=True)


    def look_at(self, where, speed, send_and_wait=True):
        """ where: PoseStamped."""

        goal = PointHeadGoal()
        goal.target = where
        goal.pointing_frame = "high_def_frame"
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = speed
        if send_and_wait:
            self.head_action_client.send_goal_and_wait(goal, rospy.Duration(5))
        else:
            self.head_action_client.send_goal(goal)


if __name__ == '__main__':
    rospy.init_node('video_head')
    try:
        node = Head()
        while not rospy.is_shutdown():
            if node.task1:
                node.look_at_default()
                node.look_left()
                if node.task1:
                    node.look_right()
                    if node.task1:
                        node.look_left()
                node.task1 = False
            if node.task2:
                node.look_down()
                node.look_down_left(0.5)
                node.look_down_right(0.5)
                node.look_down_left(0.5)
                node.look_down_right(0.5)
                node.look_down_left(0.5)
                node.look_down_right(0.5)
                node.task2 = False
    except rospy.ROSInterruptException:
        pass