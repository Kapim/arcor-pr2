#!/usr/bin/env python

import rospy

from std_srvs.srv import Trigger, TriggerRequest


def task1_init(req):
    task1HandsInit.call()
    task1HeadInit.call()


def task1_run(req):
    task1Head.call(TriggerRequest())
    task1Hands.call(TriggerRequest())


def task2_init(req):
    task2HandsInit.call()
    task2HeadInit.call()


def task2_run(req):
    task2Head.call(TriggerRequest())
    task2Hands.call(TriggerRequest())


def task3_init(req):
    task3HandsInit.call()
    task3HeadInit.call()


def task3_run(req):
    task3Hands.call(TriggerRequest())


if __name__ == '__main__':
    rospy.init_node('video_controller')
    try:
        task1Hands = rospy.ServiceProxy('/arms/task1', Trigger)
        task1Head = rospy.ServiceProxy('/head/task1', Trigger)
        task1HandsInit = rospy.ServiceProxy('/arms/task1_init', Trigger)
        task1HeadInit = rospy.ServiceProxy('/head/task1_init', Trigger)

        task2Hands = rospy.ServiceProxy('/arms/task2', Trigger)
        task2Head = rospy.ServiceProxy('/head/task2', Trigger)
        task2HandsInit = rospy.ServiceProxy('/arms/task2_init', Trigger)
        task2HeadInit = rospy.ServiceProxy('/head/task2_init', Trigger)

        task3Hands = rospy.ServiceProxy('/arms/task3', Trigger)
        task3HandsInit = rospy.ServiceProxy('/arms/task3_init', Trigger)
        task3HeadInit = rospy.ServiceProxy('/head/task3_init', Trigger)

        task1_init_srv = rospy.Service("/task1_init", Trigger,
                                       task1_init)
        task1_run_srv = rospy.Service("/task1_run", Trigger,
                                      task1_run)
        task2_init_srv = rospy.Service("/task2_init", Trigger,
                                       task2_init)
        task2_run_srv = rospy.Service("/task2_run", Trigger,
                                      task2_run)
        task3_init_srv = rospy.Service("/task3_init", Trigger,
                                       task3_init)
        task3_run_srv = rospy.Service("/task3_run", Trigger,
                                      task3_run)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass