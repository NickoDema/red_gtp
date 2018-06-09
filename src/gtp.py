#!/usr/bin/env python

##   gtp.py
##   Created on: 09.06.2018
##        Email: ndema2301@gmail.com
##               ITMO University

import rospy
import actionlib

from std_msgs.msg import String
from atwork_ros_msgs.msg import TaskInfo

import youbot_navigation.msg
from geometry_msgs.msg import Pose2D

# Benchmark State
class Benchmark_state():
    def __init__(self):
        # 1 - RUN | 2 - Pause | 3 - Fin | 4 - Stop
        self.State = 4

        # 0 - Execution | 2 - Preparation
        self.Phase = 2

        # 0 - None | 1 - BNT | 2 - BMT | 3 - BTT | 4 - PPT
        # 5 -  CBT | 6 - AWF | 7 - IRL
        self.Type = 0

        # 0, 1, ...
        self.Type_id = 0
        self.Description = ""

    def set(st, ph, t, t_id, des):
        self.State = st
        self.Phase = ph
        self.Type = t
        self.Type_id = t_id
        self.Description = des

def get_tasks():
    # try to check for correctness by messages comparing
    task_msg_first  = rospy.wait_for_message("/robot_example_ros/task_info", TaskInfo)
    while not rospy.is_shutdown():
        task_msg_second = rospy.wait_for_message("/robot_example_ros/task_info", TaskInfo)
        if check_task(task_msg_first, task_msg_second):
            task_msg = task_msg_first
            break
        task_msg_first = task_msg_second

    return task_msg.tasks

# message comparison
def check_task(first_task, second_task):
    return True

# send the goal to navi actionlib server
# task argument can take next values: 'point' and 'dist'
# in 'point' case distance argument mast not be empty
# same in 'dist' case for point, orientation and duration arguments
def navi(task, point, orientation, duration, distance):
    navi_action_client.wait_for_server()
    goal = youbot_navigation.msg.DestGoal
    goal.task = "point"
    goal.target_point = "A2"
    goal.orientation = "A0R1"
    goal.duration = 3
    pose = Pose2D
    pose.x = 0.0
    pose.y = 0.0
    pose.theta = 0.0
    goal.dist = pose
    navi_action_client.send_goal(goal)
    navi_action_client.wait_for_result()
    return navi_action_client.get_result()

if __name__ == '__main__':
    rospy.init_node('GTP')#, anonymous=True)

    #rospy.Subsc
    navi_action_client = actionlib.SimpleActionClient('navi', youbot_navigation.msg.DestAction)
    Test = Benchmark_state()
    rospy.loginfo(Test.State)

    try:
        tasks = get_tasks()
    except rospy.ROSInterruptException:
        exit()

    for task in tasks:
        rospy.loginfo(task.transportation_task.object.type)








    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #hello_str = "hello world %s" % rospy.get_time()
    #while not rospy.is_shutdown():
    #rate = rospy.Rate(10) # 10hz
    #rate.sleep()
