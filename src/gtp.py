#!/usr/bin/env python

# MIT License
#
# Copyright (c) 2018 Nikolay Dema
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

##   gtp.py
##   Created on: 09.06.2018
##        Email: ndema2301@gmail.com

import rospy
import actionlib
from terminaltables import SingleTable
import random

from std_msgs.msg import String
from atwork_ros_msgs.msg import TaskInfo
from atwork_ros_msgs.msg import BenchmarkState

import youbot_navigation.msg
import red_msgs.msg
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseWCS

# CONSTANTS:
RUN   = 1
PAUSE = 2
FINAL = 3
STOP  = 4

EXEC  = 0
CALIB = 1
PREP  = 2

# Navi task description
LOCS = { 1:"SH", 2:"WS", 3:"CB", 4:"WP", 5:"PP", 6:"ROBOT"}
ORS = {1:"NORTH", 2:"EAST", 3:"SOUTH", 4:"WEST"}

# for the weigth function
ALPHA = 1.1

Test_state = None

# Benchmark State
class Benchmark_state():
    def __init__(self):
        # 1 - RUN | 2 - Pause | 3 - Fin | 4 - Stop
        self.State = STOP

        # 0 - Execution | 2 - Preparation
        self.Phase = PREP

        # 0 - None | 1 - BNT | 2 - BMT | 3 - BTT | 4 - PPT
        # 5 -  CBT | 6 - AWF | 7 - IRL
        self.Type = 0

        # 0, 1, ...
        self.Type_id = 0
        self.Description = ""

    def set(self, st, ph, t, t_id, des):
        self.State = st
        self.Phase = ph
        self.Type = t
        self.Type_id = t_id
        self.Description = des

# try to check for correctness by messages comparing
def get_tasks():
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
    if (len(first_task.tasks) != len(first_task.tasks)):
        return False
    #for i in xrange(len(first_task.tasks)):
    #    f_task = first_task.tasks[i]
    #    s_task = second_task.tasks[i]
    #    if (f)
    return True

# set current test state by the incoming message
def state_cb(msg):
    global Test_state
    Test_state.set(msg.state.data, msg.phase.data, msg.scenario.type.data,
                   msg.scenario.type_id.data, msg.scenario.description.data,)

# scenario optimization for BTT casen
def scenario_optimization(tasks):
    for task in tasks:
        W = get_weigth(task, task[0], task[1])
        task.append(W)
    tasks.sort(key=lambda x: x[3])
    return tasks

# weigth calculation
def get_weigth(task, dest, source):
    # BNT
    # if (Test_state.Type == 1) or (Test_state.Type == 3):
    #     pose = "START"
    # if (Test_state.Type == 2):
    #     pose = "CUR"
        # pose_msg = rospy.wait_for_message("/amcl_pose", PoseWCS)
    l = random.uniform(1, 6)
    n = len(task[2])
    to_source = 1
    return l/(n*ALPHA) + to_source

# send the goal to navi actionlib server
# task argument can take next values: 'point' and 'dist'
# in 'point' case distance argument mast not be empty
# same in 'dist' case for point, orientation and duration arguments
def navi(task, point, orientation, duration, distance):
    navi_action_client.wait_for_server()
    goal = youbot_navigation.msg.DestGoal
    goal.task = task
    if (task == "point"):
        goal.target_point = point
        goal.orientation = orientation
        goal.duration = duration
        pose = Pose2D
        pose.x = 0.0
        pose.y = 0.0
        pose.theta = 0.0
        goal.dist = pose
    elif (task == "dist"):
        goal.dist = distance
        goal.target_point = ""
        goal.orientation = ""
        goal.duration = ""

    navi_action_client.send_goal(goal)
    while not rospy.is_shutdown():
        status = navi_action_client.get_status()
        if (status == "ACTIVE") or (status == "PENDING"):
            continue
        elif (status == "SUCCEEDED"):
            return True
        else:
            return False


    #navi_action_client.wait_for_result()
    return navi_action_client.get_result()

def manipulate()

if __name__ == '__main__':
    rospy.init_node('GTP')#, anonymous=True)
    rospy.Subscriber("/robot_example_ros/benchmark_state", BenchmarkState, state_cb)

    #rospy.Subsc
    navi_action_client = actionlib.SimpleActionClient('navi', youbot_navigation.msg.DestAction)
    Test_state = Benchmark_state()

    rate = rospy.Rate(10) # 10hz

    # mine spiner
    while not rospy.is_shutdown():

        # waiting for execution test phase
        while (Test_state.Phase != EXEC) or (Test_state.State == FINAL):
            if (rospy.is_shutdown()): exit()
            rate.sleep()

        rospy.loginfo("[GTP]: Current phase - RUNNING")

        try:
            task_msgs = get_tasks()
        except rospy.ROSInterruptException:
            #rospy.logerr("[GTP]: Can't get task from corresponding topic")
            continue

        # test scenario list
        tasks = []

        # BNT
        if (Test_state.Type == 1):
            for msg in task_msgs:
                location = LOCS[msg.navigation_task.location.type.data] + \
                           str(msg.navigation_task.location.instance_id.data)
                orientation = ORS[msg.navigation_task.orientation.data]
                duration = msg.navigation_task.wait_time.data.secs
                tasks.append([location, orientation, duration])

        # BMT or BTT
        elif (Test_state.Type == 2) or (Test_state.Type == 3):
            for msg in task_msgs:
                destination = LOCS[msg.transportation_task.destination.type.data] + \
                              str(msg.transportation_task.destination.instance_id.data)
                source = LOCS[msg.transportation_task.source.type.data] + \
                         str(msg.transportation_task.source.instance_id.data)
                object_type = msg.transportation_task.object.type.data
                container = msg.transportation_task.container.type.data

                # check if this source-destination pair is already in the tasks
                # if it is add new objects in the task
                new = True
                for task in tasks:
                    if ((task[0] == destination) and (task[1] == source)):
                        new = False
                        object_pose = container
                        if (object_pose != 14) and (object_pose != 15):
                            object_pose = len(task[2]) + 1
                        task[2].append([object_type, object_pose])

                # if no create new task
                if (new):
                    object_pose = container
                    if (object_pose != 14) and (object_pose != 15):
                        object_pose = 1

                    tasks.append([destination, source, [[object_type, object_pose]]])

        else:
            rospy.logerr("[GTP]: Unknown test type")
            continue


        if (Test_state.Type == 3):
            tasks = scenario_optimization(tasks)

        rospy.loginfo("[GTP]: Received tasks:")
        table = SingleTable(tasks, "tasks")
        table.inner_heading_row_border = False
        print(table.table)

    # Test Execution

    # BNT
    if (Test_state.Type == 1):
	    for i in xrange(len(tasks)):
            task = tasks.pop(0)
            if (navi("point", task[0], task[1], task[2]):
                rospy.loginfo("[GTP]: task %d is done (BNT)", i)
            else:
                rospy.logerr("[GTP]: task %d is missed (BNT)", i)

    # BMT or BTT
    elif (Test_state.Type == 2) or (Test_state.Type == 3):
        while not rospy.is_shutdown():
            try:
                task = tasks.pop(0)
            except IndexError:
                break
            if not (navi("point", task[1], task[1]+"OR", 0):
                rospy.logerr("[GTP]: Can't get %s", task[1])
                continue









    rate.sleep()
    # W = 1       # W for weigth

    # sorted(D.items(), key=lambda (k, v): v[2])




    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #hello_str = "hello world %s" % rospy.get_time()
    #while not rospy.is_shutdown():
    #rate = rospy.Rate(10) # 10hz
    #rate.sleep()
