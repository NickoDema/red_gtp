#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from atwork_ros_msgs.msg import TaskInfo

def talker():
    rospy.init_node('talker')#, anonymous=True)
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        task_msg_first  = rospy.wait_for_message("/robot_example_ros/task_info", TaskInfo)
        while 1:
            task_msg_second = rospy.wait_for_message("/robot_example_ros/task_info", TaskInfo)
            if check_task(task_msg_first, task_msg_second):
                task_msg = task_msg_first
                break
            task_msg_first = task_msg_second

        for task in task_msg.tasks:
            rospy.loginfo(task.transportation_task.object.type)
        rate.sleep()

def check_task(first_task, second_task):
    return True

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
