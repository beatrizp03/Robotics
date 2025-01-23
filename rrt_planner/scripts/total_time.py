#!/usr/bin/env python

import rospy
from actionlib_msgs.msg import GoalStatusArray
#from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

start_time = None

def goal_callback(msg):
    global start_time
    start_time = rospy.Time.now()
    rospy.loginfo("New goal received, starting timer.")

def status_callback(msg):
    global start_time
    
    if start_time and msg.status_list[-1].status == 3:  # 3 = SUCCEEDED
    #if start_time and any(status.status == 3 for status in msg.status_list):  # 3 = SUCCEEDED
        end_time = rospy.Time.now()
        duration = (end_time - start_time).to_sec()
        rospy.loginfo("Plan completed in %.4f seconds", duration)
        start_time = None  # Reset timer

rospy.init_node('plan_timing_node')

#rospy.Subscriber("/move_base_simple/goal", MoveBaseActionGoal, goal_callback)
rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)

rospy.spin()
