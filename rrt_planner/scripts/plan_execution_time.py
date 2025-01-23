#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
#from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

start_time = None
moving = False

# Define thresholds for movement (in m/s and rad/s)
LINEAR_VELOCITY_THRESHOLD = 0.05  # For example, 1 cm/s
ANGULAR_VELOCITY_THRESHOLD = 0.05  # For example, small rotations

def odom_callback(msg):
    global moving, start_time

    # Get the robot's current linear and angular velocity
    linear_velocity = msg.twist.twist.linear.x
    angular_velocity = msg.twist.twist.angular.z

    # Check if the robot is moving using threshold intervals
    if start_time and (abs(linear_velocity) > LINEAR_VELOCITY_THRESHOLD or abs(angular_velocity) > ANGULAR_VELOCITY_THRESHOLD):
        #start_time = rospy.Time.now()
        moving = True
        
        end_time = rospy.Time.now()
        duration = (end_time - start_time).to_sec()
        rospy.loginfo("Robot started executing plan after %.4f seconds", duration)
        rospy.signal_shutdown("----------------------------------------------------")


def goal_callback(msg):
    global start_time
    start_time = rospy.Time.now()
    rospy.loginfo("New goal received, starting timer.")


def main():
    rospy.init_node('movement_time_tracker', anonymous=True)

    # Subscribe to odometry to detect when the robot starts moving
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # Subscribe to move_base status to detect when the goal is reached
    #rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)
    
    # Subscribe to see when goal is called
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
