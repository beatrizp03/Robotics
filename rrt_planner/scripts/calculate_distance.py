#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
import math

class DistanceCalculator:
    def __init__(self):
        # Initialize the node
        rospy.init_node('distance_calculator')

        # Variables to store position
        self.last_x = None
        self.last_y = None
        self.total_distance = 0.0

        # Subscribe to /odom to track position
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscribe to /move_base/status to check if the goal is reached
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.goal_callback)

    def odom_callback(self, msg):
        # Get current position from odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Check if we have received a previous position
        if self.last_x is None or self.last_y is None:
            self.last_x = current_x
            self.last_y = current_y
            return

        # Calculate distance between current and last position
        dx = current_x - self.last_x
        dy = current_y - self.last_y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        # Accumulate total distance
        self.total_distance += distance

        # Update last position
        self.last_x = current_x
        self.last_y = current_y

    def goal_callback(self, msg):
        # Check if the robot has reached the goal (status == 3 means SUCCEEDED)
        if msg.status.status == 3:  # SUCCEEDED
            rospy.loginfo("Goal reached. Total distance traveled: {:.4f} meters".format(self.total_distance))
            rospy.signal_shutdown("Goal reached. Distance calculated.")

    def start(self):
        # Set the rate at which we process callbacks
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the DistanceCalculator class and start it
        distance_calculator = DistanceCalculator()
        distance_calculator.start()
    except rospy.ROSInterruptException:
        pass
