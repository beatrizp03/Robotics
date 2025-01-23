#!/usr/bin/env python
import message_filters
import rospy
import geometry_msgs.msg
import nav_msgs.msg


def callback(ground):
    print("Correcting...")
    
    msg = geometry_msgs.msg.PoseWithCovarianceStamped()


    msg.header.seq = ground.header.seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.pose.pose.position.x = ground.pose.pose.position.x
    msg.pose.pose.position.y = ground.pose.pose.position.y
    msg.pose.pose.position.z = 0.0
    
    msg.pose.pose.orientation.x = ground.pose.pose.orientation.x
    msg.pose.pose.orientation.y = ground.pose.pose.orientation.y
    msg.pose.pose.orientation.z = ground.pose.pose.orientation.z
    msg.pose.pose.orientation.w = ground.pose.pose.orientation.w
    
    for i in range(0, 6):
        j = i * 6 + i
        msg.pose.covariance[j] = 1e-5
    
    
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('correct_EKF')
    ground_sub = rospy.Subscriber('ground_truth_stamped', geometry_msgs.msg.PoseWithCovarianceStamped, callback)
   

    pub = rospy.Publisher('set_pose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
    
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        rate.sleep()
