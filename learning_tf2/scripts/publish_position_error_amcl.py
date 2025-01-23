#!/usr/bin/env python
import message_filters
import rospy
import geometry_msgs.msg
import nav_msgs.msg
import math


def callback(ground, amcl):
    global err_sqr, N
    
    msg = geometry_msgs.msg.PoseWithCovariance()

    err_x = ground.pose.position.x - amcl.pose.pose.position.x
    err_y = ground.pose.position.y - amcl.pose.pose.position.y
    
    err_sqr += math.sqrt(err_x*err_x + err_y*err_y)
    N += 1
	
    print("MEAN SQUARED ERROR:\t", err_sqr/N)

    msg.pose.position.x = ground.pose.position.x - amcl.pose.pose.position.x
    msg.pose.position.y = ground.pose.position.y - amcl.pose.pose.position.y
    msg.pose.position.z = 0.0
    
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 0.0
    
    #msg.covariance = amcl.pose.covariance
    msg.covariance = [math.sqrt(x) if x >= 0 else 0 for x in amcl.pose.covariance]
    
    pub.publish(msg)

if __name__ == '__main__':
    global err_sqr, N
    
    err_sqr = 0.0
    N = 0
    
    rospy.init_node('publish_position_error_amcl')
    ground_sub = message_filters.Subscriber('ground_truth', geometry_msgs.msg.PoseWithCovariance)
    amcl_sub = message_filters.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped)


    ts = message_filters.ApproximateTimeSynchronizer([ground_sub, amcl_sub], 100, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    
    pub = rospy.Publisher('position_error_amcl', geometry_msgs.msg.PoseWithCovariance, queue_size=1)
    
    rate = rospy.Rate(30.0)

    while not rospy.is_shutdown():
        rate.sleep()
