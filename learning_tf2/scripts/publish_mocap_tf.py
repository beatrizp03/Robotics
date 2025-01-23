#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('publish_mocap_tf')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    

    pub = rospy.Publisher('ground_truth', geometry_msgs.msg.PoseWithCovariance, queue_size=1)

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            #trans = tfBuffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time())
            trans = tfBuffer.lookup_transform('map', 'mocap_laser_link', rospy.Time())
            #print(trans.transform.translation.x)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.PoseWithCovariance()

        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = 0.0
        
        msg.pose.orientation.x = trans.transform.rotation.x
        msg.pose.orientation.y = trans.transform.rotation.y
        msg.pose.orientation.z = trans.transform.rotation.z
        msg.pose.orientation.w = trans.transform.rotation.w
        
        for i in range(0, 6):
            j = i * 6 + i
            msg.covariance[j] = 1e-9

        #print(msg)

        pub.publish(msg)

        rate.sleep()
