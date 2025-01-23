#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('publish_mocap_tf_stamped')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    

    pub = rospy.Publisher('ground_truth_stamped', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            #trans = tfBuffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time())
            trans = tfBuffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time())
            #print(trans.transform.translation.x)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.PoseWithCovarianceStamped()

        #msg.header.seq = ground.header.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = trans.transform.translation.x
        msg.pose.pose.position.y = trans.transform.translation.y
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = trans.transform.rotation.x
        msg.pose.pose.orientation.y = trans.transform.rotation.y
        msg.pose.pose.orientation.z = trans.transform.rotation.z
        msg.pose.pose.orientation.w = trans.transform.rotation.w
        
        for i in range(0, 6):
            j = i * 6 + i
            msg.pose.covariance[j] = 1e-5
        
        #print("Correction from publish_mocap_tf_stamped...")

        pub.publish(msg)

        rate.sleep()
