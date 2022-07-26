#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tf
import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from math import radians, copysign, sqrt, pow, pi
from transform_utils import quat_to_angle
from car_control.msg import Yaw

class AngelPublish():
    def __init__(self):
        
        rospy.init_node('yaw_publisher', anonymous=False)
        yaw_info_pub = rospy.Publisher('/yaw_info', Yaw, queue_size = 10)
        rate = rospy.Rate(10)
        
        nodename = rospy.get_name()
        rospy.loginfo("%s started" %nodename)
        
        self.tf_listener = tf.TransformListener()
        self.map_frame = "/map"
        try:
            self.tf_listener.waitForTransform(self.map_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.map_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        while not rospy.is_shutdown():
            (position, rotation) = self.get_map_pose()
            Yaw_msg = Yaw()
            Yaw_msg.x = position.x
            Yaw_msg.y = position.y
            Yaw_msg.z = position.z
            Yaw_msg.yaw = rotation / (2 * pi) * 360
            
            yaw_info_pub.publish(Yaw_msg)
            
            rate.sleep()
        
    
    def get_map_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
            
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        
    
if __name__ == '__main__':
    try:
        AngelPublish()
    except rospy.ROSInterruptException:
        pass
    