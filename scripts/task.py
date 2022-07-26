#!/usr/bin/env python
# -*- coding: utf-8 -*-


""" move_base_square.py - Version 1.1 2013-12-20

    Command a robot to move in a square using move_base actions..

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.htmlPoint
      
"""

import tf
import rospy
import actionlib
import json
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from transform_utils import quat_to_angle, normalize_angle
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, copysign, sqrt, pow, pi


class MoveBaseTask():  
    def __init__(self): 
        # 读取文本并转化为相应的格式,nav[i]表示第i个搜索任务,nav[i][0]表示第i个任务的搜索立面，
        # nav[i][1]表示第i个任务的搜索层，nav[i][2]表示第i个任务的搜索库位   
        nav = list() 
        with open("/home/cyc/1.txt","r") as f:
            for line in f.readlines():
                line = line.strip('\n')
                line_tuple = tuple([int(n) for n in line.split(",")])
                nav.append(line_tuple)
        nav.sort(key=lambda x:(x[0], x[1], x[2]))
        rospy.loginfo("需要导航进行搜寻的点有" + str(nav))
        
        rospy.init_node('nav_task', anonymous=False)
        nodename = rospy.get_name()
        rospy.loginfo("%s started" %nodename)
        rospy.on_shutdown(self.shutdown)
        
        # tf转换
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
        
        quaternions = list()
    
        # 欧拉角
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # 欧拉角转四元数
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)
        
        # list存map库位点，dic存map点和立面库位的映射,
        waypoints = list()
        dic = dict()
        start_to_end = dict()

        # 存取搜索整个立面或者某一层的导航的初始点,这些点是map坐标系下的
        waypoints.append(Pose(Point(11.0, -0.8, 0.0), quaternions[0]))
        waypoints.append(Pose(Point(11.0, 0.8, 0.0), quaternions[1]))
        waypoints.append(Pose(Point(-11.0, 0.8, 0.0), quaternions[2]))
        waypoints.append(Pose(Point(-11.0, -0.8, 0.0), quaternions[3]))
        waypoints.append(Pose(Point(-11.0, -7.4, 0.0), quaternions[3]))
        waypoints.append(Pose(Point(11.0, -5.8, 0.0), quaternions[1]));
        
        for i in range(8):
            # 1号立面起点
            dic.update({(1,i,-1):Pose(Point(11.0, 0.8, 0.0), quaternions[1])})
            # 1号立面终点
            dic.update({(1,i,-2):Pose(Point(-11.0, 0.8, 0.0), quaternions[1])})
            # 2号立面起点
            dic.update({(2,i,-1):Pose(Point(-11.0, -0.8, 0.0), quaternions[3])})
            # 2号立面终点
            dic.update({(2,i,-2):Pose(Point(11.0, -0.8, 0.0), quaternions[3])})
            # 3号立面起点
            dic.update({(3,i,-1):Pose(Point(11.0, -5.5, 0.0), quaternions[1])})
            # 3号立面终点
            dic.update({(3,i,-2):Pose(Point(-11.0, -5.5, 0.0), quaternions[1])})
            # 4号立面起点
            dic.update({(4,i,-1):Pose(Point(11.0, -6.8, 0.0), quaternions[3])})
            # 4号立面终点
            dic.update({(4,i,-2):Pose(Point(-11.0, -6.8, 0.0), quaternions[3])})
        
            start_to_end.update({(1,i,-1):(1,i,-2)})
            start_to_end.update({(2,i,-1):(2,i,-2)})
            start_to_end.update({(3,i,-1):(3,i,-2)})
            start_to_end.update({(4,i,-1):(4,i,-2)})
            start_to_end.update({(1,i,-2):(1,i,-1)})
            start_to_end.update({(2,i,-2):(2,i,-1)})
            start_to_end.update({(3,i,-2):(3,i,-1)})
            start_to_end.update({(4,i,-2):(4,i,-1)})

        # 将所有盘货点录入
        for i in range(8):
            waypoints.append(Pose(Point(-11 + (i + 1) * 2.5, 0.8, 0.0), quaternions[1]))
            waypoints.append(Pose(Point(-11 + (i + 1) * 2.5, -0.8, 0.0), quaternions[3]))
            waypoints.append(Pose(Point(-11 + (i + 1) * 2.5, -5.8, 0.0), quaternions[1]))
            waypoints.append(Pose(Point(-11 + (i + 1) * 2.5, -7.4, 0.0), quaternions[3]))
            # j为层数
            for j in range(8):
                # 1号立面各个库位
                dic.update({(1,j,i+1):Pose(Point(-11 + (i + 1) * 2.5, 0.8, 0.0), quaternions[1])})
                # 2号立面各个库位
                dic.update({(2,j,i+1):Pose(Point(-11 + (i + 1) * 2.5, -0.8, 0.0), quaternions[3])})
                # 3号立面各个库位
                dic.update({(3,j,i+1):Pose(Point(-11 + (i + 1) * 2.5, -5.5, 0.0), quaternions[1])})
                # 4号立面各个库位
                dic.update({(4,j,i+1):Pose(Point(-11 + (i + 1) * 2.5, -6.8, 0.0), quaternions[3])})
        
        # rviz可视化
        self.init_markers()     
        for waypoint in waypoints:           
            p = Point()
            p = waypoint.position
            self.markers.points.append(p)
            
        # 发布者
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # 订阅move_base相关服务
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
        
        
        # 输入转列表
        for i in range(len(nav)):
            # rospy.loginfo("输入立面，层数，库位号（输入格式：立面，层数，库位号）" )
            # x = raw_input()
            # xlist = tuple([int(n) for n in x.split(",")])
            
            # 需要搜索整个立面
            if nav[i][2] == -1:
                rospy.loginfo("搜索整个立面,开始前往相应货架的点!!!")
                # 更新显示
                self.marker_pub.publish(self.markers)
                # 初始化目标点
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                (position, rotation) = self.get_map_pose()
                distance1 = sqrt(pow(position.x - dic.get(nav[i]).position.x, 2) + pow(position.y - dic.get(nav[i]).position.y, 2))
                distance2 = sqrt(pow(position.x - dic.get(start_to_end.get(nav[i])).position.x, 2) + pow(position.y - dic.get(start_to_end.get(nav[i])).position.y, 2))
                # 货架的两个端点谁距离小车近，就前往哪边
                if distance1 < distance2:
                    goal.target_pose.pose = dic.get(nav[i])
                else:
                    goal.target_pose.pose = dic.get(start_to_end.get(nav[i]))
                self.move(goal)
                
                rospy.loginfo("已经到达相应的点，是否开启导航搜索整个立面(开启搜索为1)")
                a = input()
                
                if a == 1:
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    if distance1 < distance2:
                        goal.target_pose.pose = dic.get(start_to_end.get(nav[i]))
                    else:
                        goal.target_pose.pose = dic.get(nav[i])
                    self.move(goal)
                else:
                    self.shutdown()
                
            # 搜索具体某个库位 
            else:
                rospy.loginfo("开始导航前往" + str(nav[i][0]) + "号立面" + str(nav[i][1]) + "层货架" + str(nav[i][2]) + "号库位")
                # 更新显示
                self.marker_pub.publish(self.markers)
                # 初始化目标点
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()

                goal.target_pose.pose = dic.get(nav[i]);
                self.move(goal)
        
    def move(self, goal):
            # Send the goal pose to the MoveBaseAction server
            self.move_base.send_goal(goal)
            
            # Allow 1 minute to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(1000)) 
            
            # If we don't get there in time, abort the goal
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("超时了！！!")
            else:
                # We made it!
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("导航成功!到达目的点!")
                    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
    
    def get_map_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseTask()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航任务结束")