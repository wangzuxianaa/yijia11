#ifndef __AGV_H__
#define __AGV_H__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <string.h>
#include <string>
#include <iostream>
#include "common.h"
#include <car_control/Position.h>
using namespace std;



class AGV {
private:
    ros::NodeHandle m_nh;
    ros::Publisher marker_pub;
    ros::Publisher cmdVelPub;
    ros::Publisher AGVCurr_pub;
    ros::Publisher cancelGoal_pub;
    geometry_msgs::Quaternion quaternions[4];
    car_control::Position m_AGVCurr;
    
public:
    AGV(ros::NodeHandle& nh);

    void init();

    void moveToTarget(TaskPoint target, visualization_msgs::Marker &marker, int flag);

    void init_markers(visualization_msgs::Marker &marker);

    void GetAGVCurr();

    void cancelGoal();
};
#endif