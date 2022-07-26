#ifndef _MOVE_BASE_NAV_H_
#define _MOVE_BASE_NAV_H_

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class MoveBase 
{
    public:
        void init_markers(visualization_msgs::Marker *marker);
        void move(move_base_msgs::MoveBaseGoal& goal);
};



#endif // _MOVE_BASE_NAV_H_