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
#include "AGV.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "AGV");
    ros::NodeHandle nh;
    shared_ptr<AGV> agv(new AGV(nh));
    // AGV* agv = new AGV(nh);
    agv->GetAGVCurr();
    return 0;
}