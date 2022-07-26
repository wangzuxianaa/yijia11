#include <iostream>
#include <fstream>
#include <cstdlib>
#include <sys/time.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <queue>
#include "TaskPlanner.h"
#include "AGV.h"
#include "common.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <car_control/NavTask.h>
#include <car_control/Mode.h>
#include <car_control/Position.h>
#include <car_control/UAVFinshedFlag.h>
#include <car_control/UAVHeight.h>
using namespace std;

ros::Publisher UAVfinished_pub;
ros::Publisher UAVheight_pub;
int UAVmode;
MapPoint AGVCurr;
PositionVector Pr1;
double UAVtargetheight;
car_control::UAVHeight height;



void modeCallback(const car_control::ModeConstPtr& msg) {
    UAVmode = msg->mode;
    ROS_INFO("received mode: %d", msg->mode);
}

void positionCallback(const car_control::PositionConstPtr& msg) {
    AGVCurr.x = msg->AGVx;
    AGVCurr.y = msg->AGVy;
    AGVCurr.angle = msg->AGVyaw;
    Pr1.x = msg->prx;
    Pr1.y = msg->pry;
    UAVtargetheight = msg->z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv,"UAV");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    UAVfinished_pub = nh.advertise<car_control::UAVFinshedFlag>("/uavfinishedflag",100);
    UAVheight_pub = nh.advertise<car_control::UAVHeight>("/uavheight",100);

    ros::Subscriber mode_sub = nh.subscribe<car_control::Mode>("/mode",100,modeCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber position_sub = nh.subscribe<car_control::Position>("/position",100,positionCallback,ros::TransportHints().tcpNoDelay());
    while(ros::ok()) {
        height.UAVheight = 1;
        UAVheight_pub.publish(height);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}