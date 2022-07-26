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

ros::Publisher Rela_pub;
geometry_msgs::Point rela;
int flag = 1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "rela");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.2);

    Rela_pub = nh.advertise<geometry_msgs::Point>("/rela",100);

    while(ros::ok()) {
        if(flag == 0) {
            rela.x = 0;
            rela.y = 0;
            flag = 1;
        }
        else if(flag == 1) {
            rela.x = 10;
            rela.y = 10;
            flag = 0;
        }
        Rela_pub.publish(rela);
        loop_rate.sleep();
    }
}