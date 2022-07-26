#include<iostream>
#include <fstream>
#include <cstdlib>
#include <sys/time.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <string.h>
#include <signal.h>
#include <algorithm>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "AGV.h"
#include "TaskPlanner.h"
#include "car_control/NavTask.h"
using namespace std;

int countLayer = 2; // the number of layers in one facade
int countLocation = 8; // the number of locations in one layer
int flagPre = -1;
vector<vector<double>>  dict; // for point mapping, as the dictionary
ros::Publisher marker_pub;
ros::Publisher cmdVelPub;
ros::Publisher cancel_pub;
AGV* agv;
visualization_msgs::Marker line_list;
TaskPoint taskCurr;
TaskPoint taskPer;
int AGVNavigationFlag;

void moveToTarget(TaskPoint& target, int flag);

void convertToTaskPoint(double x, double y, double z, double angle, int f1, int f2, int mode);

bool equal(const TaskPoint& Curr, const TaskPoint& Per);

void navFlagCallback(const geometry_msgs::PointConstPtr& msg) {
    AGVNavigationFlag = msg->x;
}

void navTaskCallback(const car_control::NavTaskConstPtr& msg) {
    // cout << " x: " << msg->x << " y: " << msg->y << " z: " << msg->z << endl;
    convertToTaskPoint(msg->x, msg->y, msg->z, msg->angle, msg->f1, msg->f2, msg->mode);
}

// void navFlagCallback(const geometry_msgs::PointConstPtr& msg) {

// }

int main(int argc, char** argv){
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    // ros::Subscriber navFlag_sub = nh.subscribe<geometry_msgs::Point>("/nav_flag", 10, navFlagCallback);
    ros::Subscriber navTask_sub = nh.subscribe<car_control::NavTask>("/nav_task", 1, navTaskCallback);
    ros::Subscriber Navigation_sub = nh.subscribe<geometry_msgs::Point>("/nav_flag", 1, navFlagCallback);
    agv = new AGV(nh);
    TaskPlanner* task = new TaskPlanner(4, 8, "/home/cyc/guihua/taskA1.txt", "/home/cyc/guihua/map.txt", -11, -0.8, 0, 90);
    vector<vector<TaskPoint>> taskPointGroup = task->getTaskPointGroup();

    geometry_msgs::Point p;

    agv->init_markers(line_list);
    for(int i = 0; i < taskPointGroup.size(); i++) {
        for(int j = 0; j < taskPointGroup[i].size(); j++) {
            cout << "TaskPoint[" + to_string(i) + "][" + to_string(j) + "]: " << " x: " << taskPointGroup[i][j].x << " y: " << taskPointGroup[i][j].y << " z: " << taskPointGroup[i][j].z << " angle: " << taskPointGroup[i][j].angle << " f1: " << taskPointGroup[i][j].f1          << " f2: " << taskPointGroup[i][j].f2 << " mode: " << taskPointGroup[i][j].mode<<endl;
            p.x = taskPointGroup[i][j].x;
            p.y = taskPointGroup[i][j].y;
            p.z = taskPointGroup[i][j].z;
            line_list.points.push_back(p);
        }
        cout << endl;
    }
    while(ros::ok()) {
        // if(AGVNavigationFlag == 1 && !equal(taskCurr, taskPer)) {
        //     moveToTarget(taskCurr);
        //     taskPer.x = taskCurr.x;
        //     taskPer.y = taskCurr.y;
        //     taskPer.z = taskCurr.z;
        //     taskPer.mode = taskCurr.mode;
        //     taskPer.f1 = taskCurr.f1;
        //     taskPer.f2 = taskCurr.f2;
        // }
        // else if(AGVNavigationFlag == 0){
        //     agv->cancelGoal();
        // }
        moveToTarget(taskCurr, AGVNavigationFlag);


        ros::spinOnce();
    }
    
    

    return 0;

}


void convertToTaskPoint(double x, double y, double z, double angle, int f1, int f2, int mode) {
    taskCurr.x = x; taskCurr.y = y; taskCurr.z =z;
    taskCurr.angle = angle;
    taskCurr.f1 = f1;
    taskCurr.f2 = f2;
    taskCurr.mode = mode;
}

void moveToTarget(TaskPoint& target, int flag){
    agv->moveToTarget(target,line_list, flag);
    // if(!equal(taskCurr, taskPer) || flagCurr != flagPre) {
    //     cout << "ddddddd" << endl;
        
        // taskPer.x = taskCurr.x;
        // taskPer.y = taskCurr.y;
        // taskPer.z = taskCurr.z;
        // taskPer.mode = taskCurr.mode;
        // taskPer.f1 = taskCurr.f1;
        // taskPer.f2 = taskCurr.f2;
    //     flagPre = flagCurr;
    // }
    return;
}

bool equal(const TaskPoint& Curr, const TaskPoint& Per) {
    if(Curr.x == Per.x && Curr.y == Per.y && Curr.z == Per.z && Curr.mode == Per.mode && Curr.f1 == Per.f1 && Curr.f2 == Per.f2) {
        return true;
    }
    return false;
}