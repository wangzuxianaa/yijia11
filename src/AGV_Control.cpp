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
<<<<<<< HEAD
#include <car_control/UAVInfo.h>
#define PAI 3.14159265
=======
#include <car_control/UAVHeight.h>
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
using namespace std;





MapPoint AGVCurr;     // AGV current position
PositionVector Pr1;     // the same in the flow chart, means the relation position between AGV and UAV
double AGVYaw;           // current yaw of AGV
bool hasAdjusted;    // the flag to show whether UAV has finished adjust
bool UAVM4Finished; // the flag to show whether UAV has finished recognize the barcode when mode = 4
bool AGVKeep;             // AGV keeps and does not move
bool AGVCameraIsOn; // the flag to show whether the camera node in AGV is started
bool UAVCameraIsOn; // the flag to show whether the camera node in UAV is started
bool AGVHasReco;      // the flag to show whether the camera node in AGV can detect the barcode in the fixed point
bool AGVTpFinished;  // the flag to show whether AGV has reached the target taskPoint
int AGVNavigationFlag;
int countAGVRecoMax; // the maximum allowed times for AGV to wait for the recognize the barcode in mode 5
int count;
double UAVheight; // the real height of UAV
<<<<<<< HEAD
double UAVyaw;
queue<TaskPoint> taskPointStream; // the queue of taskpoint
visualization_msgs::Marker line_list; // the visualization of line_list
ros::Publisher Navigation_pub; 
=======
queue<TaskPoint> taskPointStream; // the queue of taskpoint
visualization_msgs::Marker line_list; // the visualization of line_list
ros::Publisher Navigation_pub;
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
ros::Publisher NavTask_pub;
ros::Publisher mode_pub;
ros::Publisher position_pub;
ros::Subscriber UAVfinished_sub;
<<<<<<< HEAD
ros::Subscriber UAVinfo_sub;
ros::Subscriber AGVCurr_sub;
ros::Subscriber Pr_sub;
=======
ros::Subscriber UAVheight_sub;
ros::Subscriber AGVCurr_sub;
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
geometry_msgs::Point point;
car_control::NavTask navTask;
car_control::Mode AGV_mode;
car_control::Position UAV_neededInfo;
TaskPoint taskPointCurr;  // current tastPoint
geometry_msgs::Point nav;




void convertToTask(TaskPoint taskPointCurr);


// move the AGV to taskPoint
bool AGVTotarget(TaskPoint& taskPointCurr);

// to check whether the distance between mp and tp is less than the threshold
bool checkDis(MapPoint mp, TaskPoint tp);

//to check whether UAV and AGV is coordinate in x and y
//we only need to check x-axis and y-axis
bool checkRela(PositionVector Pr1);

<<<<<<< HEAD
bool checkUAVYaw(double UAVYaw, double AGVYaw);
=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35

bool checkUAVHeight(double z, double UAVHeight);

//publish the information to UAV
void publishUAVTarget(MapPoint AGVCurr, PositionVector Pr1, double height);

//publish mode to UAV
void publishMode(int mode);

void publishUAVCamera(int f1);

//stop AGV
void stopAGV();

//control whether we need to start the camera in AGV when f1 = 1 or to stop the camera when f1 = 0
void publishAGVCamera(int f1);

//obtain the smoot target distance
MapPoint  obtainSmoothTarget(MapPoint AGVCurr, TaskPoint taskPointCurr);

// make AGV to next smooth target
void moveToTarget(TaskPoint& target);

// make UAV to land
void publishUAVLand();

//control whether we need to start the camera in UAV or to stop the camera for fixed point recognize
void publishUAVReco(int a);

<<<<<<< HEAD
////
//control whether we need to start the camera in AGV or to stop the camera for fixed point recognize
void publishAGVReco(int a);

void uavInfoCallback(const car_control::UAVInfoConstPtr& msg);
=======
//control whether we need to start the camera in AGV or to stop the camera for fixed point recognize
void publishAGVReco(int a);

void uavHeightCallback(const car_control::UAVHeightConstPtr& msg);
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35

void uavFinishedCallback(const car_control::UAVFinshedFlagConstPtr& msg);

// timer callback fuction, usually the frequency is 20Hz
void timer_callback(const ros::TimerEvent& event);

void agvCurrCallback(const car_control::PositionConstPtr& msg);

<<<<<<< HEAD
void prCallback(const geometry_msgs::PointConstPtr& msg);

=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35

int main(int argc, char** argv){

    //the same variable in taskPlan.cpp
    //here we need to obtain this variable base on taskPlan.cpp for the first step

    
    ros::init(argc, argv, "nav_task");
    ros::NodeHandle nh;

    Pr1.x = 0;
    Pr1.y = 0;

    Navigation_pub = nh.advertise<geometry_msgs::Point>("/nav_flag", 100);
    NavTask_pub = nh.advertise<car_control::NavTask>("/nav_task", 100);
    mode_pub = nh.advertise<car_control::Mode>("/mode", 100);
    position_pub = nh.advertise<car_control::Position>("/position", 100);

    UAVfinished_sub = nh.subscribe<car_control::UAVFinshedFlag>("/uavfinishedflag",100, uavFinishedCallback);
<<<<<<< HEAD
    UAVinfo_sub = nh.subscribe<car_control::UAVInfo>("/uavinfo",100,uavInfoCallback);
    AGVCurr_sub = nh.subscribe<car_control::Position>("/AGVCurr",100, agvCurrCallback);
    Pr_sub = nh.subscribe<geometry_msgs::Point>("/rela",100, prCallback);
    
    // shared_ptr<TaskPlanner> task(new TaskPlanner(4, 8, "/home/cyc/guihua/taskA1.txt", "/home/cyc/guihua/map.txt", 
    //                                             -11, -0.8, 0, 90));
=======
    UAVheight_sub = nh.subscribe<car_control::UAVHeight>("/uavheight",100,uavHeightCallback);
    AGVCurr_sub = nh.subscribe<car_control::Position>("/AGVCurr",100, agvCurrCallback);
    
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    TaskPlanner* task = new TaskPlanner(4, 8, "/home/cyc/guihua/taskA1.txt", "/home/cyc/guihua/map.txt", -11, -0.8, 0, 90);
    
    vector<vector<TaskPoint>> taskPointGroup = task->getTaskPointGroup();


    //convert taskPointGroup to 1-D vector and the variable type is queue
    for(int i = 0; i < taskPointGroup.size(); i++){
        for(int j = 0; j < taskPointGroup[i].size(); j++){
            taskPointStream.push(taskPointGroup[i][j]);
        }
    }
    

    //wait for the initialization of AGV
    //flag for initialization
    int countInit = 0;
    while(countInit < 1000){
        countInit++;
    }
    // cout << "initialization success !!!" << endl;
    ROS_INFO("initialization success !!!");

    //send command to UAV to take off
    /*
    send command to take off
    */
    //wait for the take off of UAV
    //flag for taking off
    int countTakeOff = 0;
    while(countTakeOff < 1000){
        countTakeOff++;
    }


    ros::Timer time1 = nh.createTimer(ros::Duration(0.05), timer_callback);

    taskPointCurr = taskPointStream.front();
    taskPointStream.pop();
    publishMode(taskPointCurr.mode); 

     
<<<<<<< HEAD
=======

>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    

    //the main task
    //here we go into the timer_callback function
    ros::spin();
    return 0;
}

<<<<<<< HEAD
void uavInfoCallback(const car_control::UAVInfoConstPtr& msg) {
    UAVheight = msg->UAVheight;
    UAVyaw = msg->UAVyaw;
=======
void uavHeightCallback(const car_control::UAVHeightConstPtr& msg) {
    UAVheight = msg->UAVheight;
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
}

void uavFinishedCallback(const car_control::UAVFinshedFlagConstPtr& msg) {
    UAVM4Finished = msg->UAVfinishedflag;
    if (UAVM4Finished == 1) {
        ROS_INFO("UAV has finished the process");
    }
}

<<<<<<< HEAD
void prCallback(const geometry_msgs::PointConstPtr& msg) {
    Pr1.x = msg->x;
    Pr1.y = msg->y;
}

=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
void agvCurrCallback(const car_control::PositionConstPtr& msg) {
    AGVCurr.x = msg->AGVx;
    AGVCurr.y = msg->AGVy;
    AGVCurr.angle = msg->AGVyaw;
}

void timer_callback(const ros::TimerEvent& event){
    // AGVCurr = agv->GetAGVCurr();
    cout << "-----------------------------------------------------------------------------------------------" << endl;
<<<<<<< HEAD
    ROS_INFO("AGV.x: %f     AGV.y: %f    AGV.yaw: %f", AGVCurr.x,AGVCurr.y, AGVCurr.angle);
=======
    ROS_INFO("AGV.x: %f     AGV.y: %f", AGVCurr.x,AGVCurr.y);
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    // cout << "AGV.x: " << AGVCurr.x << " AGV.y: " << AGVCurr.y << endl;
    //for each mode, we do the right operation
    switch (taskPointCurr.mode){
        case 1:
            publishMode(taskPointCurr.mode);
            //in this mode, here we only need to make AGV move to the target postion
            AGVTpFinished = AGVTotarget(taskPointCurr);
            if (AGVTpFinished == true) {
                taskPointCurr = taskPointStream.front();
                taskPointStream.pop();
            }
            break;
        case 2:
            publishMode(taskPointCurr.mode);
            //in this mode, we only need to wait until UAV has finished adjust
            //hashAdjusted is a flag from UAV so that AGV can know whether UAV has finished adjust
            // convertToTask(taskPointCurr);
            // NavTask_pub.publish(navTask);
            if(!hasAdjusted){
                publishUAVTarget(AGVCurr, Pr1, taskPointCurr.z);
                // ros::Duration(4).sleep();
<<<<<<< HEAD
                if(checkUAVHeight(taskPointCurr.z, UAVheight) && checkRela(Pr1) && checkUAVYaw(UAVyaw,  AGVCurr.angle))
                // if(true)
=======
                // if(checkUAVHeight(TaskPointCurr.z, UAVheight) && checkRela(Pr1))
                if(true)
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
                    hasAdjusted = true;
            }
            else{
                //obtain next taskPoint
                hasAdjusted = false;
                taskPointCurr = taskPointStream.front();
                taskPointStream.pop();
                //publish the next mode to UAV
                // ros::Duration(2).sleep();
            }
            break;
        case 3:
            publishMode(taskPointCurr.mode);
            //if the f1 = 1, we need to start the camera in AGV
            if(!UAVCameraIsOn && taskPointCurr.f2 == 1) {
                publishUAVCamera(1);
                UAVCameraIsOn = true;
            }

            if(!AGVCameraIsOn && taskPointCurr.f1 == 1){
                publishAGVCamera(1);
                AGVCameraIsOn = true;
            }
            //next, we need to make AGV move to the target postion
            AGVTpFinished = AGVTotarget(taskPointCurr);
            //if AGV has reached the target position, we need to stop camera node and pop next task point
            if(AGVTpFinished){
                taskPointCurr = taskPointStream.front();
                taskPointStream.pop();
                if(AGVCameraIsOn && taskPointCurr.f1 == 1){
                    publishAGVCamera(0);
                    AGVCameraIsOn = false;
                }
                if(UAVCameraIsOn && taskPointCurr.f2 == 1) {
                    publishUAVCamera(0);
                    UAVCameraIsOn = false;
                }
            }
            break;
        case 4:
            publishMode(taskPointCurr.mode);
            //the variable AGVKeep is important, details will be presented in Chinese txt
            //here, firstly, we need to move AGV to the target position
            if(!AGVKeep){
                AGVTpFinished = AGVTotarget(taskPointCurr);
                if(AGVTpFinished){
                    AGVKeep = true;
                    publishUAVReco(1);
                }
                break;
            }
            // cout <<  "Waiting for UAV finishing the process" << endl;
            ROS_INFO("Waiting for UAV finishing the process");
            //remember to update UAVM4Finished in UAV preocess
            if(UAVM4Finished){
                publishUAVReco(0);
                taskPointCurr = taskPointStream.front();
                taskPointStream.pop();
                //update AGVKeep for the next time
                AGVKeep = false;
            }
            break;
            
            
        case 5:
            cout << " " << endl;
            publishMode(taskPointCurr.mode);
            //here, firstly, we need to move AGV to the target position
            if(!AGVKeep){
                AGVTpFinished = AGVTotarget(taskPointCurr);
                if(AGVTpFinished){
                    AGVKeep = true;
                    publishAGVReco(1);
                }
                break;
            }
            ros::Duration(2).sleep();
            static int countAGVReco = 0;
            if(!AGVHasReco){
                //if time is out, we don't need to recognize this point
                if(countAGVReco > countAGVRecoMax){
                    publishAGVReco(0);
                    taskPointCurr = taskPointStream.front();
                    taskPointStream.pop();
                    publishMode(taskPointCurr.mode);
                    AGVKeep = false;
                }
                countAGVReco++;
                break;
            }
            //reserve for the logic!!!!!!!!!!
            taskPointCurr = taskPointStream.front();
            taskPointStream.pop();
            AGVKeep = false;
            publishAGVReco(0);
            break;
        case 6:
            cout << " " << endl;
            //rememeber to add the final taskPoint where position is origion f1 = 0, f2 = 0, mode = 6
            //in this case, we only need to make AGV back to the origin and make UAV land
            AGVTotarget(taskPointCurr);
            publishUAVLand();
            break;
        default :
            cout << "error for mode !!!" << endl;
    }
}

bool AGVTotarget(TaskPoint& taskPointCurr) {
    //to check whether AGV has reached the target map point
    bool hasReachedTarget = checkDis(AGVCurr, taskPointCurr);
    //to check whether UAV has reached the target height
    bool hasReachedHeight = checkUAVHeight(taskPointCurr.z, UAVheight);
    //to check whether UAV has flyed right above AGV
    bool hasConsistent = checkRela(Pr1);
    if(hasReachedTarget) {
        //stop the AGV
        stopAGV();
        // cout << "AGV has reached the goal" << endl;
        ROS_INFO("AGV has reached the goal");
        if(hasConsistent && hasReachedHeight){
            publishUAVTarget(AGVCurr, Pr1, taskPointCurr.z);
            nav.x = 1;
            Navigation_pub.publish(nav);
            convertToTask(taskPointCurr);
            NavTask_pub.publish(navTask);
            return true;
        }
        else{
            nav.x = 0;
            Navigation_pub.publish(nav);
            //publish the fly information to UAV
            publishUAVTarget(AGVCurr, Pr1, taskPointCurr.z);
        }
    }
    else{
        //first we get the smooth plan point, and make AGV move to smooth point instead of task point directly
        // MapPoint smoothTarget = obtainSmoothTarget(AGVCurr, taskPointCurr);
        // cout << "AGV has not reached the goal" << endl;
        ROS_INFO("AGV has not reached the goal");
        if(hasConsistent && hasReachedHeight) {
            nav.x = 1;
            Navigation_pub.publish(nav);
            publishUAVTarget(AGVCurr, Pr1, taskPointCurr.z);
            convertToTask(taskPointCurr);
            NavTask_pub.publish(navTask);
        }
        else {
            nav.x = 0;
            Navigation_pub.publish(nav);
            publishUAVTarget(AGVCurr, Pr1, taskPointCurr.z);   
        }
        
        //publish the fly information to UAV
    }
    return false;
}

// to check whether the distance between mp and tp is less than the threshold
bool checkDis(MapPoint mp, TaskPoint tp){
<<<<<<< HEAD
    if(mp.angle < 0)
        mp.angle += 2 * PAI;
    float dis;
    if(fabs(mp.angle - tp.angle / 180 * PAI) > PAI) {
        dis = 2 * PAI - fabs(mp.angle - tp.angle / 180 * PAI);
    } else {
        dis = fabs(mp.angle - tp.angle / 180 * PAI);
    }
    cout << dis << endl;
    if(fabs(mp.x -  tp.x) < 0.1 &&
          fabs(mp.y -  tp.y) < 0.1 && dis < 0.1)
=======
    if(fabs(mp.x -  tp.x) < 0.1 &&
          fabs(mp.y -  tp.y) < 0.1)
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
        return true;
    else
        return false;
}

//to check whether UAV and AGC is coordinate in x and y
//we only need to check x-axis and y-axis
bool checkRela(PositionVector Pr1){
    if(std::fabs(Pr1.x) < 0.1 && std::fabs(Pr1.y) < 0.1) {
        ROS_INFO("UAV has followed AGV");
        return true;
    }
    else {
        ROS_INFO("Waiting for UAV to follow AGV");
        return false;
    }
}

bool checkUAVHeight(double z, double UAVHeight) {
    if(fabs(z - UAVHeight) < 0.1) {
        ROS_INFO("UAV has reached the target height");
        return true;
    }
    else {
        ROS_INFO("Waiting for UAV to reach the correct height");
        return false;
    }
}

<<<<<<< HEAD
bool checkUAVYaw(double UAVYaw, double AGVYaw) {
    if(AGVYaw < 0) {
        AGVYaw += 2 * PAI;
    }
    if(fabs(UAVYaw - AGVYaw / PAI * 180) < 5) {
        ROS_INFO("UAV has reached the correct yaw");
        return true;
    }
    else {
        ROS_INFO("Waiting for UAV to reach the correct yaw");
        return false;
    }
}

=======
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
//publish the information to UAV
void publishUAVTarget(MapPoint AGVCurr, PositionVector Pr1, double height){
    UAV_neededInfo.AGVx = AGVCurr.x;
    UAV_neededInfo.AGVy = AGVCurr.y;
    UAV_neededInfo.AGVyaw = AGVCurr.angle;
    UAV_neededInfo.prx = Pr1.x;
    UAV_neededInfo.pry = Pr1.y;
    UAV_neededInfo.z = height;
    position_pub.publish(UAV_neededInfo);
    // cout << "publish AGVCurr, pr1, UAV height" << endl;
    ROS_INFO("publish AGVCurr, pr1, UAV height");
}

//publish mode to UAV
void publishMode(int mode){
    AGV_mode.mode = mode;
    mode_pub.publish(AGV_mode);
    // cout << "publish mode " << mode << " to UAV " << endl;
    ROS_INFO("publish mode %d to UAV", mode);
}

bool checkParam(MapPoint AGVCurr, double UAVheight, TaskPoint TaskPointCurr) {
    cout << "check AGV x,y and UAV z" << endl;
    if(fabs(AGVCurr.x - TaskPointCurr.x) < 0.1 && fabs(AGVCurr.y - TaskPointCurr.y) < 0.1) {
        cout << "AGV x, y is right" << endl;
    }
    else {
        cout << "AGV x, y is devated and ought to be solved" << endl;
        return false;
    }
    if(fabs(TaskPointCurr.z - UAVheight) < 0.1) {
        cout << "UAV height is right" << endl;
    }
    else {
        cout << "UAV height is devated and ought to be solved" << endl;
        return false;
    }
    return true;
}

//stop AGV
void stopAGV(){
    // cout << "stop AGV " << endl;
    ROS_INFO("Stop AGV...");
}

//control whether we need to start the camera in AGV when f1 = 1 or to stop the camera when f1 = 0
void publishAGVCamera(int f1){
    if(f1 == 1) 
        ROS_INFO("start the AGV camera node");
    else 
        ROS_INFO("stop the AGV camera node");
}

void publishUAVCamera(int f1) {
    if(f1 == 1) 
        ROS_INFO("start the UAV camera node");
    else 
        ROS_INFO("stop the UAV camera node");
}

//obtain the smoot target distance
MapPoint  obtainSmoothTarget(MapPoint AGVCurr, TaskPoint taskPointCurr){
    MapPoint smoothTarget;
    cout<< "obtain smooth target" << endl;
    return smoothTarget;
}

// make UAV to land
void publishUAVLand(){
    cout << "UAV land command" << endl;
}

//control whether we need to start the camera in UAV or to stop the camera for fixed point recognize
void publishUAVReco(int a){
    if(a == 1) 
        ROS_INFO("UAV starts to recognize barcode");
    else 
        ROS_INFO("UAV stops to recognize barcode");
}

//control whether we need to start the camera in AGV or to stop the camera for fixed point recognize
void publishAGVReco(int a){
    if(a == 1) 
        ROS_INFO("AGV starts to recognize barcode");
    else 
        ROS_INFO("AGV stops to recognize barcode");
}

void convertToTask(TaskPoint taskPointCurr) {
    navTask.x = taskPointCurr.x;
    navTask.y = taskPointCurr.y;
    navTask.z = taskPointCurr.z;
    navTask.angle = taskPointCurr.angle;
    navTask.f1 = taskPointCurr.f1;
    navTask.f2 = taskPointCurr.f2;
    navTask.mode = taskPointCurr.mode;
}