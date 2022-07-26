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
using namespace std;

int countLayer = 2; // the number of layers in one facade
int countLocation = 8; // the number of locations in one layer
int flag = 0;
vector<vector<double>>  dict; // for point mapping, as the dictionary
ros::Publisher marker_pub;
ros::Publisher cmdVelPub;



struct CommandPoint{
    int facade; 
    int layer; 
    int location; 
};


struct TaskPoint{
    double x, y, z, angle;
    int  f1, f2, mode;
};

struct MapPoint{
    double x, y, z, angle;
};

//generate the task point
TaskPoint taskPointGenerate(double x, double y, double z, double angle, int f1, int f2, int mode);

//generate the map point
MapPoint mapPointGenerate(double x, double y, double z, double angle);

//sort one single facade
vector<TaskPoint> sortSingleFacade(vector<CommandPoint> singleCommandGroup, MapPoint &pointCurrent);

//Comperator for raw command
bool cmp(CommandPoint a, CommandPoint b);

//read the raw command
void rawCommandRead(vector<CommandPoint>  &mission_point, string command_path);

//obtain the map dictionary
void mapPointRead(string command_path);

//convert the command into map point
void mappingConvert(int facade, int layer, int location, double &x, double &y, double &z, double & angle);

//comapre mp1 and mp2, which one is nearer for pointCurrent, i.e., which end is nearer
MapPoint compareTwoEnds(MapPoint mp1, MapPoint mp2, MapPoint pointCurrent);

void init_markers(visualization_msgs::Marker &marker);

vector<TaskPoint>  singleLayerGenerate(vector<CommandPoint> singlelayerCommandGroup, MapPoint &pointCurrent, int flag);

void shutdown(int sig);

int main(int argc, char** argv){


    ros::init(argc, argv, "nav_task");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternions[4];

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Rate rate(2);
    string map_frame = "/odom";
    string base_frame;
    try
    {
        listener.waitForTransform(map_frame, "/base_footprint", ros::Time(), ros::Duration(2.0));
        base_frame = "/base_footprint";
        ROS_INFO("base_frame = /base_footprint");
    }
    catch (tf::TransformException & ex)
    {
        try
        {
            listener.waitForTransform(map_frame, "/base_link", ros::Time()::Now(), ros::Duration(2.0));
            base_frame = "/base_link";
            ROS_INFO("base_frame = /base_link");
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("Cannot find transform between /odom and /base_link or /base_footprint");
            ros::shutdown();
        }
    }

    cout << map_frame << endl;
    cout << base_frame << endl;

    double angle = M_PI/2;
    int angle_count = 0;

    for(angle_count = 0; angle_count < 4; angle_count++){
        quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
        angle += M_PI/2;
    }


    //Define a marker publisher.
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
    visualization_msgs::Marker line_list;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);

    ROS_INFO("taskPlan.cpp started..");

    vector<CommandPoint> mission_point;
    string command_path = "/home/cyc/guihua/taskA1.txt";
    string dict_path = "/home/cyc/guihua/map.txt";

    rawCommandRead(mission_point, command_path);

    mapPointRead(dict_path);
    int n = mission_point.size();
    CommandPoint * array = new CommandPoint[n];
    for(int i = 0; i < n; i++){
        array[i] = mission_point[i];
        cout << " facade: " << mission_point[i].facade << " layer: " << mission_point[i].layer << " location: " << mission_point[i].location << endl;
    }
    sort(array, array + n, cmp);

    init_markers(line_list);

    signal(SIGINT, shutdown);

    for(int i = 0; i < n; i++) {
        mission_point[i] = array[i];
        cout << " facadeSorted: " << mission_point[i].facade << " layerSorted: " << mission_point[i].layer << " locationSorted: " << mission_point[i].location << endl;
    }

    //group CommandPoint by facade -> layer -> location

    vector<vector<CommandPoint>> commandPointGroup;
    vector<CommandPoint> currentGroup;
    for(int i = 0; i < n; i++){
        if(i == 0 || mission_point[i].facade != mission_point[i - 1].facade){
            vector<CommandPoint> preGroup(currentGroup);
            commandPointGroup.push_back(preGroup);
            currentGroup.clear();
            currentGroup.push_back(mission_point[i]);
        }
        else{
            currentGroup.push_back(mission_point[i]);
        }
    }
    commandPointGroup.push_back(currentGroup);
    commandPointGroup.erase(commandPointGroup.begin());
    int k = commandPointGroup.size();

    rate.sleep();
    listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
    double x_start = transform.getOrigin().x();
    double y_start = transform.getOrigin().y();
    double z_start = transform.getOrigin().z();
    double angle_start = fabs(tf::getYaw(transform.getRotation()));

    MapPoint pointCurrent;
    pointCurrent.x = x_start; pointCurrent.y = y_start; pointCurrent.z = z_start, pointCurrent.angle = angle_start; 
    vector<vector<TaskPoint>> taskPointGroup;
    //for each facade, do the sort operation
    for(int i = 0; i < k; i++){
        vector<TaskPoint> currentGroup = sortSingleFacade(commandPointGroup[i], pointCurrent);
        taskPointGroup.push_back(currentGroup);
    }


    for(int i = 0; i < taskPointGroup.size(); i++){
        for(int j = 0; j < taskPointGroup[i].size(); j++){
            point.x = taskPointGroup[i][j].x;
            point.y = taskPointGroup[i][j].y;
            point.z = taskPointGroup[i][j].z;
            
            line_list.points.push_back(point);
        }
    }

    ROS_INFO("Waiting for move_base action server...");
    //Wait 60 seconds for the action server to become available
    if(!ac.waitForServer(ros::Duration(60)))
    {
        ROS_INFO("Can't connected to move base server");
        return 1;
    }
    ROS_INFO("Connected to move base server");
    ROS_INFO("Starting Task Plan");

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);


    for(int i = 0; i < taskPointGroup.size(); i++){
        for(int j = 0; j < taskPointGroup[i].size(); j++){
            cout << "TaskPoint[" + to_string(i) + "][" + to_string(j) + "]: " << " x: " << taskPointGroup[i][j].x << " y: " << taskPointGroup[i][j].y << " z: " << taskPointGroup[i][j].z << " angle: " << taskPointGroup[i][j].angle << " f1: " << taskPointGroup[i][j].f1 
            << " f2: " << taskPointGroup[i][j].f2 << " mode: " << taskPointGroup[i][j].mode<<endl;
            marker_pub.publish(line_list);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            point.x = taskPointGroup[i][j].x;
            point.y = taskPointGroup[i][j].y;
            point.z = taskPointGroup[i][j].z;
            pose.position = point;
            if(i & 1 == 0)
                pose.orientation = quaternions[1];
            else 
                pose.orientation = quaternions[3];
            goal.target_pose.pose = pose;

            ac.sendGoal(goal);
            bool finished_within_time = ac.waitForResult(ros::Duration(1000));
            if(!finished_within_time)
            {
                ac.cancelGoal();
                ROS_INFO("Timed out achieving goal");
            }
            else
            {
                //We made it!
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO("Goal succeeded!");
                }
                else
                {
                    ROS_INFO("The base failed for some reason");
                }
            }
        }
        cout << endl;
    }

    // for(int i = 0; i < commandPointGroup.size(); i++){
    //     for(int j = 0; j < commandPointGroup[i].size(); j++){
    //         cout << commandPointGroup[i][j].facade << " " << commandPointGroup[i][j].layer << " " << commandPointGroup[i][j].location <<endl;
    //     }
    //     cout << endl << endl;
    // }
    // for(int i = 0; i < n; i++){
    //     cout<< array[i].facade << " " << array[i].layer << " " << array[i].location << " ";
    //     cout << endl;
    // }
}

vector<TaskPoint> sortSingleFacade(vector<CommandPoint> singleCommandGroup, MapPoint &pointCurrent){
    vector<TaskPoint> singleTaskPointGroup;
    vector<CommandPoint> singlelayerCommandGroup;
    vector<vector<CommandPoint>> singlefacadeCommandGroup;
    // bool onlyAGV = true;
    // bool onlyUAV = true;
    bool searchAllFacades = false;
    singlelayerCommandGroup.push_back(singleCommandGroup[0]);
    for(int i = 1; i < singleCommandGroup.size(); i++){
        if(singleCommandGroup[i-1].layer == singleCommandGroup[i].layer) {
            singlelayerCommandGroup.push_back(singleCommandGroup[i]);
        }
        else {
            singlefacadeCommandGroup.push_back(singlelayerCommandGroup);
            singlelayerCommandGroup.clear();
            singlelayerCommandGroup.push_back(singleCommandGroup[i]);
        }
    }
    singlefacadeCommandGroup.push_back(singlelayerCommandGroup);

    for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        for(int j = 0; j < singlefacadeCommandGroup[i].size(); j++) {
            cout <<  " facade: " << singlefacadeCommandGroup[i][j].facade << " layer: " << singlefacadeCommandGroup[i][j].layer << " location: " << singlefacadeCommandGroup[i][j].location << " ";
        }
        cout << endl;
    }

    // bug
    // for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
    //     if(singlefacadeCommandGroup[i][0].layer != 1) {
    //         onlyAGV = false;
    //     } else {
    //         onlyUAV = false;
    //     }
    // }
    
    for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        for(int j = 0; j < singlefacadeCommandGroup[i].size(); j++) {
            if(singlefacadeCommandGroup[i][j].layer == -1) {
                searchAllFacades = true;
                break;
            }
        }
    }

    if(searchAllFacades) {
        cout << "SearchAllFacades" << endl;
        MapPoint mp1, mp2;
        mappingConvert(singleCommandGroup[0].facade - 1, 0, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
        mappingConvert(singleCommandGroup[0].facade - 1, 0, countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
        MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
        MapPoint mpFar;
        if(mpNear.x == mp1.x && mpNear.y == mp1.y)
            mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
        else
            mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
        int index = 0;
        TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
        TaskPoint taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
        TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 1, 3);
        while(index < countLayer - 1) {
            if(index % 2 == 0)
                singleTaskPointGroup.push_back(taskPointNear);
            else    
                singleTaskPointGroup.push_back(taskPointFar);
            index++;
        }
        int size = singleTaskPointGroup.size();
        pointCurrent = mapPointGenerate(singleTaskPointGroup[size - 1].x, singleTaskPointGroup[size - 1].y, 
                                        singleTaskPointGroup[size - 1].z, singleTaskPointGroup[size - 1].angle);
    }
    else {
        for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
            // for(int j = 0; j < singlefacadeCommandGroup[i].size(); j++) {
            //     if(singlefacadeCommandGroup[i][j].layer == 1) {

            //     }
            // }
            if(singlefacadeCommandGroup[i][0].layer == 1) {
                if(i + 1 < singlefacadeCommandGroup.size() && (singlefacadeCommandGroup[i + 1][0].location == -1 || singlefacadeCommandGroup[i][0].location == -1)) {
                    flag = 3;
                    singlefacadeCommandGroup[i + 1][0].location = -1;
                    vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i + 1], pointCurrent, flag);
                    singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
                    i++;
                    cout << " flag: " << flag << endl;
                    cout << " current.x: " << pointCurrent.x << " current.y " << pointCurrent.y << " current.z: " << pointCurrent.z << endl;
                    continue;
                }
                flag = 1;
                vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
                singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
            } else {
                flag = 2;
                vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
                singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
            }
            cout << " flag: " << flag << endl;
            cout << " current.x: " << pointCurrent.x << " current.y " << pointCurrent.y << " current.z: " << pointCurrent.z << endl;
        }
        // if(onlyAGV) {
        //     flag = 1;
        //     vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[0], pointCurrent, flag);
        //     singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
        // } 
        // else {
        //     if(onlyUAV) {
        //         flag = 2;
        //         for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        //             vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
        //             singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
        //         }
        //     }
        //     else {
        //         flag = 3;
        //         for(int i = 0; i < singlefacadeCommandGroup.size(); i++) {
        //             vector<TaskPoint> v1 = singleLayerGenerate(singlefacadeCommandGroup[i], pointCurrent, flag);
        //             singleTaskPointGroup.insert(singleTaskPointGroup.end(),v1.begin(), v1.end());
        //         }
        //     }
        // }
    }
    return singleTaskPointGroup;
}


vector<TaskPoint>  singleLayerGenerate(vector<CommandPoint> singlelayerCommandGroup, MapPoint &pointCurrent, int flag) {
    vector<TaskPoint> singlelayerTaskPointGroup;
    switch (flag) {
        case 1:
            if(singlelayerCommandGroup[0].location == -1) {
                CommandPoint command = singlelayerCommandGroup[0];
                MapPoint mp1, mp2;
                mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
                mappingConvert(command.facade - 1, command.layer - 1, countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                MapPoint mpFar;
                //obtain the other map point
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
                else
                    mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
                TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
                TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 0, 3);
                singlelayerTaskPointGroup.push_back(taskPointNear);
                singlelayerTaskPointGroup.push_back(taskPointFar);
                pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
            }
            else {
                int n = singlelayerCommandGroup.size();
                vector<MapPoint> singleMapPointGroup;
                for(int i = 0; i < n; i++){
                    CommandPoint command = singlelayerCommandGroup[i];
                    MapPoint mp;
                    mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                    singleMapPointGroup.push_back(mp);
                }
                MapPoint mp1 = singleMapPointGroup[0];
                MapPoint mp2 = singleMapPointGroup[n - 1];
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    isAscend = true;
                if(isAscend){
                    for(int i = 0; i < n; i ++){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
                }
                else{
                    for(int i = n - 1; i >= 0; i--){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
                }
            }
            break;
        case 2:
            if(singlelayerCommandGroup[0].location == -1) {
                CommandPoint command = singlelayerCommandGroup[0];
                MapPoint mp1, mp2;
                mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
                mappingConvert(command.facade - 1, command.layer - 1, countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                MapPoint mpFar;
                //obtain the other map point
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
                else
                    mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
                TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
                TaskPoint taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
                TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 0, 1, 3);
                singlelayerTaskPointGroup.push_back(taskPointNear);
                singlelayerTaskPointGroup.push_back(taskPointRevise);
                singlelayerTaskPointGroup.push_back(taskPointFar);
                pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
            }
            else {
                int n = singlelayerCommandGroup.size();
                vector<MapPoint> singleMapPointGroup;
                for(int i = 0; i < n; i++){
                    CommandPoint command = singlelayerCommandGroup[i];
                    MapPoint mp;
                    mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                    singleMapPointGroup.push_back(mp);
                }
                MapPoint mp1 = singleMapPointGroup[0];
                MapPoint mp2 = singleMapPointGroup[n - 1];
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    isAscend = true;
                if(isAscend){
                    for(int i = 0; i < n; i ++){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
                }
                else{
                    for(int i = n - 1; i >= 0; i--){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
                }
            }
            break;
        case 3:
            if(singlelayerCommandGroup[0].location == -1) {
                CommandPoint command = singlelayerCommandGroup[0];
                MapPoint mp1, mp2;
                mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
                mappingConvert(command.facade - 1, command.layer - 1, countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
                // cout << mp1.x << " " << mp1.y << " " << mp2.x << " " << mp2.y << endl;
                // cout << pointCurrent.x << pointCurrent.y << endl;
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                MapPoint mpFar;
                //obtain the other map point
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
                else
                    mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
                // cout << mpNear.x << mpNear.y << " " << mpFar.x << mpFar.y << endl;
                TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
                TaskPoint taskPointRevise = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 2);
                TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 1, 3);
                singlelayerTaskPointGroup.push_back(taskPointNear);
                singlelayerTaskPointGroup.push_back(taskPointRevise);
                singlelayerTaskPointGroup.push_back(taskPointFar);
                pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
            }
            else {
                int n = singlelayerCommandGroup.size();
                vector<MapPoint> singleMapPointGroup;
                for(int i = 0; i < n; i++){
                    CommandPoint command = singlelayerCommandGroup[i];
                    MapPoint mp;
                    mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
                    singleMapPointGroup.push_back(mp);
                }
                MapPoint mp1 = singleMapPointGroup[0];
                MapPoint mp2 = singleMapPointGroup[n - 1];
                MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
                bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
                if(mpNear.x == mp1.x && mpNear.y == mp1.y)
                    isAscend = true;
                if(isAscend){
                    for(int i = 0; i < n; i ++){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        if(mp.z < 1)
                            targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                        else
                            targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
                }
                else{
                    for(int i = n - 1; i >= 0; i--){
                        MapPoint mp = singleMapPointGroup[i];
                        TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
                        singlelayerTaskPointGroup.push_back(targetTask);
                        if(mp.z < 1)
                            targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
                        else
                            targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 1, 4);
                        singlelayerTaskPointGroup.push_back(targetTask);
                    }
                    pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
                }
            }
            break;
        
        default:
            cout << "error" << endl;
            break;
    }
    return singlelayerTaskPointGroup;
}

// vector<TaskPoint>  singleLayerGenerate(vector<CommandPoint> singlelayerCommandGroup, MapPoint &pointCurrent, int flag) {
//     vector<TaskPoint> singlelayerTaskPointGroup;
//     if(singlelayerCommandGroup[0].location == -1) {
//         CommandPoint command = singlelayerCommandGroup[0];
//         MapPoint mp1, mp2;
//         mappingConvert(command.facade - 1, command.layer - 1, 0, mp1.x, mp1.y, mp1.z, mp1.angle);//obtain mp1 based on command
//         mappingConvert(command.facade - 1, command.layer - 1, countLocation - 1, mp2.x, mp2.y, mp2.z, mp2.angle);//obtain mp2 based on cammand
//         MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
//         MapPoint mpFar;
//         //obtain the other map point
//         if(mpNear.x == mp1.x && mpNear.y == mp1.y)
//             mpFar = mapPointGenerate(mp2.x, mp2.y, mp2.z, mp2.angle);
//         else
//             mpFar = mapPointGenerate(mp1.x, mp1.y, mp1.z, mp1.angle);
//         TaskPoint taskPointNear = taskPointGenerate(mpNear.x, mpNear.y, mpNear.z, mpNear.angle, 0, 0, 1);
//         TaskPoint taskPointFar = taskPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle, 1, 0, 1);
//         singlelayerTaskPointGroup.push_back(taskPointNear);
//         singlelayerTaskPointGroup.push_back(taskPointFar);
//         pointCurrent = mapPointGenerate(mpFar.x, mpFar.y, mpFar.z, mpFar.angle);
//     }
//     else {
//         int n = singlelayerCommandGroup.size();
//         vector<MapPoint> singleMapPointGroup;
//         for(int i = 0; i < n; i++){
//             CommandPoint command = singlelayerCommandGroup[i];
//             MapPoint mp;
//             mappingConvert(command.facade - 1, command.layer - 1, command.location - 1, mp.x, mp.y, mp.z, mp.angle);//obtain every mp based on command
//             singleMapPointGroup.push_back(mp);
//         }
//         MapPoint mp1 = singleMapPointGroup[0];
//         MapPoint mp2 = singleMapPointGroup[n - 1];
//         MapPoint mpNear = compareTwoEnds(mp1, mp2, pointCurrent);// obtain the nearer map point based on current point
//         bool isAscend = false; // check whether we need to sort the singleTaskPointGroup ascend or descend
//         if(mpNear.x == mp1.x && mpNear.y == mp1.y)
//             isAscend = true;
//         if(isAscend){
//             for(int i = 0; i < n; i ++){
//                 MapPoint mp = singleMapPointGroup[i];
//                 TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
//                 singlelayerTaskPointGroup.push_back(targetTask);
//                 targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
//                 singlelayerTaskPointGroup.push_back(targetTask);
//             }
//             pointCurrent = mapPointGenerate(singleMapPointGroup[n - 1].x, singleMapPointGroup[n - 1].y, singleMapPointGroup[n - 1].z, singleMapPointGroup[n - 1].angle);
//         }
//         else{
//             for(int i = n - 1; i >= 0; i--){
//                 MapPoint mp = singleMapPointGroup[i];
//                 TaskPoint targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 0, 0, 1);
//                 singlelayerTaskPointGroup.push_back(targetTask);
//                 targetTask = taskPointGenerate(mp.x, mp.y, mp.z, mp.angle, 1, 0, 5);
//                 singlelayerTaskPointGroup.push_back(targetTask);
//             }
//             pointCurrent = mapPointGenerate(singleMapPointGroup[0].x, singleMapPointGroup[0].y, singleMapPointGroup[0].z, singleMapPointGroup[0].angle);
//         }
//     }
//     return singlelayerTaskPointGroup;
// }

void rawCommandRead(vector<CommandPoint>  &mission_point, string command_path){
    ifstream mission_task;
    unsigned int count = 0;
    mission_task.open(command_path);  
    while(!mission_task.eof() && mission_task.peek()!= EOF)
    {
      string nextline;
      getline(mission_task, nextline);
      if(nextline == "end")
        break;
      istringstream in(nextline);
      CommandPoint commandPoint;
      in >> commandPoint.facade >> commandPoint.layer >> commandPoint.location;
      mission_point.push_back(commandPoint);
    }
    mission_task.close();
}


bool cmp(CommandPoint a, CommandPoint b){
    if(a.facade == b.facade){
        if(a.layer == b.layer){
            return a.location < b.location;
        }
        return a.layer < b.layer;
    }
    return a.facade < b.facade;
}

MapPoint mapPointGenerate(double x, double y, double z, double angle){
    MapPoint ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    ret.angle = angle;
    return ret;
}

TaskPoint taskPointGenerate(double x, double y, double z, double angle, int f1, int f2, int mode){
    TaskPoint ret;
    ret.x = x;
    ret.y = y;
    ret.z = z;
    ret.angle = angle;
    ret.f1 = f1;
    ret.f2 = f2;
    ret.mode  = mode;
    return ret;
}

MapPoint compareTwoEnds(MapPoint mp1, MapPoint mp2, MapPoint pointCurrent){
    double delta1 = sqrt((mp1.x - pointCurrent.x) * (mp1.x - pointCurrent.x) + (mp1.y - pointCurrent.y) * (mp1.y - pointCurrent.y));
    double delta2 = sqrt((mp2.x - pointCurrent.x) * (mp2.x - pointCurrent.x) + (mp2.y - pointCurrent.y) * (mp2.y - pointCurrent.y));
    if(delta1 < delta2)
        return mp1;
    else
        return mp2;
}

void mappingConvert(int facade, int layer, int location, double &x, double &y, double &z, double & angle){
    int index = facade * countLayer * countLocation + layer * countLocation + location; // it should be noticed that every base index for facade, layer, location is 0!!!
    x = dict[index][0];
    y = dict[index][1];
    z = dict[index][2];
    angle = dict[index][3];
}

void init_markers(visualization_msgs::Marker &marker) {
    marker.ns       = "waypoints";
    marker.id       = 0;
    marker.type     = visualization_msgs::Marker::CUBE_LIST;
    marker.action   = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();//0 is forever
    marker.scale.x  = 0.2;
    marker.scale.y  = 0.2;
    marker.color.r  = 1.0;
    marker.color.g  = 0.7;
    marker.color.b  = 1.0;
    marker.color.a  = 1.0;

    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
}

//obtain the dictionary
void mapPointRead(string command_path){


    ifstream dict_file;
    dict_file.open(command_path);  
    while(!dict_file.eof() && dict_file.peek()!= EOF)
    {
      string nextline;
      getline(dict_file, nextline);
      if(nextline == "end")
        break;
      istringstream in(nextline);
      vector<double> temp;
      double x, y, z, angle;
      in >> x >> y >> z >> angle;
      temp.push_back(x);
      temp.push_back(y);
      temp.push_back(z);
      temp.push_back(angle);
      dict.push_back(temp);
    }
    dict_file.close();
    for(int i = 0; i < dict.size(); i++){
        cout<< dict[i][0] << " " << dict[i][1] << " " << dict[i][2] << " " << dict[i][3] << endl;
    }
}

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());
  ros::Duration(1).sleep(); // sleep for  a second
  ROS_INFO("taskPlan ended!");
  ros::shutdown();
}