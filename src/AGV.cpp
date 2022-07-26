#include "AGV.h"

AGV::AGV(ros::NodeHandle& nh) {
    init();
    m_nh = nh;
    
    marker_pub = m_nh.advertise<visualization_msgs::Marker>("waypoint_markers", 10);
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);   
    AGVCurr_pub = nh.advertise<car_control::Position>("/AGVCurr", 10);
    cancelGoal_pub = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
}

void AGV::init() {
    double angle = M_PI/2;
    int angle_count = 0;

    for(angle_count = 0; angle_count < 4; angle_count++){
        quaternions[angle_count] = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
        angle += M_PI/2;
    }
}

void AGV::moveToTarget(TaskPoint target, visualization_msgs::Marker &marker, int flag) {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    //Wait 60 seconds for the action server to become available
    if(!ac.waitForServer(ros::Duration(60)))
    {
        return;
    }
    marker_pub.publish(marker);
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Point point;
    geometry_msgs::Pose pose;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    point.x = target.x;
    point.y = target.y;
    point.z = target.z;
    pose.position = point;
<<<<<<< HEAD
    // pose.orientation = quaternions[3];
    if(target.angle == 0)
        pose.orientation = quaternions[3];
    else if(target.angle == 90)
        pose.orientation = quaternions[0];
    else if(target.angle == 180)
        pose.orientation = quaternions[1];
    else if(target.angle == 270)
        pose.orientation = quaternions[2];
    
    goal.target_pose.pose = pose;
    ac.sendGoal(goal);
=======
    pose.orientation = quaternions[3];
    goal.target_pose.pose = pose;
    ac.sendGoal(goal);
    cout << "ddddddddddddddddddddddddddddddddddddddddddddd";
>>>>>>> 7f853810ab9b8f9e0090e9c75d2590db5153aa35
    if(flag == 0)
        ac.cancelGoal();
    // if(flag == 1)
    //     ac.sendGoal(goal);
    // else if (flag == 0)
    //     ac.cancelGoal();
    //We made it!

    // bool finished_within_time = ac.waitForResult(ros::Duration(5));
    // if(!finished_within_time)
    // {
    //     ac.cancelGoal();
    //     ROS_INFO("Timed out achieving goal");
    // }
    // else
    // {
    //     //We made it!
    //     if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     {
    //         ROS_INFO("Goal succeeded!");
    //     }
    //     else
    //     {
    //         ROS_INFO("The base failed for some reason");
    //     }
    // }
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal succeeded!");
    }
    else
    {
        ROS_INFO("The base failed for some reason");
    }
}

void AGV::init_markers(visualization_msgs::Marker &marker) {
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

void AGV::GetAGVCurr() {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate loop_rate(20);//20Hz
    while(ros::ok()) {
        string map_frame = "map";
        string base_frame = "base_footprint";
        listener.waitForTransform(map_frame, base_frame,ros::Time(0), ros::Duration(2.0));
        try {
            listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        m_AGVCurr.AGVx = transform.getOrigin().x();
        m_AGVCurr.AGVy = transform.getOrigin().y();
        m_AGVCurr.AGVyaw = tf::getYaw(transform.getRotation());   
        AGVCurr_pub.publish(m_AGVCurr);
        loop_rate.sleep();
    }
}

void AGV::cancelGoal() {
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ac.cancelGoal();
}
