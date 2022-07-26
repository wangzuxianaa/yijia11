#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <iostream>

using namespace std;

ros::Publisher cmd_pub;
geometry_msgs::Twist speed;
double linear_speed = 0.4;
double delta_t = 0.05;
struct RelaVector {
    double x, y, z;
};

RelaVector Pr;

// 根据标志位寻找相应的策略
void flagCallback(const geometry_msgs::PointConstPtr& msg) {
    if(msg->x == 1) {
        speed.linear.x = linear_speed - Pr.x/delta_t;
        cmd_pub.publish(speed);
    }
    else if(msg->x ==0) {
        speed.linear.x = linear_speed;
        cmd_pub.publish(speed);
    }
}

// 回调接收小车和无人机的相对位置
void prCallback(const geometry_msgs::PointConstPtr& msg) {
    Pr.x = msg->x;
    Pr.y = msg->y;
    Pr.z = 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AGV");
    ros::NodeHandle nh;
    
    double rate = 20;
    float goal_distance = 20;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber flag_version_sub = nh.subscribe<geometry_msgs::Point>("/far_flag",10,flagCallback);
    ros::Subscriber pr_sub = nh.subscribe<geometry_msgs::Point>("/rela_distance", 10, prCallback);

    ros::Rate loopRate(rate);

    // 获取odom和base坐标关系，得到坐标值
    tf::TransformListener listener;
    tf::StampedTransform transform;
    std::string odom_frame = "/odom";
    std::string base_frame;
    try
    {
        listener.waitForTransform(odom_frame, "/base_footprint", ros::Time(), ros::Duration(2.0) );
        base_frame = "/base_footprint";
        ROS_INFO("base_frame = /base_footprint");
    }
    catch (tf::TransformException & ex)
    {
        try
        {
            listener.waitForTransform(odom_frame, "/base_link", ros::Time(), ros::Duration(2.0) );
            base_frame = "/base_link";
            ROS_INFO("base_frame = /base_link");
        }
        catch (tf::TransformException ex)
        {
            ROS_INFO("Cannot find transform between /odom and /base_link or /base_footprint");
            cmd_pub.publish(geometry_msgs::Twist());
            ros::shutdown();
        }
    }

    // 下面就是让小车向前走
    speed.linear.x = linear_speed; 
    listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
    float x_start = transform.getOrigin().x();
    float y_start = transform.getOrigin().y();
    float pre_y = y_start;
    float pre_x = x_start;
    float distance = 0;
    while( (distance < goal_distance) && (ros::ok()) )
    {
        listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));
        // 控制旋转的z速度，让小车走直线
        speed.angular.z = -(y-y_start)-(y-pre_y)/0.01;
        loopRate.sleep();
        pre_y = y;
        pre_x = x;
        ros::spinOnce();
    }
    cmd_pub.publish(geometry_msgs::Twist());
    ros::Duration(1).sleep(); 
    speed.linear.x = 0;

    ros::shutdown();
    return 0;
}