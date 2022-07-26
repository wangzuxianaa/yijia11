#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <iostream>


int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_distance");
    ros::NodeHandle nh;
    double rate = 20;
    double delta_t = 0.05;
    geometry_msgs::Point point;
    point.x = 0;

    ros::Publisher distance_pub = nh.advertise<geometry_msgs::Point>("/rela_distance", 10);
    ros::Rate loopRate(rate);

    while(ros::ok()) {
        point.x += 0.2 * delta_t;
        distance_pub.publish(point);
        loopRate.sleep();
    }
    return 0;
}