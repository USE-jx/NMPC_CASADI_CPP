#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>
using namespace std;

nav_msgs::Odometry odom;
void odomCallback(const nav_msgs::Odometry &msg) { odom = msg; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_path");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_odom = nh.subscribe("/odom", 10, odomCallback);
    ros::Publisher pub_odom_path = nh.advertise<nav_msgs::Path>("/odom_path", 10);

    // ROS params
    int length_path = -1;
    ros::param::get("~length_path", length_path);

    nav_msgs::Path odom_path;
    geometry_msgs::PoseStamped poses;

    ros::Rate r(10);
    while (nh.ok())
    {
        if(odom.header.frame_id == "")
        {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        odom_path.header = odom.header;
        poses.header = odom_path.header;
        poses.pose = odom.pose.pose;
        odom_path.poses.push_back(poses);

        // Make sure odom path < length_path
        while(length_path != -1 && odom_path.poses.size() >= length_path)
        {
            odom_path.poses.erase(odom_path.poses.begin());
        }

        pub_odom_path.publish(odom_path);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}