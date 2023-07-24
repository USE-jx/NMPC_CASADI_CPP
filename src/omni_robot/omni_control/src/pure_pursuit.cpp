#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
using namespace std;

nav_msgs::Odometry odom;
void odomCallback(const nav_msgs::Odometry &msg) { odom = msg; }

nav_msgs::Path path;
void pathCallback(const nav_msgs::Path &msg) { path = msg; }

float quaternion2Yaw(geometry_msgs::Quaternion orientation)
{
    double q0 = orientation.x;
    double q1 = orientation.y;
    double q2 = orientation.z;
    double q3 = orientation.w;

    float yaw = atan2(2.0*(q2*q3 + q0*q1), 1.0 - 2.0*(q1*q1 + q2*q2));
    return yaw;
}

geometry_msgs::Twist purePursuit(nav_msgs::Odometry odom, nav_msgs::Path path, int count)
{
    float pose[3], vel[3], tar[3];
    // Pose
    pose[0] = odom.pose.pose.position.x;                    // x
    pose[1] = odom.pose.pose.position.y;                    // y
    pose[2] = quaternion2Yaw(odom.pose.pose.orientation);   // theta
    cout << pose[0] << "\t" << pose[1] << "\t" << pose[2] << endl;

    // Velocity:
    vel[0] = odom.twist.twist.linear.x;     // vx
    vel[1] = odom.twist.twist.linear.y;     // vy
    vel[2] = odom.twist.twist.angular.z;    // omega

    // Goal
    geometry_msgs::PoseStamped goal;
    if(count < path.poses.size()) goal = path.poses[count];
    else goal = path.poses[path.poses.size()-1];
    tar[0] = goal.pose.position.x;
    tar[1] = goal.pose.position.y;
    tar[2] = quaternion2Yaw(goal.pose.orientation);

    // PID Control
    float delta_x = tar[0] - pose[0];
    float delta_y = tar[1] - pose[1];
    float delta_th = tar[2] - pose[2];

    float Kx = 0.5; float Ky = 0.5; float Kw = 1.0;

    geometry_msgs::Twist control;
    if(count < path.poses.size() || sqrt(delta_x*delta_x + delta_y*delta_y) > 0.05)
    {
        control.linear.x = Kx*delta_x*cos(pose[2]) + Ky*delta_y*sin(pose[2]);
        control.linear.y = -Kx*delta_x*sin(pose[2]) + Ky*delta_y*cos(pose[2]);
        control.angular.z = Kw*delta_th;
    }
    else
    {
        cout << "Done!!" << endl;
        control.linear.x = 0.0;
        control.linear.y = 0.0;
        control.angular.z = 0.0;
    }
    return control;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;

    ros::Subscriber sub_odom = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber sub_path = nh.subscribe("/path", 10, pathCallback);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    int count = 0;

    ros::Rate r(10);
    while(nh.ok())
    {
        if(odom.header.frame_id == "" || path.header.frame_id == "")
        {
            ros::spinOnce();
            r.sleep();
            continue;
        }

        geometry_msgs::Twist vel = purePursuit(odom, path, count);
        pub_vel.publish(vel);
        
        count ++;

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}