// ROS library
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iostream>

using namespace std;

#define PI 3.14159265

// Global Variables
ros::Time last_vel_time;
ros::Time prev_update_time;
geometry_msgs::Twist vel;
nav_msgs::Odometry odom;


float odom_pose[3] = {0, 0, 0};     // x, y, theta
float odom_vel[3] = {0, 0, 0};      // vx, vy, omega

ros::Publisher pub_odom;
string frame_id = "odom";
string child_frame_id = "base_footprint";


// Declare functions
void velCallback(const geometry_msgs::Twist &msg);
void updateOmniFake(tf::TransformBroadcaster &tf_broadcaster);
void updateOdometry(ros::Duration diff_time);
void updateTF(geometry_msgs::TransformStamped& odom_tf);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_fake_node");
    ros::NodeHandle nh("~");

    ros::Subscriber sub_vel = nh.subscribe("/cmd_vel", 10, velCallback);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    tf::TransformBroadcaster tf_broadcaster;

    // ROS params
    ros::param::get("~init_pose_x", odom_pose[0]);
    ros::param::get("~init_pose_y", odom_pose[1]);
    ros::param::get("~init_pose_theta", odom_pose[2]);

    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    ros::Rate r(10);
    cout << "Initial Omni Robot Fake !!!" << endl;
    while(nh.ok())
    {
        updateOmniFake(tf_broadcaster);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void velCallback(const geometry_msgs::Twist &msg) { vel = msg; }

void updateOmniFake(tf::TransformBroadcaster &tf_broadcaster)
{
    ros::Time time_now = ros::Time::now();
    ros::Duration step_time = time_now - prev_update_time;
    prev_update_time = time_now;

    updateOdometry(step_time);
    odom.header.stamp = time_now;
    pub_odom.publish(odom);

    geometry_msgs::TransformStamped odom_tf;
    updateTF(odom_tf);
    tf_broadcaster.sendTransform(odom_tf);
}

void updateOdometry(ros::Duration diff_time)
{
    float dt = diff_time.toSec();

    // Robot state - velocity
    odom_vel[0] = vel.linear.x;
    odom_vel[1] = vel.linear.y;
    odom_vel[2] = vel.angular.z;

    // Robot state - position (Euler Discretize = Runge-Kutta 1st order)
    odom_pose[2] += odom_vel[2]*dt;
    if(odom_pose[2] > PI) { odom_pose[2] -= 2*PI; }
    if(odom_pose[2] < -PI) { odom_pose[2] += 2*PI; }
    
    odom_pose[0] += (odom_vel[0]*cos(odom_pose[2]) - odom_vel[1]*sin(odom_pose[2]))*dt;
    odom_pose[1] += (odom_vel[0]*sin(odom_pose[2]) + odom_vel[1]*cos(odom_pose[2]))*dt;

    // Update Odometry
    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.linear.y = odom_vel[1];
    odom.twist.twist.angular.z = odom_vel[2];
}

void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}