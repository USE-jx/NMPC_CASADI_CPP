#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <so3_disturbance_generator/DisturbanceUIConfig.h>
#include "pose_utils.h"

using namespace arma;
using namespace std;

#define CORRECTION_RATE 1

ros::Publisher pubo;
ros::Publisher pubc;
ros::Publisher pubf;
ros::Publisher pubm;
so3_disturbance_generator::DisturbanceUIConfig config;
nav_msgs::Odometry noisy_odom;
geometry_msgs::PoseStamped correction;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  noisy_odom.header = msg->header;  
  correction.header = msg->header;  
  // Get odom
  colvec pose(6);
  colvec vel(3);  
  pose(0)        = msg->pose.pose.position.x;
  pose(1)        = msg->pose.pose.position.y;
  pose(2)        = msg->pose.pose.position.z;    
  colvec q       = zeros<colvec>(4);
  q(0)           = msg->pose.pose.orientation.w;
  q(1)           = msg->pose.pose.orientation.x;
  q(2)           = msg->pose.pose.orientation.y;
  q(3)           = msg->pose.pose.orientation.z; 
  pose.rows(3,5) = R_to_ypr(quaternion_to_R(q));
  vel(0)         = msg->twist.twist.linear.x;       
  vel(1)         = msg->twist.twist.linear.y;       
  vel(2)         = msg->twist.twist.linear.z;       
  // Drift Odom 
  static colvec drift_pose      = pose;
  static colvec drift_vel       = vel;   
  static colvec correction_pose = zeros<colvec>(6);  
  static colvec prev_pose       = pose;
  static ros::Time prev_pose_t  = msg->header.stamp;  
  if (config.enable_drift_odom)
  {
    double dt       = (msg->header.stamp - prev_pose_t).toSec();
    prev_pose_t     = msg->header.stamp;    
    colvec d        = pose_update(pose_inverse(prev_pose), pose);
    prev_pose       = pose;
    d(0)           += (config.vdriftx   + config.stdvdriftxyz * as_scalar(randn(1))) * dt;    
    d(1)           += (config.vdrifty   + config.stdvdriftxyz * as_scalar(randn(1))) * dt; 
    d(2)           += (config.vdriftz   + config.stdvdriftxyz * as_scalar(randn(1))) * dt;        
    d(3)           += (config.vdriftyaw + config.stdvdriftyaw * as_scalar(randn(1))) * dt;
    drift_pose      = pose_update(drift_pose, d);
    drift_vel       = ypr_to_R(drift_pose.rows(3,5)) * trans(ypr_to_R(pose.rows(3,5))) * vel;
    correction_pose = pose_update(pose, pose_inverse(drift_pose));        
  }
  else
  {
    drift_pose      = pose;
    drift_vel       = vel;
    correction_pose = zeros<colvec>(6);
  }
  // Noisy Odom
  static colvec noisy_pose = drift_pose;
  static colvec noisy_vel  = drift_vel;    
  if (config.enable_noisy_odom)
  {
    colvec noise_pose = zeros<colvec>(6);
    colvec noise_vel  = zeros<colvec>(3);
    noise_pose(0) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(1) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(2) = config.stdxyz  * as_scalar(randn(1));
    noise_pose(3) = config.stdyaw  * as_scalar(randn(1));
    noise_pose(4) = config.stdrp   * as_scalar(randn(1)); 
    noise_pose(5) = config.stdrp   * as_scalar(randn(1));       
    noise_vel(0)  = config.stdvxyz * as_scalar(randn(1));
    noise_vel(1)  = config.stdvxyz * as_scalar(randn(1));
    noise_vel(2)  = config.stdvxyz * as_scalar(randn(1));    
    noisy_pose = drift_pose + noise_pose;
    noisy_vel  = drift_vel  + noise_vel;
    noisy_odom.pose.covariance[0+0*6]         = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[1+1*6]         = config.stdxyz * config.stdxyz;
    noisy_odom.pose.covariance[2+2*6]         = config.stdxyz * config.stdxyz;      
    noisy_odom.pose.covariance[(0+3)+(0+3)*6] = config.stdyaw * config.stdyaw;
    noisy_odom.pose.covariance[(1+3)+(1+3)*6] = config.stdrp  * config.stdrp;
    noisy_odom.pose.covariance[(2+3)+(2+3)*6] = config.stdrp  * config.stdrp;       
    noisy_odom.twist.covariance[0+0*6]        = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[1+1*6]        = config.stdvxyz * config.stdvxyz;
    noisy_odom.twist.covariance[2+2*6]        = config.stdvxyz * config.stdvxyz;     
  }
  else
  {
    noisy_pose = drift_pose;
    noisy_vel  = drift_vel;
    noisy_odom.pose.covariance[0+0*6]         = 0;
    noisy_odom.pose.covariance[1+1*6]         = 0;
    noisy_odom.pose.covariance[2+2*6]         = 0;
    noisy_odom.pose.covariance[(0+3)+(0+3)*6] = 0;
    noisy_odom.pose.covariance[(1+3)+(1+3)*6] = 0;
    noisy_odom.pose.covariance[(2+3)+(2+3)*6] = 0;    
    noisy_odom.twist.covariance[0+0*6]        = 0;
    noisy_odom.twist.covariance[1+1*6]        = 0;
    noisy_odom.twist.covariance[2+2*6]        = 0;      
  }
  // Assemble and publish odom
  noisy_odom.pose.pose.position.x    = noisy_pose(0);
  noisy_odom.pose.pose.position.y    = noisy_pose(1);
  noisy_odom.pose.pose.position.z    = noisy_pose(2);
  noisy_odom.twist.twist.linear.x    = noisy_vel(0);
  noisy_odom.twist.twist.linear.y    = noisy_vel(1);
  noisy_odom.twist.twist.linear.z    = noisy_vel(2);
  colvec noisy_q                     = R_to_quaternion(ypr_to_R(noisy_pose.rows(3,5)));
  noisy_odom.pose.pose.orientation.w = noisy_q(0);
  noisy_odom.pose.pose.orientation.x = noisy_q(1);
  noisy_odom.pose.pose.orientation.y = noisy_q(2);
  noisy_odom.pose.pose.orientation.z = noisy_q(3);    
  pubo.publish(noisy_odom);
  // Check time interval and publish correction
  static ros::Time prev_correction_t = msg->header.stamp;
  if ((msg->header.stamp - prev_correction_t).toSec() > 1.0 / CORRECTION_RATE)
  {
    prev_correction_t             = msg->header.stamp;
    correction.pose.position.x    = correction_pose(0);
    correction.pose.position.y    = correction_pose(1);
    correction.pose.position.z    = correction_pose(2);        
    colvec correction_q           = R_to_quaternion(ypr_to_R(correction_pose.rows(3,5)));
    correction.pose.orientation.w = correction_q(0);
    correction.pose.orientation.x = correction_q(1);
    correction.pose.orientation.y = correction_q(2);
    correction.pose.orientation.z = correction_q(3);   
    pubc.publish(correction);         
  }
}    

void config_callback(so3_disturbance_generator::DisturbanceUIConfig &_config, uint32_t level) 
{
  config = _config;
}

void set_disturbance()
{
  geometry_msgs::Vector3 f;
  geometry_msgs::Vector3 m;
  f.x = config.fxy  + config.stdfxy  * as_scalar(randn(1));
  f.y = config.fxy  + config.stdfxy  * as_scalar(randn(1));
  f.z = config.fz   + config.stdfz   * as_scalar(randn(1));    
  m.x = config.mrp  + config.stdmrp  * as_scalar(randn(1));    
  m.y = config.mrp  + config.stdmrp  * as_scalar(randn(1));    
  m.z = config.myaw + config.stdmyaw * as_scalar(randn(1));        
  pubf.publish(f);
  pubm.publish(m);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "so3_disturbance_generator");
  ros::NodeHandle n("~");

  ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);
  pubo = n.advertise<nav_msgs::Odometry>(        "noisy_odom",         10);    
  pubc = n.advertise<geometry_msgs::PoseStamped>("correction",         10);      
  pubf = n.advertise<geometry_msgs::Vector3>(    "force_disturbance" , 10);
  pubm = n.advertise<geometry_msgs::Vector3>(    "moment_disturbance", 10);  
  
  // Dynamic Reconfig
  dynamic_reconfigure::Server<so3_disturbance_generator::DisturbanceUIConfig> server;
  dynamic_reconfigure::Server<so3_disturbance_generator::DisturbanceUIConfig>::CallbackType ff;
  ff = boost::bind(&config_callback, _1, _2);
  server.setCallback(ff);   

  ros::Rate r(100.0);
  while(n.ok())
  {
    ros::spinOnce();
    set_disturbance();
    r.sleep();
  }

  return 0;
}
