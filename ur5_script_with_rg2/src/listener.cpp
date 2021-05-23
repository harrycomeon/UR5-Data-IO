#include "ros/ros.h"
//#include <ros/package.h>
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include "sensor_msgs/JointState.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
using namespace std;

ofstream f1, f2;

double duration=0; //time duration

double start_time = 0;


void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{

  //double total_time = ros::Time::now().toSec();

  //float pos[3],vel[3];
  // interval time
  duration = ros::Time::now().toSec() - start_time;

  f1 << duration << "\t" <<  msg->position[0] << "\t"<< msg->position[1] << "\t"<< msg->position[2] << "\t"<< msg->position[3] << "\t"<< msg->position[4] << "\t"<< msg->position[5] << "\n" ;  //saving joint positions to file
  f2 << duration << "\t" << msg->velocity[0] << "\t"<< msg->velocity[1] << "\t"<< msg->velocity[2] << "\t"<< msg->velocity[3] << "\t"<< msg->velocity[4] << "\t"<< msg->velocity[5] << "\n" ;  //saving joint velocities to file


  //ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],vel[0],vel[1],vel[2]);
  ROS_INFO("I heard: [%f] ",duration);
  //f1.close();
  //f2.close(); 
}
 
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
  
  // open the file in the file system

  f1.open("/home/harry/catkin_ws/src/ur5_script_with_rg2/results/Joint_position.txt",std::ios_base::trunc);
  f2.open("/home/harry/catkin_ws/src/ur5_script_with_rg2/results/Joint_velocities.txt",std::ios_base::trunc);
 
  ros::NodeHandle n;
  
  // start time for the interval time

  start_time = ros::Time::now().toSec();

  // publish frequency

  ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointstatesCallback);

  ros::spin();
 
  return 0;
}
