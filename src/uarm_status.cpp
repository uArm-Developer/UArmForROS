
/******************************************************************************************
# File Name : uarm_core.py
# Author : Joey Song
# Version : V1.0
# Date : 6 Jan, 2016
# Modified Date : 6 Jan, 2016
# Description : This documents is for uarm ROS Library and ROS package
# Copyright(C) 2016 uArm Team. All right reserved.
*******************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <time.h> 

// Main function
int main(int argc, char **argv)
{
  // initial node
  ros::init(argc, argv, "uarm_status");
  ros::NodeHandle n;
  ros::Publisher execute = n.advertise<std_msgs::String>("uarm_status", 10);

  ros::Rate loop_rate(10);
 
  std_msgs::String msg;
  int executeTrigger =0;
 
  // create a loop which will allow publisher publishs after set-up
  while(ros::ok()){
	// check connection - if there is a subscriber
	if (execute.getNumSubscribers()!=0)
	{
		if(executeTrigger == 0)			
		if (argc == 2)
		{
			std::string string_1 = argv[1];
			
			// attach uarm
			if ((string_1== "attach") || (string_1== "1") || (string_1== "ATTACH"))
			{
			  msg.data = "attach";
			  execute.publish(msg);
			  std::cout << "uArm: Attached uarm" << std::endl;
			  executeTrigger= 1;
		 	}
			// detach uarm
			else if ((string_1 == "detach")|| (string_1== "0") || (string_1== "DETACH"))
			{
			  msg.data = "detach";
			  execute.publish(msg);
			  std::cout << "uArm: Detached uarm" << std::endl;
			  executeTrigger= 1;
			}
			// print errors
			else
			{
				std::cout << "ERROR: Input Incorrect" << std::endl;
			}
		}
		else
		{
			std::cout << "ERROR: Input detach/0 or attach/1" << std::endl;
		}
			sleep(0.1);
	  }
	}
  return 0;
}
