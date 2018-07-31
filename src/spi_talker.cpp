/*************************************************
* Author: Brian Adams
* Date: 4/4/2018
* Revised: 
* Description: This program tests the spi read and
* write commands.   
**************************************************/
#include <ros/ros.h>
#include "<spi.c>"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
void shellClr();

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "hello_ros");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("hello, ROS!");
}
