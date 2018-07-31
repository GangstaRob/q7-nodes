/*************************************************
* Author: Brian Adams
* Date: 4/4/2018
* Revised: 
* Description: This program tests the spi read and
* write commands.   
**************************************************/
#include "spi.cpp"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
//#include "adis16480.c"
void shellClr();
//#include <spi-xilinx.c>
//#include "Documents/q7-toolchain/linux-stable/drivers/spi/spidev.c"

int main(int argc, char *argv[]){

	ros::init(argc, argv, "hello_ros");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("hello, ROS!");

	__u8 tx[]={0xE7, 0x00, 0xE7, 0x00};

	spiReadWrite(tx);
	
	return 0;
}


void shellClr(){
	char cmd[25];
	snprintf(cmd,24,"clear");	
  	int rv = system(cmd);
}
