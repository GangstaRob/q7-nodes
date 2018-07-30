/*************************************************
* Author: Brian Adams
* Date: 4/4/2018
* Revised: 
* Description: This program tests the spi read and
* write commands.   
**************************************************/
#include <ros/ros.h>
#include "spi.c"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
//#include "adis16480.c"
void shellClr();

int main(int argc, char *argv[]){

    ros::init(argc, argv, "imu_talker");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("hello, ROS!");

__u8 tx[]={(int)strtol(argv[1], NULL, 16), (int)strtol(argv[2], NULL, 16), (int)strtol(argv[3], NULL, 16), (int)strtol(argv[4], NULL, 16)};

	shellClr();
	spiReadWrite(tx);
	
	return 0;
}


void shellClr(){
	char cmd[25];
	snprintf(cmd,24,"clear");	
  	int rv = system(cmd);
}
