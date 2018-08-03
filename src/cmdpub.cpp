#include <ros/ros.h>
#include <mypkg/Command.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <string>
using namespace std;

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<mypkg::Command>("my_topic0", 1000);

  ros::Rate rate(2);
  while(ros::ok()) {

     string mystr;
     int myint;
     mypkg::Command msg;
     ROS_INFO_STREAM("Upper Command:");
     getline (cin,mystr);
     stringstream(mystr) >> myint;
     msg.upper = myint;
     ROS_INFO_STREAM("Lower Command:");
     getline (cin,mystr);
     stringstream(mystr) >> myint;
     msg.lower = myint;
     pub.publish(msg); 

    ROS_INFO_STREAM("Sending command:" << " upper=" <<  convert_int(msg.upper) << " lower=" << convert_int(msg.lower));

  }
}
