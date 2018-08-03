#include <ros/ros.h>
#include "Command.h"
#include <iomanip>
#include <sstream>
using namespace std;

mypkg::Command function(my_pkg::Command);

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

ros::Publisher pub;

void messageReceived(const my_pkg::Command& input) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "upper=" << convert_int(input.upper) << "lower=" << convert_int(input.lower));

    my_pkg::Command output;
    output = function(input);

    pub.publish(output);

    ROS_INFO_STREAM("Sending modified value:" << " upper=" <<  convert_int(output.upper) << " lower=" << convert_int(output.lower));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pubsub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("my_topic0", 1000, &messageReceived);
  pub = nh.advertise<my_pkg::Command>("my_topic1", 1000);

  ros::spin();
}

mypkg::Command function(my_pkg::Command input) {
  mypkg::Command output;
  output.upper = input.upper - 10;
  output.lower = input.lower - 10;
  return output;
}
