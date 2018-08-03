#include <ros/ros.h>
#include <my_pkg/Command.h>
#include <iomanip>
#include <sstream>
using namespace std;

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

void messageReceived(const my_pkg::Command& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "upper=" << convert_int(msg.upper) << "lower=" << convert_int(msg.lower));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("my_topic1", 1000, &messageReceived);

  ros::spin();
}
