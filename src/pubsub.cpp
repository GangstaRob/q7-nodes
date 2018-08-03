#include <ros/ros.h>
#include <mypkg/Command.h>
#include <iomanip>
#include <sstream>
using namespace std;

mypkg::Command function(mypkg::Command); 

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

ros::Publisher pub;

void messageReceived(const mypkg::Command& input) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "upper=" << convert_int(input.upper) << "lower=" << convert_int(input.lower));

    mypkg::Command output;
    output = function(input);

    pub.publish(output);

    ROS_INFO_STREAM("Sending modified value:" << " upper=" <<  convert_int(output.upper) << " lower=" << convert_int(output.lower));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pubsub");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("my_topic0", 1000, &messageReceived);
  pub = nh.advertise<mypkg::Command>("my_topic1", 1000);

  ros::spin();
}

mypkg::Command function(mypkg::Command input) {
  mypkg::Command output;
  output.upper = input.upper - 10;
  output.lower = input.lower - 10;
  return output;
}
