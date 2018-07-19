#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    str = "hello world %s" % rospy.get_time()
    rospy.loginfo(str)
    pub.publish(String(str))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
