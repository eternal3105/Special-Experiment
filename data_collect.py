#!/usr/bin/env python

import rospy

# import message file
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo( data.ranges )

def listener():
    rospy.init_node('listener', anonymous=True)

    # subscribe topic name: '/scan', data type: LaserScan, callback function
    rospy.Subscriber('/scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
