#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def keyboard_callback(data):
    key = data.data
    if key == 'w':

        pass
    elif key == 's':

        pass
    elif key == 'a':

        pass
    elif key == 'd':

        pass
    elif key == 'q':

        rospy.signal_shutdown('Quit')

if __name__ == '__main__':
    rospy.init_node('keyboard_controller')
    rospy.Subscriber('/keyboard', String, keyboard_callback)
    rospy.spin()