#!/usr/bin/env python

import rospy
import sys
import select
import termios
import tty



'''
def get_char( c ):
    temp = sys.stdin.read(1)
    if c == temp :
        return c
    else:
        return temp 


'''

'''
def get_char( c ):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
'''


def get_char():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)



def keyboard_input():
    while not rospy.is_shutdown():
        if rospy.core.is_initialized():
            c = get_char( c ) 
            if c:
                rospy.loginfo('Key pressed: %s', c)
        else:
            rospy.rostime.wallsleep(0.1)

if __name__ == '__main__':
    rospy.init_node('keyboard_input_node')
    keyboard_input()
    while True:
        print( "by" )
