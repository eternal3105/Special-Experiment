#!/usr/bin/env python

import time
import sys
import rospy
import key_control
from std_msgs.msg import String


from geometry_msgs.msg import Twist 

# from std_msgs.msg import String		# for topic

def talker():
    # set the conclu. between node and topic
    # syntex
# publisher=rospy.Publisher(topic_name, msg_class, queue_size)
    pub = rospy.Publisher('/rosky01/cmd_vel', Twist, queue_size=1)
    speed = 2
    rotate_speed = 1
    mode = False
    
    rospy.init_node('hello', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    move = Twist()
    
    '''
    linear only x can work,
    if x > 0 then it will go straight,
    if x < 0 then it will go backward.
    -------------------------------------------
    angular only z can work,
    if z > 0 then it will go counter-clockwise,
    if z < 0 then it will go clockwise.
    '''
    temp = ''
    while not rospy.is_shutdown():
        signal = key_control.get_char()
        if ( signal == "j" ):
            mode = True

        if ( mode == False ):
            if signal != "" :
                temp = signal

            if signal == "" :
                signal = temp
            
            if signal == "w" :
                move.linear.x = speed
            
            elif signal == "x":
                move.linear.x = -1 * speed

            elif signal == "a":
                move.angular.z = -1 * rotate_speed

            elif signal == "d":
                move.angular.z = rotate_speed

            elif signal == "q":
                move.linear.x = speed
                move.angular.z = -1 * rotate_speed

            elif signal == "e":
                move.linear.x = speed
                move.angular.z = rotate_speed

            elif signal == "z":
                move.linear.x = -1 * speed
                move.angular.z = -1 * rotate_speed

            elif signal == "c":
                move.linear.x = -1 * speed
                move.angular.z = rotate_speed

            elif signal == "s" :
                move.linear.x = 0
                move.angular.z = 0

            elif signal == "^c":
                exit()
        else:
            print( "enter the speed key:" )
            signal = key_control.get_char()
            print( ">" )
            print( signal )
            if signal == "r":
                speed = 2
                rotate_speed = 1

            elif signal == "u":
                speed = speed + 1

            elif signal == "m":
                speed = 0
                print( speed )

            elif signal == "h":
                rotate_speed = rotate_speed - 1

            elif signal == "k":
                rotate_speed = rotate_speed + 1

            mode = False

            
        #print( c )
        pub.publish(move)
            #time.sleep(1)
            # meter / second

    """
    move.linear.x = 2
    pub.publish(move)
    print("s")

    
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    """

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
