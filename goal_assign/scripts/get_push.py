#!/usr/bin/env python

"""
This script is used to get the position from topic /vector 
transform it to PoseStamped
and publish it to /move_base_simple/goal
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

def callback(data):
    global goal_publisher
    goal_msg = PoseStamped()
    goal_msg.header.seq = 0
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"

    # transform the data to PoseStamped
    # layout: 
    #     dim: []
    #     data_offset: 0
    # data: [655.0, 507.0]

    x = data.data[0]
    y = data.data[1]

    #  0, 0 => (-50, 50)
    #  992, 992 => (0, 0)
    #  1984, 1984 => (50, -50)
    
    x = (x - 780) / 40
    y = (330 - y) / 40

    goal_msg.pose.position.x = x  
    goal_msg.pose.position.y = y 
    goal_msg.pose.position.z = 0.0

    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.9999032988
    goal_msg.pose.orientation.w = -0.0139065829385

    goal_publisher.publish(goal_msg)
    rospy.loginfo("Goal has been sent!")
    rospy.sleep(1)  # wait for the publisher to be ready

def get_push():
    global goal_publisher
    rospy.init_node('custom_goal_publisher')
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.Subscriber("/vector", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    get_push()