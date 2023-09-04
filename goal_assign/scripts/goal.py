#!/usr/bin/env python

"""
simply assign a goal to the robot
"""

import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('custom_goal_publisher')
goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
rospy.sleep(1)  # wait for the publisher to be ready

goal_msg = PoseStamped()
goal_msg.header.seq = 0
goal_msg.header.stamp = rospy.Time.now()
goal_msg.header.frame_id = "map"

goal_msg.pose.position.x = 1.50254900455  
goal_msg.pose.position.y = -0.0506183719635 
goal_msg.pose.position.z = 0.0

goal_msg.pose.orientation.x = 0.0
goal_msg.pose.orientation.y = 0.0
goal_msg.pose.orientation.z = 0.9999032988
goal_msg.pose.orientation.w = -0.0139065829385

goal_publisher.publish(goal_msg)
rospy.loginfo("Goal has been sent!")

rospy.sleep(1)  # wait for the publisher to be ready