#!/usr/bin/env python
# -*- coding: utf-8 -*-

# param target_angle: 目標角度

"""
這個程式會持續以一個固定的半徑旋轉，直到 /imu topic 的方位角度達到目標角度。
target_angle: 目標角度
first_imu_data: 第一次接收到的方位角度
current_yaw: 當前方位角度
error: 目標角度與當前方位角度的誤差

半徑預設20cm
目標角度預設120度

目前只能逆時針旋轉
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

first_imu_data = None
current_yaw = 0

def imu_callback(msg):
    global first_imu_data, current_yaw
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    current_yaw = math.degrees(euler[2])

    if first_imu_data is None:
        first_imu_data = current_yaw

def move_in_circle():
    global first_imu_data, current_yaw
    rospy.init_node('circle_mover')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/imu', Imu, imu_callback)

    target_angle = rospy.get_param('~target_angle', 120)
    radius = 0.2

    while first_imu_data is None:
        rospy.sleep(0.1)
    
    # 計算目標角度
    # target_angle = first_imu_data + target_angle
    # 角速度 & 線速度 
    linear_velocity = 0.1
    angular_velocity = linear_velocity / radius

    twist = Twist()
    relative_yaw = current_yaw - first_imu_data
    
    error = (target_angle - relative_yaw ) % 360 

    while abs(error) > 3: # 允許1度的誤差
        print(error)
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        pub.publish(twist)
        relative_yaw = current_yaw - first_imu_data
        error = (target_angle - relative_yaw ) % 360 
        rospy.sleep(0.1)

    # 停止移動
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == '__main__':
    move_in_circle()
