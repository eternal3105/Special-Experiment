#! /usr/bin/env python
# -*- coding: utf-8 -*-

# 本範例為使用 PID 控制，讓機器人旋轉 x 度
# 可以自由調整角度 以達到正確的轉向

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

first_imu_data = None
target_angle = 90 # 目標旋轉角度
Kp = 0.05 # 比例增益
Ki = 0.0 # 積分增益
Kd = 0.5 # 微分增益
prev_error = 0
integral = 0

def imu_callback(msg):
    global first_imu_data, prev_error, integral
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
        return

    error = target_angle - (current_yaw - first_imu_data)
    error = (error + 360) % 360 - 180 # Normalize to -180 to 180

    integral += error
    derivative = error - prev_error

    # PID 控制
    angular_z = Kp * error + Ki * integral + Kd * derivative

    twist = Twist()
    twist.angular.z = angular_z
    cmd_vel_pub.publish(twist)

    prev_error = error

    if abs(error) < 1: # 允許1度的誤差
        print("90990")
        rospy.signal_shutdown('Turned 90 degrees')

if __name__ == '__main__':
    rospy.init_node('imu_subscriber')
    imu_sub = rospy.Subscriber('/imu', Imu, imu_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.spin()
