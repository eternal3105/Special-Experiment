#! /usr/bin/env python
# -*- coding: utf-8 -*-


# this code will stored the first imu data 
# callback will check the imu data and compare with the first imu data
# once the robot turn exactly 90 degree, it will print a msg "90990" and stop this node

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

first_imu_data = None

def imu_callback(msg):
    global first_imu_data
    quaternion = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    current_yaw = euler[2]

    # 儲存第一筆IMU數據
    if first_imu_data is None:
        first_imu_data = current_yaw
        return

    # 檢查與第一筆數據的差異是否正好90度
    difference = math.degrees(current_yaw - first_imu_data)
    difference = (difference + 360) % 360  # 確保範圍在0到360度之間

    if abs(difference - 90) < 1: # 允許1度的誤差
        print("90990")
        rospy.signal_shutdown('Turned 90 degrees')

if __name__ == '__main__':
    rospy.init_node('imu_subscriber')
    imu_sub = rospy.Subscriber('/imu', Imu, imu_callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 設定旋轉速度
    twist = Twist()
    twist.angular.z = 0.5 # 設定合適的旋轉速度

    # 每秒發布旋轉命令，直到節點被關閉
    rate = rospy.Rate(1) # 10 Hz
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rate.sleep()

    rospy.spin()




