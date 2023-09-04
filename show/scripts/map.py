#!/usr/bin/env python


# ****************************************
# unused
# ****************************************
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

def map_callback(msg):
    # encode
    width = msg.info.width
    height = msg.info.height
    data = np.array(msg.data).reshape((height, width))

    # show map
    plt.imshow(data, cmap='gray')
    plt.axis('off')
    plt.show()

rospy.init_node('map_subscriber')
rospy.Subscriber('/map', OccupancyGrid, map_callback)
rospy.spin()
