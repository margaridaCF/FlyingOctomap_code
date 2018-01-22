#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan

min_distance = None

def callback(data):
    latest_scan = data.ranges
    min_distance = np.nanmin(latest_scan)
    #Handle data
    pass

def listener():
    # Initiate our node as a listener
    # Implement callback to handle data received via subscription

    rospy.init_node('handbrake_listen', anonymous=True)

    rospy.Subscriber("/depth_laser_scan", LaserScan, callback=callback)
    print "DEBUG: "+str(min_distance)

    rospy.spin()

if __name__ == '__main__':
    listener()
