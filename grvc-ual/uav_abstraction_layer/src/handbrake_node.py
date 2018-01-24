#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import os

min_distance = 0.0

def callback(data):
    # print str(data.ranges)
    global min_distance
    min_distance = np.nanmin(data.ranges)
    # print min_distance
    # print str(min_distance) + "\n"


def main():
    # Initiate our node as a listener
    # Implement callback to handle data received via subscription
    # min_distance = None
    rospy.init_node('handbrake_listen')
    safety_threshold = 2.0
    rospy.Subscriber('/depth_laser_scan', LaserScan, callback)
    stop_pub = rospy.Publisher('/stop_uav', Empty, queue_size=10)
    emergency_stop_msg = Empty()
    # emergency_stop_msg = Twist()
    # emergency_stop_msg.linear.x = 0.0
    # emergency_stop_msg.linear.y = 0.0
    # emergency_stop_msg.linear.z = 0.0
    # emergency_stop_msg.angular.x = 0.0
    # emergency_stop_msg.angular.y = 0.0
    # emergency_stop_msg.angular.z = 0.0

    # rospy.spin()
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        print "Distance to closest obstacle: " + str(min_distance)
        # If the perceived minimum distance to the obstacles is less then a threshold, send a stopping command
        if min_distance <= safety_threshold and min_distance != 0.0:
            try:
                stop_pub.publish(emergency_stop_msg)
                print "Emergency message successfully sent!"
            except rospy.ROSException:
                print "Error while sending the emergency message"
        else:
            print "You're OK so far!"



if __name__ == '__main__':
    main()
