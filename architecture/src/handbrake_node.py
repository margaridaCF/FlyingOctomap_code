#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from architecture_msgs.srv import *
import os

min_distance = 0.0
handbrake_on = False

def handle_enable_handbrake(req):
    handbrake_on = True
    return []

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
    safety_threshold = rospy.get_param("safety_margin")
    enable_handbrake_s = rospy.Service('enable_handbrake_trigger', EnableHandbrakeTrigger, handle_enable_handbrake)
    rospy.Subscriber('hokuyo_laser_scan', LaserScan, callback)
    stop_state_pub = rospy.Publisher('stop_uav', Empty, queue_size=10)
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
        # print "[Handbrake] Distance to closest obstacle: " + str(min_distance)
        # If the perceived minimum distance to the obstacles is less then a threshold, send a stopping command
        if handbrake_on and (min_distance <= safety_threshold) and (min_distance != 0.0):
            rospy.logwarn("[Handbrake] Distance to closest obstacle: %s", str(min_distance))
            try:
                stop_state_pub.publish(emergency_stop_msg)
                # print "Emergency message successfully sent!"
            except rospy.ROSException:
                rospy.logerr("Error while sending the emergency message")
        # else:
        #     print "[Handbrake] You're OK so far!"



if __name__ == '__main__':
    main()
