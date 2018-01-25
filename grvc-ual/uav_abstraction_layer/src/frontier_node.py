#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import Header
import os, time

class frontier():
    """docstring for frontier."""
    def __init__(self):
        # Initiate Frontier Node
        rospy.init_node('frontier_node')

        # Setup Publisher & Subscriber
        rospy.Subscriber('/octomap', Empty, self.octomapCallback)
        self.sub_listener = rospy.SubscribeListener()
        self.sub_listener.peer_subscribe('/new_frontier', self.initialConnectionCallback, self.initialConnectionCallback)

        self.frontier_pub = rospy.Publisher('/new_frontier', PoseStamped, queue_size=10, subscriber_listener=self.sub_listener)

        # Initialise Frontier Pose message.
        self.frontier_pose_stamped_msg = PoseStamped()

        # Default values should all be 0, just explicit initialisation here.
        self.frontier_pose_stamped_msg.pose.position.x = float(0)
        self.frontier_pose_stamped_msg.pose.position.y = float(0)
        self.frontier_pose_stamped_msg.pose.position.z = float(2)
        self.frontier_pose_stamped_msg.pose.orientation.x = float(0)
        self.frontier_pose_stamped_msg.pose.orientation.y = float(0)
        self.frontier_pose_stamped_msg.pose.orientation.z = float(0)
        self.frontier_pose_stamped_msg.pose.orientation.w = float(0)

        self.frontierSpin()

    def initialConnectionCallback():
        # TODO: Remove below
        try:

            print "Call Backed"

            self.frontier_pose_stamped_msg.header.stamp = rospy.Time.now()
            self.frontier_pub.publish(self.frontier_pose_stamped_msg)
            print "Published"
        except rospy.ROSException as e:
            print e
        ###

    def octomapCallback(self, data):
        pass

    def frontierSpin(self):
        rospy.spin()

if __name__ == '__main__':
    start = frontier()
