#!/usr/bin/env python
# license removed for brevity
import rospy, tf
from geometry_msgs.msg import PoseStamped

global pub

def callback(data):
    global pub
    pub.publish(data)


def talker():
    global pub
    rospy.init_node('bridge', anonymous=True)

    pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)
    rospy.Subscriber("vicon_client/MBZIRC4/pose", PoseStamped, callback)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
