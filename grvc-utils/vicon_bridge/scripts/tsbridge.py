#!/usr/bin/env python
# license removed for brevity
import rospy, tf, socket
from geometry_msgs.msg import PoseStamped
from math import *

def py_trans2global(x,y,z,lat0,lon0,h0,Az0):
    import numpy as np

    Re = 6378
    e  = 3.352 * (10 ** -3)

    lat0 = lat0 * pi / 180
    lon0 = lon0 * pi / 180

    coslat0 = cos(lat0)
    sinlat0 = sin(lat0)
    den = ( 1 - (e ** 2)  * ( sinlat0 ** 2 ) )
    Rm = Re * ( 1 - e ** 2 ) / ( den ** (3 / 2) )
    Rn = Re / sqrt(den)

    M = np.array([cos(Az0), -sin(Az0), 0, sin(Az0), cos(Az0), 0, 0, 0, 1]).reshape(3,3)
    pos_local = np.array([x, y, z]).reshape(3,1)
    pos_global = np.dot(M,pos_local)
    xn = pos_global[0,0]
    yn = pos_global[1,0]
    zn = pos_global[2,0]

    lat = xn / Rm + lat0
    lon = yn / (Rn * coslat0 ) + lon0
    h = h0 + zn

    return floor(lat*1e7), floor(lon*1e7), h


def talker():
    rospy.init_node('tsbridge', anonymous=True)

    output = PoseStamped()
    TCP_IP = '0.0.0.0'
    TCP_PORT = 8000
    BUFFER_SIZE = 1024  # Example {-0.589207,-2.744139,-1.105899,7486517.000000}

    # TSReceiver Socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((TCP_IP, TCP_PORT))
    s.listen(1)

    conn, addr = s.accept()
    rospy.loginfo('Connection address: %s', addr)

    pub = rospy.Publisher('mavros/mocap/pose', PoseStamped, queue_size=10)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        data = conn.recv(BUFFER_SIZE)
        print data    # Debug input
        index = data.index('}')
        parseData = data[1:index]
        parseData = parseData.split(";")
        x,y,z,t = float(parseData[0]), float(parseData[1]), float(parseData[2]), float(parseData[3])

        output.pose.position.x = x
        output.pose.position.y = y
        output.pose.position.z = z

        output.pose.orientation.x = 0
        output.pose.orientation.y = 0
        output.pose.orientation.z = 0
        output.pose.orientation.w = 1

        # lat, lon, h = py_trans2global(x,y,z,0,0,0,0)
        # print "%s %d %d %f %d" % (topic, lat, lon, h, t) # Debug output
        print "%f %f %f" % (x, y, z) # Debug output

        pub.publish(output)
        # rate.sleep()

    conn.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
