#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Quaternion

def callback(data):
	data = [data.x, data.y, data.z, data.w]
	print data

def listener():
	rospy.init_node('qr_listener', anonymous=False)
	rospy.Subscriber('qr_data', Quaternion, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()	