#!/usr/bin/env python
import rospy
import roslib
import time
roslib.load_manifest('roscopter')
from geometry_msgs.msg import Quaternion
import roscopter.msg

from std_srvs.srv import *
from std_msgs.msg import String, Header, Int32
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy, Imu, NavSatStatus, NavSatFix
from datetime import datetime
from roscopter.srv import APMCommand
from roscopter.msg import VFR_HUD, State

class QR_Listener:
	def __init__(self):
		rospy.init_node('qr_listener', anonymous=False)

		self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
		self.x = 1500.
		self.y = 1500.
		self.z = 1000.
		self.yaw = 1500.
		self.alt = 0.
		self.armed = False
		self.qr_found = False
		self.data = [0., 0., 0., 0.]

		self.qr_data = rospy.Subscriber('vision', Quaternion, self.callback)
		self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
		self.sub_state = rospy.Subscriber('/state', State, self.check_armed)
		self.command_serv = rospy.ServiceProxy('command', APMCommand)

		r = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.search()
			r.sleep()

	def callback(self, data):
		old_data = self.data
		self.data = [data.x, data.y, data.z, data.w]
		self.qr_found = old_data != self.data

	def check_armed(self, data):
		self.armed = data.armed

	def parse_action(self, data):
		self.airspeed = data.airspeed
		self.groundspeed = data.groundspeed
		self.heading = data.heading
		self.throttle = data.throttle
		self.alt = data.alt
		self.plt_alt.append(self.alt)
		self.climb = data.climb

	def search(self):
		if not self.armed:
			time.sleep(10)

 			self.command_serv(3)
 			self.armed = True

 		if self.armed:
	 		if not self.qr_found:
				(self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1700, 1500, 1500, 1500)
			else:
				(self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (1500, 1500, 1500, 1500)
			self.pub_rc.publish(self.twist)

if __name__ == '__main__':
	var = QR_Listener()