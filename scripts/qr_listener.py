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
from roscopter.msg import State

class MCN():

    def __init__(self):
        rospy.init_node('listener', anonymous=False)
        self.axes = []
        self.buttons = []
        self.twist = [0, 0, 0, 0, 1500, 1500, 1500, 1500]
        self.x = 1500.0 #Side Tilt
        self.y = 1500.0 #Front Tilt
        self.z = 1000.0 #Throttle
        self.yaw = 1500 #Spin
        self.armed = False
        self.qr_found = False
        self.data = [0., 0., 0., 0.]

        self.qr_data = rospy.Subscriber('vision', Quaternion, self.vision_callback)
        self.pub_rc = rospy.Publisher('/send_rc', roscopter.msg.RC)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_state = rospy.Subscriber("/state", State, self.check_state)
        self.command_serv = rospy.ServiceProxy('command', APMCommand)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.fly()
            r.sleep()

    def check_state(self, data):
        self.armed = data.armed

    def vision_callback(self, data):
        old_data = self.data
        self.data = [data.x, data.y, data.z, data.w]
        self.qr_found = old_data != self.data

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons
        self.x = 1500-self.axes[0]*300 #Scales 1200-1800
        self.y = 1500-self.axes[1]*300 #Scales 1200-1800
        self.z = 2000+(self.axes[3])*1000 #Scales 1000-2000
        self.yaw = 1500-self.axes[2]*300 #Scales 1200-1800

    def fly(self):
        if self.buttons:
            if self.buttons[2]:
                self.command_serv(roscopter.srv.APMCommandRequest.CMD_ARM)
                print 'Arm Quad'
            if self.buttons[3]:
                self.command_serv(4)
                print 'Disarm Quad'
            if self.buttons[8]:
                self.command_serv(7)
                print 'Stabilize'
            if self.buttons[10]:
                self.command_serv(11)
                print 'Land'
            if self.buttons[11]:
            	self.search()
            	print 'Find Fiducial'
        
        if self.armed:
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))
            self.pub_rc.publish(self.twist)

    def search(self):
        while ~self.qr_found:
            self.x = 1800
            (self.twist[0], self.twist[1], self.twist[2], self.twist[3]) = (int(self.x), int(self.y), int(self.z), int(self.yaw))

if __name__ == '__main__':
    print 'process started at ' + str(datetime.now())
    try:
        var = MCN()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
    #TODO shutdown when ROS exited