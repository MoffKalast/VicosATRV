#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import sys
import os
import math

from dnn_detect.msg import DetectedObjectArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Float32, Float32MultiArray

#camera info:

#height: 480, fov: 40
#width: 640, fov: 65

class Tracking:

	def __init__(self):
		rospy.init_node('person_tracking', anonymous=False)

		self.state_pub = rospy.Publisher("tracking_status", String, queue_size=1)
		self.pantilt_pub = rospy.Publisher("ptu/cmd", JointState, queue_size=1)

		self.bridge = CvBridge()
		self.run = True

		self.c_pan = 0.0
		self.c_tilt = 0.0
		self.vel = 0.0

		self.prevdeltax = 0.0
		self.prevdeltay = 0.0

		self.pantilt_sub = rospy.Subscriber("ptu/state", JointState, self.tilt_status)
		self.dnn_sub = rospy.Subscriber("/dnn_objects", DetectedObjectArray, self.dnn_objects)

	def tilt_status(self, msg):
		print(msg)
		self.c_pan = msg.position[0]
		self.c_tilt = msg.position[1]
		self.vel = msg.velocity[0] + msg.velocity[1]

	def movehead(self, pan, tilt, speed):

		if pan < -2 or pan > 2:
			return

		if tilt < -0.5 or tilt < 0.5:
			return

		js = JointState()
		js.name = [ "ptu_pan", "ptu_tilt" ]
		js.velocity = [ speed, speed ]
		js.position = [ pan, tilt ]
		self.pantilt_pub.publish(js)

		self.c_pan = pan
		self.c_tilt = tilt

	def dnn_objects(self, msg):
		if len(msg.objects) >= 1:

			people = []
			for x in msg.objects:
				if x.class_name == "person":
					people.append(x)

			if len(people) == 0:
				return

			#print(people[0])

			hei_y = people[0].y_max - people[0].y_min

			x = (people[0].x_min + people[0].x_max) / 2.0
			y = (people[0].y_min + people[0].y_max) / 2.0 - hei_y*3/4

			# degrees off center
			deltax = (x/640.0 - 0.5) * 65
			deltay = (y/480.0 - 0.5) * 40

			if math.fabs(deltax) < 3.0:
				deltax = 0
			if math.fabs(deltay) < 3.0:
				deltay = 0

			self.prevdeltax = self.prevdeltax * 0.9 + deltax * 0.1
			self.prevdeltay = self.prevdeltay * 0.9 + deltay * 0.1

			#print(self.vel)

	def update(self):
		if self.c_pan > -2 and self.c_pan < 2 and math.fabs(self.vel) < 0.05:
			self.movehead(self.c_pan - math.radians(self.prevdeltax), self.c_tilt - math.radians(self.prevdeltay), 0.6)


try:
	track = Tracking()
	rate = rospy.Rate(1)
	while not rospy.core.is_shutdown():
		track.update()
		rate.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
