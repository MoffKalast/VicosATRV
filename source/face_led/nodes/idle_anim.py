#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import sys
import os

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32, Float32MultiArray

class IdleAnim:

	def __init__(self):
		rospy.init_node('idle_anim', anonymous=False)

		self.pantilt_pub = rospy.Publisher("ptu/cmd", JointState, queue_size=1)
		self.emotion_pub = rospy.Publisher("face_emotion", String, queue_size=1)
		self.eyes_pub = rospy.Publisher("face_eye_anim", Float32MultiArray, queue_size=1)
		self.talk_pub = rospy.Publisher("face_talk_anim", Float32, queue_size=1)

	def say(self, time):
		msg = Float32()
		msg.data = time
		self.talk_pub.publish(msg)

	def emotion(self, string):
		msg = String()
		msg.data = string
		self.emotion_pub.publish(msg)

	def blink(self, left, right):
		array = Float32MultiArray()
		array.data = [left, right]
		self.eyes_pub.publish(array)

	def movehead(self, pan, tilt, speed):
		js = JointState()
		js.name = [ "ptu_pan", "ptu_tilt" ]
		js.velocity = [ speed, speed ]
		js.position = [ pan, tilt ]
		self.pantilt_pub.publish(js)

	def update(self):
		if random.random() > 0.4:
			split = random.random()
			if split < 0.25:
				self.blink(0.4, 0.4)
			elif split < 0.5:
				self.blink(random.random()*0.5+0.3, random.random()*0.5+0.3)
			elif split < 0.75:
				self.blink(random.random()*0.5+0.3, -1)
			else:
				self.blink(-1, random.random()*0.5+0.3)

		if random.random() > 0.65:
			self.movehead(random.random()*2.0-1.0, random.random()*0.2-0.1,0.6)

		if random.random() > 0.5:
			self.say(random.random()*0.5)

		if random.random() < 0.2:
			split = random.random()
			if split < 0.25:
				self.emotion("neutral")
			elif split < 0.4:
				self.emotion("neutral_right")
			elif split < 0.6:
				self.emotion("neutral_left")
			elif split < 0.65:
				self.emotion("neutral_frown")	
			else:
				self.emotion("neutral_smile")	

try:
	idle = IdleAnim()
	rate = rospy.Rate(0.5)
	while not rospy.core.is_shutdown():
		idle.update()
		rate.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
