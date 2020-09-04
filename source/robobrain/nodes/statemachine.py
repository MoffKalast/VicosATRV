#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
import sys
import os

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32, Float32MultiArray

class RobotState:

	def __init__(self):
		rospy.init_node('robot_state', anonymous=False)
	

	def update(self):
		pass

try:
	state = RobotState()
	rate = rospy.Rate(0.5)
	while not rospy.core.is_shutdown():
		state.update()
		rate.sleep()

except rospy.ROSInterruptException:
	print("Script interrupted", file=sys.stderr)
