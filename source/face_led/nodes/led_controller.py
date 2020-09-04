#!/usr/bin/env python

import time
import sys
import serial
import random
import roslib
import rospy

from collections import deque
from std_msgs.msg import String, Float32, Float32MultiArray

def rgb2Hex(r, g, b):
	return (b << 4) | (g << 2)| r

def encode(mouth, eye1, eye2, eyecolor):
	rgb = bytearray()

	for i in range(58):
		if i < 24:
			rgb.append(eye1[i]*eyecolor);
		elif i < 48:
			rgb.append(eye2[i-24]*eyecolor);
		elif i >= 48:
			rgb.append(mouth[i-48])
		else:
			rgb.append(0x00)

	return rgb

orange = rgb2Hex(3,2,0)
yellow = rgb2Hex(3,3,0)
cyan = rgb2Hex(0,3,3)
purple = rgb2Hex(3,0,3)

green = rgb2Hex(0,3,0)
red = rgb2Hex(3,0,0)
blue = rgb2Hex(0,0,3)
white = rgb2Hex(2,2,2)

loadingeyes = deque([0,0,0,0,0,0,0,0,0,1,1,2,2,3,3,3,0,0,0,0,0,0,0,0])
loadingmouth = [0x00, 0x00, 0x00, 0x00, 0x00, white, white, white, white, white]

emotions = {}

emotions["blank"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
								[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
								[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
								white
							)

emotions["neutral"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, white, white, white, white, white], #mouth [upper row, lower row]
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # left eye mask
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # right eye mask
								white # eye colour
							)


emotions["neutral_smile"] = encode(
								[white, 0x00, 0x00, 0x00, white, 0x00, white, white, white, 0x00], #mouth [upper row, lower row]
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # left eye mask
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # right eye mask
								white # eye colour
							)

emotions["neutral_frown"] = encode(
								[white, white, white, white, white, 0x00, 0x00, 0x00, 0x00, 0x00], #mouth [upper row, lower row]
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # left eye mask
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1], # right eye mask
								white # eye colour
							)

emotions["neutral_left"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, white, white, white, white, white],
								[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
								[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
								white
							)

emotions["neutral_right"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, white, white, white, white, white],
								[1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								[1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								white
							)

emotions["furious"] = encode(
								[0x00, red, red, red, 0x00, red, 0x00, 0x00, 0x00, red],
								[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								[0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
								red
							)

emotions["mad"] = encode(
								[orange, orange, orange, orange, orange, 0x00, 0x00, 0x00, 0x00, 0x00],
								[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								[1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								orange
							)

emotions["stupid"] = encode(
								[0x00, orange, orange, orange, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
								[0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0],
								[1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1],
								orange
							)

emotions["sad"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, blue, blue, blue, 0x00],
								[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0],
								[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0],
								blue
							)

emotions["miserable"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, cyan, cyan, cyan, 0x00],
								[0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
								[0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
								cyan
							)

emotions["happy"] = encode(
								[green, 0x00, 0x00, 0x00, green, 0x00, green, green, green, 0x00],
								[0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1],
								[0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1],
								green
							)

emotions["nauseated"] = encode(
								[purple, 0x00, purple, 0x00, purple, 0x00, purple, 0x00, purple, 0x00],
								[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
								[1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
								purple
							)

emotions["surprised"] = encode(
								[yellow, yellow, yellow, yellow, yellow, yellow, yellow, yellow, yellow, yellow],
								[1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
								[1,1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1],
								yellow
							)

emotions["loading"] = 0

def sendData(rgb):
	#print ''.join('{:02x}'.format(y) for y in rgb)
	try:
		port.flushInput()
		port.write(rgb)
		port.readline()
	except:
		rospy.logerr("Serial error!")


def blink(newarray, percentage, offset):
	eye = (2 * percentage - 1) ** 2

	eye = 1.0 - eye

	up_min = 6 - eye*6 + offset
	up_max = 6 + eye*6 + offset

	dn_min = 18 - eye*6 + offset
	dn_max = 18 + eye*6 + offset

	for i in range(offset,24+offset):
		if i < 12+offset:
			if not (i <= up_min or i >= up_max):
				newarray[i] = rgb2Hex(0,0,0)
		else:
			if not (i <= dn_min or i >= dn_max):
				newarray[i] = rgb2Hex(0,0,0)

def animate(array, anim_data):
	newarray = bytearray()
	newarray[:] = array

	#talk
	if anim_data[0]:
		for i in range(48,58):
			if bool(random.getrandbits(1)):
				newarray[i] = 0
	#blink
	blink(newarray, anim_data[1], 0)
	blink(newarray, anim_data[2], 24)
	return newarray

def emotion_callback(msg): # takes one of the keys in the emotions dictonary
	global mode
	data = msg.data
	if data in emotions:
		mode = data
		rospy.loginfo("New emotion mode: %s", mode)
	else:
		rospy.logerr("Unknown emotion: %s", data)

def talktimer_callback(msg): # number of seconds for which to animate mouth
	global talk_time, talk_start_time

	if msg.data > 0:
		talk_time = msg.data
		talk_start_time = time.time()
		rospy.loginfo("Animating mouth for %f seconds.", talk_time)

def eye_callback(msg): # speeds at which to blink
	global blinkleft_time, blinkright_time, blinkleft_start_time, blinkright_start_time

	if len(msg.data) != 2:
		rospy.logerr("Eye data error! Specify two floats.")
		return

	blinkleft_start_time = time.time()
	blinkleft_time = msg.data[0]

	blinkright_start_time = time.time()
	blinkright_time = msg.data[1]

def cleanup():
	sendData(emotions["blank"])
	port.close() 

keys = emotions.keys()
port = serial.Serial('/dev/ttyACM0', 19200, timeout=0.2) #19200
mode = "happy"

talk_time = -1
talk_start_time = time.time()

blinkleft_time = -1
blinkleft_start_time = time.time()

blinkright_time = -1
blinkright_start_time = time.time()

rospy.init_node('led_controller')
rospy.Subscriber("face_emotion", String, emotion_callback)
rospy.Subscriber("face_eye_anim", Float32MultiArray, eye_callback)
rospy.Subscriber("face_talk_anim", Float32, talktimer_callback)
rospy.loginfo("Waiting for commands...")

rospy.on_shutdown(cleanup)

rate = rospy.Rate(10)

while not rospy.core.is_shutdown():

	anim = False
	anim_data = [False, 1.0, 1.0] # talk, blink eye1 %, blink eye2 %
	data = emotions[mode]

	if talk_time > 0:
		anim_data[0] = True
		anim = True
		if time.time() - talk_start_time > talk_time:
			talk_time = -1

	if blinkleft_time > 0:
		deltatime = time.time() - blinkleft_start_time
		anim_data[2] = deltatime/blinkleft_time
		anim = True
		if deltatime > blinkleft_time:
			blinkleft_time = -1

	if blinkright_time > 0:
		deltatime = time.time() - blinkright_start_time
		anim_data[1] = deltatime/blinkright_time
		anim = True
		if deltatime > blinkright_time:
			blinkright_time = -1

	if mode == "loading":
		loadingeyes.rotate(1)
		if anim:
			sendData(animate(encode(loadingmouth, loadingeyes, loadingeyes, white)))
		else:
			sendData(encode(loadingmouth, loadingeyes, loadingeyes, white))	
	else:
		sendData(animate(data, anim_data))
		if not anim:
			rate.sleep()