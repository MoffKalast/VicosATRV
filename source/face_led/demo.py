#!/usr/bin/env python

import time
import sys
import serial
import random
import roslib
import rospy

from collections import deque
from std_msgs.msg import String

# Wheel of emotions: https://i.redd.it/zs6xww1s8f911.jpg

def rgb2Hex(r, g, b):
	return (b << 4) | (g << 2)| r

def sendData(rgb):
	#print ''.join('{:02x}'.format(y) for y in rgb)
	port.flushInput()
	port.write(rgb)
	port.readline()

def talk(array):
	newarray = bytearray()
	newarray[:] = array
	for i in range(48,58):
		if bool(random.getrandbits(1)):
			newarray[i] = 0
	return newarray

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
#loadingmouth = deque([red, 0x00, 0x00, 0x00, red, 0x00, red, red, red, 0x00])
loadingmouth = deque([red, red, red, red, red, red, red, red, red, red])

def loadingLoop():
	loadingeyes.rotate(1)
	#loadingmouth.rotate(1)
	sendData(talk(encode(loadingmouth, loadingeyes, loadingeyes, rgb2Hex(1,0,0))))

emotions = {}

emotions["neutral"] = encode(
								[0x00, 0x00, 0x00, 0x00, 0x00, white, white, white, white, white],
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
								[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
								white
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

keys = emotions.keys()


port = serial.Serial('/dev/ttyACM0', 19200, timeout=0.2) #19200

mode = "neutral"
start_time = time.time()

mode = "loading"

index = 0

while True:

	if mode == "loading":
		loadingLoop()
	else:
		sendData(talk(emotions[mode]))
		time.sleep(0.01)

	if time.time() - start_time > 1.5:
		start_time = time.time()

		mode = keys[index]
		index = (index + 1) % len(keys)

		print(mode)
