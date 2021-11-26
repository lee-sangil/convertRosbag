#!/usr/bin/env python

""" 
rosbag2txt.py is a Python script that converts the .rosbag output file from the rpg_davis_simulator [1] 
and optitrack_bridge[2] into a txt file whose format is TUM-style.

[1] rpg_davis_simulator: https://github.com/uzh-rpg/rpg_davis_simulator
[2] optitrack_bridge: https://github.com/qwerty35/optitrack_bridge

Author: Sangil Lee
Created: 26-11-2021
"""

import os
import sys
import shutil
import rosbag
import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError

dvs = "/dvs"
pose = "/optitrack/davis/poseStamped"

# input file
bagFile  = sys.argv[1]
bagSplit = bagFile.replace('/','.').replace('\\','.').split(".")

# check if the output file already exists
filename = bagSplit[len(bagSplit)-2]

if len(sys.argv) == 2:
	baseDir = os.path.join(os.getcwd(), filename)
elif len(sys.argv) == 3:
	baseDir = os.path.join(sys.argv[2], filename)

print(baseDir)

eventFileName = os.path.join(baseDir, 'events.txt')
imuFileName = os.path.join(baseDir, 'imu.txt')
imgFileName = os.path.join(baseDir, 'images.txt')
poseFileName = os.path.join(baseDir, 'pose.txt')

if os.path.isfile(eventFileName):
	os.remove(eventFileName)

if os.path.isfile(imuFileName):
	os.remove(imuFileName)

if os.path.isfile(imgFileName):
	os.remove(imgFileName)

if os.path.isfile(poseFileName):
	os.remove(poseFileName)

outputDir = baseDir + "/images/"
if os.path.exists(outputDir):
	shutil.rmtree(outputDir)
os.makedirs(outputDir)

print("\nFormatting: .rosbag -> .txt (This should take a couple of minutes)\n")

# open the file and write the headers
eventFile = open(eventFileName, "w")
eventFile.write('# timestamps x y p\r\n')

imuFile = open(imuFileName, "w")
imuFile.write('# timestamps ax ay az wx wy wz\r\n')

imgFile = open(imgFileName, "w")
imgFile.write('# timestamps filename\r\n')

poseFile = open(poseFileName, "w")
poseFile.write('# timestamps px py pz qx qy qz qw\r\n')

bridge = CvBridge()

# open the rosbag file and process the events
bag = rosbag.Bag(bagFile)
for topic, msg, t in bag.read_messages():
	if topic == dvs+"/events":
#		print(len(msg.events))
		for e in msg.events:

			ts = float(e.ts.to_nsec() * 1e-9)
			p = 1 if e.polarity else 0

			# write the event using big endian format
			eventFile.write("%10.6f %d %d %d\r\n" % (ts, e.x, e.y, p) )

	if topic == dvs+"/image_raw":
		try:
			cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			cv2.imwrite(outputDir+"%010d.%06d.png" % (t.secs, t.nsecs*0.001), cv_img)
		except e:
			print(e)

		# write the image filename
		imgFile.write("%010d.%06d /images/%010d.%06d.png\r\n" % (t.secs, t.nsecs*0.001, t.secs, t.nsecs*0.001) )

	if topic == dvs+"/imu":

		w = msg.angular_velocity
		a = msg.linear_acceleration

		# write the inertial measurement
		imuFile.write("%010d.%06d %.10f %.10f %.10f %.10f %.10f %.10f\r\n" % (t.secs, t.nsecs*0.001, a.x, a.y, a.z, w.x, w.y, w.z) )

	if topic == pose:
		pos = msg.pose.position
		orient = msg.pose.orientation

		print(t)

		poseFile.write("%010d.%06d %.10f %.10f %.10f %.10f %.10f %.10f %.10f\r\n" % (t.secs, t.nsecs*0.001, pos.x, pos.y, pos.z, orient.x, orient.y, orient.z, orient.w) )

bag.close()
