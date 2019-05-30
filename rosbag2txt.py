#!/usr/bin/env python

""" 
rosbag2txt.py is a Python script that converts the .rosbag output file from the rpg_davis_simulator [1] 
into a txt file whose format is TUM-style.

[1] rpg_davis_simulator: https://github.com/uzh-rpg/rpg_davis_simulator

Author: Sangil Lee
Created: 30-05-2019
"""

import os
import sys
import shutil
import rosbag
import struct
import cv2
from cv_bridge import CvBridge, CvBridgeError

dvs = "/davis/left"

# input file
bagFile  = sys.argv[1]
bagSplit = bagFile.replace('/','.').replace('\\','.').split(".")

# check if the output file already exists
filename = bagSplit[len(bagSplit)-2]
baseDir = os.path.join(os.getcwd(), filename)
eventFileName = os.path.join(baseDir, 'events.txt')
imuFileName = os.path.join(baseDir, 'imu.txt')
imgFileName = os.path.join(baseDir, 'images.txt')

if os.path.isfile(eventFileName):
	os.remove(eventFileName)

if os.path.isfile(imuFileName):
	os.remove(imuFileName)

if os.path.isfile(imgFileName):
	os.remove(imgFileName)

outputDir = baseDir + "/images/"
if os.path.exists(outputDir):
	shutil.rmtree(outputDir)
os.makedirs(outputDir)

print "\nFormatting: .rosbag -> .txt (This should take a couple of minutes)\n"

# open the file and write the headers
eventFile = open(eventFileName, "w")
eventFile.write('# timestamps x y p\r\n');

imuFile = open(imuFileName, "w")
imuFile.write('# timestamps ax ay az wx wy wz\r\n');

imgFile = open(imgFileName, "w")
imgFile.write('# timestamps filename\r\n');

bridge = CvBridge()

# open the rosbag file and process the events
bag = rosbag.Bag(bagFile)
for topic, msg, t in bag.read_messages():
	if topic == dvs+"/events":
	    for e in msg.events:

			ts = float(e.ts.to_nsec() / 1e9)
			p = 1 if e.polarity else 0

		    # write the event using big endian format
			eventFile.write("%10.6f %d %d %d\r\n" % (ts, e.x, e.y, p) )

	if topic == dvs+"/image_raw":
		try:
			cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			cv2.imwrite(outputDir+"%010d.%06d.png" % (t.secs, t.nsecs/1000.0), cv_img)
		except CvBridgeError, e:
			print e

		# write the image filename
		imgFile.write("%010d.%06d /images/%010d.%06d.png\r\n" % (t.secs, t.nsecs/1000.0, t.secs, t.nsecs/1000.0) )

	if topic == dvs+"/imu":

		w = msg.angular_velocity
		a = msg.linear_acceleration

		# write the inertial measurement
		imuFile.write("%010d.%06d %.10f %.10f %.10f %.10f %.10f %.10f\r\n" % (t.secs, t.nsecs/1000.0, a.x, a.y, a.z, w.x, w.y, w.z) )

bag.close()
