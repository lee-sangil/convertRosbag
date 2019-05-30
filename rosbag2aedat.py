#!/usr/bin/env python

""" 
rosbag2aedat.py is a Python script that converts the .rosbag output file from the rpg_davis_simulator [1] into a 
2.0 .aedat file [2] that can be processed by the software jAER [3].

[1] rpg_davis_simulator: https://github.com/uzh-rpg/rpg_davis_simulator
[2] AEDAT file formats: https://inilabs.com/support/software/fileformat/
[3] jAER: https://sourceforge.net/p/jaer/wiki/Home/

Author: F. Paredes Valles
Modifier: Sangil Lee
Created: 03-05-2017
Modified: 30-05-2019
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
base_dir = os.path.join(os.getcwd(), filename)
aedatFile = os.path.join(base_dir, filename) + '.aedat'

if os.path.isfile(aedatFile):
	os.remove(aedatFile)

output_dir = base_dir + "/images/"
if os.path.exists(output_dir):
	shutil.rmtree(output_dir)
os.makedirs(output_dir)

print "\nFormatting: .rosbag -> .aedat (This should take a couple of minutes)\n"

# open the file and write the headers
file = open(aedatFile, "w")
file.write('#!AER-DAT2.0\r\n')
file.write('# This is a raw AE data file created by saveaerdat.m\r\n');
file.write('# Data format is int32 address, int32 timestamp (8 bytes total), repeated for each event\r\n');
file.write('# Timestamps tick is 1 us\r\n');

bridge = CvBridge()

# open the rosbag file and process the events
bag = rosbag.Bag(bagFile)
for topic, msg, t in bag.read_messages():
	if topic == dvs+"/events":
	    for e in msg.events:

			ts = int(e.ts.to_nsec() / 1000.0)
			x = '{0:07b}'.format(e.x)
			y = '{0:07b}'.format(e.y)
			p = '1' if e.polarity else '0'
			address = "0" + y + x + p

		    # write the event using big endian format
			file.write("%s" % struct.pack('>I', int(address, 2)))
			file.write("%s" % struct.pack('>f', float(ts)))

	if topic == dvs+"/image_raw":
		try:
			cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			cv2.imwrite(output_dir+"%010d.%06d.png" % (t.secs, t.nsecs/1000.0), cv_img)
		except CvBridgeError, e:
			print e

bag.close()
