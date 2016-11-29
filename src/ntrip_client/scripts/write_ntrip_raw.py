#!/usr/bin/env python

import os
import rospy
from ntrip_client.msg import ByteArray
import sys

def rxCallback(data):
	converted = bytearray(data.data)
	fHandle.write(converted)
	rospy.loginfo("I wrote {0} bytes.".format(len(converted)))

	for byte in converted:
		print "%02X" % byte



if(len(sys.argv) != 2):
	print "You must supply an output filename!"
	sys.exit(0) 

filename = os.path.abspath(sys.argv[1])
fHandle = open(filename, "wb")
print "Writing to {0}".format(filename)

rospy.init_node("listener", anonymous=True)
rospy.Subscriber("/ntrip/data", ByteMultiArray, rxCallback)

rospy.spin()

fHandle.close()
