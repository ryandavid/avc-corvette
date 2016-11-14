#!/usr/bin/env python

import rospy
from bd950_driver.msg import NavSVInfoArray

def rxCallback(data):
	print chr(27) + "[2J"

	print "*" * 40
	print "Num SV : {0}".format(len(data.sv))
	print "Time: {0}".format(data.header.stamp)
	print "*" * 40
	for sv in data.sv:
		used = ""
		if(sv.usedInSln > 0):
			used += "In Use"

		if(sv.usedInRTK > 0):
			used += "(RTK)"

		print "  PRN {0} {1}".format(sv.prn, used)
		print "    - L1 : {0}".format("x" * int(sv.snrL1))
		print "    - L2 : {0}".format("x" * int(sv.snrL2))


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("/bd950_driver_node/svInfo", NavSVInfoArray, rxCallback)
rospy.spin()

