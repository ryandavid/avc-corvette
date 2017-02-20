#!/usr/bin/env python

import argparse
import os
import rosbag
import simplekml
import sys

parser = argparse.ArgumentParser()
parser.add_argument("bag_path", help="Filepath to bag")
args = parser.parse_args()

bag_path = os.path.abspath(args.bag_path)

if os.path.exists(bag_path) is False:
    print "ERROR: Provided bag filepath is invalid."
    sys.exit()

pos_slns = []

bag = rosbag.Bag(bag_path, "r")
for topic, msg, ts in bag.read_messages(topics=["/bd950_driver_node/fix"]):
    # Note: its lon, lat, alt!
    pos_slns.append((msg.longitude, msg.latitude, msg.altitude))

bag.close()

print "Found {0} position solutions.".format(len(pos_slns))

kml_path = os.path.splitext(bag_path)[0] + ".kml"
print "Writing KML out to '{0}'".format(kml_path)

kml = simplekml.Kml()
kml.newlinestring(name="Path", coords=pos_slns)
kml.save(kml_path)
