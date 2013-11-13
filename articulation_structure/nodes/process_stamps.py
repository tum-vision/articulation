#!/usr/bin/python

import roslib; roslib.load_manifest('articulation_structure')
import rospy
import rosbag
import sys

def process_bags(infile,
								outfile):

	outbag = rosbag.Bag(outfile,"w")
	for topic,msg,t in rosbag.Bag(infile).read_messages():
		outbag.write(topic,msg,msg.header.stamp)

		if rospy.is_shutdown():
			break;
		
	outbag.close()

if __name__ == '__main__':
	if len(sys.argv) != 3:
		print "Usage: process_structure <in.bag> <out.bag>"
		sys.exit()

	infile = sys.argv[1]
	outfile = sys.argv[2]
	process_bags(infile,outfile)
