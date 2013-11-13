#!/usr/bin/env python

import roslib; roslib.load_manifest('articulation_structure')
import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32,CameraInfo,CompressedImage
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
import process_bags

def main():
	rospy.init_node('process_structure')
	if len(rospy.myargv()) != 3:
		print "Usage: process_structure <in.bag> <out.bag>"
		return

	infile = rospy.myargv()[1]
	outfile = rospy.myargv()[2]

	in_topics = ['/camera/camera_info','/camera/image_raw/compressed',
			'/camera/pose_array','/camera/visualization_marker_array']
	in_types = [CameraInfo,CompressedImage,PoseArray,MarkerArray]

	out_topics = ['/track','/model','/structure_array','/object']
	out_types = [TrackMsg,ModelMsg,MarkerArray,ArticulatedObjectMsg]

	timeout = 5

	process_bags.process_bags(infile,in_topics,in_types,
								outfile,out_topics,out_types,
								timeout)


if __name__ == '__main__':
  main()

