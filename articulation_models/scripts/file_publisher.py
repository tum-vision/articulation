#!/usr/bin/env python

"""
 usage: %(progname)s [--incremental] [--loop] <track_file1.txt> ...
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
import sys
from articulation_models.msg import TrackMsg
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import ChannelFloat32
import logging
import fileinput
import getopt

def usage(progname):
  print __doc__ % vars()


def talker(files,flag_incremental,flag_loop):
  pub = rospy.Publisher('track', TrackMsg)
	
  if flag_incremental:
    rospy.loginfo("Publishing track incrementally, (will sleep 1s after each new pose)")  
  else:
    rospy.loginfo("Publishing a single track per file, immediately")
    rospy.sleep(0.4)  

  iter=0
  while iter==0 or flag_loop:
		 
		for filename in files:
		  rospy.loginfo("Opening logfile '%s'",filename)
		  
		  msg = TrackMsg()
		  channel_w = None
		  channel_h = None
		  for line in fileinput.input(filename):
		    line = line.strip()
		    rospy.loginfo("Reading line: '%s'",line)
		    data = line.strip().split()
		    data_float = [float(column) for column in data]
		    if len(data) < 3:
		    	 continue

		    msg.header.stamp = rospy.get_rostime()
		    msg.header.frame_id = "/"

		    msg.id = 0
		    pose = Pose( Point(0,0,0), Quaternion(0,0,0,1) )
		    
		    if len(data) >= 3:

		      pose.position.x = data_float[0]
		      pose.position.y = data_float[1]
		      pose.position.z = data_float[2]
		      
		    if len(data) >= 7:

		      pose.orientation.x = data_float[3]
		      pose.orientation.y = data_float[4]
		      pose.orientation.z = data_float[5]
		      pose.orientation.w = data_float[6]
		      
		    if len(data) >= 9:
		    	if channel_w is None or channel_h is None:
						channel_w = ChannelFloat32('width', [])
						channel_h = ChannelFloat32('height', [])
						msg.channels.append(channel_w)
						msg.channels.append(channel_h)
		    	channel_w.values.append(data_float[7])
		    	channel_h.values.append(data_float[8])
		      
		    msg.pose.append(pose)
		    if flag_incremental:
		    	pub.publish(msg)
		    	rospy.sleep(1.0)
		    if rospy.is_shutdown():
		      sys.exit()
		  if not flag_incremental:
		    pub.publish(msg)
		iter += 1
		if flag_loop:
			rospy.loginfo("Infinitely looping over file list, iteration %d",iter)
			rospy.sleep(1.0)

def main():
  files = []

  optlist, files = getopt.getopt(sys.argv[1:], "", ["help", "incremental", "loop"])

  flag_incremental = False
  flag_loop = False
  for (field, val) in optlist:
    if field == "--help":
      usage(sys.argv[0])
      return
    elif field == "--incremental":
      flag_incremental = True
    elif field == "--loop":
      flag_loop = True

  if len(files)==0:
    usage(sys.argv[0])
    return

  try:
    rospy.init_node('file_publisher')
    talker(files,flag_incremental,flag_loop)
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
  main()

