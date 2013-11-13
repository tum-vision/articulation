#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> .. 
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
from articulation_models import track_utils
from articulation_models.track_utils import *

def main():
  rospy.init_node('simple_publisher')
  pub = rospy.Publisher('track', TrackMsg)

  for id,filename in enumerate(rospy.myargv()[1:]):
    track = readtrack(filename)
    
    print filename
    msg = track
    msg.pose = zero_start(msg.pose)
    msg.id = id
    pub.publish(msg)
    if rospy.is_shutdown():
      exit(0)

if __name__ == '__main__':
  main()

