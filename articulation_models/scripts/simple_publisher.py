#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> .. 
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
from articulation_models.track_utils import *

def main():
  rospy.init_node('simple_publisher')
  pub = rospy.Publisher('track', TrackMsg)

  for id,filename in enumerate(rospy.myargv()[1:]):
    track = readtrack(filename)

    loop = rospy.get_param("~loop",False)
    while True:    
      rospy.sleep(2)
      for i in range(1,len(track.pose)):
        print filename,i
        msg = sub_track(track, 0, i)
        msg.pose = zero_start(msg.pose)
        msg.id = id
        pub.publish(msg)
        rospy.sleep(0.05)
        if rospy.is_shutdown():
          exit(0)
      if not loop:
         break

if __name__ == '__main__':
  main()

