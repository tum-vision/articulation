#!/usr/bin/env python

"""
 usage: %(progname)s <track_file1.txt> .. 
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
from track_utils import *

model_pub = rospy.Publisher('model', TrackMsg)
data_pub = rospy.Publisher('data', TrackMsg)
data_aligned_pub = rospy.Publisher('data_aligned', TrackMsg)

def main():
  rospy.init_node('simple_publisher')
  
  icp_align = rospy.ServiceProxy('icp_align', AlignModelSrv)

  for id1,filename1 in enumerate(rospy.myargv()[1:]):
    for id2,filename2 in enumerate(rospy.myargv()[1:]):
      if id1 >= id2:
        continue
      track1 = readtrack(filename1)
      track2 = readtrack(filename2)
      
      print "calling service.."
      request = AlignModelSrvRequest()
      request.model.track = track1
      request.data.track = track2
      response = icp_align(request)
      
      rospy.sleep(0.05)
      request.model.track.header.frame_id="/"
      request.data.track.header.frame_id="/"
      response.data_aligned.track.header.frame_id="/"
      model_pub.publish(request.model.track)
      data_pub.publish(request.data.track)
      data_aligned_pub.publish(response.data_aligned.track)
      rospy.sleep(0.05)


if __name__ == '__main__':
  main()

