#! /usr/bin/python

# Need to explicitly enable 'with' in python 2.5
from __future__ import with_statement

PKG = 'articulation_closedchain'
NAME = 'track_assembler'
import roslib; roslib.load_manifest(PKG)

import rospy
import cv
import math
import threading
import numpy
import tf
import articulation_models
from articulation_models.transform_datatypes import mat44_to_pose,pose_to_mat44

from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion
from articulation_msgs.msg import TrackMsg,ModelMsg

class TrajAssemblerNode:
  def __init__(self):
    self.markers = rospy.get_param('~markers', 4)
#    self.last_publish_time = rospy.Time()

    self.marker_sub = []
    self.marker_sub.append(rospy.Subscriber("/marker/1", PoseStamped, self.callback1))
    self.marker_sub.append(rospy.Subscriber("/marker/2", PoseStamped, self.callback2))
    self.marker_sub.append(rospy.Subscriber("/marker/3", PoseStamped, self.callback3))
    self.marker_sub.append(rospy.Subscriber("/marker/4", PoseStamped, self.callback4))
    self.track_pub = rospy.Publisher('track', TrackMsg)
    self.mutex = threading.RLock()
    self.pose = {}
    
    self.track = TrackMsg()

  def callback1(self, pose):
    self.add_pose(1,pose)
    if(len(self.pose[1])%3 ==0):
        print "sending tracks."
        self.send_track()

  def callback2(self, pose):
    self.add_pose(2,pose)
    
  def callback3(self, pose):
    self.add_pose(3,pose)
    
  def callback4(self, pose):
    self.add_pose(4,pose)
    
  def add_pose(self,marker,pose):
    with self.mutex:
        if not marker in self.pose:
          self.pose[marker] = {}
        self.pose[marker][pose.header.stamp.to_sec()] = pose.pose
        print "added marker %d timestamp %f"%(marker,pose.header.stamp.to_sec())
    
  def send_track(self):
    with self.mutex:
        #for m1 in self.pose.keys():
        #    for m2 in self.pose.keys():
        for (m1,m2) in [(2,1),(3,2),(4,3),(1,4)]:
                if m2==m1:
                    continue
                track = TrackMsg()
                track.header.stamp = rospy.get_rostime()
                track.header.frame_id = "/camera"

                track.id = m1*len(self.pose.keys()) + m2
                keys1 = self.pose[m1].keys()
                keys2 = self.pose[m2].keys()
                keys = filter(lambda x:x in keys1,keys2)
                keys.sort()
                mat1_last = pose_to_mat44(self.pose[m1][keys[-1]])
                #mat1_last = numpy.eye(4)
                for time in keys:
                    pose1 = self.pose[m1][time]
                    pose2 = self.pose[m2][time]
                    mat1=pose_to_mat44(pose1)
                    mat2=pose_to_mat44(pose2)
                    diff = numpy.mat(mat1).I * mat2
                    diff_pose = mat44_to_pose(diff)
                    diff[2,3] = 0.00
                    pose_rel_to_last = mat44_to_pose(mat1_last * diff)
                    track.pose.append(pose_rel_to_last)
                self.track_pub.publish(track)
    
def main(argv=None):
  rospy.init_node(NAME, anonymous=False)
  assembler = TrajAssemblerNode()
  rospy.spin()

if __name__ == '__main__':
  main()

