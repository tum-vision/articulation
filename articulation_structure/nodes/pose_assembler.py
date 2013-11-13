#! /usr/bin/python

# Need to explicitly enable 'with' in python 2.5
from __future__ import with_statement

PKG = 'articulation_closedchain'
NAME = 'pose_assembler'
import roslib; roslib.load_manifest(PKG)

import rospy
from geometry_msgs.msg import PoseStamped
from articulation_closedchain.msg import PoseStampedIdMsg
import tf

def send_marker(pose,id):
  global pub,br
  msg = PoseStampedIdMsg(pose,id)
  rospy.loginfo("received marker %d @ %f"%(id,pose.header.stamp.to_sec()))
  pub.publish(msg)
  br.sendTransform(pose,"/marker/%d"%id)


def callback1(pose):
  send_marker(pose,1)
    
def callback2(pose):
  send_marker(pose,2)
    
def callback3(pose):
  send_marker(pose,3)
    
def callback4(pose):
  send_marker(pose,4)
    
def main(argv=None):
  global pub,br
  rospy.init_node(NAME, anonymous=False)
  rospy.loginfo("ready to receive /marker/[1-4], republishing to /markers")
  rospy.Subscriber("/marker/1", PoseStamped, callback1, queue_size=100)
  rospy.Subscriber("/marker/2", PoseStamped, callback2, queue_size=100)
  rospy.Subscriber("/marker/3", PoseStamped, callback3, queue_size=100)
  rospy.Subscriber("/marker/4", PoseStamped, callback4, queue_size=100)
  br = tf.TransformBroadcaster()
  pub = rospy.Publisher('/markers', PoseStampedIdMsg)
  rospy.spin()

if __name__ == '__main__':
  main()

