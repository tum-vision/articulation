#!/usr/bin/env python

import roslib; roslib.load_manifest('articulation_structure')
import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
import logging
import getopt
import colorsys
import numpy
import copy

class articulation_collector:
  def __init__(self):
    try:
      rospy.wait_for_service('fit_models', 5)
      rospy.wait_for_service('get_spanning_tree', 5)
      rospy.wait_for_service('get_fast_graph', 5)
      rospy.wait_for_service('visualize_graph', 5)
      print "services OK"
    except rospy.ROSException:
      print "Services not found"
      rospy.signal_shutdown('Quit')

    self.pose_pub = rospy.Publisher('track', TrackMsg)
    self.object_pub = rospy.Publisher('object', ArticulatedObjectMsg)
    rospy.Subscriber("/camera/pose_array", PoseArray, self.callback,queue_size=1)
    self.object_parts = []
    self.fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
    self.get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
    self.get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
    self.visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)
    
    self.sigma_position = rospy.get_param('~sigma_position',0.01)
    self.sigma_orientation = rospy.get_param('~sigma_orientation',0.3)
    self.reduce_dofs = rospy.get_param('~reduce_dofs',1)
    self.samples = rospy.get_param('~samples',50) #keep this number of samples
    self.downsample = rospy.get_param('~downsample',False)#downsample or latest obs?
    self.object_msg = ArticulatedObjectMsg()
    set_param(self.object_msg, "sigma_position", self.sigma_position, ParamMsg.PRIOR)
    set_param(self.object_msg, "sigma_orientation", self.sigma_orientation, ParamMsg.PRIOR)
    set_param(self.object_msg, "reduce_dofs", self.reduce_dofs, ParamMsg.PRIOR)
  
  def callback(self, pose_array):
    #rospy.loginfo("received pose array of length %d" % len(pose_array.poses))
    marker_array = MarkerArray()
    
    all_detected = True
    for p in pose_array.poses:
      r = [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
      if( numpy.dot(r,r) == 0 ):
        all_detected = False

    if not all_detected:
      #rospy.loginfo('not all markers detected')
      return
    
    if(len(self.object_parts)==0): #generate new TrackMsgs for all Object Parts if there aren't any yet
      self.object_parts = [ 
          TrackMsg(
                   id=i,

          ) 
          for (i,p) in enumerate(pose_array.poses) ]
      
    for (track,pose) in zip(self.object_parts,pose_array.poses):
      track.pose.append( pose )
      track.pose_headers.append(pose_array.header)
#      if(len(track.pose)>self.samples):
#        track.pose = track.pose[len(track.pose)-self.samples:]
      
    rospy.loginfo('sending tracks with '+('/'.join(["%d"%len(track.pose) for track in self.object_parts]))+' poses')
    for track in self.object_parts:
      track.header = pose_array.header
      self.pose_pub.publish(track)
      
		# downsample or cut-off?
#    self.object_msg.parts = self.object_parts
    self.object_msg.header = pose_array.header
    self.object_msg.parts = copy.deepcopy(self.object_parts)
    if self.downsample:
      print "downsampling to %d observations"%(self.samples)
      for part in self.object_msg.parts:
        if len(part.pose)>self.samples:
          part.pose = [p for (i,p) in enumerate(part.pose) if i % (len(part.pose) / self.samples + 1) == 0 or i==len(part.pose)-1] 
    else:
      print "selecting latest %d observations"%(self.samples)
      for part in self.object_msg.parts:
        if len(part.pose)>self.samples:
          part.pose = part.pose[len(part.pose) - self.samples:]

    self.object_msg.header = pose_array.header
    
    request = ArticulatedObjectSrvRequest()
    request.object = self.object_msg
    
    parts = len(self.object_parts) 
    response = self.fit_models(request)
    print '\n'.join(
       ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f sigma_pos=%f sigma_orient=%f'%(
            model.name[0:2],
            model.id/parts,
            model.id%parts,
            get_param(model,'bic'),
            get_param(model,'avg_error_position'),
            get_param(model,'avg_error_orientation'),
            get_param(model,'sigma_position'),
            get_param(model,'sigma_orientation')
        ) for model in response.object.models])
    request.object = copy.deepcopy(response.object)

    response = self.get_fast_graph(request)
    self.object_pub.publish(response.object)
    self.object_msg.models = response.object.models
      
    request.object = copy.deepcopy(response.object)
    response = self.visualize_graph(request)
    
    rospy.loginfo('received articulation model: '+(' '.join(
       ['(%s:%d,%d)'%(model.name[0:2],model.id/parts,model.id%parts) for model in response.object.models])))
  
    print '\n'.join(
       ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f'%(
            model.name[0:2],
            model.id/parts,
            model.id%parts,
            get_param(model,'bic'),
            get_param(model,'avg_error_position'),
            get_param(model,'avg_error_orientation')
        ) for model in response.object.models])
#    print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f"%(
#            %get_param(response.object,'dof'),
#            get_param(response.object,'dof.nominal'),
#            get_param(response.object,'bic'),
#            get_param(response.object,'avg_error_position'),
#            get_param(response.object,'avg_error_orientation')
#            )      
def main():
  try:
    rospy.init_node('articulation_collector')
    articulation_collector()
    rospy.spin()
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
  main()

