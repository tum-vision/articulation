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
import rosbag

def all_detected(pose_array):
  for p in pose_array.poses:
    r = [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
    if( numpy.dot(r,r) == 0 ):
      return False
  return True

def main():
  rospy.init_node('articulation_eval')
  model_pub = rospy.Publisher('/model', ModelMsg)
  track_pub = rospy.Publisher('/track', TrackMsg)
  fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
  get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
  get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
  get_graph = rospy.ServiceProxy('get_graph', ArticulatedObjectSrv)
  
  downsample = 50
  sigma_position = 0.005
  sigma_orientation = 0.05
  reduce_dofs = 1
  inbag_filename = '/home/sturm/sturm-data/sturm10tro/closedchain-pose.bag'
  #inbag_filename = '/home/sturm/sturm-data/sturm10tro/openchain-pose.bag'
  
  inbag = rosbag.Bag(inbag_filename)
  
  sequence = [] 
  for topic, msg, t in inbag.read_messages(topics=['/camera/pose_array']):
#    msg.poses = msg.poses[1:3]
    if all_detected(msg):
      sequence.append(msg)
      
  parts = len(sequence[0].poses)
  print "read %d observations of %d object parts"%(len(sequence),parts)
  
  sequence_downsampled = [
        p 
        for (i,p) in enumerate(sequence) 
        if (i % (max(1,len(sequence) / downsample)) == 0 )] \
        [0:downsample]
  print "downsampled to %d pose observations"%len(sequence_downsampled)
  
  request = ArticulatedObjectSrvRequest()
  request.object.header = sequence_downsampled[-1].header
  request.object.parts = [ 
    TrackMsg(
             id=i,

             header=request.object.header,
             pose=[p.poses[i] for p in sequence_downsampled]
    ) 
    for i in range(parts)
  ]
  
  set_param(request.object, "sigma_position", sigma_position, ParamMsg.PRIOR)
  set_param(request.object, "sigma_orientation", sigma_orientation, ParamMsg.PRIOR)
  
  set_param(request.object, "reduce_dofs", reduce_dofs, ParamMsg.PRIOR)
  
  #response = get_fast_graph(request)
  #response = get_graph(request)
  response = get_spanning_tree(request)
  
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
  print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f"%(
          get_param(response.object,'dof'),
          get_param(response.object,'dof.nominal'),
          get_param(response.object,'bic'),
          get_param(response.object,'avg_error_position'),
          get_param(response.object,'avg_error_orientation')
          )

#  model_pub.publish(response.object.models[1])
#  track_pub.publish(request.object.parts[0])
#  track_pub.publish(request.object.parts[1])
#  rospy.sleep(5)
  #model_pub.publish(response.object.models[0])
  
if __name__ == '__main__':
  main()

