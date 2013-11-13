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
  
  info_pub = rospy.Publisher('/camera/camera_info',CameraInfo)
  image_pub = rospy.Publisher('/camera/image_raw/compressed',CompressedImage)
  posearray_pub = rospy.Publisher('/camera/pose_array',PoseArray)
  model_pub = rospy.Publisher('/model', ModelMsg)
  track_pub = rospy.Publisher('/track', TrackMsg)
  
  fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
  get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
  get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
  get_graph = rospy.ServiceProxy('get_graph', ArticulatedObjectSrv)
  get_graph_all = rospy.ServiceProxy('get_graph_all', ArticulatedObjectSrv)
  visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)
  
  downsample = rospy.get_param("~downsample",100)
  first_n = rospy.get_param("~first_n",0)
  start_n = rospy.get_param("~start_n",0)
  sigma_position = rospy.get_param("~sigma_position",0.005)
  sigma_orientation = rospy.get_param("~sigma_orientation",0.4)
  sigmax_position = rospy.get_param("~sigmax_position",0.005)
  sigmax_orientation = rospy.get_param("~sigmax_orientation",0.4)
  reduce_dofs = rospy.get_param("~reduce_dofs",1)
  mode = rospy.get_param("~mode","spanningtree")
  #sigma_position = 0.007
  #sigma_orientation = 0.02
  
  #inbag_filename = '/home/sturm/sturm-data/sturm10tro/closedchain-pose.bag'
  #inbag_filename = '/home/sturm/sturm-data/sturm10tro/openchain-pose.bag'
  #inbag_filename = '/home/sturm/sturm-data/sturm10tro/lampe-pose.bag'
  inbag_filename = rospy.get_param('~bagfile','/home/sturm/sturm-data/sturm10tro/cabinet-pose.bag')
  
  # read whole bag file, group by timestamp in header
  bag = {}
  for topic, msg, t in rosbag.Bag(inbag_filename).read_messages():
    timestamp = t
    if not timestamp in bag:
      bag[timestamp] = {}
    bag[timestamp][topic] = msg
  print "read bagfile %s with %d different header timestamps"%( inbag_filename, len(bag) )
  topics = max([len(msgs.keys()) for (timestamp,msgs) in bag.iteritems()])
  for (timestamp) in bag.keys():
    if len( bag[timestamp] ) != topics or not all_detected(bag[timestamp]['/camera/pose_array']):
      del bag[timestamp]
#    else:
#      bag[timestamp]['/camera/pose_array'].poses = [
#              bag[timestamp]['/camera/pose_array'].poses[0],
#              bag[timestamp]['/camera/pose_array'].poses[1]
#              ]
  timestamps = bag.keys()
  timestamps.sort()
  if(first_n>0):
    timestamps = timestamps[0:first_n]

  # downsample sequence      
  sequence = [ bag[timestamp]['/camera/pose_array'] for timestamp in timestamps ]
  parts = len( sequence[0].poses )
  print "read %d observations of %d object parts"%(len(bag),parts)
  downsampled = [ i for i in range(len(timestamps)) if (i % (max(1,(len(sequence)-start_n) / downsample)) == 0 ) and i>=start_n]
  downsampled = downsampled[len(downsampled) - downsample:]
  sequence_downsampled = [ sequence[i] for i in downsampled ]
  print "downsampled to %d pose observations"%len(sequence_downsampled)
  
  # debugging:
  #parts = 3
  
  # build service request message  
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
  set_param(request.object, "sigmax_position", sigmax_position, ParamMsg.PRIOR)
  set_param(request.object, "sigmax_orientation", sigmax_orientation, ParamMsg.PRIOR)
  set_param(request.object, "reduce_dofs", reduce_dofs, ParamMsg.PRIOR)
  
#  b=rosbag.Bag("eval.bag","w")
#  b.write("/object",request.object,rospy.Time(0))
#  b.close()
  
  print "fitting models" 
  response = fit_models(request)
  print '\n'.join(
     ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f gamma=%f'%(
          model.name[0:2],
          model.id/parts,
          model.id%parts,
          get_param(model,'bic'),
          get_param(model,'avg_error_position'),
          get_param(model,'avg_error_orientation'),
          get_param(model,'outlier_ratio')
      ) for model in response.object.models])
  
  if mode=="fastgraph":
    print "running fastgraph" 
    response = get_fast_graph(request)
  elif mode=="graph":
    print "running graph" 
    response = get_graph(request)
  elif mode=="spanningtree":
    print "running spanningtree" 
    response = get_spanning_tree(request)
  elif mode=="all":
    print "running all structure selection methods" 
    response = get_graph_all(request)
  
  request.object = copy.deepcopy(response.object)
  response = visualize_graph(request)  
  
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
  
  #outbag = rosbag.Bag("debug-%s.bag"%mode,"w")
  #outbag.write("/object",response.object,rospy.Time(0))
  #outbag.close()

  print response.object.params
  #print response.object.models
  
  # send image and info for last pose
  last_timestamp = timestamps[ downsampled[-1] ]
  info_pub.publish( bag[last_timestamp]['/camera/camera_info'] )
  image_pub.publish( bag[last_timestamp]['/camera/image_raw/compressed'] )
  posearray_pub.publish( bag[last_timestamp]['/camera/pose_array'] )

#  model_pub.publish(response.object.models[1])
#  track_pub.publish(request.object.parts[0])
#  track_pub.publish(request.object.parts[1])
#  rospy.sleep(5)
  #model_pub.publish(response.object.models[0])
  rospy.sleep(5)
  
if __name__ == '__main__':
  main()

