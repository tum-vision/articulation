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
  steps = rospy.get_param("~steps",100)
  sigma_position = rospy.get_param("~sigma_position",0.005)
  sigma_orientation = rospy.get_param("~sigma_orientation",0.4)
  reduce_dofs = rospy.get_param("~reduce_dofs",1)
  mode = rospy.get_param("~mode","spanningtree")
  inbag_filename = rospy.get_param('~bagfile','/home/sturm/sturm-data/sturm10tro/cabinet-pose.bag')
  outbag_filename = rospy.get_param('~outfile','/home/sturm/sturm-data/sturm10tro/cabinet-eval.bag')
  
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
  timestamps = bag.keys()
  timestamps.sort()

  outbag = rosbag.Bag(outbag_filename,"w")
  request = ArticulatedObjectSrvRequest()
  for step in range(1,steps+1):
    # downsample sequence      
    sequence = [ bag[timestamp]['/camera/pose_array'] for timestamp in timestamps ]
    parts = len( sequence[0].poses )
    print "we have %d observations of %d object parts"%(len(bag),parts)
    # cutoff
    cutoff = int(max(1,step * len(sequence) / float(steps))) 
    sequence = sequence[0:cutoff]
    print "selected the first %d observations"%len(sequence)
    d = max(1,int(downsample * step / float(steps)))
    downsampled = [ i for i in range(len(sequence)) if (i % (max(1,len(sequence) / d)) == 0 )]
    #print downsampled
    downsampled = downsampled[len(downsampled) - d:]
    sequence_downsampled = [ sequence[i] for i in downsampled ]
    print "--> downsampled to %d pose observations (downsample=%d)"%(len(sequence_downsampled),downsample)
    
    # debugging:
    #parts = 3
    
    # build service request message  
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
    
  #  b=rosbag.Bag("eval.bag","w")
  #  b.write("/object",request.object,rospy.Time(0))
  #  b.close()
    
    print "fitting models" 
    response = fit_models(request)
    print '\n'.join(
       ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f'%(
            model.name[0:2],
            model.id/parts,
            model.id%parts,
            get_param(model,'bic'),
            get_param(model,'avg_error_position'),
            get_param(model,'avg_error_orientation')
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
    
   # print response.object.params
    outbag.write("/object",response.object,rospy.Time(step))
    #print response.object.models
    
    # send image and info for last pose
    last_timestamp = timestamps[ downsampled[-1] ]
    info_pub.publish( bag[last_timestamp]['/camera/camera_info'] )
    image_pub.publish( bag[last_timestamp]['/camera/image_raw/compressed'] )
    posearray_pub.publish( bag[last_timestamp]['/camera/pose_array'] )
  outbag.close()
  
if __name__ == '__main__':
  main()

