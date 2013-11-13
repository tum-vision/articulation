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
  visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)
  
  #read data
  for (topic,object,t) in rosbag.Bag("eval.bag").read_messages():
    print t
    
  #object.parts = [object.parts[1],object.parts[2]]
  for p in object.parts:
    print "publishing part %d"%p.id
    track_pub.publish(p)
  
  #sys.exit()
  
  parts = len(object.parts)
  request = ArticulatedObjectSrvRequest()
  request.object = object
  #print object
  
  response = fit_models(request)
  print '\n'.join(
     ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f \n%s'%(
          model.name[0:2],
          model.id/parts,
          model.id%parts,
          get_param(model,'bic'),
          get_param(model,'avg_error_position'),
          get_param(model,'avg_error_orientation'),
          "\n".join(["  %s = %f"%(p.name,p.value) for p in model.params if p.name[0:3]=="rot"])
      ) for model in response.object.models])

  for m in response.object.models:
    if m.name=="rotational":
      model_pub.publish(m)

  request.object = copy.deepcopy(response.object)
  response = visualize_graph(request)  

  sys.exit()
  
  if mode=="fastgraph":
    response = get_fast_graph(request)
  elif mode=="graph":
    response = get_graph(request)
  elif mode=="spanningtree":
    response = get_spanning_tree(request)
  
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

