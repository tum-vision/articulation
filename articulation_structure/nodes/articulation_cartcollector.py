#!/usr/bin/env python

import roslib; roslib.load_manifest('articulation_structure')
import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose,PoseStamped, PoseArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
from articulation_models.transform_datatypes import *
import tf
import logging
import getopt
import colorsys
import numpy
import copy

class articulation_cartcollector:
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

    self.listener = tf.TransformListener()

    self.pose_pub = rospy.Publisher('track', TrackMsg)
    self.object_pub = rospy.Publisher('object', ArticulatedObjectMsg)
    rospy.Subscriber("/r_cart/state/x", PoseStamped, self.callback,queue_size=1)
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
  
  def callback(self, pose):
    print "adding pose.."
#    try:
#      (trans,rot) = self.listener.lookupTransform('/torso_lift_link', '/wide_stereo_optical_frame', pose.header.stamp)
#    except (tf.LookupException, tf.ConnectivityException):
#      return
#    print trans,rot
#      
#    transform = numpy.dot(translation_matrix(trans),quaternion_matrix(rot))
#    transformedPose.header = pose.header
#    transformedPose.header.frame_id = '/wide_stereo_optical_frame'
#    transformedPose.pose = mat44_to_pose( numpy.dot(transform,pose_to_mat44( pose.pose) ) )
    transformedPose = pose 
      
    if(len(self.object_parts)==0):
      self.object_parts = [ 
          TrackMsg(
                   id=i,

          ) 
          for i in range(2)
          ]
      
    identity = Pose(Point(0,0,0),Quaternion(0,0,0,1))
    self.object_parts[0].pose.append(identity)
    self.object_parts[1].pose.append(transformedPose.pose)
      
    rospy.loginfo('sending tracks with '+('/'.join(["%d"%len(track.pose) for track in self.object_parts]))+' poses')
    for track in self.object_parts:
      track.header = transformedPose.header
      self.pose_pub.publish(track)
      
		# downsample or cut-off?
#    self.object_msg.parts = self.object_parts
    self.object_msg.header = transformedPose.header
    self.object_msg.parts = copy.deepcopy(self.object_parts)
    if self.downsample:
      for part in self.object_msg.parts:
        if len(part.pose)>self.samples:
          part.pose = [p for (i,p) in enumerate(part.pose) if i % (len(part.pose) / self.samples + 1) == 0 or i==len(part.pose)-1] 
    else:
      for part in self.object_msg.parts:
        if len(part.pose)>self.samples:
          part.pose = part.pose[len(part.pose) - self.samples:]

    self.object_msg.header = transformedPose.header
    
    request = ArticulatedObjectSrvRequest()
    request.object = self.object_msg
    
    parts = len(self.object_parts) 
    print "calling fit service"
    response = self.fit_models(request)
    print " fit service done"
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
    print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f"%(
            get_param(response.object,'dof'),
            get_param(response.object,'dof.nominal'),
            get_param(response.object,'bic'),
            get_param(response.object,'avg_error_position'),
            get_param(response.object,'avg_error_orientation')
            )      
    print "done evaluating new pose.."
def main():
  try:
    rospy.init_node('articulation_cartcollector')
    articulation_cartcollector()
    rospy.spin()
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
  main()

