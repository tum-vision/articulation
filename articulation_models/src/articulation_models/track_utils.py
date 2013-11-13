import sys
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import ChannelFloat32
import logging
import fileinput
import numpy
import tf
from tf import transformations
import glob
import rospy
import copy

def readtrack(filename):
  rospy.loginfo("Opening logfile '%s'", filename)
  
  msg = TrackMsg()
  channel_w = None
  channel_h = None
  for line in fileinput.input(filename):
    line = line.strip()
#    rospy.loginfo("Reading line: '%s'",line)
    data = line.strip().split()
    data_float = [float(column) for column in data]
    if len(data) < 3:
       continue

    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "/"
    msg.id = 0
    pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    
    if len(data) >= 3:
      pose.position.x = data_float[0]
      pose.position.y = data_float[1]
      pose.position.z = data_float[2]
      
    if len(data) >= 7:
      pose.orientation.x = data_float[3]
      pose.orientation.y = data_float[4]
      pose.orientation.z = data_float[5]
      pose.orientation.w = data_float[6]
      
    if len(data) >= 9:
      if channel_w is None or channel_h is None:
        channel_w = ChannelFloat32('width', [])
        channel_h = ChannelFloat32('height', [])
        msg.channels.append(channel_w)
        msg.channels.append(channel_h)
      channel_w.values.append(data_float[7])
      channel_h.values.append(data_float[8])
      
    msg.pose.append(pose)
  return msg

def readtrack2(filename):
  rospy.loginfo("Opening logfile '%s'", filename)
  
  msg = TrackMsg()
  
  for line_number,line in enumerate( fileinput.input(filename) ):
    line = line.strip()
    data = line.strip().split()
    if data[0]=="#":
      # treat the first comment as a field list (when it starts with "x y z .."
      if len(msg.channels) == 0:
        for a,b in zip(data[1:8],["x","y","z","qx","qy","qz","qw"]):
          if a!=b:
            continue
        for field in data[2:]:
          ch = ChannelFloat32(field, [])
          msg.channels.append(ch)
      continue
      
    data_float = [float(column) for column in data]
    if len(data) < 3:
       continue

    if len(data) != len(msg.channels):
      raise  BaseException("invalid number of fields!! line %d has only %d fields, should have %d! line: \n'%s'" % (
        line_number,len(data),len(msg.channels),line))

    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "/"
    msg.id = 0
    pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    
    if len(data) >= 3:
      pose.position.x = data_float[0]
      pose.position.y = data_float[1]
      pose.position.z = data_float[2]
      
    if len(data) >= 7:
      pose.orientation.x = data_float[3]
      pose.orientation.y = data_float[4]
      pose.orientation.z = data_float[5]
      pose.orientation.w = data_float[6]
      
    msg.pose.append(pose)
    
    for i,ch in enumerate(msg.channels):
      ch.values.append(data_float[i])
      
  return msg

def downsample(track,interval):
  track2 = TrackMsg()
  track2.header = track.header
  track2.id = track.id
  track2.pose = [p for i,p in enumerate(track.pose) if i%interval==0] 
  for channel in track.channels:
    ch = ChannelFloat32()
    ch.name = channel.name
    ch.values = [p for i,p in enumerate(channel.values) if i%interval==0]
    track2.channels.append(ch)
  return track2


def sub_track(track, begin, end):
  track2 = TrackMsg()
  track2.header = track.header
  track2.id = track.id
  track2.pose = track.pose[begin:end]
  for channel in track.channels:
    ch = ChannelFloat32()
    ch.name = channel.name
    ch.values = channel.values[begin:end]
    track2.channels.append(ch)
  return track2

def xyz_to_mat44(pos):
    return transformations.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return transformations.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

def pose_to_numpy( p ):
  return numpy.dot( xyz_to_mat44( p.position ), xyzw_to_mat44( p.orientation ) );  
  
def numpy_to_pose( P ):
  xyz = tuple(transformations.translation_from_matrix(P))[:3]
  quat = tuple(transformations.quaternion_from_matrix(P))
  return geometry_msgs.msg.Pose(geometry_msgs.msg.Point(*xyz), geometry_msgs.msg.Quaternion(*quat));
  
def zero_start(poses):
  if len(poses)==0:
    return poses;
  ref = pose_to_numpy( poses[0] );
  
  zero_poses = [ numpy_to_pose( numpy.dot( transformations.inverse_matrix(ref), pose_to_numpy(p) ) ) for p in poses]
  return zero_poses

def set_param(model,paramname,value,type):
  for p in model.params:
    if(p.name==paramname):
      p.value = value
      if(p.type != type):
        raise NameError('parameter %s exists but has different type'%paramname)
      return
  model.params.append(ParamMsg(paramname,value,type));
  
def get_param(model, paramname):
  list = [p.value for p in model.params if p.name == paramname]
  
  if len(list)==0:
    raise NameError('parameter %s not found in list'%paramname)
  if len(list)>1:
    raise NameError('parameter %s not unique!'%paramname)

  return list[0]

def get_channel(model, channelname):
  l = [p for p in model.track.channels if p.name == channelname]
  
  if len(l)==0:
    raise NameError('channel %s not found; channels=[%s]'%(channelname," ".join([ch.name for ch in model.track.channels])))
  if len(l)>1:
    raise NameError('channel %s not unique!'%channelname)

  l[0].values = list(l[0].values)
  return l[0].values

def get_channel_track(track, channelname):
  l = [p for p in track.channels if p.name == channelname]
  
  if len(l)==0:
    raise NameError('channel %s not found in list'%channelname)
  if len(l)>1:
    raise NameError('channel %s not unique!'%channelname)

  l[0].values = list(l[0].values)
  return l[0].values
  
def sort_with(l,k):
  lk = zip(k,l)
  lk.sort()
  l = [a[1] for a in lk]
  return l 

def sort_by_configuration(model):
  if get_param(model,"dofs")==0:
    return model
  q =  copy.deepcopy( get_channel(model,'q0') )
  
  model.track.pose = sort_with(model.track.pose,q)
  model.track.pose_projected = sort_with(model.track.pose_projected,q);
  for ch in model.track.channels:
    ch.values = sort_with(ch.values,q)

  return model
