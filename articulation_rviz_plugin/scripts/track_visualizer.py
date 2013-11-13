#!/usr/bin/env python

"""
 usage: %(progname)s [--obs] [--tracks] 
 
 This node visualizes track messages. 
"""

import roslib; roslib.load_manifest('articulation_models')
import rospy
import sys
from articulation_models.msg import TrackMsg
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Vector3,Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import ChannelFloat32
import logging
import getopt
import colorsys

def usage(progname):
  print __doc__ % vars()

class trackVisualizer:
	def __init__(self,colorize_track,colorize_obs):
		self.pub = rospy.Publisher('visualization_marker', Marker)
		self.pub_array = rospy.Publisher('visualization_marker_array', MarkerArray)
		self.colorize_track = colorize_track
		self.colorize_obs = colorize_obs
		rospy.Subscriber("track", TrackMsg, self.callback)
		self.num_poses = {}
		self.num_markers = {}
		self.old_num_markers = {}
	
	def callback(self,track):
		rospy.loginfo( "received track %d, containing %d poses",track.id,len(track.pose) )
		marker_array = MarkerArray()
		
		if track.id in self.num_markers:
			self.old_num_markers[track.id] = self.num_markers[track.id]
		else:
			self.old_num_markers[track.id] = 0
		self.num_markers[track.id] = 0
		self.num_poses[track.id] = len(track.pose)
		
		channel_w = None
		channel_h = None
		for channel in track.channels:
			if channel.name == 'width':
				channel_w = channel
			elif channel.name == 'height':
				channel_h = channel
				
				
		marker_array = MarkerArray()
		if track.track_type == TrackMsg.TRACK_FULL_POSE:
			if not ( (channel_w is None) or (channel_h is None) ):
				self.render_rectangles(track,marker_array,channel_w,channel_h)
			else:
				self.render_axes(track,marker_array)
		elif track.track_type == TrackMsg.TRACK_POSITION_ONLY:
			self.render_points(track,marker_array)
		else:
			rospy.logerr("invalid track received!")


		self.delete_old_markers(track,marker_array)		
		
		rospy.loginfo( "publishing MarkerArray, containing %d markers",
			len(marker_array.markers) )
		self.pub_array.publish(marker_array)
		
	def render_rectangles(self,track,marker_array,channel_w,channel_h):
		for i in range( len( track.pose ) ):
			marker = Marker()
			marker.header.stamp = track.header.stamp
			marker.header.frame_id = track.header.frame_id
			marker.ns = "track_visualizer-%d"%(track.id)
			marker.id = self.num_markers[track.id]
			marker.action = Marker.ADD
			
			marker.scale = Vector3(0.003,0.003,0.003)
			marker.color = self.generate_color_rectangle(track.id, i)

			marker.type = Marker.LINE_STRIP
			marker.pose = track.pose[i]
			marker.points.append( Point(0,0,0) )
			marker.points.append( Point(channel_w.values[i],0,0) )
			marker.points.append( Point(channel_w.values[i],channel_h.values[i],0) )
			marker.points.append( Point(0,channel_h.values[i],0) )
			marker.points.append( Point(0,0,0) )
			
			marker_array.markers.append(marker)
			self.num_markers[track.id] += 1
			
	def render_axes(self,track,marker_array):
		for i in range( len( track.pose ) ):
			for axis in range(3):
				marker = Marker()
				marker.header.stamp = track.header.stamp
				marker.header.frame_id = track.header.frame_id
				marker.ns = "track_visualizer-%d"%(track.id)
				marker.id = self.num_markers[track.id]
				marker.action = Marker.ADD
			
				marker.scale = Vector3(0.003,0.003,0.003)
				marker.color = self.generate_color_axis(track.id, i,axis)

				marker.type = Marker.LINE_STRIP
				marker.pose = track.pose[i]
				marker.points.append( Point(0,0,0) )
				if axis==0:
					marker.points.append( Point(0.05,0,0) )
				elif axis==1:
					marker.points.append( Point(0,0.05,0) )
				elif axis==2:
					marker.points.append( Point(0,0,0.05) )
			
				marker_array.markers.append(marker)
				self.num_markers[track.id] += 1

	def render_points(self,track,marker_array):
		for i in range( len( track.pose ) ):
			marker = Marker()
			marker.header.stamp = track.header.stamp
			marker.header.frame_id = track.header.frame_id
			marker.ns = "track_visualizer-%d"%(track.id)
			marker.id = self.num_markers[track.id]
			marker.action = Marker.ADD
			
			marker.scale = Vector3(0.003,0.003,0.003)
			marker.color = self.generate_color_rectangle(track.id, i)

			marker.type = Marker.SPHERE_LIST
			marker.pose = track.pose[i]
			marker.points.append( Point(0,0,0) )
			
			marker_array.markers.append(marker)
			self.num_markers[track.id] += 1
						
	def delete_old_markers(self,track,marker_array):
		i = self.num_markers[track.id]
		while i < self.old_num_markers[track.id]:
			marker = Marker()
			marker.header.stamp = track.header.stamp
			marker.header.frame_id = track.header.frame_id
			marker.ns = "track_visualizer-%d"%(track.id)
			marker.id = i
			marker.action = Marker.DELETE
			marker_array.markers.append(marker)
			i += 1

	def generate_color_rectangle(self,track_id,obs_id):
		h = 1
		s = 1
		v = 1
		
		if self.colorize_track and self.colorize_obs:
			h = self.num_markers.keys().index(track_id) / float(len(self.num_markers))
			a = obs_id / float(self.num_poses[track_id])
			if a<0.5:
				s = 0.5 + a
			else:
				v = 0.5 + (1-a)
		elif self.colorize_track:
			h = self.num_markers.keys().index(track_id) / float(len(self.num_markers))
		elif self.colorize_obs:
			h = obs_id / float(self.num_poses[track_id])
				
		rgb = colorsys.hsv_to_rgb(h,s,v)
		return ColorRGBA(rgb[0],rgb[1],rgb[2],1.0)

	def generate_color_axis(self,track_id,obs_id,axis):
		hue = {0: 0, 1: 1/3.0, 2: 2/3.0}
		h = hue[axis]
		s = 1
		v = 1
		
		if self.colorize_track and self.colorize_obs:
			h = self.num_markers.keys().index(track_id) / float(len(self.num_markers))
			a = obs_id / float(self.num_poses[track_id])
			if a<0.5:
				s = 0.5 + a
			else:
				v = 0.5 + (1-a)
		elif self.colorize_track:
			a = self.num_markers.keys().index(track_id) / float(len(self.num_markers))
			if a<0.5:
				s = 0.5 + a
			else:
				v = 0.5 + (1-a)
		elif self.colorize_obs:
			a = obs_id / float(self.num_poses[track_id])
			if a<0.5:
				s = 0.5 + a
			else:
				v = 0.5 + (1-a)
				
		rgb = colorsys.hsv_to_rgb(h,s,v)
		return ColorRGBA(rgb[0],rgb[1],rgb[2],1.0)


def main():
	optlist, files = getopt.getopt(sys.argv[1:], "", ["help","obs","track"])

	colorize_obs = False
	colorize_track = False
	
	for (field, val) in optlist:
		if field == "--help":
			usage(sys.argv[0])
			return
		elif field == "--obs":
			colorize_obs = True
		elif field == "--track":
			colorize_track = True
		
	try:
		rospy.init_node('track_visualizer')
		trackVisualizer(colorize_track,colorize_obs)
		rospy.spin()
	except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
  main()

