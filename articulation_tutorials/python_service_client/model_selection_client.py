#!/usr/bin/env python

import roslib; roslib.load_manifest('articulation_tutorials')
import rospy
import numpy

from articulation_msgs.msg import *
from articulation_msgs.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import ChannelFloat32


PRISMATIC = 0
ROTATIONAL = 1
MODELS={PRISMATIC:'prismatic',ROTATIONAL:'rotational'}

def sample_track(model = PRISMATIC, n = 100, sigma_position = 0.02):
	msg = TrackMsg()
	msg.header.stamp = rospy.get_rostime()
	msg.header.frame_id = "/"
	msg.id = model

	for i in range(0,n):
		q = i / float(n)
		if model == PRISMATIC:
			pose = Pose(Point(q, 0, 0), Quaternion(0, 0, 0, 1))
		elif model == ROTATIONAL:
			pose = Pose(Point(numpy.sin(q), numpy.cos(q) - 1.0, 0), Quaternion(0, 0, 0, 1))
		else:
			raise NameError("unknown model, cannot generate trajectory!")
		pose.position.x += numpy.random.rand()*sigma_position
		pose.position.y += numpy.random.rand()*sigma_position
		pose.position.z += numpy.random.rand()*sigma_position
		msg.pose.append( pose )
	return msg

def main():
	rospy.init_node('test_fitting')

	model_select = rospy.ServiceProxy('model_select', TrackModelSrv)
	model_pub = rospy.Publisher('model', ModelMsg)
	print
  
	while True:
		for model_type,model_name in MODELS.items():
			request = TrackModelSrvRequest()
			print "generating track of type '%s'" % model_name
			request.model.track = sample_track( model_type )
		 
			try:            
				response = model_select(request)
				print "selected model: '%s' (n = %d, log LH = %f)" % (
					response.model.name,
					len(response.model.track.pose),
					[entry.value for entry in response.model.params if entry.name=='loglikelihood'][0]
				)
				model_pub.publish(response.model)
			except rospy.ServiceException: 
				print "model selection failed"
				pass
			if rospy.is_shutdown():
				exit(0)
			print
			rospy.sleep(0.5)



if __name__ == '__main__':
  main()

