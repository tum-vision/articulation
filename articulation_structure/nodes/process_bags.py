#!/usr/bin/python

import rospy
import rosbag
import time


def callback(topic, msg):
	global current_time
	try:
		header = getattr(msg, "header")
		if header.stamp < current_time:
			print "received old message, skipping '%s'" % topic
			return
	except AttributeError:
		pass
	global result
	result[topic] = msg


def wait_for_result(timeout, out_topics):
	#print "  waiting for result"
	t = 0
	while len(result) != len(out_topics) and not rospy.is_shutdown() and timeout > t:
		rospy.sleep(0.1)
		t += 0.1
#	print "  timing: %f"%t

	return len(result) == len(out_topics)

def process_bags(infile, in_topics, in_types,
								outfile, out_topics, out_types,
								timeout):
	publishers = dict([ (topic, rospy.Publisher(topic, t)) 
			for topic, t in zip(in_topics, in_types) ])

#	subscribers=[
#		rospy.Subscriber(out_topic,out_type,lambda x:callback(out_topic,x))
#		for out_topic,out_type in zip(out_topics,out_types) ]

	if(len(out_topics) > 0):
		rospy.Subscriber(out_topics[0], out_types[0], lambda x:callback(out_topics[0], x))
	if(len(out_topics) > 1):
		rospy.Subscriber(out_topics[1], out_types[1], lambda x:callback(out_topics[1], x))
	if(len(out_topics) > 2):
		rospy.Subscriber(out_topics[2], out_types[2], lambda x:callback(out_topics[2], x))
	if(len(out_topics) > 3):
		rospy.Subscriber(out_topics[3], out_types[3], lambda x:callback(out_topics[3], x))
	if(len(out_topics) > 4):	
		print "lambda trouble, add more lines.."
		return
	

	global result
	result = {}

	outbag = rosbag.Bag(outfile, "w")
	global current_time
	current_time = rospy.get_rostime()
	for topic, msg, t in rosbag.Bag(infile).read_messages(topics=in_topics):
		if(t != current_time):
			print "frame: ", current_time
			current_time = t
			in_msgs = {}
		
		in_msgs[topic] = msg

		if len(in_msgs) == len(in_topics):
			result = {}
			print "  in topics:"
			for topic, msg in in_msgs.iteritems():
				print "    %s" % topic
				publishers[topic].publish(msg)

			t0 = time.time()				
			complete = wait_for_result(timeout, out_topics)

			print "  out topics (time=%f):" % (time.time() - t0)
			for topic, msg in result.iteritems():
				print "    %s" % topic

			if complete:
				if(time.time() - t0 > timeout * 0.5):
					timeout = (time.time() - t0) * 2
					print "  increasing timeout to %f" % timeout
				for topic, msg in in_msgs.iteritems():
					outbag.write(topic, msg, t)
				for topic, msg in result.iteritems():
					outbag.write(topic, msg, t)
				print "  saving frame"
			else:
				print "  skipping frame"

		if rospy.is_shutdown():
			break;
		
	outbag.close()


