#!/usr/bin/python

import roslib; roslib.load_manifest('articulation_structure')
import rospy
import sys
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import *
from std_msgs.msg import ColorRGBA,String
from sensor_msgs.msg import ChannelFloat32, CameraInfo, CompressedImage
from tf.msg import tfMessage 
from articulation_msgs.msg import *
from articulation_msgs.srv import *
from articulation_models.track_utils import *
from imagerender.srv import *
import tf
import process_bags
import rosbag
import time

def all_detected(pose_array):
  for p in pose_array.poses:
    r = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    if(numpy.dot(r, r) == 0):
      return False
  return True
 
class ProcessRobotBag:
    def __init__(self):
        rospy.wait_for_service('fit_models', 5)
        rospy.wait_for_service('get_spanning_tree', 5)
        rospy.wait_for_service('get_fast_graph', 5)
        rospy.wait_for_service('visualize_graph', 5)
        rospy.wait_for_service('render_image', 5)
        self.tf = tf.TransformListener()
        self.fit_models = rospy.ServiceProxy('fit_models', ArticulatedObjectSrv)
        self.get_spanning_tree = rospy.ServiceProxy('get_spanning_tree', ArticulatedObjectSrv)
        self.get_fast_graph = rospy.ServiceProxy('get_fast_graph', ArticulatedObjectSrv)
        self.get_graph = rospy.ServiceProxy('get_graph', ArticulatedObjectSrv)
        self.visualize_graph = rospy.ServiceProxy('visualize_graph', ArticulatedObjectSrv)
        self.render_image = rospy.ServiceProxy('render_image', RenderImageSrv)
        
        self.infile = rospy.get_param('~infile', 'in.bag')
        self.outfile = rospy.get_param('~outfile', 'out.bag')

        self.start_t = rospy.get_param('~start', 0.0)
        self.stop_t = rospy.get_param('~stop', 0.0)
        
        self.sigma_position = rospy.get_param('~sigma_position', 0.01)
        self.sigma_orientation = rospy.get_param('~sigma_orientation', 0.3)
        self.sigmax_position = rospy.get_param('~sigmax_position', 0.01)
        self.sigmax_orientation = rospy.get_param('~sigmax_orientation', 0.3)
        self.samples = rospy.get_param('~samples', 50) #keep this number of samples
        self.every = rospy.get_param('~every', 1) #skip poses?
        self.downsample = rospy.get_param('~downsample', False)#downsample or latest obs?
        self.object_msg = ArticulatedObjectMsg()
        set_param(self.object_msg, "sigma_position", self.sigma_position, ParamMsg.PRIOR)
        set_param(self.object_msg, "sigma_orientation", self.sigma_orientation, ParamMsg.PRIOR)
        set_param(self.object_msg, "sigmax_position", self.sigmax_position, ParamMsg.PRIOR)
        set_param(self.object_msg, "sigmax_orientation", self.sigmax_orientation, ParamMsg.PRIOR)
        reduce_dofs = rospy.get_param("~reduce_dofs", 1)
        set_param(self.object_msg, "reduce_dofs", reduce_dofs, ParamMsg.PRIOR)
        
        self.object_parts = []
        
        self.run()

    def run(self):
        startn = 0
        outbag = rosbag.Bag(self.outfile, "w")
        count = 0
        out_publishers = {}
        first_t = None
        graph = "0-DOF "
        object_pub = rospy.Publisher("/object", ArticulatedObjectMsg)
        graph_pub = rospy.Publisher("/graph", String)
        for topic, msg, t in rosbag.Bag(self.infile).read_messages():
            if topic == "/camera/image_raw/compressed" and count >= startn:
              print "rendering image.."
              self.render_image(RenderImageSrvRequest())
              graph_pub.publish(graph)
              print "done rendering image.."
                    
            if first_t == None:
                first_t = t
            if topic not in out_publishers:
                print "adding publisher for topic '%s' with type '%s'" % (topic, msg.__class__.__name__)
                out_publishers[topic] = rospy.Publisher(topic, msg.__class__)

            out_publishers[topic].publish(msg)
            outbag.write(topic, msg, t)
            if t < rospy.Duration(self.start_t) + first_t:
                continue
            if self.stop_t > 0 and t > rospy.Duration(self.stop_t) + first_t:
                continue
            if topic == "/camera/pose_array":
                count += 1
                if(count % self.every == 0):
                    if not all_detected(msg):
                        continue
                    if len(self.object_parts) == 0:
                        self.object_parts = \
                                [ 
                                    TrackMsg(
                                                     id=i,

                                    ) 
                                    for i in range(len(msg.poses))
                                ]
                    for i, p in enumerate(msg.poses):
                        self.object_parts[i].pose.append(p)
                    
                    if(count < startn):
                        continue
                    
#                    if len(self.object_parts[0].pose)<255:
#                        continue
                    rospy.loginfo('sending tracks with ' + ('/'.join(["%d" % len(track.pose) for track in self.object_parts])) + ' poses')
                    for track in self.object_parts:
                        track.header = msg.header
                        outbag.write('/track', track, t)
                        
                    self.object_msg.header = msg.header
                    self.object_msg.parts = copy.deepcopy(self.object_parts)
                    if self.downsample:
                        for part in self.object_msg.parts:
                            if len(part.pose) > self.samples:
                                part.pose = [p for (i, p) in enumerate(part.pose) if i % (len(part.pose) / self.samples + 1) == 0 or i == len(part.pose) - 1] 
                    else:
                        for part in self.object_msg.parts:
                            if len(part.pose) > self.samples:
                                part.pose = part.pose[len(part.pose) - self.samples:]
                    
                    rospy.loginfo('fitting tracks with ' + ('/'.join(["%d" % len(track.pose) for track in self.object_msg.parts])) + ' poses')
                    
                    request = ArticulatedObjectSrvRequest()
                    request.object = self.object_msg
                    parts = len(self.object_parts) 
                    print "fitting models.."        
                    response = self.fit_models(request)
                    print '\n'.join(
                         ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f sigma_pos=%f sigma_orient=%f' % (
                                    model.name[0:2],
                                    model.id / parts,
                                    model.id % parts,
                                    get_param(model, 'bic'),
                                    get_param(model, 'avg_error_position'),
                                    get_param(model, 'avg_error_orientation'),
                                    get_param(model, 'sigma_position'),
                                    get_param(model, 'sigma_orientation')
                            ) for model in response.object.models])
                    request.object = copy.deepcopy(response.object)
    
                    print "selecting models and structure.."        
                    response = self.get_fast_graph(request)
                    #response = self.get_graph(request)
                    outbag.write('/object', response.object, t)
                    object_pub.publish(response.object)
                    graph = "%d-DOF "%(get_param(response.object, 'dof')) \
                        + ','.join(
                         ['(%s:%d,%d)' % (
                                    model.name[0:2],
                                    (model.id / parts) + 1,
                                    (model.id % parts) + 1
                            ) for model in response.object.models])
                    self.object_msg.models = response.object.models
                        
                    request.object = copy.deepcopy(response.object)
                    print "visualizing models.."        
                    response = self.visualize_graph(request)
                    outbag.write('/structure_array', response.object.markers, t)
                    
                    rospy.loginfo('received articulation model: ' + (' '.join(
                         ['(%s:%d,%d)' % (model.name[0:2], model.id / parts, model.id % parts) for model in response.object.models])))
                
                    print '\n'.join(
                         ['(%s:%d,%d) bic=%f pos_err=%f rot_err=%f' % (
                                    model.name[0:2],
                                    model.id / parts,
                                    model.id % parts,
                                    get_param(model, 'bic'),
                                    get_param(model, 'avg_error_position'),
                                    get_param(model, 'avg_error_orientation')
                            ) for model in response.object.models])
                    print "dofs=%d, nominal dofs=%d, bic=%f pos_err=%f rot_err=%f" % (
                                    get_param(response.object, 'dof'),
                                    get_param(response.object, 'dof.nominal'),
                                    get_param(response.object, 'bic'),
                                    get_param(response.object, 'avg_error_position'),
                                    get_param(response.object, 'avg_error_orientation')
                                    )            
                    print "done evaluating new pose.."
                    

        outbag.close()        
         
if __name__ == '__main__':
    rospy.init_node('process_robotbag')

    ProcessRobotBag()




