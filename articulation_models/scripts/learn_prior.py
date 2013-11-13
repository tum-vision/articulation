#!/usr/bin/env python

"""
 usage: %(progname)s <track_file.txt> 
"""
import roslib; roslib.load_manifest('articulation_models')
from articulation_models.track_utils import *
from copy import deepcopy



def main():
  try:
    rospy.init_node('learn_prior')
    model_pub = rospy.Publisher('model', ModelMsg)
    
    tracks = []
    for filename in rospy.myargv()[1:]:
      tracks.append( readtrack( filename ) )
      
    for track in tracks:
      track.pose = zero_start( track.pose )

    model_select = rospy.ServiceProxy('model_select', TrackModelSrv)

    model_eval = rospy.ServiceProxy('model_select_eval', TrackModelSrv)

    prior_models = []
    for id,track in enumerate(tracks):
      request = TrackModelSrvRequest()
      request.model.id = id
      request.model.track = track
      response = model_select(request)
      set_param(response.model,"complexity",0,ParamMsg.PRIOR)
      prior_models.append( response.model )
      
    print "All models learned"
    
    for id,track in enumerate(tracks):
      print "working on track %d" % id
      for i in range(1,len(track.pose)):
        print "working on track %d, considering first %d observations " % (id,i)
        subtrack = sub_track(track, 0, i)
        
        request = TrackModelSrvRequest()
        request.model.id = -1 # current
        request.model.track = subtrack
        response = model_select(request)

        # construct full list
        models = []
        for id2,model2 in enumerate(deepcopy(prior_models)):
          if id!=id2:
            models.append(model2)
        models.append(response.model)
        
        # now update all models to new subtrack
        
        response_list = []
        for model in models:
          request = TrackModelSrvRequest()

          request.model = model
          request.model.track = subtrack
          response = model_eval(request)
          #print response.model
          print response.model.name,get_param(response.model,"bic"),get_param(response.model,"complexity")
          response_list.append( response.model )
        
        def compare_bic(model_a, model_b):
          return cmp( get_param(model_a,"bic"),get_param(model_b,"bic") ) 

        response_list.sort(compare_bic)
        print "best:",response_list[0].name,get_param(response_list[0],"bic"),get_param(response_list[0],"complexity")
        
        model_pub.publish(response_list[0])
  except rospy.ROSInterruptException: pass
    
if __name__ == '__main__':
  main()

