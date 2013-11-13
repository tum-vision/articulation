std_msgs/Header header

articulation_msgs/TrackMsg[] parts    # observed trajectories for each object part 
articulation_msgs/ParamMsg[] params   # global parameters
articulation_msgs/ModelMsg[] models	  # models, describing relationships between parts
visualization_msgs/MarkerArray markers # marker visualization of models/object 
