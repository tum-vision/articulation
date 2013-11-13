# Single kinematic trajectory
#
# This message contains a kinematic trajectory. The trajectory is specified
# as a vector of 6D poses. An additional flag, track_type, indicates whether
# the track is valid, and whether it includes orientation. The track id
# can be used for automatic coloring in the RVIZ track plugin, and can be 
# freely chosen by the client. 
#
# A model is fitting only from the trajectory stored in the pose[]-vector. 
# Additional information may be associated to each pose using the channels
# vector, with arbitrary # fields (e.g., to include applied/measured forces). 
#
# After model evaluation,
# also the associated configurations of the object are stored in the channels,
# named "q[0]".."q[DOF-1]", where DOF is the number of degrees of freedom.
# Model evaluation also projects the poses in the pose vector onto the model,
# and stores these ideal poses in the vector pose_projected. Further, during model
# evaluation, a new set of (uniform) configurations over the valid configuration
# range is sampled, and the result is stored in pose_resampled.
# The vector pose_flags contains additional display flags for the poses in the
# pose vector, for example, whether a pose is visible and/or
# the end of a trajectory segment. At the moment, this is only used by the
# prior_model_learner.
#

std_msgs/Header header                        # frame and timestamp
int32 id                                # used-specified track id

geometry_msgs/Pose[] pose               # sequence of poses, defining the observed trajectory
std_msgs/Header[] pose_headers                   # Timestamp and frame for each pose (and pose_projected)
geometry_msgs/Pose[] pose_projected     # sequence of poses, projected to the model 
                                        # (after model evaluation)
geometry_msgs/Pose[] pose_resampled     # sequence of poses, re-sampled from the model in
                                        # the valid configuration range
uint32[] pose_flags                     # bit-wise combination of POSE_VISIBLE and POSE_END_OF_SEGMENT

uint32 POSE_VISIBLE=1
uint32 POSE_END_OF_SEGMENT=2

# Each channel should have the same number of elements as pose array, 
# and the data in each channel should correspond 1:1 with each pose
# possible channels: "width", "height", "rgb", ...
sensor_msgs/ChannelFloat32[] channels       


