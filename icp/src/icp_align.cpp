/*
 * icp_test.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#include <ros/ros.h>

#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/AlignModelSrv.h"
#include "geometry_msgs/Pose.h"
#include "LinearMath/btTransform.h"

#include "icp/icp_utils.h"

using namespace std;
using namespace articulation_msgs;
using namespace icp;


bool icp_align(articulation_msgs::AlignModelSrv::Request &request,
		articulation_msgs::AlignModelSrv::Response &response
		)
{
  ROS_INFO("aligning model with data, poses=%lu, poses=%lu", request.model.track.pose.size(),request.data.track.pose.size());

  IcpAlign alignment(request.model.track, request.data.track);
  response.data_aligned = request.data;
  alignment.TransformData(response.data_aligned.track);
  response.model_aligned = request.model;
  alignment.TransformModel(response.model_aligned.track);
  for(int i=0;i<9;i++)
	  response.R[i] = alignment.TR[i];
  for(int i=0;i<3;i++)
	  response.T[i] = alignment.TT[i];
  btMatrix3x3 rot(
		  alignment.TR[0],alignment.TR[1],alignment.TR[2],
		  alignment.TR[3],alignment.TR[4],alignment.TR[5],
		  alignment.TR[6],alignment.TR[7],alignment.TR[8]);
  btVector3 trans(alignment.TT[0],alignment.TT[1],alignment.TT[2]);
  btTransform diff(rot,trans);
  response.dist_trans = trans.length();
  response.dist_rot = diff.getRotation().getAngle();
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp_test");
  ros::NodeHandle n;

  ros::ServiceServer icp_align_service= n.advertiseService("icp_align", icp_align);

  ROS_INFO("icp_align running, service ready");
  ros::spin();
}
