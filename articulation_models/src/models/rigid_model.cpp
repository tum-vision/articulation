/*
 * rigid_model.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#include "articulation_models/models/rigid_model.h"
#include "articulation_models/utils.h"


using namespace std;

using namespace articulation_msgs;

namespace articulation_models {

void readParamsFromModel();
void writeParamsToModel();

RigidModel::RigidModel():GenericModel() {
	rigid_position = tf::Vector3(0,0,0);
	rigid_orientation = tf::Quaternion(0,0,0,1);
	rigid_width = 1;
	rigid_height = 1;

	complexity = 6;
}

void RigidModel::readParamsFromModel() {
	GenericModel::readParamsFromModel();
	getParam("rigid_position",rigid_position);
	getParam("rigid_orientation",rigid_orientation);
	getParam("rigid_width",rigid_width);
	getParam("rigid_height",rigid_height);
}

void RigidModel::writeParamsToModel() {
	GenericModel::writeParamsToModel();
	setParam("rigid_position",rigid_position,ParamMsg::PARAM);
	setParam("rigid_orientation",rigid_orientation,ParamMsg::PARAM);
	setParam("rigid_width",rigid_width,ParamMsg::PARAM);
	setParam("rigid_height",rigid_height,ParamMsg::PARAM);
}

geometry_msgs::Pose RigidModel::predictPose(V_Configuration q) {
	return transformToPose( tf::Transform(rigid_orientation,rigid_position) );
}

void RigidModel::projectConfigurationToChannels() {
	int ch_w = openChannel("width",false);
	int ch_h = openChannel("height",false);

	size_t n = model.track.pose.size();
	for(size_t i=0;i<n;i++) {
		if(ch_w>=0) model.track.channels[ch_w].values[i] = rigid_width;
		if(ch_h>=0) model.track.channels[ch_h].values[i] = rigid_height;
	}
}

bool RigidModel::guessParameters() {
	if(model.track.pose.size() == 0)
		return false;

	size_t i = rand() % getSamples();

	tf::Transform pose = poseToTransform(model.track.pose[i]);

	rigid_position = pose.getOrigin();
	rigid_orientation = pose.getRotation();
	return true;
}

void RigidModel::updateParameters(std::vector<double> delta) {
	rigid_position = rigid_position + tf::Vector3(delta[0],delta[1],delta[2]);
	tf::Quaternion q;
	q.setEuler(delta[3],delta[4],delta[5]);
	rigid_orientation = rigid_orientation * q;
}

}
