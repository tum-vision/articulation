/*
 * prismatic_model.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#include "articulation_models/models/prismatic_model.h"
#include "articulation_models/utils.h"

using namespace std;

#include "Eigen/Core"
#include <Eigen/SVD>

using namespace Eigen;

using namespace articulation_msgs;

#include <iomanip>
#define VEC(a) setprecision(5)<<fixed<<a.x()<<" "<<a.y()<<" "<<a.z()<<" "<<a.w()<<" l="<<a.length()
#define VEC2(a) "t=["<<VEC(a.getOrigin())<<"] r=[]"<<VEC(a.getRotation())<<"]"
#define PRINT(a) cout << #a <<"=" << VEC(a)<<endl;
#define PRINT2(a) cout << #a <<"=" << VEC2(a)<<endl;

namespace articulation_models {

PrismaticModel::PrismaticModel() {
	prismatic_dir = tf::Vector3(0,0,0);

	complexity = 6+2; // rigid + 2 (orientation, without yaw)
}

// -- params
void PrismaticModel::readParamsFromModel() {
	RigidModel::readParamsFromModel();
	getParam("prismatic_dir",prismatic_dir);
}

void PrismaticModel::writeParamsToModel() {
	RigidModel::writeParamsToModel();
	setParam("prismatic_dir",prismatic_dir,ParamMsg::PARAM);
}

V_Configuration PrismaticModel::predictConfiguration(geometry_msgs::Pose pose) {
	tf::Vector3 diff = (positionToVector(pose.position) - rigid_position);

	V_Configuration q( 1 );
	q(0) = diff.dot(prismatic_dir);

	return q;
}

geometry_msgs::Pose PrismaticModel::predictPose(V_Configuration q) {
//	cout << "predictPose q(0)=" <<q(0) << endl;
//	PRINT(rigid_orientation);
//	PRINT(rigid_position);
//	PRINT(prismatic_dir);
	return transformToPose( tf::Transform( rigid_orientation, rigid_position + q(0) * prismatic_dir ) );
}

M_CartesianJacobian PrismaticModel::predictHessian(V_Configuration q,double delta) {
	M_CartesianJacobian H;
	H.setZero(3*getDOFs(),getDOFs());
	return H;
}

bool PrismaticModel::guessParameters() {
	if(model.track.pose.size() < 2)
		return false;

	size_t i,j;
	do{
		i = rand() % getSamples();
		j = rand() % getSamples();
	} while (i==j);

	tf::Transform pose1 = poseToTransform(model.track.pose[i]);
	tf::Transform pose2 = poseToTransform(model.track.pose[j]);

	rigid_position = pose1.getOrigin();
	rigid_orientation = pose1.getRotation();
	prismatic_dir =  pose2.getOrigin() - pose1.getOrigin();
	prismatic_dir.normalize();

	if(!check_values(rigid_position)) return false;
	if(!check_values(rigid_orientation)) return false;
	if(!check_values(prismatic_dir)) return false;

	return true;
}

void PrismaticModel::updateParameters(std::vector<double> delta) {
	RigidModel::updateParameters(delta);

	tf::Quaternion q;
	q.setRPY(delta[6],delta[7],0.00);
	prismatic_dir = tf::Matrix3x3(q) * prismatic_dir;
}
bool PrismaticModel::normalizeParameters() {
	if(model.track.pose.size()>2) {
		rigid_position = rigid_position + predictConfiguration(model.track.pose.front())[0] * prismatic_dir;
		if(predictConfiguration(model.track.pose.back())[0]<0)
			prismatic_dir *= -1;
	}
	return true;
}

}
