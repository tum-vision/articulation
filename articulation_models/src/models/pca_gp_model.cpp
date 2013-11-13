/*
 * pca_gp_model.cpp
 *
 *  Created on: Feb 10, 2010
 *      Author: sturm
 */

#include "articulation_models/models/pca_gp_model.h"
#include "articulation_models/utils.h"

using namespace std;
using namespace articulation_msgs;

#include "Eigen/Core"
#include <Eigen/SVD>

using namespace Eigen;

#include <iomanip>
#define VEC(a) setprecision(5)<<fixed<<a.x()<<" "<<a.y()<<" "<<a.z()<<" "<<a.w()<<" l="<<a.length()
#define VEC2(a) "t=["<<VEC(a.getOrigin())<<"] r=[]"<<VEC(a.getRotation())<<"]"
#define PRINT(a) cout << #a <<"=" << VEC(a)<<endl;
#define PRINT2(a) cout << #a <<"=" << VEC2(a)<<endl;

#include <boost/format.hpp>
using namespace boost;

namespace articulation_models {

PCAGPModel::PCAGPModel() {
	complexity = 0;

	downsample = 20;

	//todo:add parameters here
	CovFuncND initialCovFunc;
	double initialSigmaNoise;
	vector<double> params = vector<double>(2);	// DOF + 1
	params[0] = -0.5;	// length scale, dof 1
	params[1] = 0.0;	// sigma
	initialCovFunc.setHyperparameter(params);
	initialSigmaNoise = -5;

	for(size_t i=0;i<7;i++) {
		gp.push_back( new gaussian_process::SingleGP(initialCovFunc,initialSigmaNoise) );
	}

	initialized = false;
	outlier_ratio = 0.0;

}

PCAGPModel::~PCAGPModel() {
	for(size_t i=0;i<gp.size();i++) {
		delete gp[i];
	}
	gp.clear();
}


void PCAGPModel::readParamsFromModel() {
	GenericModel::readParamsFromModel();
	getParam("downsample",downsample);
	getParam("rigid_position",rigid_position);
	getParam("prismatic_dir",prismatic_dir);
	double training_samples_float=0.00;
	getParam("training_samples",training_samples_float);
	training_samples = (int)training_samples_float;
	complexity = 3 + 6*training_samples;

	checkInitialized();//gp needs to build covariance matrix
}

void PCAGPModel::writeParamsToModel() {
	GenericModel::writeParamsToModel();
	setParam("downsample",downsample,ParamMsg::PARAM);
	setParam("rigid_position",rigid_position,ParamMsg::PARAM);
	setParam("prismatic_dir",prismatic_dir,ParamMsg::PARAM);
	setParam("training_samples",training_samples,ParamMsg::PARAM);
}

tf::Transform PCAGPModel::pose(size_t index) {
	tf::Transform pose;
	getParam(str(format("pose[%d]")%index),pose);
	return pose;
}

void PCAGPModel::storeData(bool inliersOnly) {
	// copy data into params
	bool resample = false;
	size_t n = model.track.pose.size();

	vector<double> inliers_cum(n,0);
	double sum = 0;
	for(size_t i=0;i<n;i++) {
		if(inliersOnly)
			sum += 1 - model.track.channels[channelOutlier].values[i];
		else
			sum += 1;
		inliers_cum[i] = sum;
	}
//	cout <<"inlier sum is "<<sum<<endl;

	training_samples = (int)sum;
	if(downsample>0 && n>downsample) {
		training_samples  = downsample;
		resample = true;
	}
	complexity = 1 + getDOFs() + 6*training_samples;

	for(size_t i=0;i<training_samples;i++) {
		double choice = i / (double)training_samples;
		size_t j=0;
		for(j=0;j<n;j++) {
			if(inliers_cum[j]/sum>choice) break;
		}
//		cout <<"choosing j="<<j<<endl;
		setParam(str(format("pose[%d]")%i),poseToTransform(model.track.pose[j]),ParamMsg::PARAM);
	}

}

bool PCAGPModel::fitModel() {
	if(getSamples()<3) return true;




	// outlier experimental
	storeData(false);	// use all points
	outlier_ratio = 0.0;
	buildGPs();
	getLogLikelihood(true);
////	double old_outlier_ratio;
////	int iter=0;
////	do {
////		storeData(true);	// use only inliers
////		buildGPs();
////		old_outlier_ratio = outlier_ratio;
////		getLogLikelihood(true);
////		iter ++;
////		cout <<"GP EM iteration="<<iter<<" outlier_ratio="<<outlier_ratio<<endl;
////	} while(iter < 10 && fabs(outlier_ratio - old_outlier_ratio) > 0.01);
//
//	cout << "outlier ratio="<<this->outlier_ratio<<endl;

	return true;
}

void PCAGPModel::buildGPs() {
	if(training_samples<3) return;
	// fit PCA
	// compute center
	tf::Vector3 sum_p(0,0,0);
	for (size_t j = 0; j < training_samples; j++) {
		sum_p = sum_p + pose(j).getOrigin();
	}
	rigid_position = sum_p / training_samples;

	// find line direction
	MatrixXf X_trans(training_samples, 3);
	for (size_t j = 0; j < training_samples; j++) {
		tf::Vector3 diff = pose(j).getOrigin() - rigid_position;
		X_trans(j, 0) = diff.x();
		X_trans(j, 1) = diff.y();
		X_trans(j, 2) = diff.z();
	}

	VectorXf b(training_samples);
	VectorXf x(3);
	b = VectorXf::Zero(training_samples);
	JacobiSVD<MatrixXf> svdOfA(X_trans);

	const Eigen::MatrixXf U = svdOfA.matrixU();
	const Eigen::MatrixXf V = svdOfA.matrixV();
	const Eigen::VectorXf S = svdOfA.singularValues();

//	Eigen::MatrixXf Y_trans = U * S.asDiagonal();
//	cout << endl;
//	cout <<"X=["<<endl;
//	for(int i=0;i<X_trans.rows();i++) {
//		for(int j=0;j<X_trans.cols();j++) {
//			if(j!=0)
//				cout <<", ";
//			cout <<X_trans(i,j);
//		}
//		cout <<";"<<endl;
//	}
//	cout << "]"<<endl;
//	cout << X_trans<<endl;
//	cout << endl;
//	cout << Y_trans<<endl;
//	cout << endl;
//	cout << V;
//	cout << endl;
	double v0=V(0,0),v1=V(1,0),v2=V(2,0);
	prismatic_dir= tf::Vector3(v0,v1,v2);
//	PRINT(prismatic_dir);

	if(!check_values(rigid_position)) {
		cerr <<"rigid_position has NaNs"<<endl;
		return ;
	}
	if(!check_values(prismatic_dir))  {
		cerr <<"prismatic_dir has NaNs"<<endl;
		return ;
	}
//	cout << X_trans << endl;
//	PRINT(rigid_position);
//	PRINT(prismatic_dir);
//	writeParamsToModel();
//	prismatic_dir = tf::Vector3(1,-1,0);
//	prismatic_dir = tf::Vector3(0.48308,-0.87558,0.00000);
//	prismatic_dir = tf::Vector3(0.88215,0.46638,-0.06560);
//	prismatic_dir = tf::Vector3(prismatic_dir.x(),prismatic_dir.y(),prismatic_dir.z());
//	prismatic_dir.normalize();
//	cout <<"fitting:";
//	PRINT(prismatic_dir);


	TVector<TDoubleVector> input(training_samples);
	TVector<double> output[7];
	for(size_t j=0;j<7;j++) {
		output[j] = TDoubleVector(training_samples);
	}

	for(size_t i=0;i<training_samples;i++) {
		input[i] = TDoubleVector(1);

		geometry_msgs::Pose p = transformToPose(pose(i));
		input[i][0] = predictConfiguration( p )[0];
		output[0][i] = p.position.x;
		output[1][i] = p.position.y;
		output[2][i] = p.position.z;
		output[3][i] = p.orientation.w;
		output[4][i] = p.orientation.x;
		output[5][i] = p.orientation.y;
		output[6][i] = p.orientation.z;
	}

	for(size_t j=0;j<7;j++) {
//		cout << "setting data, j="<<j<<" n="<<input.size()<<endl;
		gp[j]->SetData(input,output[j]);
//		gp[j]->OptimizeGP();
	}


 //	cout << "fitting model, end.."<<endl;
}

V_Configuration PCAGPModel::predictConfiguration(geometry_msgs::Pose pose) {
	tf::Vector3 diff = (positionToVector(pose.position) - rigid_position);
//	PRINT(diff);
//	PRINT(prismatic_dir);

	V_Configuration q( 1 );
	q(0) = diff.dot(prismatic_dir);

	return q;
}

geometry_msgs::Pose PCAGPModel::predictPose(V_Configuration q) {
	geometry_msgs::Pose result;
	double mean_x=0,var_x=0;
	double mean_y=0,var_y=0;
	double mean_z=0,var_z=0;
	double mean_rw=0,var_rw=0;
	double mean_rx=0,var_rx=0;
	double mean_ry=0,var_ry=0;
	double mean_rz=0,var_rz=0;
	TDoubleVector inp(1);
	inp(0) = q[0];
	gp[0]->Evaluate( inp, mean_x, var_x );
	gp[1]->Evaluate( inp, mean_y, var_y );
	gp[2]->Evaluate( inp, mean_z, var_z );
	gp[3]->Evaluate( inp, mean_rw, var_rw );
	gp[4]->Evaluate( inp, mean_rx, var_rx );
	gp[5]->Evaluate( inp, mean_ry, var_ry );
	gp[6]->Evaluate( inp, mean_rz, var_rz );

	tf::Quaternion pred_rot(mean_rx,mean_ry,mean_rz,mean_rw);
	tf::Vector3 pred_pos(mean_x,mean_y,mean_z);
	pred_rot.normalize();
	tf::Transform pred_pose(pred_rot,pred_pos);
	result = transformToPose(pred_pose);

//	tf::Transform t( rigid_orientation, rigid_position + prismatic_dir*q[0]);
//	result = transformToPose(t);

	return result;
}

void PCAGPModel::checkInitialized() {
	if(!initialized) {
		buildGPs();
		initialized = true;
	}
}

void PCAGPModel::projectPoseToConfiguration() {
	checkInitialized();
	GenericModel::projectPoseToConfiguration();
}

void PCAGPModel::projectConfigurationToPose() {
	checkInitialized();
	GenericModel::projectConfigurationToPose();
}

void PCAGPModel::projectConfigurationToJacobian() {
	checkInitialized();
	GenericModel::projectConfigurationToJacobian();
}

bool PCAGPModel::evaluateModel() {
	checkInitialized();
	return GenericModel::evaluateModel();
}

bool PCAGPModel::normalizeParameters() {
	if(model.track.pose.size()>2) {
		rigid_position = rigid_position + predictConfiguration(model.track.pose.front())[0] * prismatic_dir;
		if(predictConfiguration(model.track.pose.back())[0]<0)
			prismatic_dir *= -1;
	}
	return true;
}
}
