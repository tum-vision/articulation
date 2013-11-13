/*
 * generic_model.cpp
 *
 *  Created on: Oct 19, 2009
 *      Author: sturm
 */
#include <boost/format.hpp>
#include "articulation_models/models/generic_model.h"

#include "articulation_models/utils.h"
#include "articulation_models/models/factory.h"
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_blas.h>

using namespace std;
using namespace boost;

using namespace Eigen;

using namespace articulation_msgs;

#include <iomanip>
#define VEC(a) setprecision(5)<<fixed<<a.x()<<" "<<a.y()<<" "<<a.z()<<" "<<a.w()<<" l="<<a.length()
#define VEC2(a) "t=["<<VEC(a.getOrigin())<<"] r=[]"<<VEC(a.getRotation())<<"]"
#define PRINT(a) cout << #a <<"=" << VEC(a)<<endl;
#define PRINT2(a) cout << #a <<"=" << VEC2(a)<<endl;

namespace articulation_models {

GenericModel::GenericModel() {
	setId(-1);
	sigma_position = 0.005;
	sigma_orientation = 360 * M_PI/180.0;

	avg_error_position = 0;
	avg_error_orientation = 0;
	bic = 0;

	last_error_jacobian = 0;
	evaluated = false;
	supress_similar = true;
	outlier_ratio = 0.5;
	sac_iterations = 100;
	optimizer_iterations = 10;
//	optimizer_iterations = 0;

	prior_outlier_ratio = log(0.01) / (- 0.05); // prior over outlier ratio

	complexity = 0;
	jacobian = Eigen::MatrixXd();
	hessian = Eigen::MatrixXd();
}

void GenericModel::setModel(const ModelMsg& model) {
	this->model = model;
	readParamsFromModel();
	prepareChannels();
}

ModelMsg GenericModel::getModel() {
    writeParamsToModel();
	model.name = getModelName();
	model.header = model.track.header;
	model.track.pose_flags.resize(model.track.pose.size(),TrackMsg::POSE_VISIBLE );
	if(model.track.pose.size())
		model.track.pose_flags[model.track.pose.size()-1] |= TrackMsg::POSE_END_OF_SEGMENT;
	return model;
}

void GenericModel::setTrack(const TrackMsg& track) {
	this->model.track = track;
	prepareChannels();
}

TrackMsg GenericModel::getTrack() {
	return model.track;
}

void GenericModel::setId(int id) {
	model.id = id;
}

int GenericModel::getId() {
	return(model.id);
}

// -- model information
std::string GenericModel::getModelName() {
	return MultiModelFactory::instance.getLongName(this);
}

size_t GenericModel::getDOFs() {
	return 0;
}

size_t GenericModel::getSamples() {
	return model.track.pose.size();
}

// -- params
void GenericModel::readParamsFromModel() {
	getParam("sigma_position",sigma_position);
	getParam("sigma_orientation",sigma_orientation);
	getParam("supress_similar",supress_similar);
	getParam("avg_error_position",avg_error_position);
	getParam("avg_error_orientation",avg_error_orientation);
	getParam("bic",bic);
	getParam("last_error_jacobian",last_error_jacobian);
	getParam("evaluated",evaluated);
//	getParam("complexity",complexity);//read-only
	getParam("jacobian",jacobian);
	getParam("hessian",hessian);
	getParam("loglikelihood",loglikelihood);
	getParam("outlier_ratio",outlier_ratio);
	getParam("sac_iterations",sac_iterations);
	getParam("prior_outlier_ratio",prior_outlier_ratio);
}

void GenericModel::writeParamsToModel() {
	setParam("sigma_position",sigma_position,ParamMsg::PRIOR);
	setParam("sigma_orientation",sigma_orientation,ParamMsg::PRIOR);
	setParam("supress_similar",supress_similar,ParamMsg::PRIOR);
	setParam("avg_error_position",avg_error_position,ParamMsg::EVAL);
	setParam("avg_error_orientation",avg_error_orientation,ParamMsg::EVAL);
	setParam("loglikelihood",loglikelihood,ParamMsg::EVAL);
	setParam("bic",bic,ParamMsg::EVAL);
	setParam("last_error_jacobian",last_error_jacobian,ParamMsg::EVAL);
	setParam("evaluated",evaluated,ParamMsg::EVAL);
	setParam("complexity",complexity,ParamMsg::PRIOR);
	setParam("jacobian",jacobian,ParamMsg::EVAL);
	setParam("hessian",hessian,ParamMsg::EVAL);
	setParam("dofs",getDOFs(),ParamMsg::EVAL);
	setParam("samples",getSamples(),ParamMsg::EVAL);
	setParam("outlier_ratio",outlier_ratio,ParamMsg::EVAL);
	setParam("sac_iterations",sac_iterations,ParamMsg::PRIOR);
	setParam("prior_outlier_ratio",prior_outlier_ratio,ParamMsg::PRIOR);
}

void GenericModel::prepareChannels() {
	channelOutlier = openChannel("outlier");
	channelLogLikelihood = openChannel("loglikelihood");
	channelInlierLogLikelihood = openChannel("loglikelihood_if_inlier");
	channelConfiguration.resize(getDOFs(),0);
	for(size_t i=0;i<getDOFs();i++) {
		channelConfiguration[i] = openChannel(str(format("q%d")%i));
	}

}

double GenericModel::getParam(std::string name) {
	for(size_t i=0;i<model.params.size();i++) {
		if(model.params[i].name == name) {
			return(model.params[i].value);
		}
	}
//	std::cerr << "WARNING (in GenericModel::getParam): undefined parameter '"<<name<<"'" << std::endl;
	return 0.00;
}

void GenericModel::getParam(std::string name,double& value) {
	if(hasParam(name))
		value = getParam(name);
}

void GenericModel::getParam(std::string name,Eigen::VectorXd &vec) {
	for(int i=0;i<vec.rows();i++)
		vec(i) = getParam( str(boost::format( name+"[%1%]") %i ) );
}

void GenericModel::getParam(std::string name,tf::Vector3 &vec) {
	vec.setX( getParam( name+".x" ) );
	vec.setY( getParam( name+".y" ) );
	vec.setZ( getParam( name+".z" ) );
}

void GenericModel::getParam(std::string name,tf::Quaternion &quat) {
	quat.setX( getParam( name+".x" ) );
	quat.setY( getParam( name+".y" ) );
	quat.setZ( getParam( name+".z" ) );
	quat.setW( getParam( name+".w" ) );
}

void GenericModel::getParam(std::string name,tf::Transform &t) {
	tf::Quaternion q;
	tf::Vector3 v;
	getParam(name+".orientation",q);
	getParam(name+".position",v);
	t = tf::Transform(q,v);
}

void GenericModel::getParam(std::string name,Eigen::MatrixXd &mat) {
	for(int r=0;r<mat.rows();r++)
		for(int c=0;c<mat.cols();c++)
			mat(r,c) = getParam( str(boost::format( name+"[%1%][%2%]") %r % c) );
}

void GenericModel::setParam(std::string name,double value,int type) {
	for(size_t i=0;i<model.params.size();i++) {
		if(model.params[i].name == name) {
			model.params[i].value = value;
			return;
		}
	}
	ParamMsg data;
	data.name = name;
	data.value = value;
	data.type = type;
	model.params.push_back(data);
}

void GenericModel::setParam(std::string name,const Eigen::VectorXd &vec,int type) {
	for(int i=0;i<vec.rows();i++)
		setParam( boost::str(boost::format( name+"[%1%]") %i ),vec(i),type );
}

void GenericModel::setParam(std::string name,const tf::Vector3 &vec,int type) {
	setParam( name+".x",vec.x(),type );
	setParam( name+".y",vec.y(),type );
	setParam( name+".z",vec.z(),type );
}

void GenericModel::setParam(std::string name,const tf::Quaternion &quat,int type) {
	setParam( name+".x",quat.x(),type );
	setParam( name+".y",quat.y(),type );
	setParam( name+".z",quat.z(),type );
	setParam( name+".w",quat.w(),type );
}

void GenericModel::setParam(std::string name,const tf::Transform &t,int type) {
	setParam(name+".position",t.getOrigin(),type);
	setParam(name+".orientation",t.getRotation(),type);
}

void GenericModel::setParam(std::string name,const Eigen::MatrixXd &mat,int type) {
	for(int r=0;r<mat.rows();r++)
		for(int c=0;c<mat.cols();c++)
			setParam( str(boost::format( name+"[%1%][%2%]") %r%c ),mat(r,c),type );
}

bool GenericModel::hasParam(std::string name) {
	for(size_t i=0;i<model.params.size();i++) {
		if(model.params[i].name == name)
			return(true);
	}
	return false;
}

// -- track data
int GenericModel::openChannel(std::string name,bool autocreate) {
	return(articulation_models::openChannel(model.track,name,autocreate));
}

// -- train model
bool GenericModel::fitModel() {
	if(!sampleConsensus()) {
//		if(DEBUG) cout <<"sampleConsesus failed of "<<getModelName()<<endl;
		return false;
	}
	if(!optimizeParameters()) {
//		if(DEBUG) cout <<"optimizeParameters failed of "<<getModelName()<<endl;
		return false;
	}
	if(!normalizeParameters()) {
		return false;
	}
    return true;
}

bool GenericModel::fitMinMaxConfigurations() {
	setParam("q_min",getMinConfigurationObserved(),ParamMsg::PARAM);
	setParam("q_max",getMaxConfigurationObserved(),ParamMsg::PARAM);
	return true;
}


// -- evaluate model
bool GenericModel::evaluateModel() {
	// need at least one data point
	if(model.track.pose.size() == 0)
		return false;

	// let getLogLikelihood() do the projection
	loglikelihood = getLogLikelihood(false);
	// evaluate some extra statistics

	// get params
	size_t n = model.track.pose.size();

	// compute pose difference and likelihoods
	double sum_position2 = 0;
	double sum_orientation2 = 0;

	double sum_position = 0;
	double sum_orientation = 0;
//	double sum_loglikelihood = 0;

	for(size_t i=0;i<n;i++) {
		tf::Transform p1 = poseToTransform(model.track.pose[i]);
		tf::Transform p2 = poseToTransform(model.track.pose_projected[i]);
		tf::Transform diff = p1.inverseTimes(p2);
		double err_position2 = diff.getOrigin().length2();
		double err_orientation2 = SQR(diff.getRotation().getAngle());
//		sum_loglikelihood += model.track.channels[channelInlierLogLikelihood].values[i];

		sum_position2 += err_position2;
		sum_orientation2 += err_orientation2;
		sum_position += sqrt(err_position2);
		sum_orientation += sqrt(err_orientation2);
	}

	// store into cache
	avg_error_position = sum_position/n;
	avg_error_orientation = sum_orientation/n;
	loglikelihood += - ((double)n)*getDOFs()*log(n);

	bic =
					-2*(loglikelihood )
					+complexity
					* log( n );

	last_error_jacobian = evalLatestJacobian();

	evaluated = true;

	if(model.track.pose.size()>=2) {
		VectorXd q1 = predictConfiguration( model.track.pose.front() );
		VectorXd q2 = predictConfiguration( model.track.pose.back() );

		jacobian  = predictJacobian( q2 );
		hessian = predictHessian( q2 );
		if(getDOFs()>=1 ) {
			VectorXd p(3);
			for(size_t i=0;i<model.track.pose.size();i++) {
				p += pointToEigen(model.track.pose[i].position) - pointToEigen(model.track.pose.front().position);
			}
			if(p.dot(jacobian.col(0))<0)	// make Jacobian point into current direction (of dof 1)
				jacobian *= -1;
				hessian *= -1;
		}
	}
	writeParamsToModel();
	return true;
}

double GenericModel::evalLatestJacobian() {
	if(model.track.pose.size()<2)
		return 0.00;

	// observed direction of motion
	VectorXd p1 = pointToEigen( model.track.pose[model.track.pose.size()-1].position );
	VectorXd p2 = pointToEigen( model.track.pose[model.track.pose.size()-2].position );
	VectorXd pD_obs = p1-p2;
	if(pD_obs.norm() == 0)
		return 0.00;
	pD_obs.normalize();
//	cout << "p1="<<p1<<endl;
//	cout << "p2="<<p2<<endl;
//	cout << "pD_obs="<<pD_obs<<endl;

	if(getDOFs()==0)
		return 2*M_PI;

	// predicted direction of motion
	VectorXd q1 = predictConfiguration( model.track.pose[model.track.pose.size()-1] );
	VectorXd q2 = predictConfiguration( model.track.pose[model.track.pose.size()-2] );
//	cout << "q1="<<q1<<endl;
//	cout << "q2="<<q2<<endl;
	MatrixXd J2 = predictJacobian( q2 );
//	cout << "J2="<<J2<<endl;
	VectorXd qD = (q1 - q2);
//	cout << "qD="<<qD<<endl;
	VectorXd pD_pred = J2 * qD; // reduce Jacobian
//	cout << "pD_pred="<<pD_pred<<endl;
	if(pD_pred.norm() == 0)
		return 0.00;
	pD_pred.normalize();
//	cout << "qD_pred.normalized="<<pD_pred<<endl;
//
//	cout << "angle_diff="<<acos( pD_obs.dot( pD_pred ) )<<endl;
//
//	// return angle between predicted motion and observed motion
	return acos( pD_obs.dot( pD_pred ) );
}

double GenericModel::getPositionError() {
	return avg_error_position;
}

double GenericModel::getOrientationError() {
	return avg_error_orientation;
}

double GenericModel::getBIC() {
	return bic;
}

// -- cartesian space
geometry_msgs::Pose GenericModel::predictPose(V_Configuration q) {
	geometry_msgs::Pose p;
	p.orientation.w = 1;
	return p;
}

M_CartesianJacobian GenericModel::predictJacobian(V_Configuration vq,double delta) {
	M_CartesianJacobian J;
	J.resize(3,getDOFs());
	VectorXd p = pointToEigen(predictPose(vq).position);
	for(size_t i=0;i<getDOFs();i++) {
		V_Configuration q = vq;
		q(i) += delta;
		J.col(i) = (pointToEigen( predictPose(q).position ) - p)/delta;
	}
	return J;
}

M_CartesianJacobian GenericModel::predictHessian(V_Configuration q,double delta) {
	M_CartesianJacobian H;
	H.resize(3*getDOFs(),getDOFs());
//	cout <<"dofs="<<getDOFs()<<" q.size"<<vq.size()<<endl;
	for(size_t i=0;i<getDOFs();i++) {
		V_Configuration qd = q;
		q(i) += delta;
		M_CartesianJacobian H_part;

		M_CartesianJacobian J = predictJacobian(q);
		M_CartesianJacobian Jd = predictJacobian(qd);

//		cout << J(0,0) << " "<< J(1,0) << " "<< J(2,0) << endl;
//		cout << "H_part "<<Jd(0,0) << " "<< Jd(1,0) << " "<< Jd(2,0) << endl;

		H_part = (Jd - J)/delta;
//		cout << "H_part "<<H_part(0,0) << " "<< H_part(1,0) << " "<< H_part(2,0) << endl;
		for(size_t r=0;r<3;r++) {
			for(size_t c=0;c<getDOFs();c++) {
				H(r+3*i,c) = H_part(r,c);
			}
		}
	}
	return H;
}

// -- configuration space
void GenericModel::setConfiguration(size_t index,V_Configuration q) {
	std::map<int,int> channel;
	// get channel ids and prepare channels
	for(size_t ch=0;ch<getDOFs();ch++) {
		std::stringstream s;
		s << "q" << ch;
		channel[ch] = openChannel( s.str() );
	}

	for(int j=0;j<q.rows();j++) {
		model.track.channels[ channel[j] ].values[index] = q[j];
	}
}

// -- configuration space
void GenericModel::setJacobian(size_t index,M_CartesianJacobian J) {
	std::map<int,std::map<int,int> > channel;
	// get channel ids and prepare channels
	for(int r=0;r<J.rows();r++)
		for(int c=0;c<J.cols();c++)
		channel[r][c] = openChannel( boost::str(boost::format( "J[%1%][%2%]") %r%c ) );

	for(int r=0;r<J.rows();r++)
		for(int c=0;c<J.cols();c++)
			model.track.channels[ channel[r][c] ].values[index] = J(r,c);
}

V_Configuration GenericModel::getConfiguration(size_t index) {
	std::map<int,int> channel;
	// get channel ids and prepare channels
	for(size_t ch=0;ch<getDOFs();ch++) {
		std::stringstream s;
		s << "q" << ch;
		channel[ch] = openChannel( s.str() );
	}

	V_Configuration q;
	if (getDOFs()) q.resize(getDOFs());
	for(int j=0;j<q.rows();j++) {
		q[j] = model.track.channels[ channel[j] ].values[index];
	}
	return(q);
}

V_Configuration GenericModel::predictConfiguration(geometry_msgs::Pose pose) {
	V_Configuration q;
	if(getDOFs()>0) q.resize(getDOFs());
	return q;
}



V_Configuration GenericModel::getMinConfigurationObserved() {
	if(getDOFs()==0) return V_Configuration();
	V_Configuration q_min(getDOFs());
	for(size_t j=0;j<getDOFs();j++) {
		q_min[j] = FLT_MAX;
	}

	for(size_t i=0;i<model.track.pose.size();i++) {
		V_Configuration q = getConfiguration(i);
		for(size_t j=0;j<getDOFs();j++) {
			q_min[j] = MIN( q_min[j], q[j] );
		}
	}

	return q_min;
}

V_Configuration GenericModel::getMaxConfigurationObserved() {
	if(getDOFs()==0) return V_Configuration();
	V_Configuration q_max(getDOFs());
	for(size_t j=0;j<getDOFs();j++) {
		q_max[j] = -FLT_MAX;
	}

	for(size_t i=0;i<model.track.pose.size();i++) {
		V_Configuration q = getConfiguration(i);
		for(size_t j=0;j<getDOFs();j++) {
			q_max[j] = MAX( q_max[j], q[j] );
		}
	}

	return q_max;
}

// -- projections of track data
void GenericModel::projectPoseToConfiguration() {
	// now estimate configurations, and store in channels q0..qn
	for(size_t i=0;i<model.track.pose.size();i++) {
		V_Configuration q;
		q = predictConfiguration( model.track.pose[i] );
		setConfiguration(i,q);
	}
}

void GenericModel::projectConfigurationToPose() {
	// now estimate poses
	model.track.pose_projected.resize(model.track.pose.size());
	for(size_t i=0;i<model.track.pose.size();i++) {
		V_Configuration q = getConfiguration(i);
		model.track.pose_projected[i] = predictPose( q );
	}
}

void GenericModel::projectConfigurationToJacobian() {
	// now estimate poses
	model.track.pose_projected.resize(model.track.pose.size());
	for(size_t i=0;i<model.track.pose.size();i++) {
		V_Configuration q = getConfiguration(i);
		setJacobian(i, predictJacobian( q ));
	}
}

void GenericModel::sampleConfigurationSpace(double resolution) {
//	cout << "resolution="<<resolution<<endl;
	// clear all poses
	model.track.pose_resampled.clear();

	if(getDOFs() == 0) {
		model.track.pose_resampled.push_back( predictPose(V_Configuration() ) );	// single sample, empty configuration
		return;
	}

	V_Configuration q_min(getDOFs());
	V_Configuration q_max(getDOFs());
	getParam("q_min",q_min);
	getParam("q_max",q_max);

	geometry_msgs::Pose p_min = predictPose( q_min );
	geometry_msgs::Pose p_max = predictPose( q_max );
	double dist =
			(positionToVector(p_min.position) - positionToVector(p_max.position)).length();
	if(isnan(dist) || isinf(dist)) return;

	// show all axes..
	for(size_t j=0;j<getDOFs();j++) {
		size_t num = MAX(2,1 + (int)( dist /resolution ));
		num = MIN(1000,MAX(num,fabs((double)(q_min[0]-q_max[0])/resolution)));
		double new_res = (q_max[j] - q_min[j]) / (num-1);

		for(size_t i=0;i<num;i++) {
			V_Configuration q(getDOFs());
			q[j] = q_min[j] + i*new_res;

			geometry_msgs::Pose pose = predictPose( q );
			model.track.pose_resampled.push_back(pose);
		}
	}
}

void GenericModel::keepLatestPoseOnly() {
	if(model.track.pose.size()<=1) return;
	model.track.pose.erase( model.track.pose.begin(), model.track.pose.begin() + (model.track.pose.size() -1) );
	for(size_t i = 0; i < model.track.channels.size(); i++) {
		model.track.channels[i].values.erase(
				model.track.channels[i].values.begin(),
				model.track.channels[i].values.begin() + (model.track.channels[i].values.size() -1)
		);
	}
}

double GenericModel::getInlierLogLikelihood( size_t index ) {
	geometry_msgs::Pose &pose_obs = model.track.pose[index];
	V_Configuration q_estimated = predictConfiguration(pose_obs);
	for(size_t i=0;i<(size_t)q_estimated.rows();i++) {
		model.track.channels[channelConfiguration[i]].values[index] =  q_estimated[i];
	}

	geometry_msgs::Pose pose_estimated = predictPose(q_estimated);
	model.track.pose_projected[index] = pose_estimated;

	tf::Transform diff = poseToTransform(pose_obs).inverseTimes( poseToTransform(pose_estimated) );
	double err_position = diff.getOrigin().length();
	double err_orientation = fabs(diff.getRotation().getAngle());

	double loglikelihood =
			- log(2*M_PI * sigma_position*sigma_orientation)
			- 0.5*(
					( SQR(err_position) / (SQR(sigma_position)) ) +
					( SQR(err_orientation) / SQR(sigma_orientation)) );	// 2-dim multivariate normal distribution
	return loglikelihood;
}

double GenericModel::getOutlierLogLikelihood() {
	// outside the 95% ellipsoid
	double chi2inv = 1.96;
//	chi2inv = 6.63;	//99% ellipsoid

	double err_position = chi2inv * sigma_position;
	double err_orientation = chi2inv * sigma_orientation;

	double loglikelihood =
			- log(2*M_PI * sigma_position*sigma_orientation)
			- 0.5*(
					( SQR(err_position) / (SQR(sigma_position)) ) +
					( SQR(err_orientation) / SQR(sigma_orientation)) );	// 2-dim multivariate normal distribution

	return( loglikelihood );
}

double GenericModel::getLogLikelihoodForPoseIndex(size_t index) {
//	double gamma_mixing = 0.1;
//	return (1-gamma_mixing)*getInlierLogLikelihood(pose_obs) + gamma_mixing * getOutlierLogLikelihood();
	double inlierLikelihood = getInlierLogLikelihood(index);
	double outlierLikelihood = getOutlierLogLikelihood();

//  mle-sac
	model.track.channels[channelInlierLogLikelihood].values[index] = inlierLikelihood;
	double pi = (1-outlier_ratio) * exp( inlierLikelihood );
	double po = outlier_ratio * exp( outlierLikelihood );
	model.track.channels[channelOutlier].values[index] =  po / (pi+po);
	double p = (log(exp(inlierLikelihood) + exp(outlierLikelihood)));
	model.track.channels[channelLogLikelihood].values[index] =  p;
	return p;

//	// ransac
//	return ((1-gamma_mixing)*inlierLikelihood + (gamma_mixing)*outlierLikelihood);

//	// m-sac
//	return (max(inlierLikelihood,outlierLikelihood));


}

double GenericModel::getLogLikelihood(bool estimate_outlier_ratio) {
	model.track.pose_projected.resize(model.track.pose.size());

	model.track.channels[channelInlierLogLikelihood].values.resize(model.track.pose.size());
	model.track.channels[channelOutlier].values.resize(model.track.pose.size());
	model.track.channels[channelLogLikelihood].values.resize(model.track.pose.size());
	for(size_t i=0;i<(size_t)getDOFs();i++) {
		model.track.channels[channelConfiguration[i]].values.resize(model.track.pose.size());
	}

	double sum_likelihood = 0;
	size_t n = getSamples();
	if(estimate_outlier_ratio) {
		outlier_ratio =0.5;
		for(size_t i=0;i<n;i++) {
			model.track.channels[channelOutlier].values[i] = 0.5;	// initial estimate of gamma
		}

		int iter = 0;
		double diff = 0;
		do{
			sum_likelihood = 0;
			for(size_t i=0;i<n;i++) {
				sum_likelihood += getLogLikelihoodForPoseIndex(i);
			}

			double outlier_ratio_new = 0;
			for(size_t i=0;i<n;i++) {
				outlier_ratio_new += model.track.channels[channelOutlier].values[i]/n;
			}
			diff = fabs(outlier_ratio - outlier_ratio_new);
			iter++;
			outlier_ratio = outlier_ratio_new;
//			cout <<"EM iter="<<iter<<" outlier_ratio="<<outlier_ratio<<endl;
		} while( diff>0.01 && iter<10 );
	} else {
		for(size_t i=0;i<n;i++) {
			sum_likelihood += getLogLikelihoodForPoseIndex(i);
		}
	}
	sum_likelihood += - prior_outlier_ratio * outlier_ratio * n;
	return sum_likelihood;
}

bool GenericModel::guessParameters() {
	// sample a minimum number of samples to compute parameters
	return false;
}

bool GenericModel::sampleConsensus() {
	// sample consensus
	writeParamsToModel();
	vector<articulation_msgs::ParamMsg> bestParams = model.params;
	double bestLikelihood = -DBL_MAX;
	bool goodGuess = false;
	for(size_t it=0;it<sac_iterations;it++) {
		if(!guessParameters()) continue;
		goodGuess = true;
		double likelihood = getLogLikelihood(true);
//		cout <<"RANSAC iteration="<<it<<", likelihood="<<likelihood<<endl;
		if(bestLikelihood < likelihood) {
			writeParamsToModel();
			bestParams = model.params;
			bestLikelihood = likelihood;
		}
	}
//	cout <<"RANSAC best likelihood="<<bestLikelihood<<endl;
	model.params = bestParams;
	readParamsFromModel();

	return goodGuess;
}

double
my_f (const gsl_vector *v, void *params)
{
  GenericModel *p = (GenericModel *)params;

  vector<double> delta( v->size );
  for(size_t i=0;i<v->size;i++) {
	  delta[i] = gsl_vector_get (v, i);
//	  cout <<"delta["<<i<<"]="<<delta[i]<<" ";
  }

  p->model.params = p->params_initial;
  p->readParamsFromModel();
  p->updateParameters(delta);
  p->writeParamsToModel();
  double likelihood = p->getLogLikelihood(true);
//  cout <<"likelihood="<<likelihood<<endl;
  return -likelihood;
}

/* The gradient of f, df = (df/dx, df/dy). */
void
my_df (const gsl_vector *v, void *params,
       gsl_vector *df)
{
	double likelihood = my_f(v,params);

	gsl_vector * v_delta =gsl_vector_alloc (v->size);

    double DELTA = 1e-3;
    for(size_t i=0;i<v->size;i++) {
        v_delta = gsl_vector_alloc (v->size);
        gsl_vector_memcpy(v_delta,v);
        gsl_vector_set(v_delta, i,gsl_vector_get(v, i)+DELTA);

    	double likelihood_delta = my_f(v_delta,params);
        gsl_vector_set(df, i,(likelihood_delta-likelihood)/DELTA);
	}
    gsl_vector_free (v_delta);
}

/* Compute both f and df together. */
void
my_fdf (const gsl_vector *x, void *params,
        double *f, gsl_vector *df)
{
  *f = my_f(x, params);
  my_df(x, params, df);
}

bool GenericModel::optimizeParameters() {
	// using gsl minimizer
	// dofs to optimize: complexity
	// stores current parameter vector as params_initial
	writeParamsToModel();
	params_initial = model.params;
	// calls updateParameters(..) to set new params vector

    size_t iter = 0;
    int status;

    const gsl_multimin_fdfminimizer_type *T;
    gsl_multimin_fdfminimizer *s;

    gsl_vector *x;
    gsl_multimin_function_fdf my_func;

    my_func.n = (int)complexity;
    my_func.f = my_f;
    my_func.df = my_df;
    my_func.fdf = my_fdf;
    my_func.params = this;

    x = gsl_vector_alloc ((int)complexity);
    gsl_vector_set_zero (x);
//    double likelihood_initial= -my_f(x,this);

    T = gsl_multimin_fdfminimizer_vector_bfgs2;
    s = gsl_multimin_fdfminimizer_alloc (T, (int)complexity);

    gsl_multimin_fdfminimizer_set (s, &my_func, x, 0.01, 0.1);

//    cout <<"optimizing "<<complexity<<" DOFs"<<endl;

//    for(size_t i=0;i<model.params.size();i++) {
//    	if(model.params[i].type != ParamMsg::PARAM) continue;
//    	cout <<" param "<<model.params[i].name<<"="<<model.params[i].value<<endl;
//    }

    if(optimizer_iterations>0) do
      {
        iter++;
//        cout <<"iter "<<iter<<".."<<endl;
        status = gsl_multimin_fdfminimizer_iterate (s);

        if (status)
          break;

        status = gsl_multimin_test_gradient (s->gradient, 1e-1);

//        if (status == GSL_SUCCESS)
//          printf ("Minimum found at:\n");

//        printf ("i=%5d ", (int)iter);
//        for(size_t i=0;i<s->x->size;i++) {
//            printf ("%5f ", gsl_vector_get(s->x,i));
//        }
//        printf("\n");
      }
    while (status == GSL_CONTINUE && iter < optimizer_iterations);

//    double likelihood_final = -my_f(s->x,this);
//    cout <<"likelihood_initial="<<likelihood_initial<<" ";
//    cout <<"likelihood_final="<<likelihood_final<<" ";
//    cout <<"after "<<iter<<" iterations";
//    cout << endl;

    gsl_multimin_fdfminimizer_free (s);
    gsl_vector_free (x);

	if(!fitMinMaxConfigurations())
		return false;

//	cout <<"type="<<getModelName()<<endl;
//    for(size_t i=0;i<model.params.size();i++) {
//    	if(model.params[i].type != ParamMsg::PARAM) continue;
//    	cout <<" param "<<model.params[i].name<<"="<<model.params[i].value<<endl;
//    }

    return true;
}


void GenericModel::updateParameters(std::vector<double> delta) {
}

bool GenericModel::normalizeParameters() {
	return true;
}

}
