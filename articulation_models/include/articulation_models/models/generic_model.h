/*
 * generic_model.h
 *
 *  Created on: Oct 19, 2009
 *      Author: sturm
 */

#ifndef GENERIC_MODEL_H_
#define GENERIC_MODEL_H_

#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"
#include "../utils.h"

namespace articulation_models {


class GenericModel {
public:
	// global params
	double sigma_position;
	double sigma_orientation;
	double supress_similar;
	double outlier_ratio;
	double sac_iterations;
	double optimizer_iterations;

	// cached variables
	double complexity;
	double avg_error_position;
	double avg_error_orientation;
	double loglikelihood;
	double bic;
	double prior_outlier_ratio;
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd hessian;

	double last_error_jacobian;

	double evaluated;

	int channelOutlier;
	int channelLogLikelihood;
	int channelInlierLogLikelihood;
	std::vector<int> channelConfiguration;

	articulation_msgs::ModelMsg model;
public:
	GenericModel();
	virtual void setModel(const articulation_msgs::ModelMsg& model);
	virtual articulation_msgs::ModelMsg getModel();
	virtual void setTrack(const articulation_msgs::TrackMsg& track);
	virtual articulation_msgs::TrackMsg getTrack();
	void setId(int id);
	int getId();
	// -- model information
	virtual std::string getModelName();
	virtual size_t getDOFs();
	virtual size_t getSamples();
	// -- params
	virtual void readParamsFromModel();
	virtual void writeParamsToModel();
	virtual void prepareChannels();
	//
	bool hasParam(std::string name);
	double getParam(std::string name);
	void getParam(std::string name,double& data);
	void getParam(std::string name,tf::Vector3 &vec);
	void getParam(std::string name,tf::Quaternion &quat);
	void getParam(std::string name,tf::Transform &t);
	void getParam(std::string name,Eigen::VectorXd &vec);
	void getParam(std::string name,Eigen::MatrixXd &mat);
	void setParam(std::string name,double value,int type);
	void setParam(std::string name,const tf::Vector3 &vec,int type);
	void setParam(std::string name,const tf::Quaternion &quat,int type);
	void setParam(std::string name,const tf::Transform &t,int type);
	void setParam(std::string name,const Eigen::VectorXd &vec,int type);
	void setParam(std::string name,const Eigen::MatrixXd &mat,int type);
	// -- track data
	virtual int openChannel(std::string name,bool autocreate=true);
	// -- train model
	virtual bool fitModel();
	virtual bool fitMinMaxConfigurations();
	// -- evaluate model
	virtual bool evaluateModel();
	virtual double evalLatestJacobian();
	virtual double getPositionError();
	virtual double getOrientationError();
	virtual double getBIC();
	// -- cartesian space
	virtual geometry_msgs::Pose predictPose(V_Configuration q);
	virtual M_CartesianJacobian predictJacobian(V_Configuration q,double delta = 1e-6);
	virtual M_CartesianJacobian predictHessian(V_Configuration q,double delta = 1e-6);
	// -- configuration space convenience functions
	virtual void setConfiguration(size_t index,V_Configuration q);
	virtual void setJacobian(size_t index,M_CartesianJacobian J);
	virtual V_Configuration getConfiguration(size_t index);
	// -- configuration space
	virtual V_Configuration predictConfiguration(geometry_msgs::Pose pose);
	virtual V_Configuration getMinConfigurationObserved();
	virtual V_Configuration getMaxConfigurationObserved();
	// -- projections of track data
	virtual void projectPoseToConfiguration();
	virtual void projectConfigurationToPose();
	virtual void projectConfigurationToJacobian();
	virtual void sampleConfigurationSpace(double resolution);
	virtual void keepLatestPoseOnly();
	// -- likelihood
	virtual double getInlierLogLikelihood( size_t index );
	virtual double getOutlierLogLikelihood();
	virtual double getLogLikelihoodForPoseIndex(size_t index);
	virtual double getLogLikelihood(bool estimate_outlier_ratio);
	virtual bool guessParameters();
	virtual bool sampleConsensus();
	virtual bool optimizeParameters();
	virtual bool normalizeParameters();
	std::vector<articulation_msgs::ParamMsg> params_initial;
	virtual void updateParameters(std::vector<double> delta);
};

typedef boost::shared_ptr<GenericModel > GenericModelPtr;
typedef boost::shared_ptr<GenericModel const> GenericModelConstPtr;

}

#endif /* GENERIC_MODEL_H_ */
