/*
 * model_learner.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#include <ros/ros.h>

#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"
#include "articulation_msgs/TrackModelSrv.h"
#include "articulation_msgs/AlignModelSrv.h"
#include "articulation_msgs/GetModelPriorSrv.h"
#include "articulation_msgs/SetModelPriorSrv.h"

#include "articulation_models/models/factory.h"
#include "articulation_models/utils.h"

#include "icp/icp_utils.h"

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

ros::Publisher model_pub;

MultiModelFactory factory;

std::map<int, GenericModelPtr> model_database;

double sigma_position = 0.01;
double sigma_orientation = 360 * M_PI / 180.0;

bool single_model = false;
bool do_align = false;

double sigma_align_position = 0.2;
double sigma_align_orientation = 20 * M_PI / 180.0;

#define SQR(a) ((a)*(a))

int next_id = 0;

double getSimpleLikelihood(GenericModelPtr model) {
	return model->getParam("loglikelihood") +
			(model->getParam("samples"))*model->getParam("dofs")*log(model->getParam("samples"));
}

bool model_select(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response, bool use_prior) {
	ROS_INFO("selecting model, id=%d, poses=%d", (int)request.model.track.id,(int)request.model.track.pose.size());

	map<double, GenericModelPtr> evaluated_models;

	// set some parameters
	request.model.id = -1; // cluster assigment not known yet
	setParamIfNotDefined(request.model.params, "sigma_position",
			sigma_position, ParamMsg::PRIOR);
	setParamIfNotDefined(request.model.params, "sigma_orientation",
			sigma_orientation, ParamMsg::PRIOR);

	double total_loglikelihood=0;
	double total_n=0;
	double total_k=0;
	for (std::map<int, GenericModelPtr>::iterator it = model_database.begin(); it
			!= model_database.end(); it++) {
		total_loglikelihood += getSimpleLikelihood(it->second) ;
		total_n += it->second->model.track.pose.size();
		total_k += it->second->getParam("complexity");
	}
//	cout << "total_loglikelihood ="<<total_loglikelihood <<endl;
//	cout << "total_k ="<<total_k <<endl;
//	cout << "total_n ="<<total_n <<endl;

	// possibility 1: fit a completely new model
	GenericModelVector candidate_models = factory.createModels(request.model);
	// fit candidates and sort
	for (size_t i = 0; i < candidate_models.size(); i++) {
		if (!candidate_models[i]->fitModel())
			continue;
		candidate_models[i]->projectPoseToConfiguration();
		if (!candidate_models[i]->fitMinMaxConfigurations())
			continue;
		if (!candidate_models[i]->evaluateModel())
			continue;
		if (isnan(candidate_models[i]->getBIC()))
			continue;

//		cout << "candidate_models->getParam(loglikelihood) ="<<candidate_models[i]->getParam("loglikelihood") <<endl;
//		cout << "candidate_models->getParam(complexity) ="<<candidate_models[i]->getParam("complexity") <<endl;
//		cout << "request.model.track.pose.size() ="<<request.model.track.pose.size() <<endl;
		double bic =
				-2*(total_loglikelihood +
						getSimpleLikelihood(candidate_models[i]))
				+( total_k + candidate_models[i]->getParam("complexity") )
				* log(total_n + request.model.track.pose.size() );
		evaluated_models[ bic ] = candidate_models[i];
//		cout << candidate_models[i]->getModelName()<< " candidate bic="<< bic <<endl;
	}

	if(single_model && use_prior) {
		if(model_database.size()>0) {
			cout << "single_model mode active!! clearing new candidates!!"<<endl;
			evaluated_models.clear();
		}
	}

	if(use_prior) {
		// possibility 2: combine trajectory with a stored model, then re-fit
		for (std::map<int, GenericModelPtr>::iterator it = model_database.begin(); it
				!= model_database.end(); it++) {
			GenericModelPtr stored_model = it->second;
			articulation_msgs::ModelMsg stored_model_msg = stored_model->getModel();

			if(do_align) {
				double k_align = 0;
				double lh_align = 0;
				if(request.model.track.pose.size()>5 && stored_model_msg.track.pose.size()>5) {
					ros::ServiceClient client = ros::NodeHandle().serviceClient<articulation_msgs::AlignModelSrv> ("icp_align");
					articulation_msgs::AlignModelSrv srv;
					srv.request.model = stored_model_msg;
					srv.request.data = request.model;
					if (client.call(srv)) {
						stored_model_msg = srv.response.model_aligned;
						k_align = 3;
						lh_align =
										( SQR(srv.response.dist_trans) / SQR(sigma_align_position) ) +
										( SQR(srv.response.dist_rot) / SQR(sigma_align_orientation) );
					}

				}
			}

				//	  if(request.model.track.pose.size()>10 && stored_model_msg.track.pose.size()>10) {
				//		  icp::IcpAlign alignment(stored_model_msg.track, request.model.track);
				//		  alignment.TransformModel(stored_model_msg.track);
				//	  }

				// now join tracks
				articulation_msgs::ModelMsg merged_model_msg = stored_model_msg;
			merged_model_msg.track.id = request.model.track.id;
			merged_model_msg.track.pose.insert(merged_model_msg.track.pose.end(),
					request.model.track.pose.begin(),
					request.model.track.pose.end());
			for (size_t i = 0; i < merged_model_msg.track.pose_flags.size(); i++) {
				merged_model_msg.track.pose_flags[i] &= ~TrackMsg::POSE_VISIBLE;
			}
			merged_model_msg.track.pose_flags.back()
					|= TrackMsg::POSE_END_OF_SEGMENT;
			merged_model_msg.track.pose_flags.insert(
					merged_model_msg.track.pose_flags.end(),
					request.model.track.pose_flags.begin(),
					request.model.track.pose_flags.end());

			//	  cout << " merged model, n="<<merged_model_msg.track.pose.size()<<endl;
			GenericModelPtr merged_model = factory.restoreModel(merged_model_msg);
			if (!merged_model->fitModel())
				continue;
			merged_model->projectPoseToConfiguration();
			if (!merged_model->fitMinMaxConfigurations())
				continue;
			if (!merged_model->evaluateModel())
				continue;
			if (isnan(merged_model->getBIC()))
				continue;

	//		cout << "stored_model->getParam(loglikelihood) ="<<stored_model->getParam("loglikelihood") <<endl;
	//		cout << "stored_model->getParam(complexity) ="<<stored_model->getParam("complexity") <<endl;
	//		cout << "request.model.track.pose.size() ="<<request.model.track.pose.size() <<endl;
	//		cout << "merged_model->getParam(loglikelihood) ="<<merged_model->getParam("loglikelihood") <<endl;
	//		cout << "merged_model->getParam(complexity) ="<<merged_model->getParam("complexity") <<endl;
			double bic =
					-2*(total_loglikelihood
							- getSimpleLikelihood(stored_model)
							+ getSimpleLikelihood(merged_model)
						)
					+ ( total_k - stored_model->getParam("complexity") + merged_model->getParam("complexity") )
					* log(total_n + request.model.track.pose.size() );
			evaluated_models[ bic ] = merged_model;
	//		cout << " relative bic="<< (bic) <<endl;
		}
	}

	if (evaluated_models.size() == 0) {
		cout << "no valid models found" << endl;
		return false;
	}

	// best model
	GenericModelPtr selected_model = evaluated_models.begin()->second;
	selected_model->bic = evaluated_models.begin()->first;

	// print some information on selected model
	cout << selected_model ->getModelName() << selected_model->getId()<<" "<< " pos_err="
			<< selected_model ->getPositionError() << " rot_err="
			<< selected_model ->getOrientationError() << " bic="
			<< selected_model ->getBIC() << " k=" << selected_model ->getParam(
			"complexity") << " n=" << selected_model ->getTrack().pose.size()
			<< endl << flush;

	selected_model->sampleConfigurationSpace(0.01);
	response.model = selected_model->getModel();
	return (true);
}

bool model_select(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	return model_select(request,response,true);
}

bool model_store(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("storing model, id=%d, poses=%d, name=%s", (int)request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());
	GenericModelPtr model = factory.restoreModel(request.model);

	if (model->getId() == -1) {
		model->setId(next_id++);
	}

	model_database[model->getId()] = model;
	response.model = model->getModel();

	return (true);
}

bool model_get_prior(articulation_msgs::GetModelPriorSrv::Request &request,
		articulation_msgs::GetModelPriorSrv::Response &response) {
	ROS_INFO("model_get_prior, returning n=%d models", (int)model_database.size());
	for (std::map<int, GenericModelPtr>::iterator it = model_database.begin(); it
			!= model_database.end(); it++) {
		GenericModelPtr stored_model = it->second;
		response.model.push_back(stored_model->getModel());

	}
	return (true);
}

bool model_set_prior(articulation_msgs::SetModelPriorSrv::Request &request,
		articulation_msgs::GetModelPriorSrv::Response &response) {
	ROS_INFO("model_set_prior, restoring n=%d models", (int)request.model.size());

	string filter_models("rigid rotational prismatic");
	ros::NodeHandle("~").getParam("filter_models", filter_models);
	factory.setFilter(filter_models);
	factory.listModelFactories();
	model_database.clear();
	next_id = 0;
	for (size_t i = 0; i < request.model.size(); i++) {
		GenericModelPtr model = factory.restoreModel(request.model[i]);
		model_database[model->getId()] = model;
		if(request.model[i].id>=next_id)
			next_id = request.model[i].id+1;
	}
	return (true);
}


bool model_select_eval(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("evaluation model selection, id=%d, poses=%d", (int)request.model.track.id,(int)request.model.track.pose.size());

	model_select(request,response,false);	// first, find the "right" model, no prior
	GenericModelPtr model = factory.restoreModel(response.model);
	if(!model) {
		ROS_INFO("sorry, no model");
		return true;
	}
	int ch = model->openChannel("avg_error_position_cutoff");
	int ch_time = model->openChannel("timing");

	articulation_msgs::TrackModelSrv::Request partial_request;
	articulation_msgs::TrackModelSrv::Response partial_response;
	for(size_t n=0;n<request.model.track.pose.size();n++) {
		partial_request = request;
		partial_request.model.track.pose.erase(
				partial_request.model.track.pose.begin() + n,
				partial_request.model.track.pose.end() );

		ros::Time t_start = ros::Time::now();
		model_select(partial_request,partial_response);
		model->model.track.channels[ ch_time ].values[n] = (ros::Time::now() - t_start ).toSec();

		GenericModelPtr partial_model = factory.restoreModel(partial_response.model);
		if(!partial_model)
			continue;
		partial_model->fitModel();	// gp need to be constructed first (=re-fitting on partial data).. :(
		partial_model->model.track = request.model.track;


		partial_model->evaluateModel();
		double avg_error_position = partial_model->getParam("avg_error_position");
		model->model.track.channels[ ch ].values[n] = avg_error_position;
		cout << n <<"/"<< request.model.track.pose.size()<<": avg_error_position="<<avg_error_position<<endl;
	}
	response.model = model->getModel();
	return true;
}

int main(int argc, char** argv) {
	// init ROS node
	ros::init(argc, argv, "model_learner_prior");
	ros::NodeHandle n;

	// read ROS params
	std::string filter_models("rigid rotational prismatic");
	ros::NodeHandle("~").getParam("filter_models", filter_models);
	factory.setFilter(filter_models);
	factory.listModelFactories();

	ros::NodeHandle("~").getParam("sigma_position", sigma_position);
	ros::NodeHandle("~").getParam("sigma_orientation", sigma_orientation);

	ros::NodeHandle("~").getParam("single_model", single_model);
	ros::NodeHandle("~").getParam("do_align", do_align);

	// advertise ROS services
	ros::ServiceServer model_select_service = n.advertiseService(
			"model_select", model_select);
	ros::ServiceServer model_store_service = n.advertiseService("model_store",
			model_store);

	ros::ServiceServer model_get_prior_service = n.advertiseService(
			"model_prior_get", model_get_prior);
	ros::ServiceServer model_set_prior_service = n.advertiseService(
			"model_prior_set", model_set_prior);

	ros::ServiceServer model_select_eval_service = n.advertiseService(
			"model_select_eval", model_select_eval);

	ROS_INFO("model_learner_prior, service ready");
	ros::spin();
}
