/*
 * model_learner.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#include <ros/ros.h>

#include "articulation_msgs/TrackModelSrv.h"

#include "articulation_models/models/factory.h"
#include "articulation_models/utils.h"

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

MultiModelFactory factory;

ros::NodeHandle *nh;

bool selectModel(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("Service call, id=%d, poses=%d",(int) request.model.track.id,(int)request.model.track.pose.size());

	response.model = request.model;

	GenericModelVector models_new = factory.createModels(request.model);
	GenericModelVector models_filtered;

	double sigma_position = 0.00;
	double sigma_orientation = 0.00;
	nh->param("sigma_position", sigma_position, 0.01);
	nh->param("sigma_orientation", sigma_orientation, 5.00);

	// filter models, and setting parameters
	for (size_t i = 0; i < models_new.size(); i++) {
		models_new[i]->setParam("sigma_position", sigma_position,
				ParamMsg::PRIOR);
		models_new[i]->setParam("sigma_orientation", sigma_orientation,
				ParamMsg::PRIOR);

		for (size_t j = 0; j < request.model.params.size(); j++) {
			models_new[i]->setParam(request.model.params[j].name,
					request.model.params[j].value, request.model.params[j].type);
		}

		if (models_new[i]->getParam("disable_all") && !models_new[i]->getParam(
				"enable_" + models_new[i]->getModelName()))
			continue;

		models_new[i]->readParamsFromModel();
		//	  cout << models_new[i]->getParam("sigma_position") << endl;
		models_filtered.push_back(models_new[i]);

	}

	GenericModelVector models_valid;

	ROS_INFO_STREAM( "number of created models: "<<models_new.size() );

	// fit new models, then add valid
	for (size_t i = 0; i < models_filtered.size(); i++) {
		if (!models_filtered[i]->fitModel()) {
			ROS_INFO_STREAM( models_filtered[i]->getModelName() << ": fitting failed" );
			continue;
		}
		models_filtered[i]->projectPoseToConfiguration();
		if (!models_filtered[i]->fitMinMaxConfigurations()) {
			ROS_INFO_STREAM( models_filtered[i]->getModelName() << ": fit min max failed" );
			continue;
		}
		if (!models_filtered[i]->evaluateModel()) {
			ROS_INFO_STREAM( models_filtered[i]->getModelName() << ": eval failed" );
			continue;
		}

		//	  ROS_INFO_STREAM( models_filtered[i]->getModelName() << ": valid, adding" );

		models_valid.push_back(models_filtered[i]);
	}

	map<double, GenericModelPtr> models_sorted;
	for (size_t i = 0; i < models_valid.size(); i++) {
		if (isnan(models_valid[i]->getBIC()))
			continue;
		models_sorted[models_valid[i]->getBIC()] = models_valid[i];
	}

	map<double, GenericModelPtr>::iterator it = models_sorted.begin();
	if (it == models_sorted.end())
		return (true);

	response.model = it->second->getModel();

	return (true);
}

bool fitModel(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("fit Service call, id=%d, poses=%d, name=%s", (int)request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());

	GenericModelPtr model = factory.restoreModel(request.model);
	if (!model) {
		ROS_WARN("could not restore model '%s'",request.model.name.c_str());
		return (true);
	}
	if (!model->fitModel()) {
		ROS_INFO_STREAM( model->getModelName() << ": fitModel() failed" );
		return (true);
	}
	model->projectPoseToConfiguration();
	if (!model->fitMinMaxConfigurations()) {
		ROS_INFO_STREAM( model->getModelName() << ": fit min max failed" );
		return (true);
	}
	if (!model->evaluateModel()) {
		ROS_INFO_STREAM( model->getModelName() << ": eval failed" );
		return (true);
	}

	response.model = model->getModel();
	return (true);
}

bool evalModel(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("eval Service call, id=%d, poses=%d, name=%s",(int) request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());

	GenericModelPtr model = factory.restoreModel(request.model);

	if (!model->evaluateModel()) {
		ROS_INFO_STREAM( model->getModelName() << ": eval failed" );
		return (true);
	}

	response.model = model->getModel();

	return (true);
}

bool resampleModel(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("resample Service call, id=%d, poses=%d, name=%s", (int)request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());

	GenericModelPtr model = factory.restoreModel(request.model);

	double density = 0.01;
	model->getParam("resample_density", density);
	model->sampleConfigurationSpace(density);
	response.model = model->getModel();

	return (true);
}

bool projPoseToConf(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("projPoseToConf Service call, id=%d, poses=%d, name=%s", (int)request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());

	GenericModelPtr model = factory.restoreModel(request.model);

	model->projectPoseToConfiguration();
	response.model = model->getModel();

	return (true);
}

bool projConfToPose(articulation_msgs::TrackModelSrv::Request &request,
		articulation_msgs::TrackModelSrv::Response &response) {
	ROS_INFO("projConfToPose Service call, id=%d, poses=%d, name=%s", (int)request.model.track.id,(int)request.model.track.pose.size(),request.model.name.c_str());

	GenericModelPtr model = factory.restoreModel(request.model);

	model->projectConfigurationToPose();
	response.model = model->getModel();

	return (true);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "track_model_server");

	nh = new ros::NodeHandle();

	string filter_models("rigid rotational prismatic");
	ros::NodeHandle("~").getParam("filter_models", filter_models);
	factory.setFilter(filter_models);
	factory.listModelFactories();

	ros::ServiceServer selectService = nh->advertiseService("model_select",
			selectModel);

	ros::ServiceServer fitService = nh->advertiseService("model_fit", fitModel);

	ros::ServiceServer evalService = nh->advertiseService("model_eval",
			evalModel);

	ros::ServiceServer resampleService = nh->advertiseService("model_resample",
			resampleModel);

	ros::ServiceServer projPoseToConfService = nh->advertiseService(
			"model_project_pose_to_configuration", projPoseToConf);

	ros::ServiceServer projConfToPoseService = nh->advertiseService(
			"model_project_configuration_to_pose", projConfToPose);

	ROS_INFO("Ready to select/fit/eval/resample models.");
	ros::spin();
}
