/*
 * model_fitting_demo.cpp
 *
 *  Created on: Jun 8, 2010
 *      Author: sturm
 */

#include <ros/ros.h>

#include "articulation_models/models/factory.h"

#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

int main(int argc, char** argv) {
	ros::init(argc, argv, "model_fitting");
	ros::NodeHandle n;
	ros::Publisher model_pub = n.advertise<ModelMsg> ("model", 5);
	ros::Rate loop_rate(5);
	int count = 0;

	boost::normal_distribution<> nd(0.0, 0.01);
	boost::mt19937 rng;
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
			var_nor(rng, nd);

	MultiModelFactory factory;

	ModelMsg model_msg;
	model_msg.name = "rotational";
	ParamMsg sigma_param;
	sigma_param.name = "sigma_position";
	sigma_param.value = 0.02;
	sigma_param.type = ParamMsg::PRIOR;
	sigma_param.name = "sigma_orientation";
	sigma_param.value = M_PI*10;
	sigma_param.type = ParamMsg::PRIOR;
	model_msg.params.push_back(sigma_param);

	model_msg.track.header.stamp = ros::Time();
	model_msg.track.header.frame_id = "/";

	while (ros::ok()) {

		geometry_msgs::Pose pose;
		pose.position.x = cos(count / 30.0)+var_nor();
		pose.position.y = sin(count / 30.0)+var_nor();
		pose.position.z = var_nor();
		pose.orientation.x = 0;
		pose.orientation.y = 0;
		pose.orientation.z = 0;
		pose.orientation.w = 1;
		model_msg.track.pose.push_back(pose);

		if(model_msg.track.pose.size()<3) continue;

		cout <<"creating object"<<endl;
		GenericModelPtr model_instance = factory.restoreModel(model_msg);
		cout <<"fitting"<<endl;
		model_instance->fitModel();
		cout <<"evaluating"<<endl;
		model_instance->evaluateModel();
		cout <<"done"<<endl;

		cout << "model class = "<< model_instance->getModelName() << endl;
		cout << "	radius = "<<model_instance->getParam("rot_radius")<< endl;
		cout << "	center.x = "<<model_instance->getParam("rot_center.x")<< endl;
		cout << "	center.y = "<<model_instance->getParam("rot_center.y")<< endl;
		cout << "	center.z = "<<model_instance->getParam("rot_center.z")<< endl;
		cout << "	log LH = " << model_instance->getParam("loglikelihood")<< endl;

		ModelMsg fitted_model_msg = model_instance->getModel();
		model_pub.publish(fitted_model_msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
}
