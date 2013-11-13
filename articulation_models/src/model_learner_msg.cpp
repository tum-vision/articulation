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

#include "articulation_models/models/factory.h"
#include "articulation_models/utils.h"
#include <boost/foreach.hpp>

#define DEBUG false

using namespace std;
using namespace articulation_models;
using namespace articulation_msgs;

ros::Publisher model_pub;

MultiModelFactory factory;

GenericModelVector models_valid;

map<string,	ros::Time> startingTime;
map<string, vector<double> > measurements;

ros::NodeHandle *nh;
double sigma_position = 0.01;
double sigma_orientation = 4*M_PI;


void TIC(string name){
	startingTime[name] = ros::Time::now();
}

void TOC(string name) {
	measurements[name].push_back( (ros::Time::now() - startingTime[name]).toSec() );
}

void ADD_DATA(string name,double data) {
	measurements[name].push_back( data );
}


#define SQR(a) ((a)*(a))
void EVAL() {
	map<string, vector<double> >::iterator it;
	for(it = measurements.begin(); it!=measurements.end(); it++) {
		size_t n = it->second.size();
		double sum = 0;
		for(size_t i=0;i<n;i++) {
			sum += it->second[i];
		}
		double mean = sum /n;
		double vsum = 0;
		for(size_t i=0;i<n;i++) {
			vsum += SQR(it->second[i] - mean);
		}
		double var = vsum / n;
		cout << it->first << " " << mean << " "<<sqrt(var)<< " ("<<n<<" obs)"<< endl;
	}
}

void trackCallback(const TrackMsgConstPtr& track)
{
  ROS_INFO("Received track id [%d]", track->id);

  articulation_msgs::ModelMsg model_track;
  model_track.track = *track;
  setParamIfNotDefined(model_track.params, "sigma_position",
			sigma_position, ParamMsg::PRIOR);
  setParamIfNotDefined(model_track.params, "sigma_orientation",
			sigma_orientation, ParamMsg::PRIOR);

  TIC("createModels");

  GenericModelVector models_new = factory.createModels( model_track );
  TOC("createModels");

  GenericModelVector models_old = models_valid;

  models_valid.clear();
  models_old.clear();

  // update old models, then add valid
  for(size_t i=0;i<models_old.size();i++) {
	  models_old[i]->setTrack(*track);
	  models_old[i]->projectPoseToConfiguration();
	  if( !models_old[i]->fitMinMaxConfigurations() ) continue;
	  if( !models_old[i]->evaluateModel() ) continue;

	  models_valid.push_back( models_old[i] );
  }

  // fit new models, then add valid
  TIC("per_track");
  for(size_t i=0;i<models_new.size();i++) {
	  TIC("fitModel" + models_new[i]->getModelName());
	  if( !models_new[i]->fitModel() ) {
		  if(DEBUG) cout <<"fitting of "<<models_new[i]->getModelName()<<" failed"<<endl;
		  continue;
	  }
	  TOC("fitModel" + models_new[i]->getModelName());
	  TIC("projectPoseToConfiguration" + models_new[i]->getModelName());
	  models_new[i]->projectPoseToConfiguration();
	  TOC("projectPoseToConfiguration" + models_new[i]->getModelName());
	  TIC("fitMinMaxConfigurations" + models_new[i]->getModelName());
	  if( !models_new[i]->fitMinMaxConfigurations() ) {
		  if(DEBUG) cout <<"fitting of min/max conf of "<<models_new[i]->getModelName()<<" failed"<<endl;
		  continue;
	  }
	  TOC("fitMinMaxConfigurations" + models_new[i]->getModelName());

	  TIC("evaluateModel" + models_new[i]->getModelName());
	  if( !models_new[i]->evaluateModel() ) {
		  if(DEBUG) cout <<"evaluation of "<<models_new[i]->getModelName()<<" failed"<<endl;
		  continue;
	  }
	  TOC("evaluateModel" + models_new[i]->getModelName());

	  models_valid.push_back( models_new[i] );

  }
  TOC("per_track");


  map<double,GenericModelPtr> models_sorted;
  for(size_t i=0;i<models_valid.size();i++) {
	  if(isnan( models_valid[i]->getBIC() )) {
		  if(DEBUG) cout <<"BIC eval of "<<models_new[i]->getModelName()<<" is nan, skipping"<<endl;
		  continue;
	  }
	  models_sorted[models_valid[i]->getBIC()] = models_valid[i];
  }

  if(models_sorted.size()==0) {
	  cout << "no valid models found"<<endl;
	  return;
  }

  for(map<double,GenericModelPtr>::iterator it=models_sorted.begin();it!=models_sorted.end();it++) {
  cout << it->second->getModelName()<<
	  " pos_err=" << it->second->getPositionError()<<
	  " rot_err=" << it->second->getOrientationError()<<
	  " bic=" << it->second->getBIC()<<
	  " k=" << it->second->getParam("complexity") <<
	  " n=" << it->second->getTrack().pose.size() <<
	  endl;
  }
//  }
  map<double,GenericModelPtr>::iterator it = models_sorted.begin();
  models_valid.clear();
  models_valid.push_back(it->second);

//  it->second->projectPoseToConfiguration();
//  it->second->fitMinMaxConfigurations();
  if(it->second->getModelName()=="rotational") {
	  it->second->sampleConfigurationSpace( 0.05 );
  } else {
	  it->second->sampleConfigurationSpace( 0.01 );
  }

  ModelMsg msg = it->second->getModel();
  msg.id = msg.track.id;
  model_pub.publish( msg );

//  EVAL();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_learner");
  ros::NodeHandle n;

  std::string filter_models("rigid rotational prismatic");
  ros::NodeHandle("~").getParam("filter_models", filter_models);
  ros::NodeHandle("~").getParam("sigma_position",sigma_position);
  ros::NodeHandle("~").getParam("sigma_orientation",sigma_orientation);
  factory.setFilter(filter_models);

  cout <<"(param) sigma_position=" << sigma_position << endl;
  cout <<"(param) sigma_orientation=" << sigma_orientation << endl;

  model_pub = n.advertise<ModelMsg>("model", 1);

  ros::Subscriber track_sub = n.subscribe("track", 1, trackCallback);
  ros::spin();
}
