/*
 * structs.cpp
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#include "structs.h"
#include "hogman_wrapper.h"
#include "boost/tuple/tuple.hpp"

using namespace std;
using namespace articulation_models;
using namespace geometry_msgs;
using namespace articulation_msgs;
using namespace std_msgs;
using namespace sensor_msgs;
using namespace articulation_structure;
using namespace Eigen;

KinematicParams::KinematicParams() {
	sigma_position = 0.005;
	sigma_orientation = 0.001;
	eval_every = 1;
	eval_every_power = 1.0;
	supress_similar = false;
	reuse_model = true;
	restricted_graphs = "";
	restrict_graphs = false;
	reduce_dofs = false;
	search_all_models = false;
	full_eval="tree";
}

void KinematicParams::LoadParams(ros::NodeHandle &nh_local,bool show) {
	string filter_models("rigid rotational prismatic");
	nh_local.getParam("filter_models", filter_models);
	nh_local.getParam("sigma_position", sigma_position);
	nh_local.getParam("sigma_orientation", sigma_orientation);
	nh_local.getParam("sigmax_position", sigmax_position);
	nh_local.getParam("sigmax_orientation", sigmax_orientation);
	nh_local.getParam("full_eval", full_eval);
	nh_local.getParam("eval_every", eval_every);
	nh_local.getParam("eval_every_power", eval_every_power);
	nh_local.getParam("restricted_graphs", restricted_graphs);
	restricted_graphs = " "+restricted_graphs+" ";
	nh_local.getParam("restrict_graphs", restrict_graphs);

	nh_local.getParam("reuse_model", reuse_model);
	nh_local.getParam("supress_similar", supress_similar);
	nh_local.getParam("reduce_dofs", reduce_dofs);
	nh_local.getParam("search_all_models", search_all_models);

	factory.setFilter(filter_models);

}

void KinematicData::addPose(PoseStamped pose1, PoseStamped pose2, size_t id1, size_t id2,KinematicParams &params) {
	TrackMsgPtr& track = trajectories[id1][id2];
	if (!track) {
		track = boost::make_shared<TrackMsg>();
		ChannelFloat32 ch;
		ch.name = "stamp";
		track->channels.push_back(ch);
	}

	tf::Transform transf1 = poseToTransform(pose1.pose);
	tf::Transform transf2 = poseToTransform(pose2.pose);
	tf::Transform diff = transf1.inverseTimes(transf2);

	for(size_t i=0;i<trajectories[id1][id2]->pose.size();i++) {
		tf::Transform diff_to_known;
		Pose p;
		p = trajectories[id1][id2]->pose[i];
		tf::Transform t;
		t = poseToTransform( p );
		diff_to_known = t.inverseTimes( diff );
		double diff_position = diff_to_known.getOrigin().length();
		double diff_orientation = fabs(diff_to_known.getRotation().getAngle());
		if(params.supress_similar && diff_position < params.sigma_position/2 && diff_orientation < params.sigma_orientation/2) {
//			cout <<"skipping relative pose information, too close to known sample"<<endl;
			return;
		}
	}

	poseIndex[id1][id2][ pose1.header.stamp.toSec() ] = track->pose.size();

	track->pose.push_back(transformToPose( diff));
	track->channels[0].values.push_back( pose1.header.stamp.toSec() );
	track->header = pose1.header;

//	updateModel(id1, id2);
}

void KinematicData::updateModel(size_t id1, size_t id2,KinematicParams &params) {
	TrackMsgPtr& track = trajectories[id1][id2];
	GenericModelPtr& model = models[id1][id2];

	articulation_msgs::ModelMsg model_track;
	model_track.track = *track;
	model_track.header = track->header;
	setParamIfNotDefined(model_track.params, "sigma_position",
			params.sigma_position, ParamMsg::PRIOR);
	setParamIfNotDefined(model_track.params, "sigma_orientation",
			params.sigma_orientation, ParamMsg::PRIOR);

	GenericModelVector models_new = params.factory.createModels(model_track);
    map<double,GenericModelPtr> models_sorted;
    models_all[id1][id2].clear();
	for (size_t i = 0; i < models_new.size(); i++) {
		if (!models_new[i]->fitModel()) continue;
		if (!models_new[i]->fitMinMaxConfigurations()) continue;
		if (!models_new[i]->evaluateModel()) continue;
		if(isnan( models_new[i]->getBIC() )) continue;

		if(params.reuse_model && model && model->getModelName()==models_new[i]->getModelName()) {
			model->setTrack(*track);
			model->optimizeParameters();
			model->evaluateModel();
			if(!isnan( model->getBIC() ) &&
					(models_new[i]->getBIC() > model->getBIC())) {
				models_new[i] = model;
			}
		}

		models_sorted[models_new[i]->getBIC()] = models_new[i];
		models_all[id1][id2].push_back( models_new[i] );
	}

	if(models_sorted.size()==0) {
		ROS_INFO_STREAM("no models!! model["<<id1<<","<<id2<<"]: poses="<<track->pose.size());
		model.reset();
		return;
	}
	map<double,GenericModelPtr>::iterator it = models_sorted.begin();
	model = it->second;

//	model->evaluateModel();
//	while(it!=models_sorted.end()) {
//		ROS_INFO_STREAM(
//			"model["<<id1<<","<<id2<<"]: poses="<<it->second->getSamples()<<
//			", type='"<<it->second->getModelName()<<
//			", BIC="<<it->first<<", logDL="<<it->second->loglikelihood<<", k="<<it->second->complexity<<
//			" pos="<<it->second->getParam("avg_error_position")<<
//			" orient="<<it->second->getParam("avg_error_orientation")
//		);
//		it++;
//	}
}

std::vector<double> KinematicData::intersectionOfStamps() {
	// timestamps where all markers were detected
	std::vector<double> intersection;

	size_t num_markers = markerStamped.size();
	for(map<double, map<int, PoseStampedIdMsgConstPtr> >::iterator it = stampedMarker.begin();
			it != stampedMarker.end();
			it++) {
		if(it->second.size() == num_markers) {
			intersection.push_back( it->first );
		}

//		cout <<"timestamp "<<(it->first - stampedMarker.begin()->first )<<" markers "<<it->second.size()<<" total "<<num_markers<<" ";
//		for(map<int, PoseStampedIdMsgConstPtr>::iterator j = it->second.begin();j!=it->second.end();j++) {
//			cout << " "<<j->first;
//		}
//		cout << endl;
	}

//	ROS_INFO_STREAM("full visibility rate: "<< intersection.size() / (double) stampedMarker.size() );
	return(intersection);
}

PoseMap KinematicData::getObservation(double timestamp) {
	PoseMap observedMarkers;
	for(std::map<int, articulation_structure::PoseStampedIdMsgConstPtr>::iterator it = stampedMarker[timestamp].begin();
			it != stampedMarker[timestamp].end();it++) {
		observedMarkers[it->first] = it->second->pose.pose;
	}
	return observedMarkers;
}

KinematicGraph::KinematicGraph() {
	BIC = DBL_MAX;
	DOF = -1;
	avg_pos_err = -1;
	avg_orient_err = -1;
	loglh = DBL_MAX;
}

KinematicGraph::KinematicGraph(const KinematicGraph &other):
	KinematicGraphType( other ), BIC(other.BIC),DOF(other.DOF),avg_pos_err(other.avg_pos_err),avg_orient_err(other.avg_orient_err),loglh(other.loglh)
{
}

KinematicGraph::KinematicGraph(const KinematicGraph &other,bool deepcopy):
			KinematicGraphType( other ), BIC(other.BIC),DOF(other.DOF),avg_pos_err(other.avg_pos_err),avg_orient_err(other.avg_orient_err),loglh(other.loglh)
{
	if(deepcopy) {
		CopyFrom(other);
	}
}

void KinematicGraph::CopyFrom(const KinematicGraph &other) {
	*this = other;
	static MultiModelFactory factory;
	for(KinematicGraph::iterator it=begin();it!=end();it++) {
		it->second = factory.restoreModel(it->second->getModel());
	}
}


size_t KinematicGraph::getNumberOfParameters() {
	size_t k = 0;
	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		k += (size_t) it->second->complexity;
	}
	return k;
}



int KinematicGraph::getNominalDOFs() {
	int DOFs = 0;
	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		DOFs += it->second->getDOFs();
	}
	return(DOFs);
}

PoseMap KinematicGraph::getPrediction(double timestamp, PoseMap &observedMarkers, PoseMap &predictedMarkersEmpty, KinematicParams &params,KinematicData &data) {
	return hogman_solver(*this, data.poseIndex, observedMarkers, predictedMarkersEmpty, params.sigma_position,params.sigma_orientation, timestamp);
}

string KinematicGraph::getTreeName(bool show_name,bool show_samples,bool show_dof) {
	stringstream s;
	if(show_dof) {
		s << DOF<<"-DOF ";
	}
	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		if(it != begin()) s<<",";
		s << "(";
		if(show_name) {
			s <<it->second->getModelName().substr(0,2)<<":";
		}
		s <<it->first.first<<","<<it->first.second;
		if(show_samples)
			s << "/"<<it->second->getSamples();
		s<<")";
	}
	return s.str();
}

void KinematicGraph::evaluateSingleStamp(double timestamp,
				double &avgPositionError,
				double &avgOrientationError,
				double &loglikelihood,KinematicParams &params,KinematicData &data) {

	if (size()==0) return;
	// compare observation versus prediction
	PoseMap observationMarkers = data.getObservation(timestamp);
	PoseMap predictedMarkers = getPrediction(timestamp,observationMarkers,observationMarkers,params,data);

	// store predicted poses
	for(PoseMap::iterator i=predictedMarkers.begin();i!=predictedMarkers.end();i++) {
		PoseStampedIdMsg p = *data.stampedMarker[timestamp][i->first];
		p.pose.pose = i->second;
		PoseStampedIdMsgConstPtr pptr = boost::make_shared<PoseStampedIdMsg>(p);
		data.stampedMarkerProjected[timestamp][i->first] = pptr;
	}

	// output data loglikelihood
	double sum_loglh = 0;
	double sum_pos2_err = 0;
	double sum_orient2_err = 0;

	double sigma2_position = SQR(params.sigmax_position);
	double sigma2_orientation = SQR(params.sigmax_orientation);

	for(PoseMap::iterator it = observationMarkers.begin();
			it != observationMarkers.end(); it++ ) {
		tf::Transform p1 = poseToTransform(observationMarkers[it->first]);
		tf::Transform p2 = poseToTransform(predictedMarkers[it->first]);

		tf::Transform diff = p1.inverseTimes(p2);
		double err_position2 = diff.getOrigin().length2();
		if(isnan(err_position2 )) err_position2 =0;
		double err_orientation2 = SQR(diff.getRotation().getAngle());
		if(isnan(err_orientation2 )) err_orientation2 =0;

		sum_pos2_err += err_position2;
		sum_orient2_err += err_orientation2;

		double loglikelihood =
				- log(2*M_PI * params.sigma_position*params.sigma_orientation)
				- 0.5*(
						( err_position2 / (sigma2_position) ) +
						( err_orientation2 / (sigma2_orientation) ) );	// 2-dim multivariate normal distribution
		sum_loglh += loglikelihood;
	}

	size_t n = observationMarkers.size();
	avgPositionError = sqrt(sum_pos2_err/n);
	avgOrientationError = sqrt(sum_orient2_err/n);
	loglikelihood = sum_loglh;
}

void KinematicGraph::evaluate(
		std::vector<double> stamps,
		KinematicParams &params,KinematicData &data) {
	// first, copy our data
	static MultiModelFactory factory;
	for(KinematicGraph::iterator it=begin();it!=end();it++) {
		it->second = factory.restoreModel(it->second->getModel());
	}

	BIC = DBL_MAX;
	avg_pos_err = DBL_MAX;
	avg_orient_err = DBL_MAX;
	loglh = DBL_MIN;

	if (size()==0) return;

	if(DOF != getNominalDOFs())
		projectConfigurationSpace(DOF,stamps,params,data);

//	cout << getTreeName(true,true,true);

	double sum_pos2_err = 0;
	double sum_orient2_err = 0;
	double sum_loglh = 0;

	double pos_err,orient_err;

	data.stampedMarkerProjected.clear();
	for(size_t i=0;i<stamps.size();i++) {
		evaluateSingleStamp(stamps[i],pos_err, orient_err, loglh,params,data );
		sum_loglh += loglh;
		sum_pos2_err += SQR(pos_err);
		sum_orient2_err += SQR(orient_err);
	}

	avg_pos_err = sqrt(sum_pos2_err/stamps.size());
	avg_orient_err = sqrt(orient_err/stamps.size());
	loglh = sum_loglh;

	k = getNumberOfParameters();
	int k_pca = 0;
	double loglh_pca;
	if(DOF != getNominalDOFs()) {
		k_pca = getNominalDOFs() * DOF; // extra parameters needed to store projection matrix
		loglh_pca = stamps.size() * (getNominalDOFs() - DOF) * log(stamps.size());	// correct likelihood for reduced dimensionality of configuration space
	}
	BIC = getBIC( loglh + loglh_pca, k + k_pca,stamps.size() );	// BIC with penalty for size of configuratino space
//	cout << ": loglh="<<loglh<<" pos="<<avg_pos_err<<" orient="<<avg_orient_err<<" k="<<k<<" BIC="<<BIC << endl;
	return;
}

void KinematicGraph::projectConfigurationSpace(int reducedDOFs,std::vector<double> stamps, KinematicParams &params,KinematicData &data) {
	if(reducedDOFs == 0) return;
	if(stamps.size() == 0) return;

	map< GenericModelPtr, int> DOFoffsets;
	int DOFs = 0;
	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		DOFoffsets[it->second] = DOFs;
		DOFs += it->second->getDOFs();
	}
	if(DOFs==0) return;

	MatrixXf X_trans(stamps.size(), DOFs);

	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		GenericModelPtr model = it->second;
		int DOFoffset = DOFoffsets[model];

		if(model->getDOFs()==0) continue;

		for(size_t timestamp_idx = 0;timestamp_idx < stamps.size();timestamp_idx++) {
			int idx = data.poseIndex[ it->first.first ][it->first.second][ stamps[timestamp_idx]];

			VectorXd c = model->getConfiguration(idx);
			for(size_t d=0;d< model->getDOFs();d++) {
				X_trans( timestamp_idx,DOFoffset + d ) = c(d);
			}
		}
	}

	VectorXf X_mean = VectorXf::Zero(DOFs,1);
	for( size_t i=0;i< stamps.size();i++) {
		for( int j=0;j< DOFs;j++) {
			X_mean(j) += X_trans(i,j);
		}
	}
	X_mean /= stamps.size();


	// check for nans or large numbers (crash eigen while svd'ing)
	for( int j=0;j< DOFs;j++) {
		if(isnan((double)X_mean(j))) {
			cout << X_trans << endl<<endl;
			cout << X_mean << endl;
			cout <<"reducing DOFs failed, NaN value"<<endl;
			return;
		}
		if(fabs((double)X_mean(j))>100) {
			cout << X_trans << endl<<endl;
			cout << X_mean << endl;
			cout <<"reducing DOFs failed, large value"<<endl;
			return;
		}
	}

	MatrixXf X_center(stamps.size(), DOFs);
	for( size_t i=0;i< stamps.size();i++) {
		for( int j=0;j< DOFs;j++) {
			X_center(i,j) = X_trans(i,j) - X_mean(j);
		}
	}

	if(stamps.size() < (size_t)DOFs) return;

	JacobiSVD<MatrixXf> svdOfA(X_center);

	const Eigen::MatrixXf U = svdOfA.matrixU();
	const Eigen::MatrixXf V = svdOfA.matrixV();
	const Eigen::VectorXf S = svdOfA.singularValues();

	//DiagonalMatrix<Eigen::VectorXf> S2(S);
	Eigen::MatrixXf V_projection = V.block(0,0,DOFs,reducedDOFs);
	MatrixXf X_reduced(stamps.size(),reducedDOFs);

	for( size_t i=0;i< stamps.size();i++) {
		for( int k=0;k< reducedDOFs;k++) {
			X_reduced(i,k) = 0.0;
			for( int j=0;j< DOFs;j++) {
				X_reduced(i,k) += V(j,k) * X_center(i,j);
			}
		}
	}

	MatrixXf X_projected(stamps.size(),DOFs);
	for( size_t i=0;i< stamps.size();i++) {
		for( int j=0;j< DOFs;j++) {
			X_projected(i,j) = X_mean(j);
			for( int k=0;k< reducedDOFs;k++) {
				X_projected(i,j) += V(j,k) * X_reduced(i,k);
			}
		}
	}

	for(KinematicGraph::iterator it=begin(); it != end();it++) {
		GenericModelPtr model = it->second;
		if(model->getDOFs()==0) continue;
		int DOFoffset = DOFoffsets[model];

		for(size_t timestamp_idx = 0;timestamp_idx < stamps.size();timestamp_idx++) {
			int idx = data.poseIndex[ it->first.first ][it->first.second][ stamps[timestamp_idx]];

			VectorXd c(model->getDOFs());
			for(size_t d=0;d< model->getDOFs();d++) {
				c(d) =X_projected( timestamp_idx,DOFoffset + d );
			}

			model->setConfiguration(idx,c);
			model->model.track.pose_projected[idx] = model->predictPose( c );
		}
	}
	DOF = reducedDOFs;
}

#include "boost/tuple/tuple.hpp"

int KinematicGraph::distanceTo(KinematicGraph &other) {
	int distance = 0;
	distance +=fabs((getNominalDOFs()-DOF) - (other.getNominalDOFs()-other.DOF));

	// first only consider edge differences
	set< pair<int,int> > my_edges,other_edges,diff_edges;
	for(KinematicGraph::iterator it=begin();it!=end();it++) my_edges.insert( pair<int,int>(it->first.first,it->first.second) );
	for(KinematicGraph::iterator it = other.begin();it!=other.end();it++) other_edges.insert(pair<int,int>(it->first.first,it->first.second) );
	set_symmetric_difference(my_edges.begin(),my_edges.end(),other_edges.begin(),other_edges.end(),insert_iterator<set< pair<int,int> > >(diff_edges,diff_edges.begin()) );
	distance += diff_edges.size();	// new/deleted edges

	// now only consider common edges and count model changes
	set< pair<pair<int,int>,string> > my_models,other_models,intersec_models,diff_models;
	for(KinematicGraph::iterator it=begin();it!=end();it++)
		if(diff_edges.find(it->first)==diff_edges.end())
			my_models.insert( pair<pair<int,int>,string>(pair<int,int>(it->first.first,it->first.second),it->second->getModelName()) );
	for(KinematicGraph::iterator it = other.begin();it!=other.end();it++)
		if(diff_edges.find(it->first)==diff_edges.end())
			other_models.insert(pair<pair<int,int>,string>(pair<int,int>(it->first.first,it->first.second),it->second->getModelName()) );
	set_symmetric_difference(my_models.begin(),my_models.end(),other_models.begin(),other_models.end(),insert_iterator<set< pair<pair<int,int>,string> > >(diff_models,diff_models.begin()) );
	distance += diff_models.size()/2;	// changed models

	return distance;
}
