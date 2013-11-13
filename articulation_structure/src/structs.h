/*
 * structs.h
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <ros/ros.h>

#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"

#include "articulation_models/models/factory.h"
#include "articulation_models/models/rotational_model.h"
#include "articulation_models/models/prismatic_model.h"

#include "articulation_structure/PoseStampedIdMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "utils.h"

#include "Eigen/Core"
#include <Eigen/SVD>
#include <sys/types.h>
#include <unistd.h>

typedef std::vector<std::pair< std::pair<int,int>,articulation_models::GenericModelPtr> > KinematicGraphType;
typedef std::map<int,geometry_msgs::Pose> PoseMap;
typedef std::map<int, std::map<int, std::map<double, int> > > PoseIndex;

class KinematicParams {
public:
	double sigma_position;
	double sigma_orientation;
	double sigmax_position;
	double sigmax_orientation;
	int eval_every;
	double eval_every_power;
	bool supress_similar;
	bool reuse_model;
	std::string restricted_graphs;
	bool restrict_graphs;
	bool reduce_dofs;
	bool search_all_models;
	std::string full_eval;
	articulation_models::MultiModelFactory factory;
	KinematicParams();
	void LoadParams(ros::NodeHandle &nh_local,bool show=true);
};

class KinematicData {
public:
	std::map<int, double > latestTimestamp;
	std::map<double, std::map<int, articulation_structure::PoseStampedIdMsgConstPtr> > stampedMarker;
	std::map<double, std::map<int, articulation_structure::PoseStampedIdMsgConstPtr> > stampedMarkerProjected;
	std::map<int, std::map<double, articulation_structure::PoseStampedIdMsgConstPtr> > markerStamped;
	PoseIndex poseIndex; // marker1 x marker2 x timestamp --> pose_index
	std::map< int, std::map<int,articulation_msgs::TrackMsgPtr> > trajectories;
	std::map< int, std::map<int,articulation_models::GenericModelPtr> > models;
	std::map< int, std::map<int,std::vector<articulation_models::GenericModelPtr> > > models_all;
	void addPose(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2, size_t id1, size_t id2,KinematicParams &params);
	void updateModel(size_t id1, size_t id2,KinematicParams &params);
	std::vector<double> intersectionOfStamps();
	virtual PoseMap getObservation(double timestamp	);
};

class KinematicGraph: public KinematicGraphType {
public:
	double BIC;
	int DOF;
	double avg_pos_err;
	double avg_orient_err;
	double loglh;
	double k;
	KinematicGraph();
	KinematicGraph(const KinematicGraph &other);	// no deepcopy
	KinematicGraph(const KinematicGraph &other,bool deepcopy);
	void CopyFrom(const KinematicGraph &other);	//deep copy
	size_t getNumberOfParameters();
	int getNominalDOFs();
	std::string getTreeName(bool show_name,bool show_samples,bool show_dof);

	virtual PoseMap getPrediction(double timestamp, PoseMap &observedMarkers, PoseMap &predictedMarkersEmpty, KinematicParams &params,KinematicData &data);
	void projectConfigurationSpace(int reducedDOFs,std::vector<double> stamps, KinematicParams &params,KinematicData &data);

	void evaluateSingleStamp(double timestamp,
					double &avgPositionError,
					double &avgOrientationError,
					double &loglikelihood,KinematicParams &params,KinematicData &data);
	void evaluate(
			std::vector<double> stamps,KinematicParams &params,KinematicData &data);
	int distanceTo(KinematicGraph &other);

};

#endif /* STRUCTS_H_ */
