/*
 * ArticulatedObject.h
 *
 *  Created on: Sep 14, 2010
 *      Author: sturm
 */

#ifndef ARTICULATEDOBJECT_H_
#define ARTICULATEDOBJECT_H_

#include "articulation_msgs/TrackModelSrv.h"
#include "articulation_msgs/ArticulatedObjectSrv.h"
#include "articulation_msgs/ArticulatedObjectMsg.h"
#include "articulation_models/models/factory.h"
#include "articulation_models/utils.h"
#include "structs.h"


class ArticulatedObject: public KinematicParams, public KinematicData {
public:
	articulation_msgs::ArticulatedObjectMsg object_msg;
	KinematicGraph currentGraph;
	std::map< std::string, KinematicGraph > graphMap;
	ArticulatedObject();
	ArticulatedObject(const KinematicParams &other);
	void SetObjectModel(const articulation_msgs::ArticulatedObjectMsg &msg,ros::NodeHandle* nh_local);
	articulation_msgs::ArticulatedObjectMsg& GetObjectModel();
	void FitModels();
	KinematicGraph getSpanningTree();
	void ComputeSpanningTree();
	void getFastGraph();
	void getGraph();
	void enumerateGraphs();
	void saveEval();
};

#endif /* ARTICULATEDOBJECT_H_ */
