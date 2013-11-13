/*
 * structure_learner_base.h
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#ifndef STRUCTURE_LEARNER_BASE_H_
#define STRUCTURE_LEARNER_BASE_H_

#include "structs.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace articulation_models;
using namespace articulation_msgs;
using namespace articulation_structure;

class EvalStampStruct {
public:
	bool bestGraph;
	bool evalInFastSearch;
	bool bestFast;
	bool bestTree;
	double BIC;
	int segment;
	EvalStampStruct():bestGraph(false),evalInFastSearch(false),bestFast(false),bestTree(false),BIC(0),segment(0) {}
};

class EvalStruct {
public:
	bool bestGraphOnce;
	bool bestTreeOnce;
	bool bestFastOnce;
	int segments;
	std::map<int,EvalStampStruct> stampEval;
	EvalStruct():bestGraphOnce(false),bestTreeOnce(false),bestFastOnce(false),segments(0) {}
};

class KinematicStructureLearner: public KinematicParams, public KinematicData {
public:
	boost::mutex frame_mutex_;

	ros::Publisher model_pub;
	ros::Publisher marker_pub;

	ros::NodeHandle nh;
	ros::NodeHandle nh_local;

	ros::Subscriber pose_sub1;
	ros::Subscriber pose_sub2;
	ros::Subscriber pose_sub3;
	ros::Subscriber pose_sub4;

	map< string, KinematicGraph > graphMap;
	std::map< std::string, EvalStruct> evalGraph;

	std::map< int, double> evalGraphBIC;
	std::map< int, double> evalFastBIC;
	std::map< int, double> evalTreeBIC;

	std::map< int, double> evalModelsRuntime;
	std::map< int, double> evalGraphRuntime;
	std::map< int, double> evalFastRuntime;
	std::map< int, double> evalTreeRuntime;

	std::map< int, double> evalDOF;
	std::map< int, double> evalLinks;

	KinematicGraph currentGraph;
	map< string, KinematicGraph > previousGraphs;

	KinematicStructureLearner();

	KinematicGraph getSpanningTree(bool increasing);
	virtual void sendModels(KinematicGraph E_new);
	virtual void sendTreeTransforms(KinematicGraph E_new);
	void saveGraphEval();
	void saveBICEval();
	void saveRuntimeEval();
	void saveDOFLinkEval();
	virtual void sendStructureVisualization(KinematicGraph graph);
	std::vector<double>  downsampleStamps(std::vector<double> vec,size_t num);
	virtual void selectSceneModel();
	virtual void sendSceneModel();
	void poseCallback(const PoseStampedIdMsgConstPtr& pose);
	void poseCallback(const PoseStampedConstPtr& pose, size_t id);
	void poseCallback1(const PoseStampedConstPtr& pose);
	void poseCallback2(const PoseStampedConstPtr& pose);
	void poseCallback3(const PoseStampedConstPtr& pose);
	void poseCallback4(const PoseStampedConstPtr& pose);
	void showEvaluation();
	void analyzeTopologyOfGraphs();
	void enumerateGraphs();
	void addPreviousGraphs();

	void evalTree();
	void evalFull();
	void evalFast(bool setCurrent);
};


#endif /* STRUCTURE_LEARNER_BASE_H_ */
