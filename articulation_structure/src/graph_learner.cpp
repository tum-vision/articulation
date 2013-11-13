/*
 * graph_learner.cpp
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#include "graph_learner.h"
#include "hogman_wrapper.h"

PoseMap GraphLearner::getPrediction(double timestamp, KinematicGraphType &tree,
		PoseMap &observedMarkers,PoseMap &predictedEmpty) {
	return hogman_solver(tree, poseIndex, observedMarkers, sigma_position,
			sigma_orientation, timestamp);
}

void GraphLearner::sendTreeTransforms(KinematicGraphType graph) {
	if (graph.size() == 0)
		return;

	double latestTimestamp = stampedMarker.rbegin()->first;

	PoseMap observation = getObservation(latestTimestamp);
	PoseMap prediction = getPrediction(intersectionOfStamps().back(), graph,
			observation);

	// send transform for root node
	static tf::TransformBroadcaster br;
	const PoseStamped &latestPose =
			stampedMarker[latestTimestamp].begin()->second->pose;
	for (auto it = prediction.begin(); it != prediction.end(); it++) {
		br.sendTransform(tf::StampedTransform(poseToTransform(it->second),
				latestPose.header.stamp, "/camera", boost::str(boost::format(
						"/pred/%1%") % it->first)));
	}
}

//void GraphLearner::selectSceneModel() {
//	std::vector<double> stamps = intersectionOfStamps();
//
//	if (stamps.size() == 0) {
//		cout << "no intersecting timestamps.." << endl;
//		return;
//	}
//	KinematicGraphType bestGraph;
//	map<double, pair<string, KinematicGraphType> > graphMap;
//	double bestBIC = DBL_MAX;
//	int bestDOF = -1;
//	double best_avg_pos_err = -1;
//	double best_avg_orient_err = -1;
//	double best_loglh = DBL_MAX;
//
//	map<int, int> marker_to_vertex;
//	map<int, int> vertex_to_marker;
//	int n = 0;
//	for (map<int, double>::iterator it = latestTimestamp.begin(); it
//			!= latestTimestamp.end(); it++) {
//		marker_to_vertex[it->first] = n;
//		vertex_to_marker[n] = it->first;
//		n++;
//	}
//
//	int vertices = models.size();
//	int edges = vertices * vertices;
//
//	for (long graph = 1; graph < (1 << (edges - 1)); graph++) {
//		// iterate over all possible graphs
//		std::vector<bool> connected(vertex_to_marker.size(), false);
//		connected[0] = true;
//		KinematicGraphType tree;
//		for (int e = 0; e < edges; e++) {
//			if ((graph & (1 << e)) > 0) {
//				// edge is present in this particular graph
//				int e_from = vertex_to_marker[e / vertices];
//				int e_to = vertex_to_marker[e % vertices];
//				if (e_from >= e_to) {
//					tree.clear();
//					break;
//				}
//				connected[e % vertices] = connected[e / vertices];
//				if (!models[e_from][e_to]) {
//					tree.clear();
//					break;
//				}
//				if (!models[e_to][e_from]) {
//					tree.clear();
//					break;
//				}
//				tree.push_back(pair<pair<int, int> , GenericModelPtr> (pair<
//						int, int> (e_from, e_to), models[e_from][e_to]));
//			}
//		}
//		if ((int) tree.size() < vertices - 1)
//			continue;
//		if (restrict_graphs) {
//			if ((restricted_graphs.find(" " + getTreeName(tree, false, false)
//					+ " ") == restricted_graphs.npos)
//					&& (restricted_graphs.find(" " + getTreeName(tree, true,
//							false) + " ") == restricted_graphs.npos)
//					&& (restricted_graphs.find(" " + getTreeName(tree, true,
//							true) + " ") == restricted_graphs.npos))
//				continue;
//		}
//
//		bool allConnected = true;
//		for (size_t i = 0; i < connected.size(); i++) {
//			if (!connected[i])
//				allConnected = false;
//		}
//		if (allConnected) {
//			// generate tree with all possible link models
//			// find the number of possible combinations
//			KinematicGraphType basetree = tree;
//			tree.clear();
//			int combination_total = 1;
//			for (KinematicGraphType::iterator it = basetree.begin(); it
//					!= basetree.end(); it++) {
//				combination_total
//						*= models_all[it->first.first][it->first.second].size();
//			}
//			if (search_all_models)
//				cout << "need to evaluate n=" << combination_total
//						<< " different versions of tree " << getTreeName(
//						basetree, true, true) << endl;
//			for (int combination = 0; combination < combination_total; combination++) {
//				if (search_all_models) {
//					tree.clear();
//					int c_tmp = combination;
//					for (KinematicGraphType::iterator it = basetree.begin(); it
//							!= basetree.end(); it++) {
//						int
//								idx =
//										c_tmp
//												% models_all[it->first.first][it->first.second].size();
//						tree.push_back(
//								pair<pair<int, int> , GenericModelPtr> (
//										pair<int, int> (it->first.first,
//												it->first.second),
//										models_all[it->first.first][it->first.second][idx]));
//						c_tmp
//								/= models_all[it->first.first][it->first.second].size();
//					}
//				} else {
//					tree = basetree;
//				}
//
//				int DOFs = getDOFs(tree);// compute sum
//				for (int reducedDOFs = (reduce_dofs ? 1 : DOFs); reducedDOFs
//						<= DOFs; reducedDOFs++) {
//					KinematicGraphType copyOfTree = copyTree(tree);
//					if (reducedDOFs != DOFs) {
//						//							cout <<"reducing dofs"<<endl;
//						projectConfigurationSpace(copyOfTree, reducedDOFs,
//								stamps);
//					}
//
//					string gname_short = boost::str(boost::format("%1%-DOF ")
//							% reducedDOFs) + getTreeName(copyOfTree, false,
//							false);
//					string gname_models = boost::str(boost::format("%1%-DOF ")
//							% reducedDOFs) + getTreeName(copyOfTree, true,
//							false);
//					string gname_models_samples = boost::str(boost::format(
//							"%1%-DOF ") % reducedDOFs) + getTreeName(
//							copyOfTree, true, false);
//					double avg_pos_err, avg_orient_err, loglh;
//					double BIC = evalTree(copyOfTree, gname_models_samples,
//							stamps, avg_pos_err, avg_orient_err, loglh);
//					if (reducedDOFs < DOFs) {
//						BIC -= (((DOFs - reducedDOFs) * stamps.size() + (DOFs
//								* reducedDOFs)) * log(stamps.size()));
//					}
//					evalGraphBIC[gname_models][stamps.size()] = BIC;
//					graphMap[BIC] = pair<string, KinematicGraphType> (
//							gname_models, copyOfTree);
//					if (BIC < bestBIC) {
//						bestGraph = tree;
//						bestBIC = BIC;
//						bestDOF = reducedDOFs;
//						best_avg_pos_err = avg_pos_err;
//						best_avg_orient_err = avg_orient_err;
//						best_loglh = loglh;
//					}
//					//					cout <<"BIC="<<BIC <<": "<<boost::str(boost::format( "%1%-DOF ") % reducedDOFs ) + graphname.str() <<endl;
//
//					// should we finish?
//					static ros::NodeHandle n;
//					if (!n.ok())
//						return;
//				}
//				if (!search_all_models)
//					break;
//			}
//		}
//	}
//	saveGraphEval();
//	sendModels(bestGraph);
//	sendStructureVisualization(bestGraph, bestBIC, bestDOF, best_avg_pos_err,
//			best_avg_orient_err, best_loglh);
//	cout << "Evaluation @ t=" << stamps.size() << endl;
//	for (map<double, pair<string, KinematicGraphType> >::iterator it =
//			graphMap.begin(); it != graphMap.end(); it++) {
//		cout << it->first << "      " << it->second.first << endl;
//	}
//	cout << "---" << endl;
//}

int main(int argc, char** argv) {
	ros::init(argc, argv, "graph_learner");

	GraphLearner learner;

	ros::NodeHandle nh;
	ros::Publisher lifebeat_pub =
			nh.advertise<std_msgs::Empty> ("/lifebeat", 1);
	while (nh.ok()) {
		cout << "lifebeat.." << learner.intersectionOfStamps().size() << endl;
		ros::Rate loop_rate(3);
		loop_rate.sleep();
		ros::spinOnce();

		lifebeat_pub.publish(std_msgs::Empty());
	}
}
