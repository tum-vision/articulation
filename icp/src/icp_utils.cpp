/*
 * icp_utils.cpp
 *
 *  Created on: Feb 15, 2010
 *      Author: sturm
 */

#include "icp/icp_utils.h"

namespace icp {

inline void multiply(double TR[9], double TT[3], geometry_msgs::Pose &track) {
	double x = track.position.x;
	double y = track.position.y;
	double z = track.position.z;
	track.position.x = TR[0] * x + TR[3] * y + TR[6] * z + TT[0];
	track.position.y = TR[1] * x + TR[4] * y + TR[7] * z + TT[1];
	track.position.z = TR[2] * x + TR[5] * y + TR[8] * z + TT[2];
}

TrajData::TrajData(const articulation_msgs::TrackMsg &track) {
	n = track.pose.size();
	points = new double[3 * n];
	pointsT = new double[3 * n];
	weights = new double[3 * n];
	index = new int[n];
	randvec = new unsigned int[n];
	std::vector<int> list;
	list.resize(n);
	for (size_t i = 0; i < n; i++) {
		list[i] = i;
	}
	random_shuffle(list.begin(), list.end());
	for (size_t i = 0; i < n; i++) {
		points[i + 0 * n] = track.pose[i].position.x;
		points[i + 1 * n] = track.pose[i].position.y;
		points[i + 2 * n] = track.pose[i].position.z;
		pointsT[i * 3 + 0] = track.pose[i].position.x;
		pointsT[i * 3 + 1] = track.pose[i].position.y;
		pointsT[i * 3 + 2] = track.pose[i].position.z;
		weights[i] = 1;
		index[i] = i;
		randvec[i] = list[i];
		//printf("%d ",randvec[i]);
	}
}

TrajData::~TrajData() {
	delete points;
	delete weights;
	delete index;
}

IcpAlign::IcpAlign(const articulation_msgs::TrackMsg &track_model,
		const articulation_msgs::TrackMsg &track_data, int iter) :
	model(track_model), data(track_data), iter(iter), tree(NULL) {
	tree = icp::build_kdtree(model.points, model.n, 3, model.index, model.n, 0);
	icp::icp(TR, TT, model.pointsT, model.n, data.pointsT, data.weights,
			data.n, data.randvec, data.n, data.n * 1.45, iter, tree);
}

IcpAlign::~IcpAlign() {
	icp::free_tree(tree->rootptr);
}

void IcpAlign::TransformData(articulation_msgs::TrackMsg &track_data_aligned) {
	for (size_t i = 0; i < track_data_aligned.pose.size(); i++) {
		multiply(TR, TT, track_data_aligned.pose[i]);
	}
}

void IcpAlign::TransformModel(articulation_msgs::TrackMsg &track_model_aligned) {
	double TR2[9];
	double TT2[3];
	for (int r = 0; r < 3; r++) {
		TT2[r] = 0;
		for (int c = 0; c < 3; c++) {
			TR2[r * 3 + c] = TR[c * 3 + r];
		}
	}
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++) {
			TT2[r] += TR[r * 3 + c] * (-TT[c]);
		}
	}
	for (size_t i = 0; i < track_model_aligned.pose.size(); i++) {
		multiply(TR2, TT2, track_model_aligned.pose[i]);
	}
}

}
