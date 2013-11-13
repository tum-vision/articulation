/*
 * icp_utils.h
 *
 *  Created on: Feb 15, 2010
 *      Author: sturm
 */

#ifndef ICP_UTILS_H_
#define ICP_UTILS_H_

#include "articulation_msgs/TrackMsg.h"
#include "icp/icpCpp.h"
#include "icp/kdtree_common.h"

namespace icp {

class TrajData {
public:
	size_t n;
	double *points;
	double *pointsT;
	double *weights;
	int *index;
	unsigned int *randvec;
	TrajData(const articulation_msgs::TrackMsg &track);
	~TrajData();
};

class IcpAlign {
public:
	TrajData model;
	TrajData data;
	int iter;
	icp::Tree *tree;
	double TR[9];
	double TT[3];
public:
	IcpAlign(const articulation_msgs::TrackMsg &track_model,const articulation_msgs::TrackMsg &track_data, int iter=40);
	~IcpAlign();
	void TransformData(articulation_msgs::TrackMsg &track_data_aligned);
	void TransformModel(articulation_msgs::TrackMsg &track_model_aligned);

};

}

#endif /* ICP_UTILS_H_ */
