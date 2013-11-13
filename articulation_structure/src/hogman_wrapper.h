/*
 * hogman_wrapper.h
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#ifndef HOGMAN_WRAPPER_H_
#define HOGMAN_WRAPPER_H_
#include "structs.h"

PoseMap hogman_solver(KinematicGraphType &graph,PoseIndex &poseIndex,PoseMap &observed, PoseMap &predictedEmpty,double sigma_position,double sigma_orientation,double timestamp);

#endif /* HOGMAN_WRAPPER_H_ */
