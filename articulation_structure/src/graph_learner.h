/*
 * graph_learner.h
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#ifndef GRAPH_LEARNER_H_
#define GRAPH_LEARNER_H_

#include "structure_learner_base.h"

class GraphLearner: public KinematicStructureLearner {
public:
	void sendTreeTransforms(KinematicGraphType graph);
//	void selectSceneModel();
//	void sendSceneModel();
	PoseMap getPrediction(PoseMap &observedMarkers,PoseMap &predictedEmpty);
};

#endif /* GRAPH_LEARNER_H_ */
