/*
 * rotational_model.h
 *
 *  Created on: Oct 22, 2009
 *      Author: sturm
 */

#ifndef ROTATIONAL_MODEL_H_
#define ROTATIONAL_MODEL_H_

#include "rigid_model.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/LinearMath/Transform.h"

namespace articulation_models {

class RotationalModel: public GenericModel {
public:
	double rot_mode;
	tf::Vector3 rot_center;
	tf::Quaternion rot_axis;

	double rot_radius;
	tf::Quaternion rot_orientation;

	RotationalModel();
	// -- params
	void readParamsFromModel();
	void writeParamsToModel();

	size_t getDOFs() { return 1; }

	V_Configuration predictConfiguration(geometry_msgs::Pose pose);
	geometry_msgs::Pose predictPose(V_Configuration q);

	bool guessParameters();
	void updateParameters(std::vector<double> delta);
	bool normalizeParameters();

};

}


#endif /* ROTATIONAL_MODEL_H_ */
