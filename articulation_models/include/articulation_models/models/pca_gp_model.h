/*
 * pca_gp_model.h
 *
 *  Created on: Feb 10, 2010
 *      Author: sturm
 */

#ifndef PCA_GP_MODEL_H_
#define PCA_GP_MODEL_H_

#include "prismatic_model.h"
#include "gaussian_process/SingleGP.h"

namespace articulation_models {

class PCAGPModel: public GenericModel {
public:
	tf::Vector3 rigid_position;
	tf::Vector3 prismatic_dir;
	size_t training_samples;

	PCAGPModel();
	virtual ~PCAGPModel();

	std::vector<gaussian_process::SingleGP*> gp;
	double downsample;
	bool initialized;

	tf::Transform pose(size_t index);
	void storeData(bool inliersOnly);

	// -- params
	void readParamsFromModel();
	void writeParamsToModel();

	size_t getDOFs() { return 1; }

	bool fitModel();
	void buildGPs();
	V_Configuration predictConfiguration(geometry_msgs::Pose pose);
	geometry_msgs::Pose predictPose(V_Configuration q);

	void checkInitialized();
	void projectPoseToConfiguration();
	void projectConfigurationToPose();
	void projectConfigurationToJacobian();
	bool evaluateModel();
	bool normalizeParameters();

};

}

#endif /* PCA_GP_MODEL_H_ */
