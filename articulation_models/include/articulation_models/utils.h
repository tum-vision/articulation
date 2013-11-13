/*
 * utils.h
 *
 *  Created on: Oct 21, 2009
 *      Author: sturm
 */

#ifndef ARTICULATION_MODELS_UTILS_H_
#define ARTICULATION_MODELS_UTILS_H_

#include "tf/LinearMath/Transform.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_msgs/ParamMsg.h"
#include <Eigen/Core>

namespace articulation_models {

#ifndef SQR
#define SQR(a) ((a)*(a))
#endif

#ifndef MIN
#define MIN(a,b) ((a<=b)?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a>=b)?(a):(b))
#endif

#define PRINT_TRANSFORM(tf) \
	"[ "<<tf.getOrigin().x() <<"; "<<tf.getOrigin().y() <<"; "<<tf.getOrigin().z() <<"]" << \
	"( "<<tf.getRotation().x() <<"; "<<tf.getRotation().y() <<"; "<<tf.getRotation().z()<<"; "<<tf.getRotation().w() <<") "

typedef Eigen::MatrixXd M_CartesianJacobian;
typedef Eigen::VectorXd V_Configuration;

inline tf::Quaternion orientationToQuaternion(geometry_msgs::Quaternion orientation) {
	return tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w);
}

inline tf::Vector3 positionToVector(geometry_msgs::Point position) {
	return tf::Vector3(position.x,position.y,position.z);
}

inline tf::Transform poseToTransform(geometry_msgs::Pose pose) {
	return(tf::Transform(
			orientationToQuaternion(pose.orientation),
			positionToVector(pose.position)
		));
}

inline geometry_msgs::Quaternion quaternionToOrientation(tf::Quaternion quat) {
	geometry_msgs::Quaternion orientation;
	orientation.x = quat.x();
	orientation.y = quat.y();
	orientation.z = quat.z();
	orientation.w = quat.w();
	return orientation;
}

inline geometry_msgs::Point vectorToPosition(tf::Vector3 point) {
	geometry_msgs::Point position;
	position.x = point.x();
	position.y = point.y();
	position.z = point.z();
	return position;
}

inline geometry_msgs::Pose transformToPose(tf::Transform transform) {
	geometry_msgs::Pose pose;
	pose.orientation = quaternionToOrientation( transform.getRotation() );
	pose.position = vectorToPosition( transform.getOrigin() );
	return pose;
}

int openChannel(articulation_msgs::TrackMsg &track,std::string name,bool autocreate=true);

articulation_msgs::TrackMsg flipTrack(articulation_msgs::TrackMsg input, int corner=0);

Eigen::VectorXd pointToEigen(geometry_msgs::Point p);
geometry_msgs::Point eigenToPoint(Eigen::VectorXd v);
void setParamIfNotDefined(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name, double value, uint8_t type=articulation_msgs::ParamMsg::PRIOR);
void setParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name, double value, uint8_t type=articulation_msgs::ParamMsg::PRIOR);
double getParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name);
bool hasParam(std::vector<articulation_msgs::ParamMsg> &vec,
		std::string name);
//Eigen::VectorXd vectorToEigen(V_Configuration q);
//Eigen::MatrixXd matrixToEigen(M_CartesianJacobian J);


bool check_values(const tf::Vector3 &vec);
bool check_values(const tf::Quaternion &vec);
bool check_values(double v);
bool check_values(float v);
}

#endif /* UTILS_H_ */
