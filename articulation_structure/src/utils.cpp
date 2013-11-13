/*
 * utils.cpp
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#include "utils.h"
using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace articulation_models;

std_msgs::ColorRGBA HSV_to_RGB(double h,double s,double v ) {
	h -= floor(h);
	h *= 6;
	int i;
	float m, n, f;

	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);
	std_msgs::ColorRGBA r;
	r.a = 1.0;
	switch (i) {
	case 6:
	case 0:
		r.r=v; r.g=n; r.b=m; break;
	case 1:
		r.r=n; r.g=v; r.b=m; break;
	case 2:
		r.r=m; r.g=v; r.b=n; break;
	case 3:
		r.r=m; r.g=n; r.b=v; break;
	case 4:
		r.r=n; r.g=m; r.b=v; break;
	case 5:
		r.r=v; r.g=m; r.b=n; break;
	default:
		r.r=1; r.g=0.5; r.b=0.5;
	}
	return r;
}

tf::Matrix3x3 RPY_to_MAT(double roll, double pitch, double yaw){
	double sphi   = sin(roll);
	double stheta = sin(pitch);
	double spsi   = sin(yaw);
	double cphi   = cos(roll);
	double ctheta = cos(pitch);
	double cpsi   = cos(yaw);

    return(tf::Matrix3x3(
      cpsi*ctheta, cpsi*stheta*sphi - spsi*cphi, cpsi*stheta*cphi + spsi*sphi,
      spsi*ctheta, spsi*stheta*sphi + cpsi*cphi, spsi*stheta*cphi - cpsi*sphi,
          -stheta,                  ctheta*sphi,                  ctheta*cphi
    ));
}

void MAT_to_RPY(const tf::Matrix3x3& mat, double &roll, double &pitch, double &yaw) {
  roll = atan2(mat[2][1],mat[2][2]);
  pitch = atan2(-mat[2][0],sqrt(mat[2][1]*mat[2][1] + mat[2][2]* mat[2][2]));
  yaw = atan2(mat[1][0],mat[0][0]);
}

#define VEC(p1) "["<< \
p1.getOrigin().x() << "," << p1.getOrigin().y()<<","<<p1.getOrigin().z()<< \
"/"<< \
p1.getRotation().x() << "," << p1.getRotation().y()<<","<<p1.getRotation().z()<<","<<p1.getRotation().w()<< \
"]"

string transform_to_string(const tf::Transform &p) {
	stringstream s;
	double roll,pitch,yaw;
	MAT_to_RPY(p.getBasis(),roll,pitch,yaw);
	s <<
			p.getOrigin().x() << " " <<
			p.getOrigin().y() << " " <<
			p.getOrigin().z() << " " <<
			roll << " " <<
			pitch <<" " <<
			yaw;

	return s.str();
}

string pose_to_string(const Pose &pose) {
	tf::Transform p = poseToTransform(pose);
	return transform_to_string(p);
}

string uncert_to_string(double sigma_position,double sigma_orientation) {
	stringstream s;
	double ip = 1/SQR(sigma_position);
	double io = 1/SQR(sigma_orientation);
	s <<
			" " << ip <<" 0 0 0 0 0" <<
			" " << ip <<" 0 0 0 0" <<
			" " << ip <<" 0 0 0" <<
			" " << io <<" 0 0" <<
			" " << io <<" 0" <<
			" " << io;
	return s.str();
}

double getBIC(double loglh, size_t k, size_t n) {
	return(	-2*( loglh ) + ( k ) * log( n ) );
}
