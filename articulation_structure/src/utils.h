/*
 * utils.h
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/format.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "articulation_models/utils.h"

#ifndef SQR
#define SQR(a) ((a)*(a))
#endif

std_msgs::ColorRGBA HSV_to_RGB(double h,double s,double v );
tf::Matrix3x3 RPY_to_MAT(double roll, double pitch, double yaw);
void MAT_to_RPY(const tf::Matrix3x3& mat, double &roll, double &pitch, double &yaw);

#define VEC(p1) "["<< \
p1.getOrigin().x() << "," << p1.getOrigin().y()<<","<<p1.getOrigin().z()<< \
"/"<< \
p1.getRotation().x() << "," << p1.getRotation().y()<<","<<p1.getRotation().z()<<","<<p1.getRotation().w()<< \
"]"

std::string transform_to_string(const tf::Transform &p);

std::string pose_to_string(const geometry_msgs::Pose &pose);

std::string uncert_to_string(double sigma_position,double sigma_orientation);

double getBIC(double loglh, size_t k, size_t n);

#endif /* UTILS_H_ */
