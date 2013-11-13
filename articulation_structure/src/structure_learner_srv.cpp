/*
 * model_learner.cpp
 */

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "ArticulatedObject.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;
using namespace articulation_models;
using namespace articulation_msgs;
using namespace articulation_structure;
using namespace visualization_msgs;

ros::NodeHandle* nh;
ros::NodeHandle* nh_local;

ros::Publisher model_pub;
ros::Publisher track_pub;
ros::Publisher marker_pub;

KinematicParams params;

bool structureFitModels(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, fitting models, object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	object.SetObjectModel(request.object,nh_local);
	object.FitModels();
	response.object = object.GetObjectModel();

	return (true);
}

bool structureSelectSpanningTree(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, select spanning tree, object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	object.SetObjectModel(request.object,nh_local);
	object.FitModels();
	object.ComputeSpanningTree();

	response.object = object.GetObjectModel();

	return (true);
}

bool structureSelectFastGraph(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, select graph, object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	ROS_INFO("Setting object model");
	object.SetObjectModel(request.object,nh_local);
	ROS_INFO("Fitting link models");
	object.FitModels();
	ROS_INFO("Selecting kinematic graph (fast)");
	object.getFastGraph();
	response.object = object.GetObjectModel();
	return (true);
}

bool structureSelectGraph(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, select graph, object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	ROS_INFO("Setting object model");
	object.SetObjectModel(request.object,nh_local);
	ROS_INFO("Fitting link models");
	object.FitModels();
	ROS_INFO("Selecting kinematic graph (full eval)");
	object.getGraph();
	response.object = object.GetObjectModel();
	return (true);
}

bool structureSelectGraphAll(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, select graph ALL (eval), object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	ROS_INFO("Setting object model");
	object.SetObjectModel(request.object,nh_local);
	ROS_INFO("Fitting link models");
	object.FitModels();
	ROS_INFO("Selecting kinematic graph (full eval)");
	object.ComputeSpanningTree();
	object.getFastGraph();
	object.getGraph();
	response.object = object.GetObjectModel();
	return (true);
}

map<string,MarkerArray> markers,old_markers;
vector<int> old_model_ids;

void AddMarkerLine(string ns,Point point_from,Point point_to,double size=0.01,double H=0,double S=1,double V=1,Point point_relative=Point()) {
	Marker marker;
	marker.id = markers[ns].markers.size();
	marker.type = Marker::LINE_LIST;
	marker.action= Marker::ADD;
	marker.pose.orientation.w = 1.00;
	marker.color =  HSV_to_RGB(H,S,V);
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.pose.position = point_relative;
	Point point_diff1;
	point_diff1.x = point_from.x - point_relative.x;
	point_diff1.y = point_from.y - point_relative.y;
	point_diff1.z = point_from.z - point_relative.z;
	Point point_diff2;
	point_diff2.x = point_to.x - point_relative.x;
	point_diff2.y = point_to.y - point_relative.y;
	point_diff2.z = point_to.z - point_relative.z;
	marker.points.push_back(point_diff1);
	marker.points.push_back(point_diff2);
	marker.ns = ns;
	markers[ns].markers.push_back(marker);
}

void AddMarkerPoint(string ns,Point point,double size=0.01,double H=0,double S=1,double V=1) {
	Marker marker;
	marker.id = markers[ns].markers.size();
	marker.type = Marker::SPHERE;
	marker.action= Marker::ADD;
	marker.pose.orientation.w = 1.00;
	marker.color =  HSV_to_RGB(H,S,V);
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.pose.position = point;
	marker.ns = ns;
	markers[ns].markers.push_back(marker);
}
void AddMarkerLine(string ns,Pose pose_from,Pose pose_to,double size=0.01,double H=0,double S=1,double V=1,Pose relative=Pose()) {
	AddMarkerLine(ns,pose_from.position,pose_to.position,size,H,S,V,relative.position);
}

void AddMarkerLine(string ns,tf::Transform pose_from,tf::Transform pose_to,double size=0.01,double H=0,double S=1,double V=1,tf::Transform relative=tf::Transform::getIdentity()) {
	AddMarkerLine(ns,transformToPose(pose_from),transformToPose(pose_to),size,H,S,V,transformToPose(relative));
}

void AddMarkerLine(string ns,tf::Vector3 pose_from,tf::Vector3 pose_to,double size=0.01,double H=0,double S=1,double V=1,tf::Vector3 relative=tf::Vector3(0,0,0)) {
	AddMarkerLine(ns,vectorToPosition(pose_from),vectorToPosition(pose_to),size,H,S,V,vectorToPosition(relative));
}

void AddText(string ns,Point point,string s="hello",double size=0.05,double H=0,double S=1,double V=1) {
	Marker marker;
	marker.id = markers[ns].markers.size();
	marker.type = Marker::TEXT_VIEW_FACING;
	marker.action= Marker::ADD;
	marker.pose.position = point;
	marker.pose.orientation.w = 1.00;
	marker.color =  HSV_to_RGB(H,S,V);
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.ns = ns;
	marker.text = s;
	markers[ns].markers.push_back(marker);
}

void DeleteOldMarkers() {
	for(map<string,MarkerArray>::iterator i=markers.begin();i!=markers.end();i++) {
		string ns = i->first;
		while(markers[ns].markers.size() < old_markers[ns].markers.size()) {
			Marker marker;
			marker.id = markers[ns].markers.size();
			marker.action= Marker::DELETE;
			marker.ns = ns;
			markers[ns].markers.push_back(marker);
		}
	}
}

tf::Vector3 getOrtho(tf::Vector3 a,tf::Vector3 b) {
	tf::Vector3 bb = b - a.dot(b)*a;
	bb.normalize();
	a.normalize();
	return a.cross(bb);
}

bool visualizeGraph(articulation_msgs::ArticulatedObjectSrv::Request &request,
		articulation_msgs::ArticulatedObjectSrv::Response &response) {
	ROS_INFO("Service call, visualize graph, object parts=%d, obs=%d",
		(int)request.object.parts.size(),
		(int)request.object.parts.size()>0? (int)request.object.parts[0].pose.size():-1);

	ArticulatedObject object(params);
	object.SetObjectModel(request.object,nh_local);

	// send tracks of object parts, and projected tracks of object parts
	for(size_t i=0;i<response.object.parts.size();i++) {
		response.object.parts[i].id = i;
		track_pub.publish(response.object.parts[i]);
	}

	// send link models, including observations, projected and resampled observations
	// first, delete old models
	for(size_t i=0;i<old_model_ids.size();i++) {
		ModelMsg m;
		m.header = object.object_msg.header;
		m.id = old_model_ids[i];
		m.track.id = old_model_ids[i];
		model_pub.publish(m);
	}
	old_model_ids.clear();

	// send link visualization (links + text)
	for(KinematicGraph::iterator i= object.currentGraph.begin();i!=object.currentGraph.end();i++) {
		int from = i->first.first;
		int to = i->first.second;
		GenericModelPtr &model = i->second;
		Pose pose_from = object.object_msg.parts[from].pose.back();
		Pose pose_to = object.object_msg.parts[to].pose.back();
//		AddText("modelname.end",
//				pose_to.position,
//				model->getModelName(),
//				0.03,
//				0,0,0);

		if(model->getModelName()=="rigid") {
			// straight line between two parts
			AddMarkerLine("link",pose_from,pose_to,0.01,0.1666,1.0,1.0);
			AddMarkerPoint("decoration.poses",
					pose_to.position,
					0.015,0.166,1.0,1.0);
			AddText("modelname",
					vectorToPosition((
							positionToVector(pose_from.position)+positionToVector(pose_to.position))*0.5
							+ tf::Vector3(0.,0.,-0.001)
					),
					model->getModelName(),
					0.03,
					0,0,1);
		} else if(model->getModelName()=="prismatic") {
			AddMarkerPoint("decoration.poses",
					pose_to.position,
					0.015,0.0,1.0,1.0);
			Pose origin = model->predictPose(model->predictConfiguration( transformToPose(tf::Transform::getIdentity()) ));
			tf::Transform pose_orthogonal = poseToTransform(pose_from)*poseToTransform(origin);

			tf::Transform pose_max = poseToTransform(pose_from)*poseToTransform(
					model->predictPose(model->getMinConfigurationObserved()));
			tf::Transform pose_min = poseToTransform(pose_from)*poseToTransform(
					model->predictPose(model->getMaxConfigurationObserved()));

			// draw tics
			for(double q = model->getMinConfigurationObserved()[0];
					q<model->getMaxConfigurationObserved()[0];
					q += 0.05) {
				V_Configuration qq(1);
				qq(0) = q;
				tf::Transform pose_q = poseToTransform(pose_from)*poseToTransform(
						model->predictPose(qq));

				tf::Vector3 dir = poseToTransform(pose_from) * boost::static_pointer_cast<PrismaticModel>(model)->prismatic_dir
						- poseToTransform(pose_from) * tf::Vector3(0,0,0);
				dir.normalize();

				AddMarkerLine("decoration.poses",
						pose_q.getOrigin(),
						pose_q.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*0.005,
						0.0025,0.0,0.0,0.3,pose_q.getOrigin());
				AddMarkerLine("decoration.poses",
						pose_q.getOrigin() ,
						pose_q.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*(-0.005),
						0.0025,0.0,0.0,0.3,pose_q.getOrigin());
			}

			//AddMarkerLine("link",pose_from,pose_to,0.005,0.0,0.0,0.3);
			AddMarkerLine("link",poseToTransform(pose_from),poseToTransform(pose_to),0.01,0.0,1.0,1.0);
			//AddMarkerLine("link",pose_orthogonal,poseToTransform(pose_to),0.01,0.0,1.0,1.0);
			AddMarkerLine("decoration",pose_min,pose_max,0.005,0.00,1.0,0.5);
			//AddMarkerLine("decoration",pose_min,pose_max,0.0025,0.0,0.2,0.0);
			AddText("modelname",
					vectorToPosition((
							positionToVector(pose_from.position)+positionToVector(pose_to.position))*0.5
							+ tf::Vector3(0.,0.,-0.001)
					),
					model->getModelName(),
					0.03,
					0,0,1);
		} else if(model->getModelName()=="rotational") {
			// draw line to rotational center
			geometry_msgs::Pose rot_center = transformToPose(
					poseToTransform(pose_from)*
					tf::Transform(
					boost::static_pointer_cast<RotationalModel>(model)->rot_axis,
					boost::static_pointer_cast<RotationalModel>(model)->rot_center)
					);

			AddMarkerLine("link",pose_from,rot_center,0.01,0.666,1.0,1.0);
			AddMarkerLine("link",rot_center,pose_to,0.01,0.666,1.0,1.0);
			AddMarkerLine("decoration",rot_center,pose_to,0.01,0.666,1.0,1.0);
			AddMarkerPoint("decoration.poses",
					pose_to.position,
					0.015,0.667,1.0,1.0);

			double q0 = model->predictConfiguration( transformToPose(tf::Transform::getIdentity()) )[0];
			double q1 = model->predictConfiguration( model->model.track.pose.back() )[0];
			if(q0>q1) {
				double q= q1;
				q1 = q0;
				q0 = q;
			}
			if(q1-q0 > M_PI) {
				// go the other way around
				double q= q1;
				q1 = q0+2*M_PI;
				q0 = q;
			}

			V_Configuration Qmin = model->getMinConfigurationObserved();
			V_Configuration Qmax = model->getMaxConfigurationObserved();

			V_Configuration Q(1);
			tf::Transform t1,t2,t3,t4;
			double STEPSIZE = M_PI/16;
			double radius = 0.05;
			for(double q=0;q<2*M_PI+STEPSIZE;q+=STEPSIZE) {
				bool in_range = q>=Qmin[0] && q<=Qmax[0];
				if(Qmin[0]<0) in_range = q>=(Qmin[0]+2*M_PI) || q<=Qmax[0];
				Q[0] = q;
				t1 = 	poseToTransform(rot_center) *
						tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-q),tf::Vector3(0,0,0)) *
						tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(radius,0,0.0));
				t2 = 	poseToTransform(rot_center) *
						tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-(q+STEPSIZE)),tf::Vector3(0,0,0)) *
						tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(radius,0,0.0));
				t3 = 	poseToTransform(rot_center) *
						tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-(q+STEPSIZE)),tf::Vector3(0,0,0)) *
						tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(radius*1.1,0,0.0));
				t4 = 	poseToTransform(rot_center) *
						tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),-(q+STEPSIZE)),tf::Vector3(0,0,0)) *
						tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(radius*0.9,0,0.0));
				AddMarkerLine("decoration",
						t1*tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(-0.015,0,0)),
						t2*tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(-0.015,0,0)),
						0.005,
						0.666,0.0,in_range?0.00:0.50,poseToTransform(rot_center));
			}

			// rotation axis:
			AddMarkerLine("decoration",
					poseToTransform(rot_center),
					poseToTransform(rot_center) *
					tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,-0.10)),
					0.01,0.666,0.2,0.2,poseToTransform(rot_center));
			AddMarkerLine("decoration",
					poseToTransform(rot_center),
					poseToTransform(rot_center) *
					tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,+0.10)),
					0.01,0.666,0.2,0.2,poseToTransform(rot_center));
			AddText("modelname",
					vectorToPosition(
							positionToVector(rot_center.position)
							+ tf::Vector3(0.,0.,-0.001)
					),
					model->getModelName(),
					0.03,
					0,0,1);

			double STEPSIZE2 = M_PI/64;
//			bool first = true;

			// draw arc
			tf::Transform pose_q2;
			tf::Transform pose_q1;
			for(double q = Qmin[0]; q<=Qmax[0]; q += (Qmax[0]-Qmin[0])/32) {
//				cout <<"q="<<q<<endl;
				pose_q2 = pose_q1;
				V_Configuration qq(1);
				qq(0) = q;
				pose_q1 = poseToTransform(pose_from)*poseToTransform(model->predictPose(qq));

				if(q != Qmin[0]) {
					AddMarkerLine("decoration.poses",
								pose_q1.getOrigin(),
								pose_q2.getOrigin(),
								0.005,0.667,1.0,0.5);
				}
			}


			// draw tics
			for(double q = model->getMinConfigurationObserved()[0];
					q<model->getMaxConfigurationObserved()[0];
					q += STEPSIZE2) {
				V_Configuration qq(1);
				qq(0) = q;
				tf::Transform pose_q1 = poseToTransform(pose_from)*poseToTransform(
						model->predictPose(qq));
				qq(0) = q+STEPSIZE2;
				tf::Transform pose_q2 = poseToTransform(pose_from)*poseToTransform(
						model->predictPose(qq));

				tf::Vector3 dir = pose_q2.getOrigin()-pose_q1.getOrigin();
				dir.normalize();

				AddMarkerLine("decoration.poses",
						pose_q1.getOrigin(),
						pose_q1.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*0.005,
						0.0025,0.0,0.0,0.3,pose_q1.getOrigin());
				AddMarkerLine("decoration.poses",
						pose_q1.getOrigin() ,
						pose_q1.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*(-0.005),
						0.0025,0.0,0.0,0.3,pose_q1.getOrigin());
			}
		} else if(model->getModelName()=="pca_gp") {
			AddMarkerPoint("decoration.poses",
					pose_to.position,
					0.015,0.333,1.0,1.0);
			for(double q = model->getMinConfigurationObserved()[0];
					q<model->getMaxConfigurationObserved()[0];
					q += 0.05) {
				V_Configuration qq(1);
				qq(0) = q;
				tf::Transform pose_q2 = poseToTransform(pose_from)*poseToTransform(
						model->predictPose(qq));
				qq(0) = q+0.05;
				tf::Transform pose_q1 = poseToTransform(pose_from)*poseToTransform(
						model->predictPose(qq));

				AddMarkerLine("decoration.poses",
						pose_q1.getOrigin(),
						pose_q2.getOrigin(),
						0.0025,0.0,0.0,0.3);

				tf::Vector3 dir = pose_q2.getOrigin()-pose_q1.getOrigin();
				dir.normalize();

				AddMarkerLine("decoration.poses",
						pose_q1.getOrigin(),
						pose_q1.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*0.005,
						0.0025,0.0,0.0,0.3,pose_q1.getOrigin());
				AddMarkerLine("decoration.poses",
						pose_q1.getOrigin() ,
						pose_q1.getOrigin() + getOrtho(dir,tf::Vector3(0,0,1))*(-0.005),
						0.0025,0.0,0.0,0.3,pose_q1.getOrigin());
			}


			AddMarkerLine("link",pose_from,pose_to,0.01,0.333,1.0,1.0);
			AddText("modelname",
					vectorToPosition((
							positionToVector(pose_from.position)+positionToVector(pose_to.position))*0.5
							+ tf::Vector3(0.,0.,-0.001)
					),
					model->getModelName(),
					0.03,
					0,0,1);
		} else {
			AddMarkerLine("link",pose_from,pose_to,0.01,5.0/6,1.0,1.0);
			AddText("modelname",
					vectorToPosition((
							positionToVector(pose_from.position)+positionToVector(pose_to.position))*0.5
							+ tf::Vector3(0.,0.,-0.001)
					),
					model->getModelName(),
					0.03,
					0,0,1);
		}
	}
	map<string,MarkerArray> new_markers = markers;
	DeleteOldMarkers();
	old_markers = new_markers;
	MarkerArray all_markers;
	for(map<string,MarkerArray>::iterator i =markers.begin();i!=markers.end();i++) {
		BOOST_FOREACH(Marker& m,i->second.markers) {
			m.header = object.object_msg.header;
			all_markers.markers.push_back(m);
//			cout <<"ns="<<m.ns<<" id="<<m.id<<" action="<<m.action<<" points="<<m.points.size()<<endl;
		}
	}
//	cout <<endl;
	markers.clear();
	marker_pub.publish(all_markers);

	// transform observed tracks into global frame, using estimated pose of latest observed pose
	for(KinematicGraph::iterator i= object.currentGraph.begin();i!=object.currentGraph.end();i++) {
		// note: observations are relative to the first marker
		int from = i->first.first;
//		int to = i.first.second;
		GenericModelPtr &model = i->second;
		model->sampleConfigurationSpace(0.02);

		Pose origin = object.object_msg.parts[from].pose.back();
		BOOST_FOREACH(Pose& p, model->model.track.pose) {
			p = transformToPose(poseToTransform(origin) * poseToTransform( p ));
		}
		BOOST_FOREACH(Pose& p, model->model.track.pose_projected) {
			p = transformToPose(poseToTransform(origin) * poseToTransform( p ));
		}
		BOOST_FOREACH(Pose& p, model->model.track.pose_resampled) {
			p = transformToPose(poseToTransform(origin) * poseToTransform( p ));
		}

		ModelMsg m = model->getModel();
		m.header = object.object_msg.header;
		model_pub.publish(m);
		old_model_ids.push_back(model->model.id);
	}


	response.object = object.GetObjectModel();
	response.object.markers = all_markers;
	return (true);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "structure_learner_server");
	nh = new ros::NodeHandle();
	nh_local = new ros::NodeHandle("~");

	params.LoadParams(*nh_local,false);

	model_pub = nh->advertise<articulation_msgs::ModelMsg> ("model", 0);
	track_pub = nh->advertise<articulation_msgs::TrackMsg> ("track", 0);
	marker_pub = nh->advertise<visualization_msgs::MarkerArray> ("structure_array", 0);
	ros::Publisher marker2_pub = nh->advertise<visualization_msgs::Marker> ("structure", 0);

	ros::ServiceServer fitService = nh->advertiseService("fit_models",
			structureFitModels);
	ros::ServiceServer selectServiceSpanningTree = nh->advertiseService("get_spanning_tree",
			structureSelectSpanningTree);
	ros::ServiceServer selectServiceFastGraph = nh->advertiseService("get_fast_graph",
			structureSelectFastGraph);
	ros::ServiceServer selectServiceGraph = nh->advertiseService("get_graph",
			structureSelectGraph);
	ros::ServiceServer selectServiceGraphAll = nh->advertiseService("get_graph_all",
			structureSelectGraphAll);
	ros::ServiceServer selectVisualizeGraph = nh->advertiseService("visualize_graph",
			visualizeGraph);

	ROS_INFO("Ready to fit articulation models and structure to articulated objects.");
	ros::spin();
}
