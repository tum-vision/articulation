/*
 * hogman_wrapper.cpp
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#include "hogman_minimal/graph_optimizer_hogman/graph_optimizer3d_chol.h"
#include <sstream>

#include "hogman_wrapper.h"
#include "structs.h"
#include "utils.h"
#include "geometry_msgs/Pose.h"
#include "articulation_models/utils.h"

using namespace AISNavigation;
using namespace geometry_msgs;
using namespace articulation_models;

void add_vertex(GraphOptimizer3D* optimizer, int id, tf::Transform pose) {
	Vector6 p;
	double roll, pitch, yaw;
	MAT_to_RPY(pose.getBasis(), roll, pitch, yaw);
	p[0] = pose.getOrigin().x();
	p[1] = pose.getOrigin().y();
	p[2] = pose.getOrigin().z();
	p[3] = roll;
	p[4] = pitch;
	p[5] = yaw;
	Transformation3 t = Transformation3::fromVector(p);
	Matrix6 identity = Matrix6::eye(1.0);
	GraphOptimizer3D::Vertex* v = optimizer->addVertex(id, t, identity);
	if (!v) {
		cerr << "adding vertex id=" << id << "failed" << endl;
	}
}

void add_edge(GraphOptimizer3D* optimizer, int id1, int id2, tf::Transform pose,
		double sigma_position, double sigma_orientation) {
	Vector6 p;
	double roll, pitch, yaw;
	MAT_to_RPY(pose.getBasis(), roll, pitch, yaw);
	p[0] = pose.getOrigin().x();
	p[1] = pose.getOrigin().y();
	p[2] = pose.getOrigin().z();
	p[3] = roll;
	p[4] = pitch;
	p[5] = yaw;

	Matrix6 m = Matrix6::eye(1.0);
	double ip = 1 / SQR(sigma_position);
	double io = 1 / SQR(sigma_orientation);
	m[0][0] = ip;
	m[1][1] = ip;
	m[2][2] = ip;
	m[3][3] = io;
	m[4][4] = io;
	m[5][5] = io;
	GraphOptimizer3D::Vertex* v1 = optimizer->vertex(id1);
	GraphOptimizer3D::Vertex* v2 = optimizer->vertex(id2);
	if (!v1) {
		cerr << "adding edge, id1=" << id1 << " not found" << endl;
	}
	if (!v2) {
		cerr << "adding edge, id2=" << id2 << " not found" << endl;
	}
	Transformation3 t = Transformation3::fromVector(p);
	GraphOptimizer3D::Edge* e = optimizer->addEdge(v1, v2, t, m);
	if (!e) {
		cerr << "adding edge failed" << endl;
	}
}

PoseMap hogman_solver(KinematicGraphType &graph, PoseIndex &poseIndex,
		PoseMap &observed, PoseMap &predictedEmpty, double sigma_position, double sigma_orientation,
		double timestamp) {
	bool visualize = false;
	bool verbose = false;
	bool guess = 0;
	int iterations = 10;

	GraphOptimizer3D* optimizer = new CholOptimizer3D();

	optimizer->verbose() = verbose;
	optimizer->visualizeToStdout() = visualize;
	optimizer->guessOnEdges() = guess;

	optimizer->clear();

	// reference
	tf::Transform root = poseToTransform(observed.begin()->second);

	int n=1000;

	// add predicted markers
	for (PoseMap::iterator it = predictedEmpty.begin(); it != predictedEmpty.end(); it++) {
		tf::Transform m = poseToTransform(it->second);
		add_vertex(optimizer, it->first, root.inverseTimes(m));
	}

	// add observed markers
	for (PoseMap::iterator it = observed.begin(); it != observed.end(); it++) {
		tf::Transform m = poseToTransform(it->second);
		add_vertex(optimizer, it->first + n, root.inverseTimes(m));
	}

	for (KinematicGraphType::iterator it = graph.begin(); it != graph.end(); it++) {
		int idx = poseIndex[it->first.first][it->first.second][timestamp];
		Pose& pose_pred = it->second->model.track.pose_projected[idx];

		ROS_ASSERT(!(isnan(pose_pred.position.x) || isnan(pose_pred.position.y) || isnan(pose_pred.position.z) ||
				isnan(pose_pred.orientation.x) || isnan(pose_pred.orientation.y) || isnan(pose_pred.orientation.z)|| isnan(pose_pred.orientation.w)));
		add_edge(optimizer, it->first.first, it->first.second, poseToTransform(
				pose_pred), sigma_position, sigma_orientation);
	}

	// add strong edges between observed markers (actually, the observed markers should be kept fixed)
	for (PoseMap::iterator it = observed.begin(); it != observed.end(); it++) {
		if (it == observed.begin())
			it++;// skip first
		// relative transformation between marker 0 and marker it
		tf::Transform m = poseToTransform(observed.begin()->second).inverseTimes(
				poseToTransform(it->second));
		add_edge(optimizer, observed.begin()->first + n,
				it->first + n, m, sigma_position / 100,
				sigma_orientation / 100);
	}

	// add weak edges between predicted markers and observed markers (keep them in place)
	for (PoseMap::iterator it = observed.begin(); it != observed.end(); it++) {
		// relative transformation between marker 0 and marker it
		tf::Transform m = tf::Transform::getIdentity();
		add_edge(optimizer, it->first, it->first + n, m,
				sigma_position * 100, sigma_orientation * 100);
	}

	optimizer->initialize(0);
	optimizer->optimize(iterations, false);

	PoseMap predictedMarkers;
	GraphOptimizer3D::VertexIDMap &vertices = optimizer->vertices();
	for (PoseMap::iterator it = predictedEmpty.begin(); it
			!= predictedEmpty.end(); it++) {
		const PoseGraph3D::Vertex* v;
		v= reinterpret_cast<const PoseGraph3D::Vertex*>(vertices[it->first]);
		if (v == 0) {
			cerr << "VERTEX not found!!" << endl;
		}
		Transformation3 t = v->transformation;
		Vector6 p = t.toVector();
		tf::Transform d = tf::Transform(RPY_to_MAT(p[3], p[4], p[5]), tf::Vector3(
				p[0], p[1], p[2]));
		predictedMarkers[it->first] = transformToPose(root * d);
	}

	const PoseGraph3D::Vertex* v = reinterpret_cast<const PoseGraph3D::Vertex*>(vertices[observed.begin()->first + n]);
	Transformation3 t = v->transformation;
	Vector6 p = t.toVector();
	tf::Transform d = tf::Transform(RPY_to_MAT(p[3], p[4], p[5]), tf::Vector3(p[0],
			p[1], p[2]));
	for (PoseMap::iterator it = predictedMarkers.begin(); it
			!= predictedMarkers.end(); it++) {
		// now correct
		it->second = transformToPose(root * d.inverseTimes(root.inverseTimes(
				poseToTransform(it->second))));
	}

	delete optimizer;

	return predictedMarkers;
}

