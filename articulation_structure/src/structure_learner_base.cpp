/*
 * structure_learner_base.cpp
 *
 *  Created on: Jul 25, 2010
 *      Author: sturm
 */

#include "structure_learner_base.h"
#include "utils.h"


KinematicStructureLearner::KinematicStructureLearner():
nh(),nh_local("~") {
	KinematicParams::LoadParams(nh_local);

	model_pub = nh.advertise<ModelMsg> ("model", 0);
	marker_pub = nh.advertise<visualization_msgs::Marker> ("structure", 0);

//	ros::Subscriber pose_sub = nh.subscribe("markers", 100, poseCallback);
	pose_sub1 = nh.subscribe("/marker/1", 5, &KinematicStructureLearner::poseCallback1,this);
	pose_sub2 = nh.subscribe("/marker/2", 5, &KinematicStructureLearner::poseCallback2,this);
	pose_sub3 = nh.subscribe("/marker/3", 5, &KinematicStructureLearner::poseCallback3,this);
	pose_sub4 = nh.subscribe("/marker/4", 5, &KinematicStructureLearner::poseCallback4,this);
}


KinematicGraph KinematicStructureLearner::getSpanningTree(bool increasing) {
	//http://en.wikipedia.org/wiki/Prim%27s_algorithm
	map< int, map<int,GenericModelPtr> > E = models;
	vector<int> V;
	for(map< int, map<int,GenericModelPtr> >::iterator i=E.begin();
			i != E.end();i++) {
		if(!i->second.begin()->second) continue;
		V.push_back(i->first);
	}
	vector<int> V_new;
	KinematicGraph E_new;

	if(V.size()==0) {
		ROS_INFO_STREAM("V.size()==0: empty graph");
		return KinematicGraph();
	}

	V_new.push_back( V[0] );
	while(V_new.size() != V.size()) {
		map<double, pair< pair<int,int>,GenericModelPtr> > E_weighted;

		// find all possible edges
		for(map< int, map<int,GenericModelPtr> >::iterator i=E.begin();
				i != E.end();i++) {
			// i has to be in V_new
			vector< int >::iterator result = find( V_new.begin(), V_new.end(), i->first );
			if (result == V_new.end()) {
				continue;
			}
			for(map<int,GenericModelPtr>::iterator j=i->second.begin();
				j != i->second.end();j++) {

				if(increasing && i->first >= j->first) continue;

				if(!j->second) continue;

				// j not yet in V_new
				vector< int >::iterator result = find( V_new.begin(), V_new.end(), j->first );
				if (result == V_new.end()) {
					E_weighted[ j->second->getBIC() ] =
							pair< pair<int,int>,GenericModelPtr>(
									pair<int,int>(i->first,j->first),j->second );
				}
			}
		}

		if(E_weighted.size()==0) {
			break; // done
		}

		V_new.push_back( E_weighted.begin()->second.first.second);
		E_new.push_back( E_weighted.begin()->second );
	}

	if(V_new.size() != V.size()) {
		ROS_INFO_STREAM("V.size()!=V_new.size(): disconnected subset exists");
		return KinematicGraph();
	}
	E_new.DOF = E_new.getNominalDOFs();
	E_new.evaluate(intersectionOfStamps(),*this,*this);
	return(E_new);
}

void KinematicStructureLearner::sendModels(KinematicGraph E_new) {
	stringstream s;
	size_t id =0;
	for(KinematicGraphType::iterator
			it = E_new.begin(); it != E_new.end();it++) {
		s <<" ["<<it->first.first<<","<<it->first.second<<":"<<
				it->second->getModelName()<<"/"<<it->second->getSamples()<<"]";

		it->second->projectConfigurationToPose();
		it->second->sampleConfigurationSpace( 0.1 );
		ModelMsg msg = it->second->getModel();
		msg.track.id = id;
		msg.id = id;
		msg.header.frame_id = boost::str(boost::format( "/pred/%1%") % it->first.first );
		model_pub.publish(msg);
		id++;
//		it->second->model.track.pose_projected.back()
	}
	static size_t old_ids = 0;
	size_t new_ids = id;
	while(id<old_ids) {
		ModelMsg msg;
		msg.track.id = id;
		msg.id = id;
		msg.header.frame_id = "/pred/1";
		model_pub.publish(msg);
		id++;
	}
	old_ids = new_ids;

	ROS_INFO_STREAM("tree: "<<s.str());
}

void KinematicStructureLearner::sendTreeTransforms(KinematicGraph graph) {
	if (graph.size() == 0)
		return;

	double latestTimestamp = stampedMarker.rbegin()->first;
	PoseMap observation = getObservation(latestTimestamp);
	PoseMap predictionEmpty = getObservation(intersectionOfStamps().back());
	PoseMap prediction = graph.getPrediction(intersectionOfStamps().back(),observation,predictionEmpty,*this,*this);

	// send transform for root node
	static tf::TransformBroadcaster br;
	const PoseStamped &latestPose =
			stampedMarker[latestTimestamp].begin()->second->pose;
	for (PoseMap::iterator it = prediction.begin(); it != prediction.end(); it++) {
		br.sendTransform(tf::StampedTransform(poseToTransform(it->second),
				latestPose.header.stamp, "/camera", boost::str(boost::format(
						"/pred/%1%") % it->first)));
	}
}

void KinematicStructureLearner::saveGraphEval() {
	ofstream in_file;
	string pid = boost::str(boost::format( "%1%") % getpid() );
	in_file.open ((string("")+"graph_eval"+pid+".gnuplot").c_str());
	in_file.precision(10);
	in_file<<"set xlabel \"number of samples\""<<endl;
	in_file<<"set ylabel \"BIC\""<<endl;
	in_file<<"set key top left"<<endl;
	in_file<<"set output \"graph_eval"<<pid<<".eps\""<<endl;
	in_file<<"set terminal postscript eps enh color \"Times\""<<endl;
	in_file<<"set size 1.0,1.0"<<endl;
	in_file<<"plot \\"<<endl;

//	double minBIC = DBL_MAX;
//	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
//		if(it->second.second.size()==0) continue;
//		if(minBIC > it->second.second.rbegin()->second)
//			minBIC = it->second.second.rbegin()->second;
//	}

	// compute segments
	map<int,int> x;
	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		for(std::map<int,EvalStampStruct>::iterator j=it->second.stampEval.begin();j!=it->second.stampEval.end();j++) {
			x[j->first] = j->first;
		}
	}
	if(x.size()==0) return;
	map<int,int>::iterator j=x.begin(); j++;
	map<int,int>::iterator i=x.begin();
	while(j!=x.end()) {
		j->second = i->first;
		i++;
		j++;
	}

//	cout << endl;
	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		int segment = -1;
		int lastX = -1;
//		cout <<"segmenting "<<it->first<<":"<<endl;
		for(std::map<int,EvalStampStruct>::iterator j=it->second.stampEval.begin();j!=it->second.stampEval.end();j++) {
			if(lastX != x[j->first]) {
				segment++;
			}
//			cout <<"x="<<j->first <<" segment="<<segment << endl;;
			lastX = j->first;
			j->second.segment = segment;
			j->second.segment = 0;
		}
		it->second.segments = segment+1;
		it->second.segments = 1;
//		cout <<"-- segments="<<it->second.segments<<endl;
	}

	bool first = true;
	int lt = 1;
	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		if(it->second.stampEval.size()==0) continue;
		if(it->second.bestGraphOnce) continue; // skip all named graphs
		for(int i=0;i<it->second.segments;i++) {
			if(first) first = false; else in_file << ", \\"<< endl;
			in_file<<"'-' with lines ";
			in_file <<" lw 1 lt "<<lt<<" lc rgb '#7f7f7f' t''";
		}
	}
	if(!first) lt++;
	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		if(it->second.stampEval.size()==0) continue;
		if(!it->second.bestGraphOnce) continue; // skip all unnamed graphs
		for(int i=0;i<it->second.segments;i++) {
			if(first) first = false; else in_file << ", \\"<< endl;
			in_file<<"'-' with lines lt "<<lt;
			lt++;
			if(i==0) {
				if(it->second.stampEval.rbegin()->second.bestGraph) {
					in_file <<" lw 5 t'"<<it->first<<"*'";
				} else {
					in_file <<" lw 3 t'"<<it->first<<"'";
				}
			} else {
				if(it->second.stampEval.rbegin()->second.bestGraph) {
					in_file <<" lw 5 t''";
				} else {
					in_file <<" lw 3 t''";
				}
			}
		}
	}
	in_file<<endl<<endl;
	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		if(it->second.stampEval.size()==0) continue;
		if(it->second.bestGraphOnce) continue;
		int lastSeg = 0;
		for( std::map<int,EvalStampStruct>::iterator j = it->second.stampEval.begin();j!=it->second.stampEval.end();j++ ) {
			in_file<< j->first <<"\t"<<j->second.BIC<<endl;
			if(lastSeg != j->second.segment) {
				in_file<<"e"<<endl;
				lastSeg = j->second.segment;
			}
		}
		in_file<<"e"<<endl;
	}
	for(std::map< std::string, EvalStruct>::iterator  it=evalGraph.begin();it!=evalGraph.end();it++) {
		if(it->second.stampEval.size()==0) continue;
		if(!it->second.bestGraphOnce) continue;
		int lastSeg = 0;
		for( std::map<int,EvalStampStruct>::iterator  j = it->second.stampEval.begin();j!=it->second.stampEval.end();j++ ) {
			in_file<< j->first <<"\t"<<j->second.BIC <<endl;
			if(lastSeg != j->second.segment) {
				in_file<<"e"<<endl;
				lastSeg = j->second.segment;
			}
		}
		in_file<<"e"<<endl;
	}

	in_file.close();
}

void saveIntDouble(ofstream &in_file,std::map< int, double> values) {
	for(std::map< int, double>::iterator i=values.begin();i!=values.end();i++) {
		in_file << i->first << " " << i->second<<endl;
	}
	in_file<<"e"<<endl;
}

void KinematicStructureLearner::saveBICEval() {
	ofstream in_file;
	string pid = boost::str(boost::format( "%1%") % getpid() );
	in_file.open ((string("")+"bic_eval"+pid+".gnuplot").c_str());
	in_file.precision(10);
	in_file<<"set xlabel \"number of samples\""<<endl;
	in_file<<"set ylabel \"BIC\""<<endl;
	in_file<<"set key top left"<<endl;
	in_file<<"set output \"bic_eval"<<pid<<".eps\""<<endl;
	in_file<<"set terminal postscript eps enh color \"Times\""<<endl;
	in_file<<"set size 1.0,1.0"<<endl;
	in_file<<"plot \\"<<endl;

	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		if(it->second.stampEval.size()==0) continue;
		in_file<<"'-' with lines  lw 1 lt 1 lc rgb '#7f7f7f' t'', \\"<<endl;
	}

	in_file <<" '-' with lines lw 3 lt 1 lc rgb '#0000ff' t'spanning tree', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 2 lc rgb '#00ff00' t'greedy graph search', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 3 lc rgb '#ff0000' t'brute force graph search';"<<endl;

	for(std::map< std::string, EvalStruct>::iterator it=evalGraph.begin();it!=evalGraph.end();it++) {
		for( std::map<int,EvalStampStruct>::iterator j = it->second.stampEval.begin();j!=it->second.stampEval.end();j++ ) {
			in_file<< j->first <<"\t"<<j->second.BIC<<endl;
		}
		in_file<<"e"<<endl;
	}

	saveIntDouble(in_file,evalTreeBIC);
	saveIntDouble(in_file,evalFastBIC);
	saveIntDouble(in_file,evalGraphBIC);


	in_file.close();
}

void KinematicStructureLearner::saveRuntimeEval() {
	ofstream in_file;
	string pid = boost::str(boost::format( "%1%") % getpid() );
	in_file.open ((string("")+"runtime_eval"+pid+".gnuplot").c_str());
	in_file.precision(10);
	in_file<<"set xlabel \"number of samples\""<<endl;
	in_file<<"set ylabel \"time [s]\""<<endl;
	in_file<<"set key top left"<<endl;
	in_file<<"set output \"runtime_eval"<<pid<<".eps\""<<endl;
	in_file<<"set terminal postscript eps enh color \"Times\""<<endl;
	in_file<<"set size 1.0,1.0"<<endl;
	in_file<<"plot \\"<<endl;

	in_file <<" '-' with lines lw 3 lt 4 lc rgb '#000000' t'local models', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 1 lc rgb '#0000ff' t'spanning tree', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 2 lc rgb '#00ff00' t'greedy graph search', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 3 lc rgb '#ff0000' t'brute force graph search';"<<endl;

	saveIntDouble(in_file,evalModelsRuntime);
	saveIntDouble(in_file,evalTreeRuntime);
	saveIntDouble(in_file,evalFastRuntime);
	saveIntDouble(in_file,evalGraphRuntime);

	in_file.close();
}

void KinematicStructureLearner::saveDOFLinkEval() {
	ofstream in_file;
	string pid = boost::str(boost::format( "%1%") % getpid() );
	in_file.open ((string("")+"doflink_eval"+pid+".gnuplot").c_str());
	in_file.precision(10);
	in_file<<"set xlabel \"number of samples\""<<endl;
	in_file<<"set ylabel \"time [s]\""<<endl;
	in_file<<"set key top left"<<endl;
	in_file<<"set output \"doflink_eval"<<pid<<".eps\""<<endl;
	in_file<<"set terminal postscript eps enh color \"Times\""<<endl;
 	in_file<<"set size 1.0,1.0"<<endl;
	in_file<<"plot \\"<<endl;

	in_file <<" '-' with lines lw 3 lt 4 lc rgb '#000000' t'DOFs', \\"<<endl;
	in_file <<" '-' with lines lw 3 lt 1 lc rgb '#0000ff' t'Links'"<<endl;

	saveIntDouble(in_file,evalDOF);
	saveIntDouble(in_file,evalLinks);

	in_file.close();
}


void KinematicStructureLearner::sendStructureVisualization(KinematicGraph graph) {
	// send transform for root node
	if(!graph.size()) return;

	std::vector<double> stamps = intersectionOfStamps();
	PoseMap observationMarkers = getObservation(stamps.back());
	PoseMap predictionEmpty = getObservation(intersectionOfStamps().back());
	PoseMap predictedMarkers = graph.getPrediction(stamps.back(),observationMarkers,predictionEmpty,*this,*this);

	static tf::TransformListener listener;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/camera";
    marker.header.stamp = ros::Time::now();
    marker.ns = "structure";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    visualization_msgs::Marker text_marker = marker;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = "model_names";
    text_marker.scale.x = 0.035;
    text_marker.scale.y = 0.035;
    text_marker.scale.z = 0.035;

    static int old_marker_id = 0;
    static int old_text_marker_id = 0;
    int model_count=0;
	for(KinematicGraphType::iterator
			it = graph.begin(); it != graph.end();it++)
	{
		double model_r = model_count / (double)graph.size();
//		if(it->first.first > it->first.second) continue;
		if(it->second->model.track.pose_projected.size()==0) continue;

		if(it->second->getModelName() == "rigid") {
			marker.color = HSV_to_RGB(0.0,0.0,model_r/3 + 0.3);
		    marker.id++;
			marker.points.clear();
			marker.points.push_back(predictedMarkers[it->first.first ].position);
			marker.points.push_back(predictedMarkers[it->first.second].position);
		    marker_pub.publish(marker);

		} else
			if(it->second->getModelName() == "prismatic") {
				marker.color = HSV_to_RGB((model_r-0.5)/6 + 0.15,1.0,1.0);
			    marker.id++;
				marker.points.clear();
				marker.points.push_back(predictedMarkers[it->first.first ].position);
				marker.points.push_back(predictedMarkers[it->first.second].position);
			    marker_pub.publish(marker);
			} else
				if(it->second->getModelName() == "rotational") {
					geometry_msgs::Point rot_center = vectorToPosition( poseToTransform(predictedMarkers[it->first.first ])*
							boost::static_pointer_cast<RotationalModel>(it->second)->rot_center);

					marker.color = HSV_to_RGB((model_r-0.5)/6 +  0.66,0.5,0.5);

				    marker.id++;
					marker.points.clear();
					marker.points.push_back(predictedMarkers[it->first.first ].position);
					marker.points.push_back(rot_center );
				    marker_pub.publish(marker);

					marker.color = HSV_to_RGB((model_r-0.5)/6 +  0.66,1.0,1.0);

				    marker.id++;
					marker.points.clear();
					marker.points.push_back(rot_center );
					marker.points.push_back(predictedMarkers[it->first.second].position);
				    marker_pub.publish(marker);

				} else
					if(it->second->getModelName() == "pca_gp") {
						marker.color = HSV_to_RGB((model_r-0.5)/6 +  0.33,1.0,1.0);
					    marker.id++;
						marker.points.clear();
						marker.points.push_back(predictedMarkers[it->first.first ].position);
						marker.points.push_back(predictedMarkers[it->first.second].position);
					    marker_pub.publish(marker);
					} else
					{
						marker.color = HSV_to_RGB(0,0,0);
					    marker.id++;
						marker.points.clear();
						marker.points.push_back(predictedMarkers[it->first.first ].position);
						marker.points.push_back(predictedMarkers[it->first.second].position);
					    marker_pub.publish(marker);
					}


	    text_marker.id++;
	    text_marker.text = it->second->getModelName();
	    text_marker.pose.position.x = (predictedMarkers[it->first.first ].position.x + predictedMarkers[it->first.second ].position.x)/2;
	    text_marker.pose.position.y = (predictedMarkers[it->first.first ].position.y + predictedMarkers[it->first.second ].position.y)/2;
	    text_marker.pose.position.z = (predictedMarkers[it->first.first ].position.z + predictedMarkers[it->first.second ].position.z)/2;
	    text_marker.color = marker.color;
	    marker_pub.publish(text_marker);

	    model_count++;
	}

    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = +0.16;
    text_marker.pose.position.z = 1;
    text_marker.color.r=1.0;
    text_marker.color.g=1.0;
    text_marker.color.b=1.0;
    text_marker.scale.x = 0.025;
    text_marker.scale.y = 0.025;
    text_marker.scale.z = 0.025;
    text_marker.text = boost::str(boost::format( "selected model: %d DOF, %d links") % graph.DOF % graph.size()  );
    text_marker.id++;
    marker_pub.publish(text_marker);
    text_marker.pose.position.y += 0.028;

    text_marker.text = boost::str(boost::format( "global error: %0.3fm, %0.3frad (%d samples)") % graph.avg_pos_err % graph.avg_orient_err % intersectionOfStamps().size());
    text_marker.id++;
    marker_pub.publish(text_marker);
    text_marker.pose.position.y += 0.028;

    text_marker.scale.x = 0.02;
    text_marker.scale.y = 0.02;
    text_marker.scale.z = 0.02;
    text_marker.text = boost::str(boost::format( "kinematic structure: %s") % graph.getTreeName(true,true,false) );
    text_marker.id++;
    marker_pub.publish(text_marker);


	int new_marker_id = marker.id;
	int new_text_marker_id = text_marker.id;
	while(marker.id < old_marker_id) {
		marker.id++;
		marker.action = visualization_msgs::Marker::DELETE;
	    marker_pub.publish(marker);
	}

	while(text_marker.id < old_text_marker_id) {
		text_marker.id++;
		text_marker.action = visualization_msgs::Marker::DELETE;
		marker_pub.publish(text_marker);
	}
	old_marker_id = new_marker_id;
	old_text_marker_id = new_text_marker_id;

}

std::vector<double>  KinematicStructureLearner::downsampleStamps(std::vector<double> vec,size_t num) {
	std::vector<int> idx;
	for(size_t i=0;i<vec.size();i++) idx.push_back(i);
	random_shuffle( idx.begin(), idx.end() );
	if(num<vec.size())
		idx.erase(idx.begin()+num,idx.end());
	sort(idx.begin(),idx.end());
	std::vector<double> res;
	for(size_t i=0;i<vec.size() && i<num;i++)
		res.push_back( vec[ idx[i] ] );
	return(res);
}

int dist(string a,string b) {
	int d = fabs(a.length()-b.length());
	for(size_t i=0;i<a.length() && i<b.length();i++) {
		if(a[i] != b[i]) d++;
	}
	return( d );
}

void KinematicStructureLearner::addPreviousGraphs() {
	cout <<"adding previous graphs, n="<<previousGraphs.size()<<endl;

	for(map< string, KinematicGraph >::iterator it= previousGraphs.begin();it!=previousGraphs.end();it++) {
		if(graphMap.find(it->first) != graphMap.end()) continue;

		KinematicGraph graph(it->second,true);
		bool skip_this = false;
		for(KinematicGraph::iterator j=graph.begin();j!=graph.end();j++) {
			bool updated = false;
			for(std::vector<articulation_models::GenericModelPtr>::iterator
					k=models_all[j->first.first][j->first.second].begin();k!=models_all[j->first.first][j->first.second].end();k++) {
				if(k->get()->getModelName() == j->second->getModelName()) {
					j->second = *k;	// update model pointer
					updated = true;
					break;
				}
			}
			if(!updated) {
				skip_this = true;
				cout <<"skipping model because not all models updatable:"<<graph.getTreeName(true,false,true)<<endl;
				break;
			}
		}
		if(!skip_this) {
			graphMap[ it->first ] = graph;
			cout <<"adding previous "<<graph.getTreeName(true,false,true)<<endl;
		}
	}
}

void KinematicStructureLearner::selectSceneModel() {
	std::vector<double> stamps= intersectionOfStamps();

	if(stamps.size()==0) {
		cout <<"no intersecting timestamps.."<<endl;
		return;
	}

	if(full_eval=="tree") {
		evalTree();
	}

	if(full_eval=="full") {
		evalFull();
	} else

	if(full_eval=="fast") {
		// initialize possible graphs
		evalFast(true);
	} else {
		cerr <<"warning!!! unknown evaluation type: full_eval="<<full_eval<<endl;
	}
}

void KinematicStructureLearner::evalTree() {
	ros::Time t = ros::Time::now();

	std::vector<double> stamps= intersectionOfStamps();
	currentGraph = getSpanningTree(true);
	currentGraph.evaluate(stamps,*this,*this);
	evalTreeRuntime[stamps.size()] = (ros::Time::now() - t).toSec();

	evalDOF[stamps.size()] = currentGraph.DOF;
	evalLinks[stamps.size()] = currentGraph.size();

	sendTreeTransforms(currentGraph);

	evalGraph[currentGraph.getTreeName(true,false,true)].stampEval[stamps.size()].bestTree = true;
	evalGraph[currentGraph.getTreeName(true,false,true)].stampEval[stamps.size()].BIC = currentGraph.BIC;
	evalTreeBIC[stamps.size()] = currentGraph.BIC;
	saveGraphEval();
	saveBICEval();
	saveRuntimeEval();

}

void KinematicStructureLearner::evalFull() {
	ros::Time t = ros::Time::now();

	std::vector<double> stamps= intersectionOfStamps();
	KinematicGraph tree;

	KinematicGraph bestGraph;
	enumerateGraphs();
	addPreviousGraphs();

	for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
		KinematicGraph& copyOfTree = j->second;
		string gname_short = copyOfTree.getTreeName(false,false,true);
		string gname_models = copyOfTree.getTreeName(true,false,true);
		string gname_models_samples = copyOfTree.getTreeName(true,false,true);
		copyOfTree.evaluate(stamps,*this,*this);
		if(evalGraph.find(gname_models) == evalGraph.end()) {


			// spawn this new model from somewhere
			// find a model that has the previous time slice defined
			string previous_model;
			int previous_time=0;
			for(std::map< std::string, EvalStruct>::iterator k=evalGraph.begin();k != evalGraph.end();k++) {
				int t = 0;
				for(std::map<int,EvalStampStruct>::iterator l=k->second.stampEval.begin();l!=k->second.stampEval.end();l++) {
					if(l->first > t && l->first < (int)stamps.size())
						t = l->first;
				}
				if( k->second.stampEval.find(t) != k->second.stampEval.end() && (t>previous_time || dist(previous_model,gname_models) > dist(k->first,gname_models)) ) {
					previous_model = k->first;
					previous_time = t;
				}
			}
			if(previous_model.length()) {
				evalGraph[gname_models].stampEval[previous_time] = evalGraph[previous_model].stampEval[previous_time];
			}

		}
		evalGraph[gname_models].stampEval[stamps.size()].BIC = copyOfTree.BIC;

		if(copyOfTree.BIC < bestGraph.BIC) {
			bestGraph = copyOfTree;
		}
		cout <<"adding "<<copyOfTree.getTreeName(true,true,true)<<endl;;

		// should we finish?
		static ros::NodeHandle n;
		if(!n.ok()) return;

	}

	currentGraph = bestGraph;
	if(currentGraph.DOF != -1) {
		string gname_models = currentGraph.getTreeName(true,false,true);
		evalGraph[gname_models].stampEval[stamps.size()].BIC = currentGraph.BIC;
		cout <<"*** STORING evalGraph"<<gname_models<<" bic" <<currentGraph.BIC<<endl;
		evalGraph[gname_models].stampEval[stamps.size()].bestGraph = true;
		evalGraph[gname_models].bestGraphOnce= true;

		evalDOF[stamps.size()] = currentGraph.DOF;
		evalLinks[stamps.size()] = currentGraph.size();

		previousGraphs[currentGraph.getTreeName(true,false,true)] = currentGraph;

		evalGraphBIC[stamps.size()] = currentGraph.BIC;
	}
	evalGraphRuntime[stamps.size()] = (ros::Time::now() - t).toSec();

	showEvaluation();
//	analyzeTopologyOfGraphs();

	evalFast(false);

	saveGraphEval();
	saveBICEval();
	saveRuntimeEval();
	saveDOFLinkEval();
}

void KinematicStructureLearner::evalFast(bool setCurrent) {
	ros::Time t = ros::Time::now();

	std::vector<double> stamps= intersectionOfStamps();
	if(setCurrent)
		enumerateGraphs();

	// initialize with spanning tree
	KinematicGraph tree = getSpanningTree(true);
	tree.evaluate(stamps,*this,*this);
	evalTreeRuntime[stamps.size()] = (ros::Time::now() - t).toSec();

	cout <<"spanning tree: "<<tree.getTreeName(true,false,true)<<endl;
	// find it graph list (ordering might be different, but topological distance should be zero)
	string current;
	for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
//				cout <<"  "<<j->second.getTreeName(true,false,true)<< " "<< tree.distanceTo(j->second)<<endl;
		if(tree.distanceTo(j->second)==0) {
			current = j->second.getTreeName(true,false,true);
			break;
		}
	}
	cout <<"starting from current graph: "<<current<<endl;
	cout <<"finding neighbors, total graphs = "<<graphMap.size()<<endl;

	if(graphMap.find(current)==graphMap.end()) {
		cout <<"current graph is not in graphMap???"<<endl;
		return;
	}
//	if(setCurrent)
	graphMap[current].evaluate(stamps,*this,*this);
	evalGraph[current].stampEval[stamps.size()].BIC = graphMap[current].BIC;
	evalGraph[current].stampEval[stamps.size()].bestTree = true;
	evalGraph[current].stampEval[stamps.size()].evalInFastSearch = true;
	evalTreeBIC[stamps.size()] = graphMap[current].BIC;


//	cout <<"pre computed BIC of tree is "<<evalGraph[current].stampEval[stamps.size()].BIC<<endl;
//	graphMap[current].evaluate(stamps,*this,*this);
//	cout <<"now computed BIC of tree is "<<graphMap[current].BIC<<endl;

	int evals = 0;
	cout <<"  starting   "<<graphMap[current].BIC<<"  "<<current<<" pos="<<graphMap[current].avg_pos_err<<" orient="<<graphMap[current].avg_orient_err<< endl;
	string previous;
	while(current!=previous) {
		previous = current;


		for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
			if(graphMap[current].distanceTo(j->second)==1) {
				evals ++;
//				if(setCurrent){
				j->second.evaluate(stamps,*this,*this);
				evalGraph[j->second.getTreeName(true,false,true)].stampEval[stamps.size()].BIC = j->second.BIC;
//				}
				evalGraph[j->second.getTreeName(true,false,true)].stampEval[stamps.size()].evalInFastSearch = true;
				cout <<"  evaluating "<< j->second.BIC<<" ("<< evalGraph[j->second.getTreeName(true,true,true)].stampEval[stamps.size()].BIC <<"): "<<j->second.getTreeName(true,true,true)<<" pos="<<j->second.avg_pos_err<<" orient="<<j->second.avg_orient_err<< endl;
				if(j->second.BIC < graphMap[current].BIC) {
					current = j->second.getTreeName(true,false,true);
					break;
				}
			} else {
//					cout <<"  not evaluating " <<j->second.getTreeName(true,true,true)<<" dist="<< graphMap[current].distanceTo(j->second)<< endl;
			}
		}
		cout <<graphMap[current].BIC<<"  "<<current<<endl;
	}
	evalGraph[current].stampEval[stamps.size()].bestFast = true;
	evalFastBIC[stamps.size()] = graphMap[current].BIC;

	cout <<"final:  "<<graphMap[current].BIC<<"  "<<current<<" pos="<<graphMap[current].avg_pos_err<<" orient="<<graphMap[current].avg_orient_err<< " stamps="<<stamps.size()<<endl;
	cout <<" evals: "<<evals;
	cout << endl;

	if(setCurrent) {
		currentGraph = graphMap[current];
		evalDOF[stamps.size()] = currentGraph.DOF;
		evalLinks[stamps.size()] = currentGraph.size();

		saveGraphEval();
		saveBICEval();
		saveRuntimeEval();
		saveDOFLinkEval();
	}
	evalFastRuntime[stamps.size()] = (ros::Time::now() - t).toSec();
}

void KinematicStructureLearner::enumerateGraphs() {
//	cout <<"enumerating"<<endl;
	graphMap.clear();
	KinematicGraph tree;
	map<int,int> marker_to_vertex;
	map<int,int> vertex_to_marker;
	int n=0;
	for(map<int,double>::iterator it = latestTimestamp.begin();
			it != latestTimestamp.end(); it++ ) {
		marker_to_vertex[it->first] = n;
		vertex_to_marker[n] = it->first;
		n++;
	}

	int vertices = models.size();
	int edges = vertices*vertices;

//	cout <<"vertices="<<vertices<<endl;
//	cout <<"edges="<<edges<<endl;
	for(long graph=1; graph < (1<<(edges-1)); graph++) {
		// iterate over all possible graphs
		std::vector<bool> connected(vertex_to_marker.size(),false);
		connected[0] = true;
		tree.clear();
		for(int e=0;e<edges;e++) {
			if((graph & (1<<e))>0) {
				// edge is present in this particular graph
				int e_from = vertex_to_marker[e / vertices];
				int e_to = vertex_to_marker[e % vertices];
				if(e_from>=e_to){
					tree.clear();
					break;
				}
				connected[e % vertices] = connected[e / vertices];
				if(!models[e_from][e_to]) {
					tree.clear();
					break;
				}
				if(!models[e_to][e_from]) {
					tree.clear();
					break;
				}
				tree.push_back(pair< pair<int,int>,GenericModelPtr>(pair<int,int>(e_from,e_to),models[e_from][e_to]));
//					tree.push_back(pair< pair<int,int>,GenericModelPtr>(pair<int,int>(e_to,e_from),models[e_to][e_from]));
			}
		}
		tree.DOF = tree.getNominalDOFs();// compute sum
		if((int)tree.size()<vertices-1) continue;
		if( restrict_graphs ) {
			if(
					(restricted_graphs.find(" "+tree.getTreeName(false,false,false)+" ")==restricted_graphs.npos) &&
					(restricted_graphs.find(" "+tree.getTreeName(true,false,false)+" ")==restricted_graphs.npos) &&
					(restricted_graphs.find(" "+tree.getTreeName(true,true,false)+" ")==restricted_graphs.npos)  &&
					(restricted_graphs.find(" "+tree.getTreeName(true,true,true)+" ")==restricted_graphs.npos) )
				continue;
		}

		bool allConnected = true;
		for(size_t i=0;i<connected.size();i++) {
			if(!connected[i]) allConnected=false;
		}
		if(allConnected) {
			// generate tree with all possible link models
			// find the number of possible combinations
			KinematicGraph basetree = tree;
			tree.clear();
			int combination_total = 1;
			for(KinematicGraphType::iterator it=basetree.begin();it!=basetree.end();it++) {
				combination_total *= models_all[it->first.first][it->first.second].size();
			}
//			cout <<"combination_total="<<combination_total<<endl;
			for(int combination=0;combination<combination_total;combination++) {
				if(search_all_models) {
					tree.clear();
					int c_tmp = combination;
					for(KinematicGraphType::iterator it=basetree.begin();it!=basetree.end();it++) {
						int idx = c_tmp % models_all[it->first.first][it->first.second].size();
						tree.push_back( pair< pair<int,int>,GenericModelPtr> ( pair<int,int>(it->first.first,it->first.second), models_all[it->first.first][it->first.second][idx]));
						c_tmp /= models_all[it->first.first][it->first.second].size();
					}
				} else {
					tree = basetree;
				}

				int DOFs = tree.getNominalDOFs();// compute sum
				tree.DOF = DOFs;
//				cout <<"DOFs="<<DOFs<<endl;
				for(int reducedDOFs=(reduce_dofs?(DOFs==0?0:1):DOFs);reducedDOFs<=DOFs;reducedDOFs++) {
					KinematicGraph copyOfTree(tree,true);
					copyOfTree.DOF = reducedDOFs;
					graphMap[copyOfTree.getTreeName(true,false,true)] = KinematicGraph(copyOfTree,true);
				}
				if(!search_all_models) break;
			}
		}
	}
}

void KinematicStructureLearner::showEvaluation() {
	cout <<"Evaluation @ t="<<intersectionOfStamps().size()<<endl;
	map<double, KinematicGraph> sorted;
	for(map< string, KinematicGraph >::iterator it = graphMap.begin();it!=graphMap.end();it++) {
		sorted[it->second.BIC] = it->second;
	}
	for(map<double, KinematicGraph>::iterator it = sorted.begin();it!=sorted.end();it++) {
		cout << it->first <<"      " << it->second.getTreeName(true,true,true)<<" pos="<<it->second.avg_pos_err<<" orient="<<it->second.avg_orient_err<< endl;
	}
	cout <<"---"<<endl;
}

void KinematicStructureLearner::sendSceneModel() {
	// need to update models first!!
	sendTreeTransforms(currentGraph);
	sendModels(currentGraph);
	sendStructureVisualization(currentGraph);
}

void KinematicStructureLearner::poseCallback(const PoseStampedIdMsgConstPtr& pose) {
//	ROS_INFO("Received pose [%d]", pose->id);
	boost::mutex::scoped_lock a(frame_mutex_);
	double stamp = pose->pose.header.stamp.toSec();
	size_t id = pose->id;

	// compare to previous markers, skip if too close (bad perception)
	tf::Transform seen_marker = poseToTransform(pose->pose.pose);
	for (map<int, map<double, PoseStampedIdMsgConstPtr> >::iterator it =
			markerStamped.begin(); it != markerStamped.end(); it++) {
		if((size_t)it->first == id) continue;
		if(it->second.size()==0) continue;

		Pose p;
		p = it->second.rbegin()->second->pose.pose;
		tf::Transform other_marker = poseToTransform( p );
		tf::Transform diff_to_known = seen_marker.inverseTimes( other_marker);
		double diff_position = diff_to_known.getOrigin().length();
//		double diff_orientation = fabs(diff_to_known.getRotation().getAngle());
//		if(diff_position < sigma_position/2 && diff_orientation < sigma_orientation/2) {
		if(diff_position < sigma_position) {
			cout <<"skipping false positive marker detection"<<endl;
			return;
		}

	}



	// lookup any existing data
	for (map<int, PoseStampedIdMsgConstPtr>::iterator it =
			stampedMarker[stamp].begin(); it != stampedMarker[stamp].end(); it++) {
		if (it->second->id == id)
			continue;
		addPose(pose->pose, it->second->pose, id, it->second->id,*this);
		addPose(it->second->pose, pose->pose, it->second->id, id,*this);

		static tf::TransformBroadcaster br;
		br.sendTransform( tf::StampedTransform(
				poseToTransform( pose->pose.pose ),
				pose->pose.header.stamp,
				"/camera",
				boost::str(boost::format( "/marker/%1%") % pose->id)
		));

	}

	stampedMarker[stamp][pose->id] = pose;
	markerStamped[pose->id][stamp] = pose;
}

void KinematicStructureLearner::poseCallback(const PoseStampedConstPtr& pose, size_t id) {
	latestTimestamp[id] = pose->header.stamp.toSec();

	bool invalid = false;
	if(pose->pose.position.x == 0 && pose->pose.position.y == 0 && pose->pose.position.z == 0 &&
			pose->pose.orientation.x == 0 && pose->pose.orientation.y == 0 && pose->pose.orientation.z == 0 && pose->pose.orientation.w == 0
	) invalid = true;

	if(!invalid) {
		PoseStampedIdMsgPtr p = boost::make_shared<PoseStampedIdMsg>();
		p->pose = *pose;
		p->id = id;
		poseCallback(p);
	}

	bool allDone = true;
//	cout<<setprecision(15) <<"process? "<<latestTimestamp.begin()->second;
	for(map<int, double >::iterator it=latestTimestamp.begin();it!=latestTimestamp.end();it++) {
		if(latestTimestamp.begin()->second != it->second) allDone = false;
//		cout <<" "<<it->second;
	}
//	cout << endl;

//	cout <<"intersections="<<intersectionOfStamps().size()<<endl;
	static size_t last_stamp = 0;
	if(allDone) {
		int a = ((int)pow(intersectionOfStamps().size(),1.00/eval_every_power));
		int b = (((int)pow(last_stamp,1.00/eval_every_power)));
		if ( (a%eval_every == 0)
				&& ( a>b ) ) {

			last_stamp = intersectionOfStamps().size();
			b = (((int)pow(last_stamp,1.00/eval_every_power)));
			cout << "stamps="<<intersectionOfStamps().size()<<endl;
			ros::Time t1 = ros::Time::now();
			cout << "updating models.."<<endl;
			//update all models
			for(map< int, map<int,TrackMsgPtr> >::iterator i=trajectories.begin();
				i != trajectories.end();i++) {
				for(map<int,TrackMsgPtr >::iterator j = i->second.begin();
					j != i->second.end();j++) {
//						if(i->first < j->first) {
							updateModel(i->first,j->first,*this);
//						}
				}
			}
			ros::Time t2 = ros::Time::now();
			evalModelsRuntime[last_stamp] = (t2-t1).toSec();

			cout << "selecting scene model.."<<endl;
			selectSceneModel();
			ros::Time t3 = ros::Time::now();
			cout <<"updating models took "<< (t2-t1).toSec() <<"s, graph selection took "<<(t3-t2).toSec()<<"s"<<endl;
			sendSceneModel();
		}
	}
}


void KinematicStructureLearner::poseCallback1(const PoseStampedConstPtr& pose) {
	poseCallback( pose,1 );
}

void KinematicStructureLearner::poseCallback2(const PoseStampedConstPtr& pose) {
	poseCallback( pose,2);
}

void KinematicStructureLearner::poseCallback3(const PoseStampedConstPtr& pose) {
	poseCallback( pose,3 );
}

void KinematicStructureLearner::poseCallback4(const PoseStampedConstPtr& pose) {
	poseCallback( pose,4 );
}

void KinematicStructureLearner::analyzeTopologyOfGraphs() {
	// does every node have a neighbor that has a lower BIC? (except for the optimal one?)
	map<string,bool> isHead;
	for(map< string, KinematicGraph >::iterator i=graphMap.begin();i!=graphMap.end();i++) {
		// consider node i, might be a head in the graph (no incoming arrows)
		isHead[i->second.getTreeName(true,false,true)] = true;
		for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
			// is node j a neighbor?
//			cout << i->second.getTreeName(true,false,true) <<" -- " << j->second.getTreeName(true,false,true) << " dist="<<i->second.distanceTo(j->second)<<endl;
			if(i->second.distanceTo(j->second)<=1) {
				// has the neighbor a lower BIC?
				if(j->second.BIC < i->second.BIC) {
					isHead[i->second.getTreeName(true,false,true)] = false;	// we have a better neighbor
				}
			}
		}
	}

	// now check for heads
	cout <<"found heads: ";//<<isHead.size()<<endl;
	for(map<string,bool>::iterator i=isHead.begin();i!=isHead.end();i++) {
		if(i->second) {
			cout <<"  "<<i->first;
			if(i->first == currentGraph.getTreeName(true,false,true) ) {
				cout <<"*";
			}
			cout<<endl;
		}
	}
	cout <<"---"<<endl;

	// would we find the optimium, starting from the spanning tree and using a greedy strategy?
	KinematicGraph tree = getSpanningTree(true);
	string current = tree.getTreeName(true,false,true);
	string previous;
	cout <<"spanning tree: "<<current<<endl;
	// find it in list
	for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
		if(tree.distanceTo(j->second)==0) {
			current = j->first;
			cout <<"distance zero. "<<current<<endl;
			break;
		}
	}
	cout <<"(modified) spanning tree: "<<current<<endl;

	if(graphMap.find(current)==graphMap.end()) {
		cout <<"spanning tree not in graphMap???"<<endl;
		return;
	}
	int evals = 0;
	cout <<graphMap[current].BIC<<"  "<<current<<endl;
	while(current!=previous) {
		previous = current;
		for(map< string, KinematicGraph >::iterator  j=graphMap.begin();j!=graphMap.end();j++) {
			if(graphMap[current].distanceTo(j->second)<=1) {
				evals ++;
				if(j->second.BIC < graphMap[current].BIC) {
					current = j->second.getTreeName(true,false,true);
					break;
				}
			}
		}
		cout <<graphMap[current].BIC<<"  "<<current<<endl;
	}
	cout <<"finished: ";
	if(current == currentGraph.getTreeName(true,false,true) ) {
		cout <<" optimal!!";
	} else {
		cout <<" not optimal!!";
		cout <<"currently best graph is: "<<currentGraph.getTreeName(true,false,true);
	}
	cout <<" evals: "<<evals;
	cout << endl;
}
