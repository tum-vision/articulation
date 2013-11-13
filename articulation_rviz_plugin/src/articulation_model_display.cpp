#include <boost/format.hpp>
#include "articulation_model_display.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <ogre_tools/axes.h>

#include <tf/transform_listener.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "utils.h"
#include "articulation_models/utils.h"

using namespace std;

namespace articulation_rviz_plugin {

ModelDisplay::ModelDisplay(const std::string& name,
		rviz::VisualizationManager* manager) :
	Display(name, manager), model_topic_("/model"), unique_(u_track),color_(0.5f, 0.0f, 1.0f),
			alpha_(0.5f), lineWidth_(0.003f), trackColor_(cs_fixed),modelColor_(cs_fixed),
					poseColor_(cs_fixed), modelTypeColor_(cs_fixed), likelihoodColor_(cs_fixed), forceColor_(cs_fixed), displayStyle_(ds_line),
			sampleConfigurations_(true), projectConfigurations_(true),
			showJacobians_(false), showHessians_(false), showLatestOnly_(false),
			sampleDensity_(0.01), showModelName_(false),showForces_(false),forceScaling_(0.01),tf_filter_(*manager->getTFClient(), "",
					1000, update_nh_) {
	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	tf_filter_.connectInput(sub_);
	tf_filter_.registerCallback(boost::bind(&ModelDisplay::incomingModel, this,
			_1));
}

ModelDisplay::~ModelDisplay() {
	unsubscribe();

}

void ModelDisplay::clearMap() {
	std::map<int, std::vector<ogre_tools::BillboardLine*> >::iterator it;
	for (it = lines.begin(); it != lines.end(); it++) {
		clearVector(it->second);
	}
	std::map<int, std::vector<ogre_tools::Arrow*> >::iterator it2;
	for (it2 = arrows.begin(); it2 != arrows.end(); it2++) {
		clearVector(it2->second);
	}
	std::map<int, std::vector<ogre_tools::MovableText*> >::iterator it3;
	for (it3 = text.begin(); it3 != text.end(); it3++) {
		clearVector(it3->second);
	}
}

void ModelDisplay::clearVector(std::vector<ogre_tools::BillboardLine*>& vec) {
	for (size_t j = 0; j < vec.size(); j++) {
		delete vec[j];
	}
	vec.clear();
}

void ModelDisplay::clearVector(std::vector<ogre_tools::Arrow*>& vec) {
	for (size_t j = 0; j < vec.size(); j++) {
		delete vec[j];
	}
	vec.clear();
}

void ModelDisplay::clearVector(std::vector<ogre_tools::MovableText*>& vec) {
	for (size_t j = 0; j < vec.size(); j++) {
		delete vec[j];
	}
	vec.clear();
}

void ModelDisplay::setTopic(const std::string& topic) {
	unsubscribe();
	model_topic_ = topic;
	subscribe();

	propertyChanged(topic_property_);
}

void ModelDisplay::onEnable() {
	subscribe();

	scene_node_->setVisible(true);
}

void ModelDisplay::onDisable() {
	unsubscribe();
	scene_node_->setVisible(false);
	clearDisplay();
}

void ModelDisplay::reset() {
	clearDisplay();
}

void ModelDisplay::subscribe() {
	if (!isEnabled()) {
		return;
	}

	if (!model_topic_.empty()) {
		sub_.subscribe(update_nh_, model_topic_, 0);
	}

}

void ModelDisplay::unsubscribe() {
	sub_.unsubscribe();
}

void ModelDisplay::incomingModel(
		const articulation_msgs::ModelMsg::ConstPtr& msg) {
	incoming_model_message_ = msg;
	boost::mutex::scoped_lock lock(queue_mutex_);

	message_queue_.push_back(msg);
}

void ModelDisplay::clearResources(int uniqueId) {
	for (size_t l = 0; l < lines[uniqueId].size(); l++)
		lines[uniqueId][l]->clear();
	recycleLines.insert(recycleLines.begin(),
			lines[uniqueId].begin(),
			lines[uniqueId].end());
	lines[uniqueId].clear();

	for (size_t l = 0; l < arrows[uniqueId].size(); l++)
		arrows[uniqueId][l]->set(0,0,0,0);
	recycleArrows.insert(recycleArrows.begin(),
			arrows[uniqueId].begin(),
			arrows[uniqueId].end());
	arrows[uniqueId].clear();

	for (size_t l = 0; l < text[uniqueId].size(); l++) {
		text[uniqueId][l]->setCaption("-");
		text[uniqueId][l]->setVisible(false);
	}
	recycleText.insert(recycleText.begin(),
			text[uniqueId].begin(),
			text[uniqueId].end());
	text[uniqueId].clear();
}

void ModelDisplay::update(float wall_dt, float ros_dt) {
	V_ModelMsg local_queue;

	{
		boost::mutex::scoped_lock lock(queue_mutex_);

		local_queue.swap(message_queue_);
	}

	if(unique_ == u_modelstored) clearResources(-1);

	for (size_t t = 0; t < local_queue.size(); t++) {
		const articulation_msgs::ModelMsg::ConstPtr& model_message_orig =
				local_queue[t];

		int uniqueId;
		static int uniqueCounter=0;
		switch(unique_) {
		case u_all: uniqueId=uniqueCounter++;break;
		case u_single: uniqueId=0;break;
		case u_track: uniqueId=model_message_orig->track.id;break;
		case u_model: uniqueId=model_message_orig->id;break;
		case u_modelstored: uniqueId=model_message_orig->id;break;
		case u_modeltype: uniqueId=factory.getModelIndex(model_message_orig->name); break;
		}

		clearResources(uniqueId);

		btTransform framePose;
		btVector3 lineScale(lineWidth_, lineWidth_, lineWidth_);
		transform(model_message_orig, framePose);

		// restore model
		articulation_models::GenericModelPtr model = factory.restoreModel(model_message_orig);
		if(!model) continue;

		articulation_msgs::ModelMsg model_message = model->getModel();

		int channel_w = articulation_models::openChannel(model_message.track,"width",false);
		int channel_h = articulation_models::openChannel(model_message.track,"height",false);
		int channel_logli = articulation_models::openChannel(model_message.track,"loglikelihood",false);
		int force_x = articulation_models::openChannel(model_message.track,"force.x",false);
		int force_y = articulation_models::openChannel(model_message.track,"force.y",false);
		int force_z = articulation_models::openChannel(model_message.track,"force.z",false);
		int force_mag = articulation_models::openChannel(model_message.track,"force",false);

		float avg_w = lineWidth_*5;
		float avg_h = lineWidth_*5;
		if(channel_w!=-1) {
			for(size_t i=0;i<model_message.track.channels[channel_w].values.size();i++) {
				avg_w +=model_message.track.channels[channel_w].values[i];
			}
			avg_w /= model_message.track.channels[channel_w].values.size();
		}
		if(channel_h!=-1) {
			for(size_t i=0;i<model_message.track.channels[channel_w].values.size();i++) {
				avg_h +=model_message.track.channels[channel_h].values[i];
			}
			avg_h /= model_message.track.channels[channel_w].values.size();
		}

		Ogre::Vector3 old_pos(0, 0, 0);

		std::vector<geometry_msgs::Pose> pose;
		if(sampleConfigurations_)
			pose = model_message.track.pose_resampled;
		else if(projectConfigurations_)
			pose = model_message.track.pose_projected;
		else
			pose =  model_message.track.pose;

		btTransform rectangle_pose(btQuaternion(0,0,0,1),btVector3(0,0,0));
		Ogre::Vector3 pos, scale;
		btVector3 color;
		Ogre::Quaternion orient;
		bool isConnected = false;
		size_t start_from=0;
		if (pose.size()>0 && showLatestOnly_) start_from = pose.size()-1;
		for (size_t i = start_from; i < pose.size(); i++) {
			if(!sampleConfigurations_ &&
				(model_message.track.pose_flags[i] & articulation_msgs::TrackMsg::POSE_VISIBLE)==0 ) {
				isConnected = false;
				continue;	// skip, because invisible
			}
			const geometry_msgs::Pose& geo_pose = pose[i];

			rectangle_pose = btTransform(btQuaternion(geo_pose.orientation.x,
					geo_pose.orientation.y, geo_pose.orientation.z,
					geo_pose.orientation.w), btVector3(geo_pose.position.x,
					geo_pose.position.y, geo_pose.position.z));

			transform(framePose * rectangle_pose, lineScale, pos, orient, scale);

			color = btVector3(color_.r_, color_.g_, color_.b_); // fixed color

			double f = model_message.track.id / 11.0;

			color = modifyColor(color, trackColor_, f - floor(f));

			double f2 = model_message.id / 11.0;

			color = modifyColor(color, modelColor_, f2 - floor(f2));

			color = modifyColor(color, poseColor_, i
					/ (double) pose.size());

			color = modifyColor(color, modelTypeColor_,
					factory.getModelIndex(model_message.name)/(double)factory.getFactoryCount());

			if(channel_logli >=0 && !sampleConfigurations_) {
				double logli = model_message.track.channels[ channel_logli ].values[i];
				double chi2_70 = 2.41;// alpha=70%, N=2
				double chi2_95 = 5.99;// alpha=95%, N=2
				double chi2_99 = 9.21;// alpha=99%, N=2
				double chi2_999 = 13.816;// alpha=99.9%, N=2

				double loglikelihood_ellipse_70=
						- log(2*M_PI * model->sigma_position*model->sigma_orientation)
						- 0.5*( chi2_70 );
				double loglikelihood_ellipse_95 =
						- log(2*M_PI * model->sigma_position*model->sigma_orientation)
						- 0.5*( chi2_95 );
				double loglikelihood_ellipse_99 =
						- log(2*M_PI * model->sigma_position*model->sigma_orientation)
						- 0.5*( chi2_99 );
				double loglikelihood_ellipse_999 =
						- log(2*M_PI * model->sigma_position*model->sigma_orientation)
						- 0.5*( chi2_999 );

				double ratio  = (logli - loglikelihood_ellipse_70)/(loglikelihood_ellipse_99 - loglikelihood_ellipse_70);
				// ratio is 0 when on ellipse
				// ratio is >0 when inside ellipse
				// ratio is <0 when outside ellipse

//				cout << "logli="<<logli<<" ellipse_95="<<loglikelihood_ellipse_95<<" ellipse_99="<<loglikelihood_ellipse_99<<" ratio="<<ratio << endl;

				double col = max(0.00,min(1.00,1-ratio));
				if(likelihoodColor_ == cs_hue) {
					col /= 3;
				}
				color = modifyColor(color, likelihoodColor_, col);
			}

			if (force_mag!=-1) {
				double force = model_message.track.channels[force_mag].values[i];
				if(forceColor_ == cs_hue)
					color = modifyColor(color, forceColor_, 1.00/3.00 - max(0.00,min(1.00,force/30.0))/3);
				else
					color = modifyColor(color, forceColor_, max(0.00,min(1.00,force/30.0)));
			} else
			if(force_x!=-1 && force_y!=-1 && force_z!=-1) {
				double force = sqrt(
						SQR(model_message.track.channels[force_x].values[i]) +
						SQR(model_message.track.channels[force_y].values[i]) +
						SQR(model_message.track.channels[force_z].values[i]) );
				if(forceColor_ == cs_hue)
					color = modifyColor(color, forceColor_, 1.00/3.00 - max(0.00,min(1.00,force/30.0))/3);
				else
					color = modifyColor(color, forceColor_, max(0.00,min(1.00,force/30.0)));
			}

			if (displayStyle_ == ds_line) {
				if (isConnected)
					createLine(pos, old_pos, scale, color, lines[uniqueId],false,true);
				old_pos = pos;
			} else if(displayStyle_ == ds_cross_line) {
				if (isConnected)
					createLine(pos, old_pos, scale, color, lines[uniqueId],true,true);
				old_pos = pos;
			} else if(displayStyle_ == ds_cross) {
				if (isConnected)
					createLine(pos, old_pos, scale, color, lines[uniqueId],true,false);
				old_pos = pos;
			} else if(displayStyle_ == ds_arrow) {
				if (isConnected)
					createArrow(pos, old_pos, scale, color, arrows[uniqueId]);
				old_pos = pos;
			} else if (displayStyle_ == ds_axes) {
				createAxes(pos, orient, scale, color, lines[uniqueId]);
//				cout << " adding axis:"<<i<<" pos.x="<<pos.x<<" pos.y="<<pos.y<<" pos.z="<<pos.z<<endl;
			} else if (displayStyle_ == ds_rectangle) {
				float w = avg_w;
				float h = avg_h;
				if(channel_w != -1) {
					if(!sampleConfigurations_ && !projectConfigurations_) {
						w=model_message.track.channels[channel_w].values[i];
						h=model_message.track.channels[channel_h].values[i];
					}
				}
//				cout <<"adding rectangle "<<i<<" w="<<w<<
//						"h="<<h<<endl;
				createRectangle(
						pos,
						orient,
						scale,
						w,h,
						color, lines[uniqueId]);
			}
			if(sampleConfigurations_)
				isConnected = true;
			else
				isConnected = (model_message.track.pose_flags[i] & articulation_msgs::TrackMsg::POSE_END_OF_SEGMENT)==0;

			if(showForces_ && force_x!=-1 && force_y!=-1 && force_z!=-1) {
				Ogre::Vector3 pos2, scale2;
				rectangle_pose = btTransform(
					btQuaternion(
						geo_pose.orientation.x,
						geo_pose.orientation.y,
						geo_pose.orientation.z,
						geo_pose.orientation.w),
					btVector3(
							geo_pose.position.x + model_message.track.channels[force_x].values[i]*forceScaling_,
							geo_pose.position.y + model_message.track.channels[force_y].values[i]*forceScaling_,
							geo_pose.position.z + model_message.track.channels[force_z].values[i]*forceScaling_));

				transform(framePose * rectangle_pose, lineScale, pos2, orient, scale2);

				createArrow(pos2, pos, scale2, color, arrows[uniqueId]);
			}

		}

		// draw jacobian?
		if(showJacobians_ && pose.size()>0 && model->getDOFs()>0) {
			geometry_msgs::Pose geo_pose;
			if(projectConfigurations_)
				geo_pose=model_message.track.pose_projected.back();
			else
				geo_pose=model_message.track.pose.back();

			rectangle_pose = btTransform(btQuaternion(geo_pose.orientation.x,
					geo_pose.orientation.y, geo_pose.orientation.z,
					geo_pose.orientation.w), btVector3(geo_pose.position.x,
					geo_pose.position.y, geo_pose.position.z));
			transform(framePose * rectangle_pose, lineScale, pos, orient, scale);

			Eigen::MatrixXd J(3,model->getDOFs());
			model->getParam("jacobian",J);

			//cout << "j="<<J<<endl;
			scale *= 5;

			for(int r=0;r<J.cols();r++) {
				btVector3 dir(J(0,r),J(1,r),J(2,r));
				dir.normalize();
				dir *= 20*lineScale;

				btVector3 endpoint = rectangle_pose.getOrigin() + dir;
				Ogre::Vector3 pos2 = Ogre::Vector3(endpoint.x(), endpoint.y(), endpoint.z());
				rviz::robotToOgre(pos2);

				createArrow(pos2, pos, scale, color, arrows[uniqueId]);
			}

		}

		// draw hessian?
		if(showHessians_ && pose.size()>0 && model->getDOFs()>0) {
			geometry_msgs::Pose geo_pose;
			if(projectConfigurations_)
				geo_pose=model_message.track.pose_projected.back();
			else
				geo_pose=model_message.track.pose.back();

			rectangle_pose = btTransform(btQuaternion(geo_pose.orientation.x,
					geo_pose.orientation.y, geo_pose.orientation.z,
					geo_pose.orientation.w), btVector3(geo_pose.position.x,
					geo_pose.position.y, geo_pose.position.z));
			transform(framePose * rectangle_pose, lineScale, pos, orient, scale);

			Eigen::MatrixXd H((int)(3*model->getDOFs()),(int)(model->getDOFs()));
			model->getParam("hessian",H);

			//cout << "j="<<J<<endl;
			scale *= 5;

			for(int i=0;i<H.rows()/3;i++)
			for(int c=0;c<H.cols();c++) {
				btVector3 dir(H(0+3*i,c),H(1+3*i,c),H(2+3*i,c));
//				cout << "hessian dir "<<i<<" "<<c<<": "<<dir.x()<<" "<<dir.y()<<" "<<dir.z()<<endl;
				if(dir.length()==0) continue;
				dir.normalize();
				dir *= 10*lineScale;

				btVector3 endpoint = rectangle_pose.getOrigin() + dir;
				Ogre::Vector3 pos2 = Ogre::Vector3(endpoint.x(), endpoint.y(), endpoint.z());
				rviz::robotToOgre(pos2);

				createArrow(pos2, pos, scale, color, arrows[uniqueId]);
			}

		}


		// show model name?
		if(showModelName_)
		{
//			string caption = boost::str( boost::format("%1% (t%3%/m%2%)") %  model_message.name % model_message.id % model_message.track.id);
			string caption = boost::str( boost::format("%1%") %  model_message.name );

			const geometry_msgs::Pose& geo_pose = pose.back();
			rectangle_pose = btTransform(btQuaternion(geo_pose.orientation.x,
					geo_pose.orientation.y, geo_pose.orientation.z,
					geo_pose.orientation.w), btVector3(geo_pose.position.x,
					geo_pose.position.y, geo_pose.position.z));

			transform(framePose * rectangle_pose, lineScale, pos, orient, scale);

			createText(pos,orient, scale, color,caption, text[uniqueId]);
		}

}
}

void ModelDisplay::targetFrameChanged() {
}

void ModelDisplay::fixedFrameChanged() {
	tf_filter_.setTargetFrame(fixed_frame_);

	clearDisplay();
}

void ModelDisplay::createProperties() {
	topic_property_ = property_manager_->createProperty<
			rviz::ROSTopicStringProperty> ("Topic", property_prefix_,
			boost::bind(&ModelDisplay::getTopic, this), boost::bind(
					&ModelDisplay::setTopic, this, _1), parent_category_, this);

	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType(articulation_msgs::ModelMsg::__s_getDataType());

	unique_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Unique by", property_prefix_, boost::bind(
							&ModelDisplay::getUnique, this), boost::bind(
							&ModelDisplay::setUnique, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop1a = unique_property_.lock();
	enum_prop1a->addOption("all", u_all);
	enum_prop1a->addOption("single", u_single);
	enum_prop1a->addOption("track id", u_track);
	enum_prop1a->addOption("model id", u_model);
	enum_prop1a->addOption("model id (stored only)", u_modelstored);
	enum_prop1a->addOption("model type", u_modeltype);

	color_property_ = property_manager_->createProperty<rviz::ColorProperty> (
			"Color", property_prefix_, boost::bind(&ModelDisplay::getColor,
					this), boost::bind(&ModelDisplay::setColor, this, _1),
			parent_category_, this);

	alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> (
			"Alpha", property_prefix_, boost::bind(&ModelDisplay::getAlpha,
					this), boost::bind(&ModelDisplay::setAlpha, this, _1),
			parent_category_, this);

	lineWidth_property_
			= property_manager_->createProperty<rviz::FloatProperty> (
					"Line Width", property_prefix_, boost::bind(
							&ModelDisplay::getLineWidth, this), boost::bind(
							&ModelDisplay::setLineWidth, this, _1),
					parent_category_, this);

	trackColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Track number", property_prefix_, boost::bind(
							&ModelDisplay::getTrackColor, this), boost::bind(
							&ModelDisplay::setTrackColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop1 = trackColor_property_.lock();
	enum_prop1->addOption("not used", cs_fixed);
	//	  enum_prop1->addOption( "channel", cs_channel );
	enum_prop1->addOption("as hue", cs_hue);
	enum_prop1->addOption("as brightness", cs_brightness);

	modelColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Model number", property_prefix_, boost::bind(
							&ModelDisplay::getModelColor, this), boost::bind(
							&ModelDisplay::setModelColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop11 = modelColor_property_.lock();
	enum_prop11->addOption("not used", cs_fixed);
	//	  enum_prop1->addOption( "channel", cs_channel );
	enum_prop11->addOption("as hue", cs_hue);
	enum_prop11->addOption("as brightness", cs_brightness);

	poseColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Sequence number", property_prefix_, boost::bind(
							&ModelDisplay::getPoseColor, this), boost::bind(
							&ModelDisplay::setPoseColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop2 = poseColor_property_.lock();
	enum_prop2->addOption("not used", cs_fixed);
	//	  enum_prop2->addOption( "channel", cs_channel );
	enum_prop2->addOption("as hue", cs_hue);
	enum_prop2->addOption("as brightness", cs_brightness);

	modelTypeColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Model type", property_prefix_, boost::bind(
							&ModelDisplay::getModelTypeColor, this), boost::bind(
							&ModelDisplay::setModelTypeColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop22 = modelTypeColor_property_.lock();
	enum_prop22->addOption("not used", cs_fixed);
	enum_prop22->addOption("as hue", cs_hue);
	enum_prop22->addOption("as brightness", cs_brightness);

	likelihoodColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Pose Likelihood", property_prefix_, boost::bind(
							&ModelDisplay::getLikelihoodColor, this), boost::bind(
							&ModelDisplay::setLikelihoodColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop3 = likelihoodColor_property_.lock();
	enum_prop3->addOption("not used", cs_fixed);
	enum_prop3->addOption("as hue", cs_hue);
	enum_prop3->addOption("as brightness", cs_brightness);

	forceColor_property_
			= property_manager_->createProperty<rviz::EnumProperty> (
					"Force (x/y/z or mag)", property_prefix_, boost::bind(
							&ModelDisplay::getForceColor, this), boost::bind(
							&ModelDisplay::setForceColor, this, _1),
					parent_category_, this);
	rviz::EnumPropertyPtr enum_prop3f = forceColor_property_.lock();
	enum_prop3f->addOption("not used", cs_fixed);
	enum_prop3f->addOption("as hue", cs_hue);
	enum_prop3f->addOption("as brightness", cs_brightness);

	displayStyle_property_ = property_manager_->createProperty<
			rviz::EnumProperty> ("Style", property_prefix_, boost::bind(
			&ModelDisplay::getDisplayStyle, this), boost::bind(
			&ModelDisplay::setDisplayStyle, this, _1), parent_category_, this);
	rviz::EnumPropertyPtr enum_prop4 = displayStyle_property_.lock();
	enum_prop4->addOption("line", ds_line);
	enum_prop4->addOption("cross line", ds_cross_line);
	enum_prop4->addOption("cross", ds_cross);
	enum_prop4->addOption("arrow", ds_arrow);
	enum_prop4->addOption("axes", ds_axes);
	enum_prop4->addOption("rectangle", ds_rectangle);
	enum_prop4->addOption("none", ds_none);

	sampleConfigurations_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Sample Configurations", property_prefix_,
			boost::bind(&ModelDisplay::getSampleConfigurations, this),
			boost::bind(&ModelDisplay::setSampleConfigurations, this, _1),
			parent_category_, this);

	sampleDensity_property_ = property_manager_->createProperty<
			rviz::FloatProperty> ("Sample Density", property_prefix_,
			boost::bind(&ModelDisplay::getSampleDensity, this), boost::bind(
					&ModelDisplay::setSampleDensity, this, _1),
			parent_category_, this);


	projectConfigurations_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Project Configurations", property_prefix_,
			boost::bind(&ModelDisplay::getProjectConfigurations, this),
			boost::bind(&ModelDisplay::setProjectConfigurations, this, _1),
			parent_category_, this);

	showJacobians_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Jacobians", property_prefix_, boost::bind(
			&ModelDisplay::getShowJacobians, this), boost::bind(
			&ModelDisplay::setShowJacobians, this, _1), parent_category_, this);

	showHessians_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Hessians", property_prefix_, boost::bind(
			&ModelDisplay::getShowHessians, this), boost::bind(
			&ModelDisplay::setShowHessians, this, _1), parent_category_, this);

	showLatestOnly_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Latest obs only", property_prefix_,
			boost::bind(&ModelDisplay::getShowLatestOnly, this), boost::bind(
					&ModelDisplay::setShowLatestOnly, this, _1),
			parent_category_, this);

	showModelName_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Show Model Name", property_prefix_,
			boost::bind(&ModelDisplay::getShowModelName, this), boost::bind(
					&ModelDisplay::setShowModelName, this, _1),
			parent_category_, this);

	showForces_property_ = property_manager_->createProperty<
			rviz::BoolProperty> ("Show Forces", property_prefix_,
			boost::bind(&ModelDisplay::getShowForces, this), boost::bind(
					&ModelDisplay::setShowForces, this, _1),
			parent_category_, this);

	forceScaling_property_ = property_manager_->createProperty<
			rviz::FloatProperty> ("Force scaling", property_prefix_,
			boost::bind(&ModelDisplay::getForceScaling, this), boost::bind(
					&ModelDisplay::setForceScaling, this, _1),
			parent_category_, this);

}

void ModelDisplay::clearDisplay() {
	//	  markers_.clear();
	//	tf_filter_.clear();
	clearMap();
}

bool ModelDisplay::transform(
		const articulation_msgs::ModelMsg::ConstPtr& message,
		btTransform &transform) {
	std::string fixed_frame = vis_manager_->getFixedFrame();

	std::string frame_id = message->header.frame_id;
	if (frame_id.empty()) {
		frame_id = fixed_frame;
	}

	tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0, 0, 0, 1), btVector3(
			0, 0, 0)), message->header.stamp, frame_id);
	try {
		vis_manager_->getTFClient()->transformPose(fixed_frame, pose, pose);
	} catch (tf::TransformException& e) {
		ROS_ERROR( "Error transforming track/model '%d'/'%d' from frame '%s' to frame '%s': %s\n",message->track.id,message->id, frame_id.c_str(), fixed_frame.c_str(), e.what() );
		return false;
	}

	transform = pose;

	return (true);
}

bool ModelDisplay::transform(const btTransform &pose, const btVector3 &scaleIn,
		Ogre::Vector3& pos, Ogre::Quaternion& orient, Ogre::Vector3& scaleOut) {

	pos = Ogre::Vector3(pose.getOrigin().x(), pose.getOrigin().y(),
			pose.getOrigin().z());
	rviz::robotToOgre(pos);

	btQuaternion quat;
	pose.getBasis().getRotation(quat);
	orient = Ogre::Quaternion::IDENTITY;
	rviz::ogreToRobot(orient);
	orient = Ogre::Quaternion(quat.w(), quat.x(), quat.y(), quat.z()) * orient;
	rviz::robotToOgre(orient);

	scaleOut = Ogre::Vector3(scaleIn.x(), scaleIn.y(), scaleIn.z());
	rviz::scaleRobotToOgre(scaleOut);

	return true;
}

ogre_tools::BillboardLine* ModelDisplay::newBillboardLine() {
	if (recycleLines.size() > 0) {
		ogre_tools::BillboardLine* line = recycleLines.back();
		recycleLines.pop_back();
		return (line);
	}
	return (new ogre_tools::BillboardLine(vis_manager_->getSceneManager(),
			scene_node_));
}

ogre_tools::Arrow* ModelDisplay::newArrow() {
	if (recycleArrows.size() > 0) {
		ogre_tools::Arrow* arrow = recycleArrows.back();
		recycleArrows.pop_back();
		return (arrow);
	}
	return (new ogre_tools::Arrow(vis_manager_->getSceneManager(),
			scene_node_));
}

ogre_tools::MovableText* ModelDisplay::newText() {
	if (recycleText.size() > 0) {
		ogre_tools::MovableText* line = recycleText.back();
		recycleText.pop_back();
		return (line);
	}
	ogre_tools::MovableText* t = new ogre_tools::MovableText( "test", "Arial", 0.03 );
	t->setTextAlignment(ogre_tools::MovableText::H_CENTER, ogre_tools::MovableText::V_BELOW);

	Ogre::SceneNode* n = scene_node_->createChildSceneNode();
	n->attachObject(t);
	return (t);
}



void ModelDisplay::createRectangle(Ogre::Vector3 pos, Ogre::Quaternion orient,
		Ogre::Vector3 scale, double w, double h, btVector3 color, std::vector<
				ogre_tools::BillboardLine*> &vec) {

	ogre_tools::BillboardLine* lines = newBillboardLine();

	lines->setPosition(pos);
	lines->setOrientation(orient);
	lines->setScale(scale);
	lines->setColor(color.x(), color.y(), color.z(), alpha_);

	lines->clear();
	lines->setLineWidth(lineWidth_);
	lines->setMaxPointsPerLine(5);
	lines->setNumLines(1);

	Ogre::Vector3 v;

	v = Ogre::Vector3(0, 0, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);
	v = Ogre::Vector3(w, 0, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);
	v = Ogre::Vector3(w, h, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);
	v = Ogre::Vector3(0, h, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);
	v = Ogre::Vector3(0, 0, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);

	vec.push_back(lines);
}

void ModelDisplay::createAxes(Ogre::Vector3 pos, Ogre::Quaternion orient,
		Ogre::Vector3 scale, btVector3 color, std::vector<
				ogre_tools::BillboardLine*> &vec) {

	for (int i = 0; i < 3; i++) {
		ogre_tools::BillboardLine* lines = newBillboardLine();

		lines->clear();
		lines->setPosition(pos);
		lines->setOrientation(orient);
		lines->setScale(scale);

		btVector3 hsv = RGB_to_HSV(color);
		hsv.setX(i / 3.0);
		btVector3 axis_color = HSV_to_RGB(hsv);
		lines->setColor(axis_color.x(), axis_color.y(), axis_color.z(), alpha_);

		lines->setLineWidth(lineWidth_);
		lines->setMaxPointsPerLine(2);
		lines->setNumLines(1);

		Ogre::Vector3 v;

		v = Ogre::Vector3(0, 0, 0);
		v[i] = lineWidth_ * 5;
		rviz::robotToOgre(v);
		lines->addPoint(v);

		v = Ogre::Vector3(0, 0, 0);
//		v[i] = -lineWidth_ * 5;
		rviz::robotToOgre(v);
		lines->addPoint(v);

		vec.push_back(lines);
	}
}

void ModelDisplay::createLine(Ogre::Vector3 pos, Ogre::Vector3 old_pos,
		Ogre::Vector3 scale, btVector3 color, std::vector<
				ogre_tools::BillboardLine*> &vec,bool add_cross,bool with_line) {

	ogre_tools::BillboardLine* lines = newBillboardLine();
	lines->setPosition(pos);
	lines->setOrientation(Ogre::Quaternion(1, 0, 0, 0));
	lines->setScale(scale);

	lines->setColor(color.x(), color.y(), color.z(), alpha_);
	lines->clear();

	lines->setLineWidth(lineWidth_);
	lines->setMaxPointsPerLine(2);
	lines->setNumLines(4);

	Ogre::Vector3 v;
	if(add_cross) {
		for (int i = 0; i < 3; i++) {

			v = Ogre::Vector3(0, 0, 0);
			v[i] = lineWidth_ * 5;
			rviz::robotToOgre(v);
			lines->addPoint(v);

			v = Ogre::Vector3(0, 0, 0);
			v[i] = -lineWidth_ * 5;
			rviz::robotToOgre(v);
			lines->addPoint(v);
			lines->newLine();
		}
	}

	if(with_line) {
		v = Ogre::Vector3(0, 0, 0);
		rviz::robotToOgre(v);
		lines->addPoint(v);

		v = old_pos - pos;
		//	rviz::robotToOgre(v);
		lines->addPoint(v);
	}

	vec.push_back(lines);
}

void ModelDisplay::createArrow(Ogre::Vector3 point2, Ogre::Vector3 point1,
		Ogre::Vector3 scale, btVector3 color, std::vector<
				ogre_tools::Arrow*> &vec) {

	ogre_tools::Arrow* arrow = newArrow();

	arrow->setColor(color.x(), color.y(), color.z(), alpha_);

    Ogre::Vector3 direction = point2 - point1;
    float distance = direction.length();
    direction.normalise();
    Ogre::Quaternion orient = Ogre::Vector3::NEGATIVE_UNIT_Z.getRotationTo( direction );
    arrow->setPosition(point1);
    arrow->setOrientation(orient);
    arrow->setScale(Ogre::Vector3(1.0f, 1.0f, 1.0f));

    float head_length = 0.3*distance;
    float shaft_length = distance - head_length;
    arrow->set(shaft_length, scale.x, head_length, scale.y*2);

	vec.push_back(arrow);
}

void ModelDisplay::createText(Ogre::Vector3 pos, Ogre::Quaternion orient,
		Ogre::Vector3 scale, btVector3 color, std::string s, std::vector<
				ogre_tools::MovableText*> &vec) {

	ogre_tools::MovableText* text = newText();
	text->setCharacterHeight(lineWidth_/0.05);
	text->setTextAlignment(ogre_tools::MovableText::H_LEFT,ogre_tools::MovableText::V_BELOW);

	text->setCaption(s);
//	text->setGlobalTranslation(pos);
	text->setColor(Ogre::ColourValue(color.x(), color.y(), color.z(), alpha_));
	text->getParentNode()->setPosition(pos);
	text->setVisible(true);

	vec.push_back(text);
}

void ModelDisplay::setColor(const rviz::Color& color) {
	color_ = color;

	propertyChanged(color_property_);
}

void ModelDisplay::setAlpha(float a) {
	alpha_ = a;

	propertyChanged(alpha_property_);
}

void ModelDisplay::setLineWidth(float a) {
	lineWidth_ = a;

	propertyChanged(lineWidth_property_);
}

void ModelDisplay::setTrackColor(int cs) {
	trackColor_ = cs;

	propertyChanged(trackColor_property_);
}

void ModelDisplay::setUnique(int cs) {
	unique_ = cs;

	propertyChanged(unique_property_);
}

void ModelDisplay::setModelColor(int cs) {
	modelColor_ = cs;

	propertyChanged(modelColor_property_);
}

void ModelDisplay::setPoseColor(int cs) {
	poseColor_ = cs;

	propertyChanged(poseColor_property_);
}

void ModelDisplay::setModelTypeColor(int cs) {
	modelTypeColor_ = cs;

	propertyChanged(modelTypeColor_property_);
}

void ModelDisplay::setLikelihoodColor(int cs) {
	likelihoodColor_ = cs;

	propertyChanged(likelihoodColor_property_);
}

void ModelDisplay::setForceColor(int cs) {
	forceColor_ = cs;

	propertyChanged(forceColor_property_);
}

btVector3 ModelDisplay::modifyColor(btVector3 color, int colorStyle, float f) {
	switch (colorStyle) {
	case cs_fixed:
		break;
	case cs_channel:
		break;
	case cs_hue:
		color = RGB_to_HSV(color);
		color.setX(f*0.7);
		color = HSV_to_RGB(color);
		break;
	case cs_brightness:
		if (f < 0.5) {
			color = color * (f + 0.5);
		} else {
			color = btVector3(1, 1, 1) - ((btVector3(1, 1, 1) - color) * (0.5
					+ (1 - f)));
		}
		break;
	}
	return (color);
}

void ModelDisplay::setDisplayStyle(int ds) {
	displayStyle_ = ds;

	propertyChanged(displayStyle_property_);
}

void ModelDisplay::setSampleConfigurations(bool sc) {
	sampleConfigurations_ = sc;
	propertyChanged(sampleConfigurations_property_);

	if(sc) {
		projectConfigurations_ = false;
		propertyChanged(projectConfigurations_property_);
	}
}

void ModelDisplay::setProjectConfigurations(bool sc) {
	projectConfigurations_ = sc;
	propertyChanged(projectConfigurations_property_);
	if(!sc) {
		sampleConfigurations_ = false;
		propertyChanged(sampleConfigurations_property_);
	}
}

void ModelDisplay::setShowJacobians(bool sc) {
	showJacobians_ = sc;

	propertyChanged(showJacobians_property_);
}

void ModelDisplay::setShowHessians(bool sc) {
	showHessians_ = sc;

	propertyChanged(showHessians_property_);
}

void ModelDisplay::setShowLatestOnly(bool sc) {
	showLatestOnly_ = sc;

	propertyChanged(showLatestOnly_property_);
}

void ModelDisplay::setSampleDensity(float sc) {
	sampleDensity_ = sc;

	propertyChanged(sampleDensity_property_);
}

void ModelDisplay::setShowModelName(bool sc) {
	showModelName_ = sc;

	propertyChanged(showModelName_property_);
}

void ModelDisplay::setShowForces(bool sc) {
	showForces_ = sc;

	propertyChanged(showForces_property_);
}

void ModelDisplay::setForceScaling(float sc) {
	forceScaling_ = sc;

	propertyChanged(forceScaling_property_);
}


}

