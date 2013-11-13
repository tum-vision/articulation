#include "articulation_display.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <ogre_tools/axes.h>

#include <tf/transform_listener.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "utils.h"

namespace articulation_rviz_plugin {

TrackDisplay::TrackDisplay(const std::string& name,
		rviz::VisualizationManager* manager) :
	Display(name, manager), track_topic_("/track"), color_(0.5f, 0.0f, 1.0f),
			alpha_(0.5f), lineWidth_(0.003f)
					,trackColor_(cs_fixed),poseColor_(cs_fixed),
					displayStyle_(ds_line),
					tf_filter_(
					*manager->getTFClient(), "", 1000, update_nh_) {
	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	tf_filter_.connectInput(sub_);
	tf_filter_.registerCallback(boost::bind(&TrackDisplay::incomingTrack, this,
			_1));
}

TrackDisplay::~TrackDisplay() {
	unsubscribe();

}

void TrackDisplay::clearMap() {
	std::map<int, std::vector<ogre_tools::BillboardLine*> >::iterator it;
	for (it = lines.begin(); it != lines.end(); it++) {
		clearVector(it->second);
	}
}

void TrackDisplay::clearVector(std::vector<ogre_tools::BillboardLine*>& vec) {
	for (size_t j = 0; j < vec.size(); j++) {
		delete vec[j];
	}
	vec.clear();
}

void TrackDisplay::setTopic(const std::string& topic) {
	unsubscribe();
	track_topic_ = topic;
	subscribe();

	propertyChanged(topic_property_);
}

void TrackDisplay::onEnable() {
	subscribe();

	scene_node_->setVisible(true);
}

void TrackDisplay::onDisable() {
	unsubscribe();
	scene_node_->setVisible(false);
	clearDisplay();
}

void TrackDisplay::reset() {
	clearDisplay();
}

void TrackDisplay::subscribe() {
	if (!isEnabled()) {
		return;
	}

	if (!track_topic_.empty()) {
		sub_.subscribe(update_nh_, track_topic_, 0);
	}

}

void TrackDisplay::unsubscribe() {
	sub_.unsubscribe();
}

void TrackDisplay::incomingTrack(
		const articulation_msgs::TrackMsg::ConstPtr& msg) {
	incoming_track_message_ = msg;
	boost::mutex::scoped_lock lock(queue_mutex_);

	message_queue_.push_back(msg);
}

void TrackDisplay::update(float wall_dt, float ros_dt) {
	V_TrackMsg local_queue;

	{
		boost::mutex::scoped_lock lock(queue_mutex_);

		local_queue.swap(message_queue_);
	}

	for (size_t t = 0; t < local_queue.size(); t++) {
		const articulation_msgs::TrackMsg::ConstPtr& track_message =
				local_queue[t];

		for(size_t l=0;l<lines[track_message->id].size();l++)
			lines[track_message->id][l]->clear();
		recycleLines.insert(recycleLines.begin(),
				lines[track_message->id].begin(),
				lines[track_message->id].end());

		lines[track_message->id].clear();
	}

	for (size_t t = 0; t < local_queue.size(); t++) {
		const articulation_msgs::TrackMsg::ConstPtr& track_message =
				local_queue[t];

		btTransform framePose;
		btVector3 lineScale(lineWidth_, lineWidth_, lineWidth_);
		transform(track_message, framePose);

		int channel_w = -1;
		int channel_h = -1;
		for (size_t i = 0; i < track_message->channels.size(); i++) {
			if (track_message->channels[i].name == "width")
				channel_w = (int) i;
			if (track_message->channels[i].name == "height")
				channel_h = (int) i;
		}

		Ogre::Vector3 old_pos(0,0,0);
		for (size_t i = 0; i < track_message->pose.size(); i++) {
			const geometry_msgs::Pose& geo_pose = track_message->pose[i];

			btTransform rectangle_pose(btQuaternion(geo_pose.orientation.x,
					geo_pose.orientation.y, geo_pose.orientation.z,
					geo_pose.orientation.w), btVector3(geo_pose.position.x,
					geo_pose.position.y, geo_pose.position.z));

			Ogre::Vector3 pos, scale;
			Ogre::Quaternion orient;
			transform(framePose * rectangle_pose, lineScale, pos, orient, scale);

			btVector3 color(color_.r_, color_.g_, color_.b_);	// fixed color

			double f =track_message->id / 7.0;
			color = modifyColor( color,  trackColor_, f - floor(f) );
			color = modifyColor( color,  poseColor_, i / (double)track_message->pose.size() );

			if(displayStyle_ == ds_line) {
				if(i==0) old_pos = pos;
				createLine(pos, old_pos, scale,
						color,
						lines[track_message->id],false);
				old_pos = pos;
			} else
			if(displayStyle_ == ds_cross_line) {
				if(i==0) old_pos = pos;
				createLine(pos, old_pos, scale,
						color,
						lines[track_message->id],true);
				old_pos = pos;
			} else
			if(displayStyle_ == ds_axes) {
				createAxes(pos, orient, scale,
						color,
						lines[track_message->id]);
			} else
			if(displayStyle_ == ds_rectangle) {
				createRectangle(pos, orient, scale,
						channel_w==-1? lineWidth_*5 : track_message->channels[channel_w].values[i],
						channel_h==-1? lineWidth_*5 : track_message->channels[channel_h].values[i],
						color,
						lines[track_message->id]);
			}
		}
	}
}

void TrackDisplay::targetFrameChanged() {
}

void TrackDisplay::fixedFrameChanged() {
	tf_filter_.setTargetFrame(fixed_frame_);

	clearDisplay();
}

void TrackDisplay::createProperties() {
	topic_property_ = property_manager_->createProperty<
			rviz::ROSTopicStringProperty> ("Topic", property_prefix_,
			boost::bind(&TrackDisplay::getTopic, this), boost::bind(
					&TrackDisplay::setTopic, this, _1), parent_category_, this);
	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType(articulation_msgs::TrackMsg::__s_getDataType());

	color_property_ = property_manager_->createProperty<rviz::ColorProperty> (
			"Color", property_prefix_, boost::bind(&TrackDisplay::getColor,
					this), boost::bind(&TrackDisplay::setColor, this, _1),
			parent_category_, this);

	alpha_property_ = property_manager_->createProperty<rviz::FloatProperty> (
			"Alpha", property_prefix_, boost::bind(&TrackDisplay::getAlpha,
					this), boost::bind(&TrackDisplay::setAlpha, this, _1),
			parent_category_, this);

	lineWidth_property_ = property_manager_->createProperty<rviz::FloatProperty> (
			"Line Width", property_prefix_, boost::bind(&TrackDisplay::getLineWidth,
					this), boost::bind(&TrackDisplay::setLineWidth, this, _1),
			parent_category_, this);

	  trackColor_property_ = property_manager_->createProperty<rviz::EnumProperty>( "Track Color", property_prefix_, boost::bind( &TrackDisplay::getTrackColor, this ),
	                                                                     boost::bind( &TrackDisplay::setTrackColor, this, _1 ), parent_category_, this );
	  rviz::EnumPropertyPtr enum_prop1 = trackColor_property_.lock();
	  enum_prop1->addOption( "fixed", cs_fixed );
//	  enum_prop1->addOption( "channel", cs_channel );
	  enum_prop1->addOption( "hue", cs_hue );
	  enum_prop1->addOption( "brightness", cs_brightness );

	  poseColor_property_ = property_manager_->createProperty<rviz::EnumProperty>( "Pose Color", property_prefix_, boost::bind( &TrackDisplay::getPoseColor, this ),
	                                                                     boost::bind( &TrackDisplay::setPoseColor, this, _1 ), parent_category_, this );
	  rviz::EnumPropertyPtr enum_prop2 = poseColor_property_.lock();
	  enum_prop2->addOption( "fixed", cs_fixed );
//	  enum_prop2->addOption( "channel", cs_channel );
	  enum_prop2->addOption( "hue", cs_hue );
	  enum_prop2->addOption( "brightness", cs_brightness );

	  displayStyle_property_ = property_manager_->createProperty<rviz::EnumProperty>( "Style", property_prefix_, boost::bind( &TrackDisplay::getDisplayStyle, this ),
	                                                                     boost::bind( &TrackDisplay::setDisplayStyle, this, _1 ), parent_category_, this );
	  rviz::EnumPropertyPtr enum_prop3 = displayStyle_property_.lock();
	  enum_prop3->addOption( "line", ds_line );
	  enum_prop3->addOption( "cross line", ds_cross_line );
	  enum_prop3->addOption( "axes", ds_axes );
	  enum_prop3->addOption( "rectangle", ds_rectangle );

}

void TrackDisplay::clearDisplay() {
	//	  markers_.clear();
	//	tf_filter_.clear();
	clearMap();
}

bool TrackDisplay::transform(
		const articulation_msgs::TrackMsg::ConstPtr& message,
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
		ROS_ERROR( "Error transforming track '%d' from frame '%s' to frame '%s': %s\n",message->id, frame_id.c_str(), fixed_frame.c_str(), e.what() );
		return false;
	}

	transform = pose;

	return (true);
}

bool TrackDisplay::transform(const btTransform &pose, const btVector3 &scaleIn,
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

ogre_tools::BillboardLine* TrackDisplay::newBillboardLine() {
	if (recycleLines.size() > 0) {
		ogre_tools::BillboardLine* line = recycleLines.back();
		recycleLines.pop_back();
		return (line);
	}
	return (new ogre_tools::BillboardLine(vis_manager_->getSceneManager(),
			scene_node_));
}

void TrackDisplay::createRectangle(Ogre::Vector3 pos, Ogre::Quaternion orient,
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

void TrackDisplay::createAxes(Ogre::Vector3 pos, Ogre::Quaternion orient,
		Ogre::Vector3 scale, btVector3 color, std::vector<
				ogre_tools::BillboardLine*> &vec) {


	for(int i=0;i<3;i++) {
		ogre_tools::BillboardLine* lines = newBillboardLine();

		lines->clear();
		lines->setPosition(pos);
		lines->setOrientation(orient);
		lines->setScale(scale);

		btVector3 hsv = RGB_to_HSV(color);
		hsv.setX(i/3.0);
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
		v[i] = -lineWidth_ * 5;
		rviz::robotToOgre(v);
		lines->addPoint(v);

		vec.push_back(lines);
	}
}

void TrackDisplay::createLine(Ogre::Vector3 pos, Ogre::Vector3 old_pos,
		Ogre::Vector3 scale, btVector3 color, std::vector<
				ogre_tools::BillboardLine*> &vec,bool add_cross) {

	ogre_tools::BillboardLine* lines = newBillboardLine();
	lines->setPosition(pos);
	lines->setOrientation(Ogre::Quaternion(1,0,0,0));
	lines->setScale(scale);

	lines->setColor(color.x(), color.y(), color.z(), alpha_);
	lines->clear();

	lines->setLineWidth(lineWidth_);
	lines->setMaxPointsPerLine(2);
	lines->setNumLines(4);

	Ogre::Vector3 v;
	if(add_cross) {
		for(int i=0;i<3;i++) {

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

	v = Ogre::Vector3(0, 0, 0);
	rviz::robotToOgre(v);
	lines->addPoint(v);

	v = old_pos - pos;
//	rviz::robotToOgre(v);
	lines->addPoint(v);

	vec.push_back(lines);
}


void TrackDisplay::setColor(const rviz::Color& color) {
	color_ = color;

	propertyChanged(color_property_);
}

void TrackDisplay::setAlpha(float a) {
	alpha_ = a;

	propertyChanged(alpha_property_);
}

void TrackDisplay::setLineWidth(float a) {
	lineWidth_ = a;

	propertyChanged(lineWidth_property_);
}

void TrackDisplay::setTrackColor(int cs) {
	trackColor_ = cs;

	propertyChanged(trackColor_property_);
}

void TrackDisplay::setPoseColor(int cs) {
	poseColor_ = cs;

	propertyChanged(poseColor_property_);
}

btVector3 TrackDisplay::modifyColor( btVector3 color,  int colorStyle, float f ) {
	switch(colorStyle) {
	case cs_fixed:
		break;
	case cs_channel:
		break;
	case cs_hue:
		color = RGB_to_HSV(color);
		color.setX( f );
		color = HSV_to_RGB(color);
		break;
	case cs_brightness:
		if(f < 0.5) {
			color = color * (f+0.5);
		} else {
			color = btVector3(1,1,1) - ( (btVector3(1,1,1) - color) * (0.5 + (1-f)) );
		}
		break;
	}
	return(color);
}

void TrackDisplay::setDisplayStyle(int ds) {
	displayStyle_ = ds;

	propertyChanged(displayStyle_property_);
}



}

