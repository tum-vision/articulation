#include "articulation_display.h"
#include "rviz/common.h"
#include "rviz/visualization_manager.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/link_updater.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"

#include <ogre_tools/axes.h>

#include <tf/transform_listener.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace articulation_rviz_plugin {

TrackDisplay::TrackDisplay(const std::string& name,
		rviz::VisualizationManager* manager) :
	Display(name, manager) {
}

TrackDisplay::~TrackDisplay() {
	unsubscribe();
}

void TrackDisplay::setTopic(const std::string& topic) {
	unsubscribe();
	track_topic_ = topic;
	subscribe();

	propertyChanged(topic_property_);
}

void TrackDisplay::onEnable() {
	subscribe();
}

void TrackDisplay::onDisable() {
	unsubscribe();
}

void TrackDisplay::subscribe() {
	if (!isEnabled()) {
		return;
	}

	if (!track_topic_.empty()) {
		sub_ = update_nh_.subscribe(track_topic_, 2,
				&TrackDisplay::incomingTrack, this);
	}

}

void TrackDisplay::unsubscribe() {
	sub_.shutdown();
}

void TrackDisplay::update(float wall_dt, float ros_dt) {
}

void TrackDisplay::incomingTrack(
		const articulation_models::TrackMsg::ConstPtr& msg) {
	incoming_track_message_ = msg;
}

void TrackDisplay::targetFrameChanged() {
}

void TrackDisplay::createProperties() {
	topic_property_ = property_manager_->createProperty<
			rviz::ROSTopicStringProperty> ("Topic", property_prefix_,
			boost::bind(&TrackDisplay::getTopic, this), boost::bind(
					&TrackDisplay::setTopic, this, _1), parent_category_, this);
	rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
	topic_prop->setMessageType(articulation_models::TrackMsg::__s_getDataType());
}

}

