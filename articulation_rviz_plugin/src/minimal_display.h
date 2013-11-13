#ifndef ARTICULATION_RVIZ_PLUGIN_PLANNING_DISPLAY_H
#define ARTICULATION_RVIZ_PLUGIN_PLANNING_DISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/forwards.h"

#include "articulation_models/TrackMsg.h"

#include <ros/ros.h>

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace articulation_rviz_plugin
{

/**
 * \class TrackDisplay
 * \brief
 */
class TrackDisplay : public rviz::Display
{
public:
  TrackDisplay( const std::string& name, rviz::VisualizationManager* manager );
  virtual ~TrackDisplay();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return track_topic_; }

  virtual void update(float wall_dt, float ros_dt);

  // Overrides from Display
  virtual void targetFrameChanged();
  virtual void fixedFrameChanged() {}
  virtual void createProperties();


protected:
  void subscribe();
  void unsubscribe();

  void incomingTrack(const articulation_models::TrackMsg::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  ros::Subscriber sub_;
  std::string track_topic_;
  articulation_models::TrackMsg::ConstPtr incoming_track_message_;
  articulation_models::TrackMsg::ConstPtr displaying_track_message_;

  rviz::ROSTopicStringPropertyWPtr topic_property_;
};

}

 #endif


