#ifndef ARTICULATION_RVIZ_PLUGIN_TRACK_DISPLAY_H
#define ARTICULATION_RVIZ_PLUGIN_TRACK_DISPLAY_H

#include "rviz/display.h"
#include "rviz/common.h"
#include "rviz/properties/forwards.h"
#include "rviz/helpers/color.h"

#include <ros/ros.h>

#include "articulation_msgs/TrackMsg.h"

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <ogre_tools/arrow.h>
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
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
  virtual void createProperties();
  virtual ~TrackDisplay();

  virtual void onEnable();
  virtual void onDisable();

  std::string track_topic_;
  void setTopic( const std::string& topic );
  const std::string& getTopic() { return track_topic_; }
  rviz::ROSTopicStringPropertyWPtr topic_property_;

  rviz::Color color_;
  void setColor( const rviz::Color& color );
  const rviz::Color& getColor() { return color_; }
  rviz::ColorPropertyWPtr color_property_;

  float alpha_;
  void setAlpha(float a);
  float getAlpha() { return alpha_; }
  rviz::FloatPropertyWPtr alpha_property_;

  float lineWidth_;
  void setLineWidth(float a);
  float getLineWidth() { return lineWidth_; }
  rviz::FloatPropertyWPtr lineWidth_property_;

  enum ColorStyle
  {
    cs_fixed,
    cs_channel,
    cs_hue,
    cs_brightness,
    ColorStyleCount,
  };

  int trackColor_;
  void setTrackColor(int cs);
  int getTrackColor() { return trackColor_; }
  rviz::EnumPropertyWPtr trackColor_property_;

  int poseColor_;
  void setPoseColor(int cs);
  int getPoseColor() { return poseColor_; }
  rviz::EnumPropertyWPtr poseColor_property_;

  enum DisplayStyle
  {
    ds_line,
    ds_cross_line,
    ds_axes,
    ds_rectangle,
    DisplayStyleCount
  };

  int displayStyle_;
  void setDisplayStyle(int ds);
  int getDisplayStyle() { return displayStyle_; }
  rviz::EnumPropertyWPtr displayStyle_property_;

  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

protected:
  std::map<int, std::vector<ogre_tools::BillboardLine*> > lines;
  std::vector<ogre_tools::BillboardLine*> recycleLines;
  ogre_tools::BillboardLine* newBillboardLine();

  void subscribe();
  void unsubscribe();

  void clearVector(std::vector<ogre_tools::BillboardLine*>& vec);
  void clearMap();
  void clearDisplay();

  void incomingTrack(const articulation_msgs::TrackMsg::ConstPtr& msg);

  message_filters::Subscriber<articulation_msgs::TrackMsg> sub_;
  tf::MessageFilter<articulation_msgs::TrackMsg> tf_filter_;

  articulation_msgs::TrackMsg::ConstPtr incoming_track_message_;

  typedef std::vector<articulation_msgs::TrackMsg::ConstPtr> V_TrackMsg;
  V_TrackMsg message_queue_;
  boost::mutex queue_mutex_;

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  bool transform(const articulation_msgs::TrackMsg::ConstPtr& message, btTransform &transform);
  bool transform(const btTransform &pose, const btVector3 &scaleIn,Ogre::Vector3& pos, Ogre::Quaternion& orient, Ogre::Vector3& scaleOut);

  void createRectangle(Ogre::Vector3 pos,Ogre::Quaternion orient,Ogre::Vector3 scale,double w,double h,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec);

  void createAxes(Ogre::Vector3 pos,Ogre::Quaternion orient,Ogre::Vector3 scale,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec);

  void createLine(Ogre::Vector3 pos,Ogre::Vector3 old_pos,Ogre::Vector3 scale,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec,bool add_cross);

  btVector3 modifyColor( btVector3 color,  int colorStyle, float f );
};

}

 #endif


