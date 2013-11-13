#ifndef ARTICULATION_RVIZ_PLUGIN_MODEL_DISPLAY_H
#define ARTICULATION_RVIZ_PLUGIN_MODEL_DISPLAY_H

#include "rviz/display.h"
#include "rviz/common.h"
#include "rviz/properties/forwards.h"
#include "rviz/helpers/color.h"

#include <ros/ros.h>

#include "articulation_msgs/ModelMsg.h"
#include "articulation_msgs/TrackMsg.h"
#include "articulation_models/models/factory.h"

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <ogre_tools/arrow.h>
#include <ogre_tools/shape.h>
#include <ogre_tools/billboard_line.h>
#include <ogre_tools/movable_text.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
namespace articulation_rviz_plugin
{

/**
 * \class ModelDisplay
 * \brief
 */
class ModelDisplay : public rviz::Display
{
public:
	articulation_models::MultiModelFactory factory;
	ModelDisplay( const std::string& name, rviz::VisualizationManager* manager );
  virtual void createProperties();
  virtual ~ModelDisplay();

  virtual void onEnable();
  virtual void onDisable();

  std::string model_topic_;
  void setTopic( const std::string& topic );
  const std::string& getTopic() { return model_topic_; }
  rviz::ROSTopicStringPropertyWPtr topic_property_;

  enum UniqueStyle
  {
    u_all,
    u_single,
    u_track,
    u_model,
    u_modelstored,
    u_modeltype
  };

  int unique_;
  void setUnique(int cs);
  int getUnique() { return unique_; }
  rviz::EnumPropertyWPtr unique_property_;

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

  int modelColor_;
  void setModelColor(int cs);
  int getModelColor() { return modelColor_; }
  rviz::EnumPropertyWPtr modelColor_property_;

  int poseColor_;
  void setPoseColor(int cs);
  int getPoseColor() { return poseColor_; }
  rviz::EnumPropertyWPtr poseColor_property_;

  int modelTypeColor_;
  void setModelTypeColor(int cs);
  int getModelTypeColor() { return modelTypeColor_; }
  rviz::EnumPropertyWPtr modelTypeColor_property_;

  int likelihoodColor_;
  void setLikelihoodColor(int cs);
  int getLikelihoodColor() { return likelihoodColor_; }
  rviz::EnumPropertyWPtr likelihoodColor_property_;

  int forceColor_;
  void setForceColor(int cs);
  int getForceColor() { return forceColor_; }
  rviz::EnumPropertyWPtr forceColor_property_;

  enum DisplayStyle
  {
    ds_line,
    ds_cross_line,
    ds_cross,
    ds_arrow,
    ds_axes,
    ds_rectangle,
    ds_none,
    DisplayStyleCount
  };

  int displayStyle_;
  void setDisplayStyle(int ds);
  int getDisplayStyle() { return displayStyle_; }
  rviz::EnumPropertyWPtr displayStyle_property_;

  bool sampleConfigurations_;
  void setSampleConfigurations(bool sc);
  bool getSampleConfigurations() { return sampleConfigurations_; }
  rviz::BoolPropertyWPtr sampleConfigurations_property_;

  bool projectConfigurations_;
  void setProjectConfigurations(bool sc);
  bool getProjectConfigurations() { return projectConfigurations_; }
  rviz::BoolPropertyWPtr projectConfigurations_property_;

  bool showJacobians_;
  void setShowJacobians(bool sc);
  bool getShowJacobians() { return showJacobians_; }
  rviz::BoolPropertyWPtr showJacobians_property_;

  bool showHessians_;
  void setShowHessians(bool sc);
  bool getShowHessians() { return showHessians_; }
  rviz::BoolPropertyWPtr showHessians_property_;

  bool showLatestOnly_;
  void setShowLatestOnly(bool sc);
  bool getShowLatestOnly() { return showLatestOnly_; }
  rviz::BoolPropertyWPtr showLatestOnly_property_;

  float sampleDensity_;
  void setSampleDensity(float a);
  float getSampleDensity() { return sampleDensity_; }
  rviz::FloatPropertyWPtr sampleDensity_property_;

  bool showModelName_;
  void setShowModelName(bool sc);
  bool getShowModelName() { return showModelName_; }
  rviz::BoolPropertyWPtr showModelName_property_;

  bool showForces_;
  void setShowForces(bool sc);
  bool getShowForces() { return showForces_; }
  rviz::BoolPropertyWPtr showForces_property_;

  float forceScaling_;
  void setForceScaling(float a);
  float getForceScaling() { return forceScaling_; }
  rviz::FloatPropertyWPtr forceScaling_property_;

  virtual void targetFrameChanged();
  virtual void fixedFrameChanged();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

protected:
  std::map<int, std::vector<ogre_tools::Arrow*> > arrows;
  std::map<int, std::vector<ogre_tools::BillboardLine*> > lines;
  std::map<int, std::vector<ogre_tools::MovableText* > > text;
  std::vector<ogre_tools::BillboardLine*> recycleLines;
  std::vector<ogre_tools::Arrow*> recycleArrows;
  std::vector<ogre_tools::MovableText*> recycleText;
  ogre_tools::BillboardLine* newBillboardLine();
  ogre_tools::Arrow* newArrow();
  ogre_tools::MovableText* newText();

  void subscribe();
  void unsubscribe();

  void clearVector(std::vector<ogre_tools::BillboardLine*>& vec);
  void clearVector(std::vector<ogre_tools::Arrow*>& vec);
  void clearVector(std::vector<ogre_tools::MovableText*>& vec);
  void clearMap();
  void clearDisplay();
  void clearResources(int uniqueId);

  void incomingModel(const articulation_msgs::ModelMsg::ConstPtr& msg);

  message_filters::Subscriber<articulation_msgs::ModelMsg> sub_;
  tf::MessageFilter<articulation_msgs::ModelMsg> tf_filter_;

  articulation_msgs::ModelMsg::ConstPtr incoming_model_message_;

  typedef std::vector<articulation_msgs::ModelMsg::ConstPtr> V_ModelMsg;
  V_ModelMsg message_queue_;
  boost::mutex queue_mutex_;

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  bool transform(const articulation_msgs::ModelMsg::ConstPtr& message, btTransform &transform);
  bool transform(const btTransform &pose, const btVector3 &scaleIn,Ogre::Vector3& pos, Ogre::Quaternion& orient, Ogre::Vector3& scaleOut);

  void createRectangle(Ogre::Vector3 pos,Ogre::Quaternion orient,Ogre::Vector3 scale,double w,double h,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec);

  void createAxes(Ogre::Vector3 pos,Ogre::Quaternion orient,Ogre::Vector3 scale,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec);

  void createLine(Ogre::Vector3 pos,Ogre::Vector3 old_pos,Ogre::Vector3 scale,btVector3 color,
		  std::vector<ogre_tools::BillboardLine*> &vec,bool add_cross,bool with_line);

  void createArrow(Ogre::Vector3 pos,Ogre::Vector3 old_pos,Ogre::Vector3 scale,btVector3 color,
		  std::vector<ogre_tools::Arrow*> &vec);

  void createText(Ogre::Vector3 pos,Ogre::Quaternion orient,Ogre::Vector3 scale,btVector3 color,std::string title,
		  std::vector<ogre_tools::MovableText*> &vec);

  btVector3 modifyColor( btVector3 color,  int colorStyle, float f );
};

}

 #endif


