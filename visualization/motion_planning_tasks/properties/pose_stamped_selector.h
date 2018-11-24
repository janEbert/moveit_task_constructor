#pragma once

#include <rviz/properties/property.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>

namespace agni_tf_tools
{
class RotationProperty;
}

namespace planning_scene
{
class PlanningScene;
}

namespace rviz
{
class BoolProperty;
class FloatProperty;
class EnumProperty;
class EditableEnumProperty;
class StatusProperty;
class VectorProperty;
class InteractiveMarker;
class DisplayContext;
}

namespace moveit_rviz_plugin
{

class PoseStampedSelector : public rviz::Property
{
	Q_OBJECT
public:
	PoseStampedSelector(const QString& name = QString(),
	                    const geometry_msgs::PoseStamped& default_value = geometry_msgs::PoseStamped(),
	                    const QString& description = QString(),
	                    Property* parent = nullptr,
	                    const char* changed_slot = nullptr,
	                    QObject* receiver = nullptr);
	~PoseStampedSelector();

	void setPlanningScene(const planning_scene::PlanningScene*scene);
	void setContext(rviz::DisplayContext* context);

private Q_SLOTS:
	void onRefFrameChanged();
	void onFramesChanged();
	void onTransformChanged();
	void onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
	void onMarkerScaleChanged();
	void createInteractiveMarker();

private:
	void add6DOFControls(visualization_msgs::InteractiveMarker &im);
	void addFrameControls(visualization_msgs::InteractiveMarker &im,
	                      double scale, bool interactive);
	bool fillPoseStamped(std_msgs::Header &header, geometry_msgs::Pose &pose);

	// rviz properties
	rviz::EditableEnumProperty* parent_frame_property_;
	rviz::VectorProperty* translation_property_;
	agni_tf_tools::RotationProperty* rotation_property_;
	rviz::EnumProperty* marker_property_;
	rviz::FloatProperty* marker_scale_property_;

	// interactive marker
	boost::shared_ptr<rviz::InteractiveMarker> imarker_;
	Ogre::SceneNode* marker_node_ = nullptr;
	bool ignore_updates_ = false;

	rviz::DisplayContext* context_ = nullptr;
};

} // end namespace moveit_rviz_plugin
