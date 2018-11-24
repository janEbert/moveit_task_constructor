#include "pose_stamped_selector.h"

#include "frame_selector.h"
#include <agni_tf_tools/rotation_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>

#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <interactive_markers/tools.h>

#include <moveit/planning_scene/planning_scene.h>

const QString FIXED_FRAME_STRING = "<Fixed Frame>";
enum MarkerType { NONE, FRAME, IFRAME, DOF6 };

namespace moveit_rviz_plugin
{

PoseStampedSelector::PoseStampedSelector(const QString& name,
                                         const geometry_msgs::PoseStamped &default_value,
                                         const QString& description, rviz::Property* parent,
                                         const char* changed_slot, QObject* receiver)
   : rviz::Property(name, QVariant(), description, parent, changed_slot, receiver)
{
	parent_frame_property_ = new rviz::EditableEnumProperty("Parent Frame", FIXED_FRAME_STRING, QString(),
	                                                        this, SLOT(onRefFrameChanged()), this);
	translation_property_ = new rviz::VectorProperty("Translation", Ogre::Vector3::ZERO,
	                                                 "Relative translation to parent frame",
	                                                 this, SLOT(onTransformChanged()), this);
	rotation_property_ = new agni_tf_tools::RotationProperty(this, "Rotation");

	connect(rotation_property_, SIGNAL(quaternionChanged(Eigen::Quaterniond)),
	        this, SLOT(onTransformChanged()));

	marker_property_ = new rviz::EnumProperty("Marker Type", "Interactive frame",
	                                          "Which type of interactive marker to use",
	                                          this, SLOT(createInteractiveMarker()), this);
	marker_property_->addOption("none", NONE);
	marker_property_->addOption("static frame", FRAME);
	marker_property_->addOption("interactive frame", IFRAME);
	marker_property_->addOption("6 DoF handles", DOF6);

	marker_scale_property_ = new rviz::FloatProperty("Marker Scale", 0.2, QString(), marker_property_,
	                                                 SLOT(onMarkerScaleChanged()), this);
	marker_property_->hide();  // hide as long as we don't have a DisplayContext
}

PoseStampedSelector::~PoseStampedSelector() {}

void PoseStampedSelector::setPlanningScene(const planning_scene::PlanningScene* scene)
{
	parent_frame_property_->clearOptions();
	if (scene) fillFrameList(*parent_frame_property_, *scene);
}

void PoseStampedSelector::setContext(rviz::DisplayContext* context)
{
	if (marker_node_) {
		imarker_.reset();
		context_->getSceneManager()->getRootSceneNode()->removeChild(marker_node_);
		delete marker_node_;
		marker_node_ = nullptr;
	}
	if ((context_ = context)) {
		marker_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
		createInteractiveMarker();
	}

	if (marker_node_)
		marker_property_->show();
	else
		marker_property_->hide();
}

static void updatePose(geometry_msgs::Pose &pose,
                       const Eigen::Quaterniond &q,
                       Ogre::Vector3 p = Ogre::Vector3::ZERO)
{
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  pose.position.x = p.x;
  pose.position.y = p.y;
  pose.position.z = p.z;
}

inline void setOrientation(geometry_msgs::Quaternion &q, double w, double x, double y, double z) {
  q.w = w;
  q.x = x;
  q.y = y;
  q.z = z;
}

static visualization_msgs::Marker createArrowMarker(double scale,
                                                    const Eigen::Vector3d &dir,
                                                    const QColor &color) {
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::ARROW;
	marker.scale.x = scale;
	marker.scale.y = 0.1*scale;
	marker.scale.z = 0.1*scale;

	updatePose(marker.pose,
	           Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir));

	marker.color.r = color.redF();
	marker.color.g = color.greenF();
	marker.color.b = color.blueF();
	marker.color.a = color.alphaF();

	return marker;
}

void PoseStampedSelector::add6DOFControls(visualization_msgs::InteractiveMarker &im) {
	visualization_msgs::InteractiveMarkerControl ctrl;
	ctrl.always_visible = false;

	setOrientation(ctrl.orientation, 1, 1,0,0);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	ctrl.name = "x pos";
	im.controls.push_back(ctrl);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	ctrl.name = "x rot";
	im.controls.push_back(ctrl);

	setOrientation(ctrl.orientation, 1, 0,1,0);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	ctrl.name = "y pos";
	im.controls.push_back(ctrl);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	ctrl.name = "y rot";
	im.controls.push_back(ctrl);

	setOrientation(ctrl.orientation, 1, 0,0,1);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	ctrl.name = "z pos";
	im.controls.push_back(ctrl);
	ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	ctrl.name = "z rot";
	im.controls.push_back(ctrl);
}

void PoseStampedSelector::addFrameControls(visualization_msgs::InteractiveMarker &im, double scale, bool interactive)
{
	visualization_msgs::InteractiveMarkerControl ctrl;
	setOrientation(ctrl.orientation, 1, 0,0,0);
	ctrl.always_visible = true;
	if (interactive) {
		ctrl.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
		ctrl.independent_marker_orientation = true;
		ctrl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
	}
	ctrl.name = "frame";

	ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitX(), QColor("red")));
	ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitY(), QColor("green")));
	ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitZ(), QColor("blue")));

	im.controls.push_back(ctrl);
}

void PoseStampedSelector::createInteractiveMarker()
{
	int type = marker_property_->getOptionInt();
	if (type == NONE || !marker_node_) {
		if (imarker_) imarker_.reset();
		return;
	}

	float scale = marker_scale_property_->getFloat();

	visualization_msgs::InteractiveMarker im;
	im.name = "";
	im.scale = scale;
	if (!fillPoseStamped(im.header, im.pose))
		return;

	if (type == FRAME || type == IFRAME)
		addFrameControls(im, 1.0, type == IFRAME);
	else if (type == DOF6) {
		addFrameControls(im, 0.5, type == IFRAME);
		add6DOFControls(im);
	}

	imarker_.reset(new rviz::InteractiveMarker(marker_node_, context_));
	connect(imarker_.get(), SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)),
	        this, SLOT(onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback&)));

	// fill in default controls
	interactive_markers::autoComplete(im, true);

	imarker_->processMessage(im);
	imarker_->setShowVisualAids(false);
	imarker_->setShowAxes(false);
	imarker_->setShowDescription(false);

	marker_property_->show();
}

bool PoseStampedSelector::fillPoseStamped(std_msgs::Header &header,
                                          geometry_msgs::Pose &pose)
{
	const std::string &parent_frame = parent_frame_property_->getStdString();
	std::string error;
	if (context_->getFrameManager()->transformHasProblems(parent_frame, ros::Time(), error)) {
		return false;
	}

	const Eigen::Quaterniond &q = rotation_property_->getQuaternion();
	const Ogre::Vector3 &p = translation_property_->getVector();
	updatePose(pose, q, p);
	header.frame_id = parent_frame;
	// frame-lock marker to update marker pose with frame updates
	header.stamp = ros::Time();
	return true;
}

void PoseStampedSelector::onRefFrameChanged()
{
	onFramesChanged();
}

void PoseStampedSelector::onFramesChanged()
{
	// update marker pose
	visualization_msgs::InteractiveMarkerPose marker_pose;
	fillPoseStamped(marker_pose.header, marker_pose.pose);
	if (imarker_) imarker_->processMessage(marker_pose);
}

void PoseStampedSelector::onTransformChanged()
{
	if (ignore_updates_) return;

	visualization_msgs::InteractiveMarkerPose marker_pose;
	fillPoseStamped(marker_pose.header, marker_pose.pose);

	// update marker pose
	ignore_updates_ = true;
	if (imarker_) imarker_->processMessage(marker_pose);
	ignore_updates_ = false;
}

void PoseStampedSelector::onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
	if (ignore_updates_) return;
	if (feedback.event_type != visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
		return;

	// convert to parent frame
	const geometry_msgs::Point &p_in = feedback.pose.position;
	const geometry_msgs::Quaternion &q_in = feedback.pose.orientation;

	tf::Stamped<tf::Pose> pose_in(tf::Transform(tf::Quaternion(q_in.x, q_in.y, q_in.z, q_in.w),
	      tf::Vector3(p_in.x, p_in.y, p_in.z)),
	      feedback.header.stamp, feedback.header.frame_id);
	tf::Stamped<tf::Pose> pose_out;
	try {
		context_->getTFClient()->transformPose(parent_frame_property_->getStdString(), pose_in, pose_out);
	} catch(const std::runtime_error &e) {
		ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s",
		          feedback.header.frame_id.c_str(),
		          qPrintable(parent_frame_property_->getString()),
		          e.what());
		return;
	}

	const tf::Vector3 &p = pose_out.getOrigin();
	const tf::Quaternion &q = pose_out.getRotation();

	ignore_updates_ = true;
	translation_property_->setVector(Ogre::Vector3(p.x(), p.y(), p.z()));
	rotation_property_->setQuaternion(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
	ignore_updates_ = false;

	updatePose(feedback.pose, rotation_property_->getQuaternion(),
	           translation_property_->getVector());
}

void PoseStampedSelector::onMarkerScaleChanged()
{
	if (marker_scale_property_->getFloat() <= 0)
		marker_scale_property_->setFloat(0.2);
	createInteractiveMarker();
}

} // end namespace moveit_rviz_plugin
