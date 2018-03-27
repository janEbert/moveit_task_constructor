#include "frame_selector.h"
#include <moveit/task_constructor/properties.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz/properties/editable_enum_property.h>

namespace moveit_rviz_plugin {

rviz::Property* createFrameSelector(const QString& name, moveit::task_constructor::Property& mtc_prop,
                                    const planning_scene::PlanningScene* scene)
{
	std::string value;
	if (!mtc_prop.value().empty())
		value = boost::any_cast<std::string>(mtc_prop.value());

	rviz::EditableEnumProperty* rviz_prop = new rviz::EditableEnumProperty(name, QString::fromStdString(value),
	                                                                       QString::fromStdString(mtc_prop.description()));
	if (scene) fillFrameList(*rviz_prop, *scene);
	QObject::connect(rviz_prop, &rviz::EditableEnumProperty::changed,
	                 [rviz_prop, &mtc_prop]() {mtc_prop.setValue(rviz_prop->getStdString());});
	return rviz_prop;
}

void fillFrameList(rviz::EditableEnumProperty& property,
                   const planning_scene::PlanningScene& scene)
{
	property.clearOptions();

	for (const std::string& name : scene.getWorld()->getObjectIds())
		property.addOptionStd(name);

	for (const std::string& name : scene.getRobotModel()->getLinkModelNames())
		property.addOptionStd(name);
}

} // end namespace moveit_rviz_plugin
