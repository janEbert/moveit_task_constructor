#include "move_group_selector.h"
#include <moveit/task_constructor/properties.h>
#include <moveit/robot_model/robot_model.h>
#include <rviz/properties/editable_enum_property.h>

namespace moveit_rviz_plugin {

rviz::Property* createMoveGroupSelector(const QString& name, moveit::task_constructor::Property& mtc_prop,
                                        const moveit::core::RobotModel* robot_model)
{
	std::string value;
	if (!mtc_prop.value().empty())
		value = boost::any_cast<std::string>(mtc_prop.value());

	rviz::EditableEnumProperty* rviz_prop = new rviz::EditableEnumProperty(name, QString::fromStdString(value),
	                                                                       QString::fromStdString(mtc_prop.description()));
	if (robot_model) fillMoveGroupList(*rviz_prop, *robot_model);
	QObject::connect(rviz_prop, &rviz::EditableEnumProperty::changed,
	                 [rviz_prop, &mtc_prop]() {mtc_prop.setValue(rviz_prop->getStdString());});
	return rviz_prop;
}

void fillMoveGroupList(rviz::EditableEnumProperty& property,
                       const moveit::core::RobotModel& robot_model)
{
	property.clearOptions();
	for (const std::string& name : robot_model.getJointModelGroupNames())
		property.addOptionStd(name);
	property.sortOptions();
}

} // end namespace moveit_rviz_plugin
