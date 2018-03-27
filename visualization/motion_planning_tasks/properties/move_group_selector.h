#pragma once

#include "property_factory.h"

namespace moveit { namespace core {
class RobotModel;
} }
namespace rviz {
class EditableEnumProperty;
}

namespace moveit_rviz_plugin
{

rviz::Property* createMoveGroupSelector(const QString& name,
                                        moveit::task_constructor::Property& prop,
                                        const moveit::core::RobotModel *robot_model);

void fillMoveGroupList(rviz::EditableEnumProperty& property,
                       const moveit::core::RobotModel& robot_model);

} // end namespace moveit_rviz_plugin
