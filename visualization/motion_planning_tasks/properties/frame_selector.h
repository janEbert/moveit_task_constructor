#pragma once

#include "property_factory.h"

namespace planning_scene {
class PlanningScene;
}
namespace rviz {
class EditableEnumProperty;
}

namespace moveit_rviz_plugin
{

rviz::Property* createFrameSelector(const QString& name,
                                    moveit::task_constructor::Property& prop,
                                    const planning_scene::PlanningScene *ps);

void fillFrameList(rviz::EditableEnumProperty& property,
                   const planning_scene::PlanningScene& scene);

} // end namespace moveit_rviz_plugin
