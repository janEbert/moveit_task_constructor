// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/stage.h>

namespace moveit { namespace task_constructor { namespace stages {

class GenerateGraspPose : public Generator {
public:
	GenerateGraspPose(std::string name);

	void init(const planning_scene::PlanningSceneConstPtr& scene) override;
	bool canCompute() const override;
	bool compute() override;

	void setEndEffector(std::string eef);

	void setGroup(std::string group_name);

	void setLink(std::string ik_link);

	void setGripperGraspPose(std::string pose_name);

	void setObject(std::string object);

	void setGraspOffset(double grasp_offset);

	void setTimeout(double timeout);

	void setAngleDelta(double delta);

	void setMaxIKSolutions(uint32_t n);

	void ignoreCollisions(bool flag);

protected:
	planning_scene::PlanningSceneConstPtr scene_;

	std::string eef_;

	std::string group_;

	std::string ik_link_;

	double grasp_offset_ = 0.0;

	uint32_t max_ik_solutions_;

	bool ignore_collisions_ = false;

	std::string gripper_grasp_pose_;

	std::string object_;

	double timeout_ = 0.1;

	double angle_delta_ = 0.1;

	/* temp values */

	double current_angle_ = 0.0;

	double remaining_time_;

	bool tried_current_state_as_seed_ = false;

	std::vector< std::vector<double> > previous_solutions_;
};

} } }