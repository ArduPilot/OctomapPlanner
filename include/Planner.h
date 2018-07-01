#ifndef PLANNER_H
#define PLANNER_H

#include <octomap/octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
// #include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include "debug_definitions.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner {
public:
	Planner();
	~Planner();

	void init_start(void);

	void setStart(double x, double y, double z);

	void setGoal(double x, double y, double z);

	void updateMap(octomap::OcTree* tree_oct);

	void plan(void);

	void replan(void);

	std::vector<std::tuple<double, double, double>> getSmoothPath();

private:

	// construct the state space we are planning in
	ob::StateSpacePtr space;

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si;

	// create a problem instance
	ob::ProblemDefinitionPtr pdef;

	// goal state
	double prev_goal[3];

	og::PathGeometric* path_smooth = NULL;

	bool replan_flag = false;

	// std::shared_ptr<fcl::CollisionGeometry<double>> Quadcopter;

	// std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj;

	std::shared_ptr<fcl::CollisionObject<double>> treeObj;

	std::shared_ptr<fcl::CollisionObject<double>> aircraftObject;
	// Flag for initialization
	bool set_autostart = false;


	bool isStateValid(const ob::State *state);

	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);


};

#endif