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
#include <fcl/octree.h>
#include <fcl/traversal/traversal_node_octree.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/math/transform.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner {
public:
	Planner();
	~Planner();	
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

	std::shared_ptr<fcl::CollisionGeometry> Quadcopter;

	std::shared_ptr<fcl::CollisionGeometry> tree_obj;

	// Flag for initialization
	bool set_start = false;

	void init_start(void);

	void setStart(double x, double y, double z);

	void setGoal(double x, double y, double z);

	void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);

	void plan(void);

	void replan(void);

	bool isStateValid(const ob::State *state);

	ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);

};


// void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, planner* planner_ptr)
// {


//     //loading octree from binary
// 	 // const std::string filename = "/home/rrc/power_plant.bt";
// 	 // octomap::OcTree temp_tree(0.1);
// 	 // temp_tree.readBinary(filename);
// 	 // fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
	

// 	// convert octree to collision object
// 	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
// 	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
	
// 	// Update the octree used for collision checking
// 	planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
// 	planner_ptr->replan();
// }

// void odomCb(const nav_msgs::Odometry::ConstPtr &msg, planner* planner_ptr)
// {
// 	planner_ptr->setStart(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
// 	planner_ptr->init_start();
// }

// void startCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
// {
// 	planner_ptr->setStart(msg->point.x, msg->point.y, msg->point.z);
// 	planner_ptr->init_start();
// }

// void goalCb(const geometry_msgs::PointStamped::ConstPtr &msg, planner* planner_ptr)
// {
// 	planner_ptr->setGoal(msg->point.x, msg->point.y, msg->point.z);
// }

#endif