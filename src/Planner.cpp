/*
*   OctomapPlanner
*
*   Copyright (C) 2018  ArduPilot
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*   Author Ayush Gaud <ayush.gaud[at]gmail.com>
*/

#include "Planner.h"

// Constructor
Planner::Planner(void)
{
	aircraftObject = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(1.5, 1.5, 1.5)));
	// fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
	// tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
	// treeObj = std::make_shared<fcl::CollisionObject<double>>((std::shared_ptr<fcl::CollisionGeometry<double>>(tree)));
	
	space = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));

	// create a start state
	ob::ScopedState<ob::RealVectorStateSpace> start(space);
	
	// create a goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(space);

	// set the bounds for the R^3
	ob::RealVectorBounds bounds(3);

	bounds.setLow(0,-10);
	bounds.setHigh(0,10);
	bounds.setLow(1,-10);
	bounds.setHigh(1,10);
	bounds.setLow(2,0.5);
	bounds.setHigh(2,3.5);

	space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

	// construct an instance of  space information from this state space
	si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

	// start->setXYZ(0,0,0);
	start->values[0] = 0;
	start->values[1] = 0;
	start->values[2] = 0;
	// start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// start.random();

	// goal->setXYZ(0,0,0);
	goal->values[0] = 0;
	goal->values[1] = 0;
	goal->values[2] = 0;

	// goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// goal.random();
	
    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));

	// create a problem instance
	pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

    // set Optimizattion objective
	pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

    // create a planner for the defined space
	o_plan = ob::PlannerPtr(new og::InformedRRTstar(si));

    // set the problem we are trying to solve for the planner
	o_plan->setProblemDefinition(pdef);

    // perform setup steps for the planner
	o_plan->setup();

	INFO("Planner Initialized");

}

// Destructor
Planner::~Planner()
{
}

bool Planner::setStart(double x, double y, double z)
{
	ob::ScopedState<ob::RealVectorStateSpace> start(space);
	start->values[0] = x;
	start->values[1] = y;
	start->values[2] = z;
	ob::State *state =  space->allocState();
	state->as<ob::RealVectorStateSpace::StateType>()->values = start->values;
	if(isStateValid(state)) // Check if the start state is valid
	{	
		pdef->clearStartStates();
		pdef->addStartState(start);
		DBG("Start point set to: " << x << " " << y << " " << z);
		return true;
	}
	else
	{
		ERROR("Start state: " << x << " " << y << " " << z << " invalid");
		return false;
	}
}

bool Planner::setGoal(double x, double y, double z)
{
	ob::ScopedState<ob::RealVectorStateSpace> goal(space);
	goal->values[0] = x;
	goal->values[1] = y;
	goal->values[2] = z;
	pdef->clearGoal();
	pdef->setGoalState(goal);
	ob::State *state =  space->allocState();
	state->as<ob::RealVectorStateSpace::StateType>()->values = goal->values;
	if(isStateValid(state)) // Check if the goal state is valid
	{	
		DBG("Goal point set to: " << x << " " << y << " " << z);
		return true;
	}
	else
	{
		ERROR("Goal state: " << x << " " << y << " " << z << " invalid");
		return false;
	}
}

void Planner::updateMap(octomap::OcTree tree_oct)
{
	// convert octree to collision object
	fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::make_shared<const octomap::OcTree>(tree_oct));
	std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
	treeObj = std::make_shared<fcl::CollisionObject<double>>((tree_obj));
}

bool Planner::replan(void)
{	
	if(path_smooth != NULL)
	{
		og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
		DBG("Total Points:" << path->getStateCount ());
		double distance;
		if(pdef->hasApproximateSolution())
		{
			DBG("Goal state not satisfied and distance to goal is: " << pdef->getSolutionDifference());
			replan_flag = true;
		}
		else
		{
			for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
			{
				if(!replan_flag)
				{
					replan_flag = !isStateValid(path->getState(idx));
				}
				else
					break;
			}
		}
	}
	if(replan_flag)
	{
		pdef->clearSolutionPaths();
		DBG("Replanning");
		plan();
		return true;
	}
	else
	{
		DBG("Replanning not required");
		return false;
	}
}

void Planner::plan(void)
{

    // attempt to solve the problem within four seconds of planning time
	ob::PlannerStatus solved = o_plan->solve(4);

	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		DBG("Found solution:");
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
		
        //Path smoothing using bspline
		og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(*path_smooth);
		// pathBSpline->collapseCloseVertices(*path_smooth);
		
		replan_flag = false;

	}
	else
		DBG("No solution found");
}

bool Planner::isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
	const ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();

    // check validity of state defined by pos
	fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);
	// INFO("State: " << translation);
	aircraftObject->setTranslation(translation);
	fcl::CollisionRequest<double> requestType(1,false,1,false);
	fcl::CollisionResult<double> collisionResult;
	fcl::collide(aircraftObject.get(), treeObj.get(), requestType, collisionResult);

	return(!collisionResult.isCollision());
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.

ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	// obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}


std::vector<std::tuple<double, double, double>> Planner::getSmoothPath()
{
	std::vector<std::tuple<double, double, double>> path;
	for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
	{
        // cast the abstract state type to the type we expect
		const ob::RealVectorStateSpace::StateType *pos = path_smooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();
		path.push_back(std::tuple<double, double, double>(pos->values[0], pos->values[1], pos->values[2]));
	}
	return path;
}