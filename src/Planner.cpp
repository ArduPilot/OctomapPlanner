#include "Planner.h"

// Constructor
Planner::Planner(void)
{
	Quadcopter = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(0.3, 0.3, 0.1));
	// fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.15)));
	// tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
	
	space = ob::StateSpacePtr(new ob::SE3StateSpace());

	// create a start state
	ob::ScopedState<ob::SE3StateSpace> start(space);
	
	// create a goal state
	ob::ScopedState<ob::SE3StateSpace> goal(space);

	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);

	bounds.setLow(0,-20);
	bounds.setHigh(0,20);
	bounds.setLow(1,-20);
	bounds.setHigh(1,20);
	bounds.setLow(2,0);
	bounds.setHigh(2,20);

	space->as<ob::SE3StateSpace>()->setBounds(bounds);

	// construct an instance of  space information from this state space
	si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

	start->setXYZ(0,0,0);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// start.random();

	goal->setXYZ(0,0,0);
	prev_goal[0] = 0;
	prev_goal[1] = 0;
	prev_goal[2] = 0;
	goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	// goal.random();
	
    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1 ));

	// create a problem instance
	pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);

    // set Optimizattion objective
	pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));

	std::cout << "Initialized: " << std::endl;
}

// Destructor
Planner::~Planner()
{
}

void Planner::init_start(void)
{
	if(!set_start)
		std::cout << "Initialized" << std::endl;
	set_start = true;
}
void Planner::setStart(double x, double y, double z)
{
	ob::ScopedState<ob::SE3StateSpace> start(space);
	start->setXYZ(x,y,z);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
	pdef->clearStartStates();
	pdef->addStartState(start);
}
void Planner::setGoal(double x, double y, double z)
{
	if(prev_goal[0] != x || prev_goal[1] != y || prev_goal[2] != z)
	{
		ob::ScopedState<ob::SE3StateSpace> goal(space);
		goal->setXYZ(x,y,z);
		prev_goal[0] = x;
		prev_goal[1] = y;
		prev_goal[2] = z;
		goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
		pdef->clearGoal();
		pdef->setGoalState(goal);
		std::cout << "Goal point set to: " << x << " " << y << " " << z << std::endl;
		if(set_start)
			plan();
		
	}
}
void Planner::updateMap(octomap::OcTree* tree_oct)
{
	// convert octree to collision object
	fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(tree_oct));
	tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
}
void Planner::replan(void)
{
	if(path_smooth != NULL && set_start)
	{
		std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
		if(path_smooth->getStateCount () <= 2)
			plan();
		else
		{
			for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
			{
				if(!replan_flag)
					replan_flag = !isStateValid(path_smooth->getState(idx));
				else
					break;

			}
			if(replan_flag)
				plan();
			else
				std::cout << "Replanning not required" << std::endl;
		}
	}
}

void Planner::plan(void)
{

    // create a planner for the defined space
	ob::PlannerPtr plan(new og::InformedRRTstar(si));

    // set the problem we are trying to solve for the planner
	plan->setProblemDefinition(pdef);

    // perform setup steps for the planner
	plan->setup();

    // print the settings for this space
	si->printSettings(std::cout);

    // print the problem settings
	pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
	ob::PlannerStatus solved = plan->solve(2);

	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);
        // print the path to screen
        // path->print(std::cout);
		// path_smooth = pth;
		
        //Path smoothing using bspline

		og::PathSimplifier* pathBSpline = new og::PathSimplifier(si);
		path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
		pathBSpline->smoothBSpline(*path_smooth,3);

		// std::cout << "Smoothed Path" << std::endl;
		// path_smooth.print(std::cout);
		
		// Clear memory
		pdef->clearSolutionPaths();
		replan_flag = false;

	}
	else
		std::cout << "No solution found" << std::endl;
}

bool Planner::isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	fcl::CollisionObject<double> treeObj((tree_obj));
	fcl::CollisionObject<double> aircraftObject(Quadcopter);

    // check validity of state defined by pos & rot
	fcl::Vector3<double> translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion<double> rotation(rot->w, rot->x, rot->y, rot->z);
	aircraftObject.setTransform(rotation, translation);
	fcl::CollisionRequest<double> requestType(1,false,1,false);
	fcl::CollisionResult<double> collisionResult;
	fcl::collide(&aircraftObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());
}

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr Planner::getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	// obj->setCostThreshold(ob::Cost(1.51));
	return obj;
}

ob::OptimizationObjectivePtr Planner::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}

std::vector<std::tuple<double, double, double>> Planner::getSmoothPath()
{
	std::vector<std::tuple<double, double, double>> path;

	for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
	{
        // cast the abstract state type to the type we expect
		const ob::SE3StateSpace::StateType *se3state = path_smooth->getState(idx)->as<ob::SE3StateSpace::StateType>();
		// extract the first component of the state and cast it to what we expect
		const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

		path.push_back(std::tuple<double, double, double>(pos->values[0], pos->values[1], pos->values[2]));
	}

	return path;
}