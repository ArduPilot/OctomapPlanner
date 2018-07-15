#ifndef GAZEBO_VISUALIZATION_H
#define GAZEBO_VISUALIZATION_H

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>
#include <octomap/octomap.h>

#include "debug_definitions.h"

class GazeboVis{

public:
	GazeboVis();
	
	bool addLine(std::vector<std::tuple<double, double, double>>& points);
	
	bool addPoint(double x, double y, double z);

	bool clearAll();
	
	bool clearPreviousLine();

	bool clearPreviousPoint();

	void visOctree(octomap::OcTree& octree);

	void clearOctree();

private:
	std::unique_ptr<ignition::transport::Node> _node;

	std::unique_ptr<ignition::msgs::Marker> _markerMsg;
	
	std::unique_ptr<ignition::msgs::Marker> _markerMsgPoint;

	std::unique_ptr<ignition::msgs::Marker> _markerMsgTree;

	int _prev_id_line;

	int _prev_id_point;
	
	int _prev_id_tree;
};

#endif