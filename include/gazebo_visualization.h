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