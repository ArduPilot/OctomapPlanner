#include "gazebo_visualization.h"

GazeboVis::GazeboVis(){
	
	_node = std::unique_ptr<ignition::transport::Node> (new ignition::transport::Node);

	// Create the marker message
	_markerMsg = std::unique_ptr<ignition::msgs::Marker> (new ignition::msgs::Marker);
	_markerMsg->set_ns("default");
	_markerMsg->set_type(ignition::msgs::Marker::LINE_LIST);
	_markerMsg->mutable_material()->mutable_script()->set_name("Gazebo/Green");
	
	_markerMsgPoint = std::unique_ptr<ignition::msgs::Marker> (new ignition::msgs::Marker);
	_markerMsgPoint->set_ns("default");
	_markerMsgPoint->set_type(ignition::msgs::Marker::SPHERE);
	_markerMsgPoint->mutable_material()->mutable_script()->set_name("Gazebo/Red");
	ignition::msgs::Set(_markerMsgPoint->mutable_scale(),
	                  ignition::math::Vector3d(0.2, 0.2, 0.2));

	_prev_id_line = 1;
	_prev_id_point = 0;
}

bool GazeboVis::addLine(std::vector<std::tuple<double, double, double>>& points){

	_markerMsg->clear_point();
	_markerMsg->set_action(ignition::msgs::Marker::ADD_MODIFY);
	_markerMsg->set_id(_prev_id_line);
	_prev_id_line += 2;
	for(auto point : points)
		ignition::msgs::Set(_markerMsg->add_point(),
		    ignition::math::Vector3d(std::get<0>(point), std::get<1>(point), std::get<2>(point)));
	return _node->Request("/marker", *_markerMsg.get());
}

bool GazeboVis::addPoint(double x, double y, double z){

	_markerMsgPoint->set_action(ignition::msgs::Marker::ADD_MODIFY);
	_markerMsgPoint->set_id(_prev_id_point);
	_prev_id_point += 2;
	DBG("ID: " << _prev_id_point << " Pos: " << x << " " << y << " " << z);
	ignition::msgs::Set(_markerMsgPoint->mutable_pose(),
	                    ignition::math::Pose3d(x, y, z, 0, 0, 0));
	return _node->Request("/marker", *_markerMsgPoint.get());
}

bool GazeboVis::clearAll(){

	_markerMsg->set_action(ignition::msgs::Marker::DELETE_ALL);
	_markerMsgPoint->set_action(ignition::msgs::Marker::DELETE_ALL);

	if(_node->Request("/marker", *_markerMsg.get()) && _node->Request("/marker", *_markerMsgPoint.get()))
	{
		_prev_id_line = 0;
		_prev_id_point = 0;
		return true;
	}
	else
		return false;
}

bool GazeboVis::clearPreviousLine(){
	
	_markerMsg->set_id(_prev_id_line - 2);	
	_markerMsg->set_action(ignition::msgs::Marker::DELETE_MARKER);
	return _node->Request("/marker", *_markerMsg.get());
}

bool GazeboVis::clearPreviousPoint(){
	
	_markerMsg->set_id(_prev_id_point - 2);	
	_markerMsg->set_action(ignition::msgs::Marker::DELETE_MARKER);
	return _node->Request("/marker", *_markerMsgPoint.get());
}