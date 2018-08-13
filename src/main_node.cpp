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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

#include <arduplanner.h>

std::shared_ptr<ArduPlanner> o_arduplanner;
int count = 0;
// Function is called everytime an image message is received from Gazebo
void imageCb(ConstImagesStampedPtr &msg)
{
  if(o_arduplanner->is_cloud_processed.load())
  {
    o_arduplanner->updateSensorToWorld();
    int width;
    int height;
    char *data_l;
    char *data_r;

    width = (int) msg->image(0).width();
    height = (int) msg->image(0).height();
    data_l = new char[msg->image(0).data().length() + 1];
    data_r = new char[msg->image(1).data().length() + 1];

    memcpy(data_l, msg->image(0).data().c_str(), msg->image(0).data().length());
    cv::Mat image_l(height, width, CV_8UC3, data_l);

    memcpy(data_r, msg->image(1).data().c_str(), msg->image(1).data().length());
    cv::Mat image_r(height, width, CV_8UC3, data_r);
    cv::cvtColor(image_l, image_l, CV_BGR2RGB);
    cv::cvtColor(image_r, image_r, CV_BGR2RGB);

    auto startTime = std::chrono::system_clock::now();
    
    cv::Mat disparity = o_arduplanner->o_stereo->matchPair(image_l,image_r);
    
    std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
    // DBG("Disparity computation time: " << total_elapsed.count());
    
    cv::imshow("disparity", disparity);

    cv::waitKey(10);
    delete data_l;
    delete data_r;
    if(std::abs(o_arduplanner->sensorToWorld(2,3)) > 1e-2)
    {
      if(count < 90)
        o_arduplanner->initializeManeuver(count * 4 * 3.14/180.0);
      else
      {
        o_arduplanner->processCloudThread();
      }
      count++;
    }
    else
      DBG("Mavlink not initialized, current height " << std::abs(o_arduplanner->sensorToWorld(2,3)));
  }
}

int main(int _argc, char **_argv)
{
  int replan_interval, executor_interval;
  cv::Mat start, goal;
  bool return_back, visualize_octomap;
  double velocity, min_range, max_range;
  std::string params_file = std::string(SRC_DIR) + std::string("/config/planner_params.yaml");

  // Read planner parameters from the file
  cv::FileStorage fs(params_file, cv::FileStorage::READ);
  if(!fs.isOpened())
  {
    ERROR("Failed to open planner_params.yaml\nExiting");
    std::raise(SIGKILL);
  }
  else
  {
    fs["start"] >> start;
    fs["goal"] >> goal;
    fs["velocity"] >> velocity;
    fs["return_back"] >> return_back;
    fs["min_range"] >> min_range;
    fs["max_range"] >> max_range;
    fs["replan_interval"] >> replan_interval;
    fs["executor_interval"] >> executor_interval;
    fs["visualize_octomap"] >> visualize_octomap;
  }

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  o_arduplanner = std::make_shared<ArduPlanner>(start, goal, velocity, return_back, replan_interval, executor_interval, visualize_octomap, min_range, max_range);
  // Listen to Gazebo topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/iris_demo/cam_link/stereo_camera/images", imageCb);

  // Busy wait loop
  while (true)
  {  
    gazebo::common::Time::MSleep(10);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}