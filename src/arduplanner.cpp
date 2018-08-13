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

#include <arduplanner.h>

ArduPlanner::ArduPlanner(cv::Mat _start, cv::Mat _goal, double _velocity, bool _return_back, int _replan_interval, int _executor_interval, bool _visualize_octomap, double _min_range, double _max_range):
  start(_start),
  goal(_goal),
  velocity(_velocity),
  return_back(_return_back),
  replan_interval(_replan_interval),
  executor_interval(_executor_interval),
  visualize_octomap(_visualize_octomap),
  min_range(_min_range),
  max_range(_max_range)
{
  // Initialize octomap object
  o_map = std::make_shared<OctomapServer>();

  // Initialize stereo matcher object
  o_stereo = std::make_shared<StereoMatcher>(std::string(SRC_DIR) + std::string("/config/camera_calibration.yaml"));

  // Initialize Gazebo visualization object
  o_gazebovis = std::make_shared<GazeboVis>();
  o_gazebovis->clearAll();

  // Initialize Planner object
  o_planner = std::make_shared<Planner>();

  // Initialize mavlink object
  mavlink_service = std::make_shared<boost::asio::io_service>();
  mavlink_work = std::make_shared<boost::asio::io_service::work>(*mavlink_service);
  o_mavlink = std::make_shared<MavlinkComm>(14551, 14550, &*mavlink_service);

  // Run mavlink in a seperate thread for async polling
  boost::thread mavlink_thread(boost::bind(&boost::asio::io_service::run, &*mavlink_service));
  mavlink_service->post(boost::bind(&MavlinkComm::run, o_mavlink));
  mavlink_thread.detach();

  // Setup plan executor async thread
  executor_service = std::make_shared<boost::asio::io_service>();
  executor_work = std::make_shared<boost::asio::io_service::work>(*executor_service);
  executor_timer = std::make_shared<boost::asio::deadline_timer>(*executor_service, boost::posix_time::seconds(replan_interval));
  boost::thread executor_thread(boost::bind(&boost::asio::io_service::run, &*executor_service));
  executor_timer->async_wait(boost::bind(&ArduPlanner::executePlan,this));
  executor_thread.detach();

  // Setup replanning thread as async callback
  replanning_service = std::make_shared<boost::asio::io_service>();
  replan_work = std::make_shared<boost::asio::io_service::work>(*replanning_service);
  replan_timer = std::make_shared<boost::asio::deadline_timer>(*replanning_service, boost::posix_time::seconds(1.5 * replan_interval));
  boost::thread replan_thread(boost::bind(&boost::asio::io_service::run, &*replanning_service));
  replan_timer->async_wait(boost::bind(&ArduPlanner::replanAsync,this));
  replan_thread.detach();

}

ArduPlanner::~ArduPlanner()
{
 mavlink_work.reset();
 mavlink_service->stop();
 executor_work.reset();
 executor_service->stop();
 replan_work.reset();
 replanning_service->stop();
}

// Function to insert pointclouds to octomap
void ArduPlanner::insertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz)
{
  octomap_fused = false;
  Eigen::Matrix4f transform_cam_world;
  transform_cam_world << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1; //Rotate camera optical to ENU;
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz, transform_cam_world);

  std::lock_guard<std::mutex> guard(octomap_mutex);
  o_map->insertCloudCallback(cloud_xyz, sensorToWorld);
  octomap_fused = true;
  is_octomap_processed.notify_all();
}

// Function to fuse multiple pointclouds accoring to their transformations
// void ArduPlanner::fuseCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_xyzrgb)
// {
//   pcl::transformPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, sensorToWorld);
//   final_cloud += *cloud_xyzrgb;
// }

// Function to process pointcloud data 
void ArduPlanner::processCloud()
{
  cv::Mat points = o_stereo->getPointcloud();
  
  float scale = 10; // Not sure why its needed but gives correct metric scale pointcloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  // Insert points to PCL pointcloud
  for (int rows = 0; rows < points.rows; ++rows) { 
  for (int cols = 0; cols < points.cols; ++cols) { 
     cv::Point3f point = points.at<cv::Point3f>(rows, cols);

     // Filter out points outside certain distance
     float dist = scale * std::sqrt(point.z * point.z + point.y * point.y + point.x * point.x);
     if(dist > min_range && dist < max_range)
     {
      pcl::PointXYZ pcl_point(scale * point.x, scale * point.y, scale * point.z); // normal PointCloud 
      cloud_xyz->push_back(pcl_point); 
     }
    } 
  }
  // Process octomap from the pointclouds in a new thread
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, [this](){return octomap_fused;});
  boost::thread octomapThread(boost::bind(&ArduPlanner::insertCloud, this, cloud_xyz));
  octomapThread.detach();
  lk.unlock();
  is_cloud_processed = true;
}

void ArduPlanner::executePlan()
{
  if(new_plan.load())
  {
    new_plan = false;
    DBG("Recieved new plan for executing");
    auto path = o_planner->getSmoothPath();
    double x, y, z, prev_x = o_mavlink->pos_msg.x, prev_y = o_mavlink->pos_msg.y, prev_z = o_mavlink->pos_msg.z, pause, distance_nearest, prev_distance = std::numeric_limits<double>::max();
    bool init_start_pose = false;

    for(auto pose: path)
    {
      if(!new_plan.load())
      { 
        x = std::get<0>(pose);
        y = -std::get<1>(pose);
        z = -std::get<2>(pose);
        distance_nearest =  std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2) + std::pow(z - prev_z, 2));
        if(distance_nearest < prev_distance && !init_start_pose)
          prev_distance = distance_nearest;
        else
        {
          init_start_pose = true;
          pause = distance_nearest/velocity;
          o_mavlink->gotoNED(x, y, z, std::atan2(y - prev_y,x - prev_x));
          prev_x = x;
          prev_y = y;
          prev_z = z;
          DBG("Pose: " << x << " " << y << " " << z);
          std::this_thread::sleep_for(std::chrono::duration<double>(pause));
        }
      }
      else
        break;
    }
    if(new_plan.load()) // If the above loop was break due to new plan then create a new executor thread
    {
      DBG("New plan found creating new thread");
      executor_timer->expires_from_now(boost::posix_time::milliseconds(1));
      executor_timer->async_wait(boost::bind(&ArduPlanner::executePlan,this));
    }
    else
    {
      INFO("Reached the goal");

      // Stop the replanner
      replan_work.reset();
      replanning_service->stop();

      // Save the map
      std::unique_lock<std::mutex> lk(octomap_mutex);
      is_octomap_processed.wait(lk, [this](){return octomap_fused;});
      o_map->m_octree->writeBinary("map.bt");
      lk.unlock();
      INFO("octomap saved");

      if(return_back)
      {
        INFO("Returning Back");
        for(auto pose: boost::adaptors::reverse(path))
        {
          x = std::get<0>(pose);
          y = -std::get<1>(pose);
          z = -std::get<2>(pose);
          distance_nearest =  std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2) + std::pow(z - prev_z, 2));
          pause = distance_nearest/velocity;
          o_mavlink->gotoNED(x, y, z, std::atan2(y - prev_y,x - prev_x));
          pause = distance_nearest/velocity;
          prev_x = x;
          prev_y = y;
          prev_z = z;
          std::this_thread::sleep_for(std::chrono::duration<double>(pause));
        }
        o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
        std::raise(SIGKILL);
      }
    }
  }
  else // If no new plan available then wait for a while and again call executor
  {
    DBG("No new plan available, waiting for " << executor_interval << " seconds before next callback");
    executor_timer->expires_from_now(boost::posix_time::seconds(executor_interval));
    executor_timer->async_wait(boost::bind(&ArduPlanner::executePlan,this));
  }
}

// Function to carryout replanning
void ArduPlanner::replanCb()
{
  DBG("Replanner called");
  replan_finished = false;
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, [this](){return octomap_fused;});
  o_planner->updateMap(*o_map->m_octree);
  if(visualize_octomap)
  {
    o_gazebovis->clearOctree();
    o_gazebovis->visOctree(*o_map->m_octree);
  }
  lk.unlock();
  
  o_planner->setGoal(goal.at<float>(0), goal.at<float>(1), goal.at<float>(2));
  if(o_planner->setStart(o_mavlink->pos_msg.x, -o_mavlink->pos_msg.y, -o_mavlink->pos_msg.z))
  {
    if(o_planner->replan())
    {
      DBG("New plan generated");
      // Visualize the plan
      o_gazebovis->clearAll();
      auto path = o_planner->getSmoothPath();
      o_gazebovis->addLine(path);
      new_plan = true;
    }
  }
  else
  {
    ERROR("Invalid start state");
    
    std::unique_lock<std::mutex> lk(octomap_mutex);
    is_octomap_processed.wait(lk, [this](){return octomap_fused;});
    o_map->m_octree->writeBinary("map.bt");
    lk.unlock();
    INFO("octomap saved");
    // o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
    // ERROR("EXITING");
    // std::raise(SIGKILL);
  }

  std::lock_guard<std::mutex> guard(planner_mutex);
  replan_finished = true;
  is_replan_processed.notify_all();
}

// Async worker which calls replanner at replan_interval
void ArduPlanner::replanAsync()
{ 

  if(!replan_finished)
  {
    DBG("Waiting for previous replanner to terminate");
    std::unique_lock<std::mutex> lk(planner_mutex, std::try_to_lock);
    is_replan_processed.wait(lk, [this](){return replan_finished;});
  }
  boost::thread replanner_thread(boost::bind(&ArduPlanner::replanCb, this));
  replanner_thread.detach();

  replan_timer->expires_from_now(boost::posix_time::seconds(replan_interval));
  replan_timer->async_wait(boost::bind(&ArduPlanner::replanAsync,this));
}

// Rotate drone 360 degree to create an initial map before planning
void ArduPlanner::initializeManeuver(double yaw_feedforward)
{
  processCloudThread();
  INFO("Desired Yaw: " << yaw_feedforward);
  o_mavlink->gotoNED(start.at<float>(0), -start.at<float>(1), -start.at<float>(2), yaw_feedforward);
}

void ArduPlanner::updateSensorToWorld()
{
  sensorToWorld = Eigen::MatrixXf::Identity(4,4);
  Eigen::Matrix3f orientation;

  orientation = Eigen::AngleAxisf(-o_mavlink->orientation_msg.yaw, Eigen::Vector3f::UnitZ())
              * Eigen::AngleAxisf(o_mavlink->orientation_msg.roll, Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(-o_mavlink->orientation_msg.pitch, Eigen::Vector3f::UnitY());
  sensorToWorld.block<3,3>(0,0) = orientation;
  sensorToWorld.block<3,1>(0,3) = Eigen::Vector3f(o_mavlink->pos_msg.x, -o_mavlink->pos_msg.y, -o_mavlink->pos_msg.z);
}

void ArduPlanner::processCloudThread()
{
  is_cloud_processed = false;
  boost::thread pointcloudThread(boost::bind(&ArduPlanner::processCloud, this));
  pointcloudThread.detach();
}