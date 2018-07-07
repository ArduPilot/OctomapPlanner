/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl/common/common_headers.h"
#include "pcl/io/io.h"
#include "pcl/visualization//pcl_visualizer.h"
#include <pcl/io/pcd_io.h>
#include "pcl/point_cloud.h"
#include <pcl/filters/voxel_grid.h>
#include "pcl/visualization/cloud_viewer.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <csignal> // for SIGTERM
#include <mutex>

#include "stereo_matcher.h"
#include "OctomapServer.h"
#include "mavlink_comm.h"

#include <condition_variable>

#include "gazebo_visualization.h"
#include "Planner.h"


// Global variables

// Basic class objects from libraries
std::shared_ptr<MavlinkComm> o_mavlink;
std::shared_ptr<OctomapServer> o_map;
std::shared_ptr<StereoMatcher> o_stereo;
std::shared_ptr<GazeboVis> o_gazebovis;
std::shared_ptr<Planner> o_planner;

// Other variables
boost::mutex pointcloud_mutex;
std::mutex octomap_mutex;
std::mutex planner_mutex;
std::mutex newplan_mutex;
bool is_cloud_processed = true, octomap_fused = true, planner_called = false, planner_init = false, replan_finished = true, plan_executed = false, new_plan = false;
std::condition_variable is_octomap_processed;
std::condition_variable is_replan_processed;
pcl::PointCloud<pcl::PointXYZRGB> final_cloud;
double velocity = 0.1;

int count = 0;
// For async replanner worker
int replan_interval = 5; //replan every 5 second
std::shared_ptr<boost::asio::deadline_timer> replan_timer;

// For plan executor
std::shared_ptr<boost::asio::deadline_timer> executor_timer;

// Function to insert pointclouds to octomap
void insertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Eigen::Matrix4f sensorToWorld)
{
  octomap_fused = false;
  Eigen::Matrix4f transform_cam_world;
  transform_cam_world << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1; //Rotate camera optical to ENU;
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz, transform_cam_world);

  std::lock_guard<std::mutex> ugard(octomap_mutex);
  o_map->insertCloudCallback(cloud_xyz, sensorToWorld);
  is_octomap_processed.notify_one();
  octomap_fused = true;
}

// Function to fuse multiple pointclouds accoring to their transformations
void fuseCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_xyzrgb, Eigen::Matrix4f sensorToWorld)
{
  pcl::transformPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, sensorToWorld);
  final_cloud += *cloud_xyzrgb;
}

// (Overloaded) Function to process pointcloud data 
/*
void processCloud(cv::Mat image_l, Eigen::Matrix4f sensorToWorld)
{
  auto startTime = std::chrono::system_clock::now();
  cv::Mat points = o_stereo->getPointcloud();
  
  std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  // DBG("Disparity to points: " << total_elapsed.count());
  
  float scale = 10; // Not sure why its needed but gives correct metric scale pointcloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  // Insert points to PCL pointcloud
  for (int rows = 0; rows < points.rows; ++rows) { 
  for (int cols = 0; cols < points.cols; ++cols) { 
     cv::Point3f point = points.at<cv::Point3f>(rows, cols);

     // Filter out points outside certain range of X Y and Z
     if(scale * point.z <= 7.0 && scale * point.z >= 0.5 && scale * point.y >= -2.5 && scale * point.y <= 2.5 && scale * point.x <= 5.0 && scale * point.x >= -5.0)
     {
       pcl::PointXYZ pcl_point(scale * point.x, scale * point.y, scale * point.z); // normal PointCloud 

       pcl::PointXYZRGB pcl_point_rgb;
       pcl_point_rgb.x = scale * point.x;    // rgb PointCloud 
       pcl_point_rgb.y = scale * point.y; 
       pcl_point_rgb.z = scale * point.z; 
       cv::Vec3b intensity = image_l.at<cv::Vec3b>(rows,cols); //BGR 
       uint32_t rgb = (static_cast<uint32_t>(intensity[2]) << 16 | static_cast<uint32_t>(intensity[1]) << 8 | static_cast<uint32_t>(intensity[0])); 
       pcl_point_rgb.rgb = *reinterpret_cast<float*>(&rgb);

       cloud_xyz->push_back(pcl_point); 
       cloud_xyzrgb->push_back(pcl_point_rgb);
     }
    } 
  }
  
  total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  // DBG("Pointcloud insertion time: " << total_elapsed.count());

  // INFO("Last Pos: " << o_mavlink->pos_msg.x << " " << o_mavlink->pos_msg.y << " " << o_mavlink->pos_msg.z << " " << o_mavlink->orientation_msg.yaw);
  
  // Process octomap from the pointclouds in a new thread
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, []{return octomap_fused;});
  boost::thread octomapThread(insertCloud, cloud_xyz, sensorToWorld);
  octomapThread.detach();

  boost::lock_guard<boost::mutex> guard(pointcloud_mutex);
  is_cloud_processed = true;
}
*/
// Function to process pointcloud data 
void processCloud(Eigen::Matrix4f sensorToWorld)
{
  auto startTime = std::chrono::system_clock::now();
  cv::Mat points = o_stereo->getPointcloud();
  
  std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  // DBG("Disparity to points: " << total_elapsed.count());
  
  float scale = 10; // Not sure why its needed but gives correct metric scale pointcloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  // Insert points to PCL pointcloud
  for (int rows = 0; rows < points.rows; ++rows) { 
  for (int cols = 0; cols < points.cols; ++cols) { 
     cv::Point3f point = points.at<cv::Point3f>(rows, cols);

     // Filter out points outside certain range of X Y and Z
     if(scale * point.z <= 5.0 && scale * point.z >= 0.5 && scale * point.y >= -3 && scale * point.y <= 3 && scale * point.x <= 3.0 && scale * point.x >= -3.0)
     {
       pcl::PointXYZ pcl_point(scale * point.x, scale * point.y, scale * point.z); // normal PointCloud 
       cloud_xyz->push_back(pcl_point); 
     }
    } 
  }
  
  total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  // DBG("Pointcloud insertion time: " << total_elapsed.count());

  // INFO("Last Pos: " << o_mavlink->pos_msg.x << " " << o_mavlink->pos_msg.y << " " << o_mavlink->pos_msg.z << " " << o_mavlink->orientation_msg.yaw);
  
  // Process octomap from the pointclouds in a new thread
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, []{return octomap_fused;});
  boost::thread octomapThread(insertCloud, cloud_xyz, sensorToWorld);
  octomapThread.detach();

  boost::lock_guard<boost::mutex> guard(pointcloud_mutex);
  is_cloud_processed = true;
}
// Function to initialize planner with basic parameters
void initPlanner()
{
  o_gazebovis->clearAll();
  o_mavlink->gotoNED(0, 0, -1,0);
  // o_gazebovis->addPoint(0, 2, 1.5);
  // o_mavlink->gotoNED(0, -2, -1.5,0); // goto start
  // usleep(2e6); // Give 2 seconds to reach start

  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, []{return octomap_fused;});
  o_planner->updateMap(*o_map->m_octree);

  o_planner->setStart(0, 0, 1);
  o_planner->setGoal(5, 2, 1.5);

  planner_init = true;
  DBG("Planner initialized");
}

void executePlan(const boost::system::error_code& /*e*/)
{
  newplan_mutex.lock();
  if(new_plan)
  {
    new_plan = false;
    newplan_mutex.unlock();
    DBG("Recieved new plan for executing");
    auto path = o_planner->getSmoothPath();
    double x, y, z, prev_x = o_mavlink->pos_msg.x, prev_y = -o_mavlink->pos_msg.y, prev_z = -o_mavlink->pos_msg.z, pause;
    for (auto pose: path)
    {
      newplan_mutex.lock();
      if(!new_plan)
      { 
        newplan_mutex.unlock();
        x = std::get<0>(pose);
        y = -std::get<1>(pose);
        z = -std::get<2>(pose);
        pause = std::sqrt(std::pow(x - prev_x, 2) + std::pow(y - prev_y, 2) + std::pow(z - prev_z, 2))/velocity;
        o_mavlink->gotoNED(x, y, z,0);
        prev_x = x;
        prev_y = y;
        prev_z = z;
        DBG("Pause " << pause);
        usleep(pause * 1e6);
      }
      else
      {
        newplan_mutex.unlock();
        break;
      }
    }
    newplan_mutex.lock();
    if(new_plan) // If the above loop was break due to new plan then create a new executor thread
    {
      DBG("New plan found creating new thread");
      newplan_mutex.unlock();
      executor_timer->expires_from_now(boost::posix_time::seconds(0));
      executor_timer->async_wait(executePlan); 
    }
    else
    {
      INFO("Reached the goal");
      o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
      std::raise(SIGKILL);
    }
  }
  else // If no new plan available then wait for a while and again call executor
  {
    newplan_mutex.unlock();
    DBG("No new plan available, waiting for a second before next callback");
    executor_timer->expires_from_now(boost::posix_time::seconds(1));
    executor_timer->async_wait(executePlan);
  }
}

// Function to carryout replanning
void replanCb()
{
  DBG("Replanner Called");
  replan_finished = false;
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, []{return octomap_fused;});
  o_planner->updateMap(*o_map->m_octree);
  if(o_planner->setStart(o_mavlink->pos_msg.x, -o_mavlink->pos_msg.y, -o_mavlink->pos_msg.z))
  {
    if(o_planner->replan())
    {
      // Visualize the plan
      o_gazebovis->clearAll();
      auto path = o_planner->getSmoothPath();
      o_gazebovis->addLine(path);
      newplan_mutex.lock();
      new_plan = true;
      newplan_mutex.unlock();
    }
  }
  else
  {
    ERROR("Invalid start state\nEXITING");
    std::unique_lock<std::mutex> lk(octomap_mutex);
    is_octomap_processed.wait(lk, []{return octomap_fused;});
    o_map->m_octree->writeBinary("map.bt");
    o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
    std::raise(SIGKILL);
  }
  // INFO("Pose: " << o_mavlink->pos_msg.x << " " << -o_mavlink->pos_msg.y << " " << -o_mavlink->pos_msg.z);

  std::lock_guard<std::mutex> guard(planner_mutex);
  is_replan_processed.notify_one();
  replan_finished = true;
}

// Async worker which calls replanner at replan_interval
void replanAsync(const boost::system::error_code& /*e*/)
{ 
  // DBG("Async Test");
  if(planner_init)
  {
    std::unique_lock<std::mutex> lk(planner_mutex);
    is_replan_processed.wait(lk, []{return replan_finished;});
    boost::thread replanner_thread(replanCb);
    replanner_thread.detach();
  }
  replan_timer->expires_from_now(boost::posix_time::seconds(replan_interval));
  replan_timer->async_wait(replanAsync);
}

// Rotate drone 360 degree to create an initial map before planning
void initializeManeuver(double yaw_feedforward, Eigen::Matrix4f sensorToWorld)
{
  is_cloud_processed = false;
  boost::thread pointcloudThread(processCloud, sensorToWorld);
  pointcloudThread.detach();
  INFO("Desired Yaw: " << yaw_feedforward);
  o_mavlink->gotoNED(0, 0, -1, yaw_feedforward);
}

// Function is called everytime an image message is received from Gazebo
void imageCb(ConstImagesStampedPtr &msg)
{
  pointcloud_mutex.lock();
  if(is_cloud_processed == true)
  {
    pointcloud_mutex.unlock();
    Eigen::Matrix4f sensorToWorld = Eigen::MatrixXf::Identity(4,4);
    Eigen::Matrix3f orientation;

    orientation = Eigen::AngleAxisf(-o_mavlink->orientation_msg.yaw, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(o_mavlink->orientation_msg.roll, Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(-o_mavlink->orientation_msg.pitch, Eigen::Vector3f::UnitY());
    sensorToWorld.block<3,3>(0,0) = orientation;
    sensorToWorld.block<3,1>(0,3) = Eigen::Vector3f(o_mavlink->pos_msg.x, -o_mavlink->pos_msg.y, -o_mavlink->pos_msg.z);

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
    
    cv::Mat disparity = o_stereo->matchPair(image_l,image_r);
    
    std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
    // DBG("Disparity computation time: " << total_elapsed.count());
    
    cv::imshow("disparity", disparity);

    cv::waitKey(10);
    delete data_l;
    delete data_r;
    if(std::abs(sensorToWorld(2,3)) > 1e-2)
    {
      if(count < 90)
        initializeManeuver(count * 4 * 3.14/180.0 , sensorToWorld);
      else
      {
        if(!planner_called)
        {
          planner_called = true;
          boost::thread planner_thread(initPlanner);
          planner_thread.detach();
        }
        else
        {
          if(count < 1000) // Keep on processing till 1000 counts and then terminate
          {
            is_cloud_processed = false;
            boost::thread pointcloudThread(processCloud, sensorToWorld);
            pointcloudThread.detach();
          }
          else
          {
            std::unique_lock<std::mutex> lk(octomap_mutex);
            is_octomap_processed.wait(lk, []{return octomap_fused;});
            o_map->m_octree->writeBinary("map.bt");
            o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
            std::raise(SIGKILL);
          }
        }
      }
      count++;
    }
    else
      DBG("Mavlink not initialized, current height" << std::abs(sensorToWorld(2,3)));
  }
  else
    pointcloud_mutex.unlock();
}

int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  
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
  boost::asio::io_service io_service;
  o_mavlink = std::make_shared<MavlinkComm>(14551, 14550, &io_service);

  // Run mavlink in a seperate thread for async polling
  boost::thread io_thread(boost::bind(&boost::asio::io_service::run, &io_service));
  io_service.post(boost::bind(&MavlinkComm::run, o_mavlink));
  io_thread.detach();

  // Setup replanning thread as async callback
  boost::asio::io_service replanning_service;
  boost::asio::io_service::work replan_work(replanning_service);
  replan_timer = std::make_shared<boost::asio::deadline_timer>(replanning_service, boost::posix_time::seconds(replan_interval));
  boost::thread replan_thread(boost::bind(&boost::asio::io_service::run, &replanning_service));
  replan_timer->async_wait(replanAsync);
  replan_thread.detach();

  // Setup plan executor async thread
  boost::asio::io_service executor_service;
  boost::asio::io_service::work executor_work(executor_service);
  executor_timer = std::make_shared<boost::asio::deadline_timer>(replanning_service, boost::posix_time::seconds(1));
  boost::thread executor_thread(boost::bind(&boost::asio::io_service::run, &executor_service));
  executor_timer->async_wait(executePlan);
  executor_thread.detach();

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
