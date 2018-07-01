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

std::shared_ptr<MavlinkComm> o_mavlink;
std::shared_ptr<OctomapServer> o_map;
std::shared_ptr<StereoMatcher> o_stereo;
std::shared_ptr<GazeboVis> o_gazebovis;
std::shared_ptr<Planner> o_planner;
pcl::PointCloud<pcl::PointXYZRGB> final_cloud;
int count = 0;
boost::mutex pointcloud_mutex;
std::mutex octomap_mutex;
bool is_cloud_processed = true, octomap_fused = true, planner_called = false;
std::condition_variable is_octomap_processed;

void insertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Eigen::Matrix4f sensorToWorld)
{
  octomap_fused = false;
  Eigen::Matrix4f transform_cam_world;
  transform_cam_world << 0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1; //Rotate camera optical to ENU;
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz, transform_cam_world);
  // DBG(sensorToWorld);
  o_map->insertCloudCallback(cloud_xyz, sensorToWorld);
  
  std::lock_guard<std::mutex> guard(octomap_mutex);
  is_octomap_processed.notify_one();
  octomap_fused = true;
}

void fuseCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_xyzrgb, Eigen::Matrix4f sensorToWorld)
{
  pcl::transformPointCloud(*cloud_xyzrgb, *cloud_xyzrgb, sensorToWorld);
  final_cloud += *cloud_xyzrgb;
}
void processCloud(cv::Mat image_l, Eigen::Matrix4f sensorToWorld)
{
  auto startTime = std::chrono::system_clock::now();
  cv::Mat points = o_stereo->getPointcloud();
  
  std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  DBG("Disp to pointcloud: " << total_elapsed.count());
  
  float scale = 10; // Not sure why its needed but gives correct metric scale pointcloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  for (int rows = 0; rows < points.rows; ++rows) { 
  for (int cols = 0; cols < points.cols; ++cols) { 
     cv::Point3f point = points.at<cv::Point3f>(rows, cols);
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
  DBG("Pointcloud insertion time: " << total_elapsed.count());

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud_xyz);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (0.1);
  // sor.filter (*cloud_xyz);

  // pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
  // sor1.setInputCloud (cloud_xyzrgb);
  // sor1.setMeanK (50);
  // sor1.setStddevMulThresh (0.1);
  // sor1.filter (*cloud_xyzrgb);

  // total_elapsed = std::chrono::system_clock::now() - startTime;
  // startTime = std::chrono::system_clock::now();
  // DBG"Filter time: " << total_elapsed.count());

  INFO("Last Pos: " << o_mavlink->pos_msg.x << " " << o_mavlink->pos_msg.y << " " << o_mavlink->pos_msg.z << " " << o_mavlink->orientation_msg.yaw);
  
  std::unique_lock<std::mutex> lk(octomap_mutex);
  is_octomap_processed.wait(lk, []{return octomap_fused;});
  boost::thread octomapThread(insertCloud, cloud_xyz, sensorToWorld);
  octomapThread.detach();
  
  // Integrate the pointcloud (needed only for visualization)
  fuseCloud(cloud_xyzrgb, sensorToWorld);
  
  total_elapsed = std::chrono::system_clock::now() - startTime;
  startTime = std::chrono::system_clock::now();
  DBG("Fusion time: " << total_elapsed.count());

  boost::lock_guard<boost::mutex> guard(pointcloud_mutex);
  is_cloud_processed = true;
}
void plannerCb()
{
	o_gazebovis->clearAll();
	o_gazebovis->addPoint(0, 2, 1.5);
	o_mavlink->gotoNED(0, -2, -1.5,0); // goto start
	usleep(3e6);
	o_planner->updateMap(o_map->m_octree);
	o_planner->setStart(0, 2, 1.5);
	o_planner->setGoal(5, 2, 1.5);
	o_planner->plan();
	auto path = o_planner->getSmoothPath();
	o_gazebovis->addLine(path);
	size_t num_points = path.size();
	double pause = 1e7/(double)num_points; // 10 seconds for path i.e. 0.5m/s
	for (auto pose: path)
	{
		o_mavlink->gotoNED(std::get<0>(pose), -std::get<1>(pose), -std::get<2>(pose),0);
		usleep(pause);
	}
	usleep(5e6);
	for (auto pose: boost::adaptors::reverse(path))
	{
		o_mavlink->gotoNED(std::get<0>(pose), -std::get<1>(pose), -std::get<2>(pose),0);
		usleep(pause);
	}
	usleep(2e6);
	// pcl::io::savePCDFile ("test_pcd.pcd", final_cloud, false);
	o_mavlink->gotoNED(0, 0, o_mavlink->pos_msg.z, 0);
	std::raise(SIGKILL);
}

// Function is called everytime an image message is received.
void cb(ConstImagesStampedPtr &msg)
{
  pointcloud_mutex.lock();
  if(is_cloud_processed == true)
  {
    pointcloud_mutex.unlock();
    Eigen::Matrix4f sensorToWorld = Eigen::MatrixXf::Identity(4,4);
    Eigen::Matrix3f orientation;
    // orientation = Eigen::AngleAxisf(o_mavlink->orientation_msg.yaw, Eigen::Vector3f::UnitY())
    //             * Eigen::AngleAxisf(o_mavlink->orientation_msg.roll, Eigen::Vector3f::UnitZ())
    //             * Eigen::AngleAxisf(o_mavlink->orientation_msg.pitch, Eigen::Vector3f::UnitX());
    // sensorToWorld.block<3,3>(0,0) = orientation;
    // sensorToWorld.block<3,1>(0,3) = Eigen::Vector3f(o_mavlink->pos_msg.y,o_mavlink->pos_msg.z,o_mavlink->pos_msg.x);

    orientation = Eigen::AngleAxisf(-o_mavlink->orientation_msg.yaw, Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(-o_mavlink->orientation_msg.roll, Eigen::Vector3f::UnitZ())
                * Eigen::AngleAxisf(o_mavlink->orientation_msg.pitch, Eigen::Vector3f::UnitX());
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
    DBG("Disparity computation time: " << total_elapsed.count());
    
    cv::imshow("disparity", disparity);

    cv::waitKey(10);
    delete data_l;
    delete data_r;
    count++;
    if(count > 50)
    {
      if(!planner_called)
      {
	      planner_called = true;
	      std::unique_lock<std::mutex> lk(octomap_mutex);
	      is_octomap_processed.wait(lk, []{return octomap_fused;});
	      o_map->m_octree->writeBinary("map.bt");
	      boost::thread planner_thread(plannerCb);
	      planner_thread.detach();
	  }
    }
    else
    {
      // DBG(sensorToWorld);
      if(abs(sensorToWorld(2,3)) < 1e-2)
      {
        // DBG("Z " << abs(sensorToWorld(2,3)));
        count--;
      }
      else
      {
        is_cloud_processed = false;
        boost::thread pointcloudThread(processCloud, image_l, sensorToWorld);
        pointcloudThread.detach();
        o_mavlink->gotoNED(0, -count/10.0, -1.5, 0);
        o_gazebovis->addPoint(0.0, count/10.0, 1.5);
      }
    }
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
  // Listen to Gazebo topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/iris_demo/cam_link/stereo_camera/images", cb);
  
  // Initialize octomap object
  o_map = std::make_shared<OctomapServer>();

  // Initialize stereo matcher object
  o_stereo = std::make_shared<StereoMatcher>(std::string(SRC_DIR) + std::string("/config/camera_calibration.yaml"));

  boost::asio::io_service io_service;

  // Initialize Gazebo visualization object
  o_gazebovis = std::make_shared<GazeboVis>();
  o_gazebovis->clearAll();

  // Initialize Planner object
  o_planner = std::make_shared<Planner>();
  // Initialize mavlink object
  o_mavlink = std::make_shared<MavlinkComm>(14551, 14550, &io_service);

  // Run mavlink in a seperate thread for async polling
  boost::thread io_thread(boost::bind(&boost::asio::io_service::run, &io_service));
  io_service.post(boost::bind(&MavlinkComm::run, o_mavlink));
  io_thread.join();

  // Busy wait loop
  while (true)
  {  
    gazebo::common::Time::MSleep(100);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
