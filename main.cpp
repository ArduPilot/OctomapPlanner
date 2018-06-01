/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

#include "stereo_matcher.h"
#include "OctomapServer.h"
#include "mavlink_comm.h"


std::shared_ptr<MavlinkComm> o_mavlink;
std::shared_ptr<OctomapServer> o_map;
std::shared_ptr<StereoMatcher> o_stereo;
int count = 0;

void insertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_xyz)
{
  Eigen::Matrix4f identity = Eigen::MatrixXf::Identity(4,4);
  Eigen::Matrix4f sensorToWorld = Eigen::MatrixXf::Identity(4,4);
  sensorToWorld.block<3,1>(1,1) = Eigen::Vector3f(o_mavlink->pos_msg.x,o_mavlink->pos_msg.y,o_mavlink->pos_msg.z);
  o_map->insertCloudCallback(cloud_xyz, identity, identity, sensorToWorld);
  o_map->m_octree->writeBinary("map.bt");
}

void processCloud(cv::Mat image_l)
{
  cv::Mat points = o_stereo->getPointcloud();
  float scale = 10; // Not sure why its needed but gives correct metric scale pointcloud

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>); 

  for (int rows = 0; rows < points.rows; ++rows) { 
  for (int cols = 0; cols < points.cols; ++cols) { 
     cv::Point3f point = points.at<cv::Point3f>(rows, cols); 

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

  // pcl::PassThrough<pcl::PointXYZ> pass_y;
  // pass_y.setFilterFieldName("y");
  // pass_y.setFilterLimits(-4, 0.1);
  // pass_y.setInputCloud(cloud_xyz);
  // pass_y.filter(*cloud_xyz);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_xyz);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud_xyz);

  // Create the voxel filtering object
  // pcl::VoxelGrid<pcl::PointXYZ> vf;
  // vf.setInputCloud (cloud_xyz);
  // vf.setLeafSize (0.02f, 0.02f, 0.02f);
  // vf.filter(*cloud_xyz);

  // pcl::PassThrough<pcl::PointXYZRGB> pass_y1;
  // pass_y1.setFilterFieldName("y");
  // pass_y1.setFilterLimits(-4, 0.01);
  // pass_y1.setInputCloud(cloud_xyzrgb);
  // pass_y1.filter(*cloud_xyzrgb);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor1;
  sor1.setInputCloud (cloud_xyzrgb);
  sor1.setMeanK (50);
  sor1.setStddevMulThresh (0.1);
  sor1.filter (*cloud_xyzrgb);

  // pcl::VoxelGrid<pcl::PointXYZRGB> vf1;
  // vf1.setInputCloud (cloud_xyzrgb);
  // vf1.setLeafSize (0.01f, 0.01f, 0.01f);
  // vf1.filter(*cloud_xyzrgb);

  pcl::io::savePCDFile ("test_pcd.pcd", *cloud_xyzrgb,false);
  insertCloud(cloud_xyz);

}

// Function is called everytime an image message is received.
void cb(ConstImagesStampedPtr &msg)
{
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
  // cv::imwrite("left.jpg",image_l);
  // cv::imwrite("right.jpg",image_r);
  cv::imshow("disparity",o_stereo->matchPair(image_l,image_r));

  cv::waitKey(10);
  processCloud(image_l);
  delete data_l;
  delete data_r;
  count++;
  if(count > 10)
  {
    throw;
  }

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
  o_stereo = std::make_shared<StereoMatcher>(std::string(SRC_DIR) + std::string("/camera_calibration.yaml"));

  boost::asio::io_service io_service;
  
  // Initialize mavlink object
  o_mavlink = std::make_shared<MavlinkComm>(14551, 14556, &io_service);

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
