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

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pcl/common/common_headers.h"
#include "pcl/io/io.h"
#include <pcl/io/pcd_io.h>
#include "pcl/point_cloud.h"

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

class ArduPlanner
{
   public:
      // Basic class objects from libraries
      std::shared_ptr<MavlinkComm> o_mavlink;
      std::shared_ptr<OctomapServer> o_map;
      std::shared_ptr<StereoMatcher> o_stereo;
      std::shared_ptr<GazeboVis> o_gazebovis;
      std::shared_ptr<Planner> o_planner;
      
      std::atomic_bool new_plan{false}, planner_init{false}, is_cloud_processed{true};
      
      Eigen::Matrix4f sensorToWorld;
      
      void initializeManeuver(double yaw_feedforward);

      void updateSensorToWorld();

      void processCloudThread();

      // void initPlannerThread();

      ArduPlanner(cv::Mat _start, cv::Mat _goal, double _velocity, bool _return_back, int _replan_interval, int _executor_interval, bool _visualize_octomap, double _min_rangem, double _max_range);
      ~ArduPlanner();

   private:
      
      cv::Mat start;
      cv::Mat goal;

      std::mutex octomap_mutex;
      std::mutex planner_mutex;

      bool replan_finished = true, octomap_fused = true, visualize_octomap;

      std::condition_variable is_octomap_processed;
      std::condition_variable is_replan_processed;
      // pcl::PointCloud<pcl::PointXYZRGB> final_cloud;
      double velocity = 0.2;

      double min_range, max_range;

      // For mavlink async worker
      std::shared_ptr<boost::asio::io_service> mavlink_service;
      std::shared_ptr<boost::asio::io_service::work> mavlink_work;
      
      int replan_interval;
      int executor_interval;
      bool return_back;
      
      // For async replanner worker
      std::shared_ptr<boost::asio::deadline_timer> replan_timer;
      std::shared_ptr<boost::asio::io_service> replanning_service;
      std::shared_ptr<boost::asio::io_service::work> replan_work;

      // For async plan executor
      std::shared_ptr<boost::asio::deadline_timer> executor_timer;
      std::shared_ptr<boost::asio::io_service> executor_service;
      std::shared_ptr<boost::asio::io_service::work> executor_work;

      void insertCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz);

      // void fuseCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_xyzrgb);

      void processCloud();

      // void initPlanner();

      void executePlan();

      void replanCb();

      void replanAsync();

};