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

#include "OctomapServer.h"

OctomapServer::OctomapServer()
: m_octree(NULL),
  m_maxRange(-1.0),
  m_res(0.15),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_compressMap(true)

{
  double probHit, probMiss, thresMin, thresMax;

  probHit = 0.55;
  probMiss = 0.4;
  thresMin = 0.12;
  thresMax = 0.97;

  // initialize octomap object & params
  m_octree = new OcTreeT(m_res);
  m_octree->setProbHit(probHit);
  m_octree->setProbMiss(probMiss);
  m_octree->setClampingThresMin(thresMin);
  m_octree->setClampingThresMax(thresMax);
  m_treeDepth = m_octree->getTreeDepth();
  m_maxTreeDepth = m_treeDepth;

  INFO("Octomap Initalized");
}

OctomapServer::~OctomapServer(){

  if (m_octree){
    delete m_octree;
    m_octree = NULL;
  }

}

void OctomapServer::insertCloudCallback(PCLPointCloud::Ptr pc, const Eigen::Matrix4f& sensorToWorld){

  auto startTime = std::chrono::system_clock::now();

  PCLPointCloud pc_cloud;
  
  // directly transform to map frame:
  pcl::transformPointCloud(*pc, *pc, sensorToWorld);
  pc_cloud = *pc;

  pc_cloud.header = pc->header;

  insertScan(sensorToWorld.block(0,3,3,1), pc_cloud);

  std::chrono::duration<double> total_elapsed = std::chrono::system_clock::now() - startTime;
  // DBG("Pointcloud insertion in OctomapServer done ( " << pc_cloud.size() << " pts (cloud), " << total_elapsed.count()<< " sec)");

}

void OctomapServer::insertScan(const Eigen::Vector3f& sensorOriginTf, const PCLPointCloud& cloud){
  octomap::point3d sensorOrigin (sensorOriginTf.x(),sensorOriginTf.y(),sensorOriginTf.z());

  if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin)
    || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax))
  {
    ERROR("Could not generate Key for origin "<<sensorOrigin);
  }

  octomap::KeySet free_cells, occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it){
    octomap::point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (m_octree->coordToKeyChecked(point, key)){
        occupied_cells.insert(key);
      }
    } else {// ray longer than maxrange:;
      octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());

        octomap::OcTreeKey endKey;
        if (m_octree->coordToKeyChecked(new_end, endKey)){
          free_cells.insert(endKey);
          updateMinKey(endKey, m_updateBBXMin);
          updateMaxKey(endKey, m_updateBBXMax);
        } else{
          ERROR("Could not generate Key for endpoint "<<new_end);
        }


      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      m_octree->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
    m_octree->updateNode(*it, true);
  }

  if (m_compressMap)
    m_octree->prune();
}