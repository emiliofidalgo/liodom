/*
* This file is part of liodom.
*
* Copyright (C) 2020 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
*
* liodom is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* liodom is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with liodom. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef INCLUDE_LIODOM_FEATURE_EXTRACTOR_H
#define INCLUDE_LIODOM_FEATURE_EXTRACTOR_H

// C++
#include <atomic>
#include <chrono>
#include <omp.h>
#include <thread>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/search.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <liodom/shared_data.h>
#include <liodom/stats.h>

#include <boost/thread/thread.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/console/parse.h>

namespace liodom {

// Point stats
struct PointInfo {

  inline PointInfo() :
    point_index(-1),
    coords(0.0, 0.0, 0.0),
    normal(0.0, 0.0, 0.0),
    curvature(DBL_MAX),
    scale(0.0),
    valid(false) {};
  
  bool isValid() {
    return valid;
  }  

  friend std::ostream& operator<<(std::ostream& os, const PointInfo& pi);

  int point_index;
  Eigen::Vector3d coords;
  Eigen::Vector3d normal;
  double curvature;
  double scale;
  bool valid;
  std::vector<int> knns;
};

std::ostream& operator<<(std::ostream& os, const PointInfo& pi) {
    return os << "Point " << pi.point_index << ": "
              << "N(" << pi.normal << "), "
              << "C(" << pi.curvature << "), "
              << "S(" << pi.scale << ")" << std::endl;
}

// Plane
struct Plane {

  inline Plane() :
    plane_id(-1),
    mean_point(0.0, 0.0, 0.0),
    normal(0.0, 0.0, 0.0),
    curvature(DBL_MAX),
    scale(0.0) {};

  void addPoint(const PointInfo& p) {
    points.push_back(p);
  }

  size_t size() {
    return points.size();
  }

  void computeStats();
  void mergePlane(const Plane& p);

  friend std::ostream& operator<<(std::ostream& os, const Plane& p);

  int plane_id;
  std::vector<PointInfo> points;
  Eigen::Vector3d mean_point;
  Eigen::Vector3d normal;  
  double curvature;
  double scale; 
};

std::ostream& operator<<(std::ostream& os, const Plane& p) {
    return os << "Plane with " << p.points.size() << " points";
}

// Smoothness
struct SmoothnessItem {

  inline SmoothnessItem() :
    point_index(-1),
    smoothness(-1.0) {};

  inline explicit SmoothnessItem(const int index, const double smooth) :
    point_index(index),
    smoothness(smooth) {};

  int point_index;
  double smoothness;

  bool operator<(const SmoothnessItem& s) const {
    return smoothness > s.smoothness;
  }
};

class FeatureExtractor {
  public:  
    explicit FeatureExtractor(const ros::NodeHandle& nh);
    virtual ~FeatureExtractor();

    void initialize();
    void operator()(std::atomic<bool>& running);

  private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Publisher pc_edges_pub_;    

    // Params    
    double min_range_;
    double max_range_;
    int lidar_type_;
    int scan_lines_;
    int scan_regions_;
    int edges_per_region_;
    size_t min_points_per_scan_;
    bool save_results_;

    // Variables
    bool picked_[400000];
    SharedData* sdata;
    Stats* stats;
    int ncores_;

    bool isValidPoint(const double x, const double y, const double z, double* dist);
    void splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans);
    void extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges);
    void extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges);

    void extractPlanes(const PointCloud::Ptr& pc_in, std::vector<Plane>& planes);
    void computePointInfo(const PointCloud::Ptr& pc_in, std::vector<PointInfo>& point_info);
    void regionGrowing(const std::vector<PointInfo>& point_info, std::vector<Plane>& planes);
    void regionMerging(const std::vector<PointInfo>& point_info, const std::vector<Plane>& planes_in, std::vector<Plane>& planes_out);
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_FEATURE_EXTRACTOR_H