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
#include <thread>

// PCL
#include <pcl_conversions/pcl_conversions.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <liodom/shared_data.h>

namespace liodom {

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

    // Variables
    bool picked_[400000];
    SharedData* sdata;

    bool isValidPoint(const double x, const double y, const double z, double* dist);
    void splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans);
    void extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges);
    void extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges);
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_FEATURE_EXTRACTOR_H