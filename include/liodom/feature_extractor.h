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

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <liodom/params.h>
#include <liodom/shared_data.h>
#include <liodom/stats.h>

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

    void operator()(std::atomic<bool>& running);

  private:
    // ROS variables
    ros::NodeHandle nh_;    

    // Variables
    bool picked_[400000];           
    SharedData* sdata;
    Stats* stats;
    Params* params;
    int ncores_;

    bool isValidPoint(const double x, const double y, const double z, double* dist);
    void splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans);
    void extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges);
    void extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges);
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_FEATURE_EXTRACTOR_H