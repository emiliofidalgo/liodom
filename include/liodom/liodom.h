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

#ifndef INCLUDE_LIODOM_LIODOM_H
#define INCLUDE_LIODOM_LIODOM_H

// C++
#include <chrono>
#include <iostream>
#include <utility>

// Ceres
#include <ceres/ceres.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

// Liodom
#include <liodom/factors.hpp>

namespace liodom {

// PCL typedefs
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> PointCloudFPFH;

// Chrono typedefs
typedef std::chrono::high_resolution_clock Clock;

// Matchings
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Match;

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

// Odometry
class LidarOdometry {
 public:
  explicit LidarOdometry(const ros::NodeHandle& nh);
  virtual ~LidarOdometry();

  void initialize();

 private:
  // ROS variables
  ros::NodeHandle nh_;
  ros::Subscriber pc_subs_;
  ros::Publisher pc_edges_pub_;
  ros::Publisher laser_odom_pub_;

  // Params
  double min_range_;
  double max_range_;
  int lidar_type_;
  int scan_lines_;
  int scan_regions_;
  int edges_per_region_;
  size_t min_points_per_scan_;

  // Global variables
  bool init_;
  bool picked_[400000];
  Eigen::Isometry3d prev_odom_;
  Eigen::Isometry3d odom_;
  std::vector<PointCloud::Ptr> prev_edges_;
  double param_q[4] = {0, 0, 0, 1};
  double param_t[3] = {0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q_curr;
  Eigen::Map<Eigen::Vector3d> t_curr;
  
  // Private methods
  void filter(const PointCloud::Ptr& pc_in, PointCloud::Ptr& pc_out);
  void computeNormals(const PointCloud::Ptr& pc_in, const PointCloud::Ptr& pc_search, PointCloudNormal::Ptr& pc_normals);
  void computeDescriptors(const PointCloud::Ptr& pc_in, const PointCloudNormal::Ptr& pc_normals, PointCloudFPFH::Ptr& descriptors);

  bool isValidPoint(const double x, const double y, const double z, double* dist);
  void splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans);
  void extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges);
  void extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges);
  void estimatePose(const PointCloud::Ptr& pc_in, const PointCloud::Ptr& pc_edges);

  // Callbacks
  void lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg);  
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_LIODOM_H
