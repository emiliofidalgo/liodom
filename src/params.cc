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

#include "liodom/params.h"

namespace liodom {

Params* Params::pinstance_{nullptr};
std::mutex Params::params_mutex_;

Params* Params::getInstance() {
  std::lock_guard<std::mutex> lock(params_mutex_);
  
  if (pinstance_ == nullptr) {
    pinstance_ = new Params();
  }
  
  return pinstance_;
}

void Params::readParams(const ros::NodeHandle& nh) {
  // Reading parameters
  // Minimum range in meters
  nh.param("min_range", min_range_, 3.0);
  ROS_INFO("Minimum range: %.2f", min_range_);

  // Maximum range in meters
  nh.param("max_range", max_range_, 75.0);
  ROS_INFO("Maximum range: %.2f", max_range_);

  // Lidar model: 0 for Velodyne, 1 for Ouster
  nh.param("lidar_type", lidar_type_, 0);
  ROS_INFO("Lidar type: %i", lidar_type_);

  // Horizontal scan lines
  nh.param("scan_lines", scan_lines_, 64);
  ROS_INFO("Scan lines: %i", scan_lines_);

  // Scan regions
  nh.param("scan_regions", scan_regions_, 8);
  ROS_INFO("Scan regions: %i", scan_regions_);

  // Edges per region
  nh.param("edges_per_region", edges_per_region_, 10);
  ROS_INFO("Edges per region: %i", edges_per_region_);

  min_points_per_scan_ = scan_regions_ * edges_per_region_ + 10;

  // Save results
  nh.param("save_results", save_results_, false);
  ROS_INFO("Save results: %s", save_results_ ? "Yes" : "No");

  // Results directory
  nh.param<std::string>("save_results_dir", results_dir_, "~/");
  ROS_INFO("Results directory: %s", results_dir_.c_str());

  // Fixed frame
  nh.param<std::string>("fixed_frame", fixed_frame_, "odom");
  ROS_INFO("Fixed frame: %s", fixed_frame_.c_str());

  // Base frame
  nh.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("Fixed frame: %s", base_frame_.c_str());

  // Laser frame
  nh.param<std::string>("laser_frame", laser_frame_, "");
  if (laser_frame_ == ""){
    ROS_INFO("Using laser frame from header");
  }else{
    ROS_INFO("Laser frame: %s", laser_frame_.c_str());
  }

  // Number of previous frames to create a local map
  int pframes = 5;
  nh.param("prev_frames", pframes, 5);
  ROS_INFO("Sliding window size: %i", pframes);
  local_map_size_ = (size_t)pframes;

  // Use IMU
  nh.param("use_imu", use_imu_, false);
  ROS_INFO("Use IMU: %s", use_imu_ ? "Yes" : "No");
}

}  // namespace liodom