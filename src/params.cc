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

void Params::declareParams(const rclcpp::Node::SharedPtr& nh) {
  // Minimum range in meters
  nh->declare_parameter("min_range", 0.1);

  // Maximum range in meters
  nh->declare_parameter("max_range", 75.0);

  // Lidar model: 0 for Velodyne, 1 for Ouster
  nh->declare_parameter("lidar_type", 0);

  // Horizontal scan lines
  nh->declare_parameter("scan_lines", 64);

  // Scan regions
  nh->declare_parameter("scan_regions", 8);

  // Edges per region
  nh->declare_parameter("edges_per_region", 10);

  // Save results
  nh->declare_parameter("save_results", false);

  // Results directory
  nh->declare_parameter("save_results_dir", "~/");

  // Fixed frame
  nh->declare_parameter("fixed_frame", "odom");

  // Base frame
  nh->declare_parameter("base_frame", "base_link");

  // Laser frame
  nh->declare_parameter("laser_frame", "");

  // Previous frames
  nh->declare_parameter("prev_frames", 5);

  // Use IMU
  nh->declare_parameter("use_imu", false);

  // Filter local map
  nh->declare_parameter("filter_local_map", false);

  // Mapping
  nh->declare_parameter("mapping", false);

  // Publish TF
  nh->declare_parameter("publish_tf", true);
}


void Params::readParams(const rclcpp::Node::SharedPtr& nh) {
  // Reading parameters
  // Minimum range in meters
  min_range_ = nh->get_parameter("min_range").as_double();
  RCLCPP_INFO(nh->get_logger(), "Minimum range: %.2f", min_range_);

  // Maximum range in meters
  max_range_ = nh->get_parameter("max_range").as_double();
  RCLCPP_INFO(nh->get_logger(), "Maximum range: %.2f", max_range_);

  // Lidar model: 0 for Velodyne, 1 for Ouster
  lidar_type_ = nh->get_parameter("lidar_type").as_int();
  RCLCPP_INFO(nh->get_logger(), "Lidar type: %i", lidar_type_);

  // Horizontal scan lines
  scan_lines_ = nh->get_parameter("scan_lines").as_int();
  RCLCPP_INFO(nh->get_logger(), "Scan lines: %i", scan_lines_);

  // Scan regions
  scan_regions_ = nh->get_parameter("scan_regions").as_int();
  RCLCPP_INFO(nh->get_logger(), "Scan regions: %i", scan_regions_);

  // Edges per region
  edges_per_region_ = nh->get_parameter("edges_per_region").as_int();
  RCLCPP_INFO(nh->get_logger(), "Edges per region: %i", edges_per_region_);

  min_points_per_scan_ = scan_regions_ * edges_per_region_ + 10;

  // Save results
  save_results_ = nh->get_parameter("save_results").as_bool();
  RCLCPP_INFO(nh->get_logger(), "Save results: %s", save_results_ ? "Yes" : "No");

  // Results directory
  results_dir_ = nh->get_parameter("save_results_dir").as_string();
  RCLCPP_INFO(nh->get_logger(), "Results directory: %s", results_dir_.c_str());

  // Fixed frame
  fixed_frame_ = nh->get_parameter("fixed_frame").as_string();
  RCLCPP_INFO(nh->get_logger(), "Fixed frame: %s", fixed_frame_.c_str());

  // Base frame
  base_frame_ = nh->get_parameter("base_frame").as_string();
  RCLCPP_INFO(nh->get_logger(), "Base frame: %s", base_frame_.c_str());

  // Laser frame
  laser_frame_ = nh->get_parameter("laser_frame").as_string();
  if (laser_frame_ == ""){
    RCLCPP_INFO(nh->get_logger(), "Using laser frame from header");
  }else{
    RCLCPP_INFO(nh->get_logger(), "Laser frame: %s", laser_frame_.c_str());
  }

  // Number of previous frames to create a local map
  int pframes = 5;
  pframes = nh->get_parameter("prev_frames").as_int();
  RCLCPP_INFO(nh->get_logger(), "Sliding window size: %i", pframes);
  local_map_size_ = (size_t)pframes;

  // Use IMU
  use_imu_ = nh->get_parameter("use_imu").as_bool();
  RCLCPP_INFO(nh->get_logger(), "Use IMU: %s", use_imu_ ? "Yes" : "No");

  // Filter local map
  filter_local_map_ = nh->get_parameter("filter_local_map").as_bool();
  RCLCPP_INFO(nh->get_logger(), "Filter local map: %s", filter_local_map_ ? "Yes" : "No");

  // Mapping
  mapping_ = nh->get_parameter("mapping").as_bool();
  RCLCPP_INFO(nh->get_logger(), "Mapping: %s", mapping_ ? "Yes" : "No");

  // Publish TF
  publish_tf_ = nh->get_parameter("publish_tf").as_bool();
  RCLCPP_INFO(nh->get_logger(), "Publish TF: %s", publish_tf_ ? "Yes" : "No");
}

}  // namespace liodom