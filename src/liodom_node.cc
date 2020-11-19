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

// C++
#include <atomic>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <liodom/defs.h>
#include <liodom/feature_extractor.h>
#include <liodom/laser_odometry.h>
#include <liodom/shared_data.h>
#include <liodom/stats.h>

liodom::SharedData* sdata;
liodom::Stats* stats;

void lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  liodom::PointCloud::Ptr pc_new(new liodom::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);
  
  ROS_DEBUG("---");
  ROS_DEBUG("Initial cloud: %lu points", pc_new->points.size());

  sdata->pushPointCloud(pc_new, lidar_msg->header);
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init (argc, argv, "liodom");
  ros::NodeHandle nh("~");

  // Reading common parameters
  // Save results
  bool save_results;
  nh.param("save_results", save_results, false);
  // Results directory
  std::string results_dir;
  nh.param<std::string>("save_results_dir", results_dir, "~/");

  Eigen::initParallel();

  // Creating threads
  liodom::FeatureExtractor fext(nh);
  fext.initialize();
  liodom::LaserOdometer lodom(nh);
  lodom.initialize();

  // Launching threads
  std::atomic<bool> running {true};
  std::thread fext_thread(fext, std::ref(running));
  std::thread lodom_thread(lodom, std::ref(running));

  // Shared stuff
  sdata = liodom::SharedData::getInstance();
  stats = liodom::Stats::getInstance();

  // Subscribers  
  ros::Subscriber pc_subs_ = nh.subscribe("points", 1000, lidarClb);

  // Receiving messages
  ros::spin();

  // Exit gracefully
  running = false;
  fext_thread.join();
  lodom_thread.join();

  // Saving results if required
  if (save_results) {
    std::cout << "Writing results to ";
    std::cout << results_dir.c_str() << std::endl;
    stats->writeResults(results_dir);
  }
  
  return 0;
}