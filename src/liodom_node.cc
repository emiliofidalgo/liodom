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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>

// Liodom
#include <liodom/defs.h>
#include <liodom/feature_extractor.h>
#include <liodom/laser_odometry.h>
#include <liodom/params.h>
#include <liodom/shared_data.h>
#include <liodom/stats.h>

liodom::Params* params;
liodom::SharedData* sdata;
liodom::Stats* stats;

rclcpp::Node::SharedPtr node = nullptr;

// liDAR Callback
void lidarClb(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
  
  // Converting ROS message to PCL
  liodom::PointCloud::Ptr pc_new(new liodom::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);

  RCLCPP_DEBUG(node->get_logger(), "---");
  RCLCPP_DEBUG(node->get_logger(), "Initial cloud: %lu points", pc_new->points.size());
  
  if (params->save_results_) {
    auto start = liodom::Clock::now();
    stats->startFrame(start);
  }

  sdata->pushPointCloud(pc_new, lidar_msg->header);
}

void mapClb(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg) {
  
  // Converting ROS message to PCL
  liodom::PointCloud::Ptr pc_new(new liodom::PointCloud);
  pcl::fromROSMsg(*map_msg, *pc_new);

  sdata->setLocalMap(pc_new);
}

void imuClb(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {

  Eigen::Quaterniond new_ori(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
  sdata->setLastIMUOri(new_ori);
}

int main(int argc, char** argv) {
  
  // Initializing node
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("liodom");

  // Declaring and reading parameters
  params = liodom::Params::getInstance();
  params->declareParams(node);
  params->readParams(node);

  Eigen::initParallel();

  // // Creating threads
  liodom::FeatureExtractor fext(node);
  liodom::LaserOdometer lodom(node);

  // // Launching threads
  std::atomic<bool> running {true};
  std::thread fext_thread(fext, std::ref(running));
  std::thread lodom_thread(lodom, std::ref(running));

  // // Shared stuff
  sdata = liodom::SharedData::getInstance();
  stats = liodom::Stats::getInstance();

  // Subscribers  
  auto pc_subs_ = node->create_subscription<sensor_msgs::msg::PointCloud2>("points", 1, lidarClb);
  auto map_subs_ = node->create_subscription<sensor_msgs::msg::PointCloud2>("map", 1, mapClb);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subs_;
  if (params->use_imu_) {
    imu_subs_ = node->create_subscription<sensor_msgs::msg::Imu>("imu", 1, imuClb);
  }

  // Receiving messages
  rclcpp::spin(node);
  rclcpp::shutdown();

  // Exit gracefully
  running = false;
  fext_thread.join();
  lodom_thread.join();

  // Saving results if required
  if (params->save_results_) {
    std::cout << "Writing results to ";
    std::cout << params->results_dir_.c_str() << std::endl;
    stats->writeResults(params->results_dir_);
  }
  
  return 0;
}
