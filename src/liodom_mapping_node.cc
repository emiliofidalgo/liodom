/*
* This file is part of liodom.
*
* Copyright (C) 2021 Emilio Garcia-Fidalgo <emilio.garcia@uib.es> (University of the Balearic Islands)
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
#include <iostream>
#include <string>
#include <chrono>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Liodom
#include <liodom/defs.h>
#include <liodom/map.h>

// Mapper
liodom::Map* mapper;
std::string fixed_frame;
std::string base_frame;
int cells_xy;
int cells_z;

// ROS 2
rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subs;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_local;
rclcpp::TimerBase::SharedPtr timer;
rclcpp::Time last_pub_time;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;

using namespace std::chrono_literals;

void lidarClb(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg) {
  
  // Converting ROS message to PCL
  liodom::PointCloud::Ptr pc_new(new liodom::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);
  RCLCPP_INFO(node->get_logger(), "Received cloud with %lu points.", pc_new->points.size());

  // Waiting for the current position
  geometry_msgs::msg::TransformStamped transform;
  try {    
    transform = tf_buffer->lookupTransform(
      fixed_frame,
      base_frame,
      lidar_msg->header.stamp,
      rclcpp::Duration(5.0, 0));
  } catch (const tf2::TransformException& ex) {    
    RCLCPP_WARN(node->get_logger(), "Could not get transform from %s to %s: %s", fixed_frame.c_str(), base_frame.c_str(), ex.what());
    return;
  }

  // Converting transform to an Eigen Isometry3d
  Eigen::Isometry3d trans = tf2::transformToEigen(transform);

  // Updating the map with the current PC
  mapper->updateMap(pc_new, trans);

  // Publishing the current map
  if (pc_pub->get_subscription_count() > 0) {
    liodom::PointCloud::Ptr map = mapper->getMap();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    cloud_msg.header.stamp = lidar_msg->header.stamp;
    pc_pub->publish(cloud_msg);
  }

  // Publishing the current local map
  if (pc_pub_local->get_subscription_count() > 0) {
    liodom::PointCloud::Ptr map = mapper->getLocalMap(trans, cells_xy, cells_z);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    cloud_msg.header.stamp = lidar_msg->header.stamp;
    pc_pub_local->publish(cloud_msg);
  }

  last_pub_time = node->now();
}

void timerClb() {

  // Publish the accumulated cloud if last publication was more than 5 seconds before.
  rclcpp::Duration elapsed_time = node->now() - last_pub_time;
  if (pc_pub->get_subscription_count() > 0 && elapsed_time.seconds() > 5.0) {
    liodom::PointCloud::Ptr map = mapper->getMap();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);    
    cloud_msg.header.frame_id = fixed_frame;
    // Print fixed frame
    pc_pub->publish(cloud_msg);   
  }
}

int main(int argc, char** argv) {
  
  // Initializing node
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("liodom_mapping");

  // Declare parameters
  node->declare_parameter("voxel_xysize", 40.0);
  node->declare_parameter("voxel_zsize", 50.0);
  node->declare_parameter("resolution", 0.4);
  node->declare_parameter("fixed_frame", std::string("world"));
  node->declare_parameter("base_frame", std::string("base_link"));
  node->declare_parameter("cells_xy", 2);
  node->declare_parameter("cells_z", 2);

  // Reading parameters
  double voxel_xysize = node->get_parameter("voxel_xysize").as_double();
  RCLCPP_INFO(node->get_logger(), "Voxel size (XY): %.2f", voxel_xysize);

  double voxel_zsize = node->get_parameter("voxel_zsize").as_double();
  RCLCPP_INFO(node->get_logger(), "Voxel size (Z): %.2f", voxel_zsize);

  double reso = node->get_parameter("resolution").as_double();
  RCLCPP_INFO(node->get_logger(), "Resolution: %.2f", reso);

  fixed_frame = node->get_parameter("fixed_frame").as_string();
  RCLCPP_INFO(node->get_logger(), "Fixed frame: %s", fixed_frame.c_str());

  base_frame = node->get_parameter("base_frame").as_string();
  RCLCPP_INFO(node->get_logger(), "Base frame: %s", base_frame.c_str());

  cells_xy = node->get_parameter("cells_xy").as_int();
  RCLCPP_INFO(node->get_logger(), "Local Map Cells XY: %i", cells_xy);

  cells_z = node->get_parameter("cells_z").as_int();
  RCLCPP_INFO(node->get_logger(), "Local Map Cells Z: %i", cells_z);

  mapper = new liodom::Map(voxel_xysize, voxel_zsize, reso);

  // Subscribers
  pc_subs = node->create_subscription<sensor_msgs::msg::PointCloud2>("points", 1, lidarClb);  
  // Publishers
  pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("map", 1);
  pc_pub_local = node->create_publisher<sensor_msgs::msg::PointCloud2>("map_local", 1);  
  timer = node->create_wall_timer(3s, timerClb);
  last_pub_time = node->now();
  // Listener
  tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  // Receiving messages
  rclcpp::spin(node);
  rclcpp::shutdown();  

  return 0;
}
