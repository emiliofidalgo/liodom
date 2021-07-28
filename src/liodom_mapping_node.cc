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

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

// Liodom
#include <liodom/defs.h>
#include <liodom/map.h>

// Mapper
liodom::Map* mapper;
std::string fixed_frame;
std::string base_frame;
int cells_xy;
int cells_z;

// ROS 
ros::Subscriber pc_subs;
ros::Publisher pc_pub;
ros::Publisher pc_pub_local;
ros::WallTimer timer;
ros::WallTime last_pub_time;
tf::TransformListener* tf_listener;

void lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  liodom::PointCloud::Ptr pc_new(new liodom::PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_new);
  ROS_INFO_STREAM("Received cloud with " << pc_new->points.size() << " points.");

  // Waiting for the current position
  tf::StampedTransform transform;
  try {
    tf_listener->waitForTransform(fixed_frame, base_frame, lidar_msg->header.stamp, ros::Duration(5.0));
    //tf_listener->waitForTransform(fixed_frame, base_frame, ros::Time(0), ros::Duration(5.0));
    tf_listener->lookupTransform(fixed_frame, base_frame, lidar_msg->header.stamp, transform);
    //tf_listener->lookupTransform(fixed_frame, base_frame, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();    
  }

  // Converting to an Eigen Isometry3D
  Eigen::Isometry3d trans;
  tf::transformTFToEigen(transform, trans);

  // Updating the map with the current PC
  mapper->updateMap(pc_new, trans);

  // Publishing the current map  
  if (pc_pub.getNumSubscribers() > 0) {
    liodom::PointCloud::Ptr map = mapper->getMap();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    pc_pub.publish(cloud_msg);
  }

  // Publishing the current local map
  if (pc_pub_local.getNumSubscribers() > 0) {
    liodom::PointCloud::Ptr map = mapper->getLocalMap(trans, cells_xy, cells_z);
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    pc_pub_local.publish(cloud_msg);
  }

  last_pub_time = ros::WallTime::now();
}

void timerClb(const ros::WallTimerEvent&) {
  // Publish the accumulated cloud if last publication was more than 5 seconds before.
  ros::WallDuration elapsed_time = ros::WallTime::now() - last_pub_time;
  if (pc_pub.getNumSubscribers() > 0 && elapsed_time.toSec() > 5.0) {
    liodom::PointCloud::Ptr map = mapper->getMap();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*map, cloud_msg);
    cloud_msg.header.frame_id = fixed_frame;
    pc_pub.publish(cloud_msg);
  }
}

int main(int argc, char** argv) {
  
  // Initializing node
  ros::init(argc, argv, "liodom_mapping");
  ros::NodeHandle nh("~");

  // Reading parameters
  double voxel_xysize;
  nh.param("voxel_xysize", voxel_xysize, 40.0);
  ROS_INFO("Voxel size (XY): %.2f", voxel_xysize);

  double voxel_zsize;
  nh.param("voxel_zsize", voxel_zsize, 50.0);
  ROS_INFO("Voxel size (Z): %.2f", voxel_zsize);

  double reso;
  nh.param("resolution", reso, 0.4);
  ROS_INFO("Resolution: %.2f", reso);

  nh.param("fixed_frame", fixed_frame, std::string("world"));
  nh.param("base_frame", base_frame, std::string("base_link"));

  nh.param("cells_xy", cells_xy, 2);
  ROS_INFO("Local Map Cells XY: %i", cells_xy);

  nh.param("cells_z", cells_z, 1);
  ROS_INFO("Local Map Cells Z: %i", cells_z);

  mapper = new liodom::Map(voxel_xysize, voxel_zsize, reso);

  // Subscribers  
  pc_subs = nh.subscribe("points", 1, lidarClb);
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
  pc_pub_local = nh.advertise<sensor_msgs::PointCloud2>("map_local", 1, true);
  timer = nh.createWallTimer(ros::WallDuration(3.0), timerClb);
  // Listener
  tf_listener = new tf::TransformListener();

  // Receiving messages
  ros::spin();
  
  return 0;
}
