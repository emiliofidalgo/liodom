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

#include "liodom/liodom.h"

namespace liodom {

LidarOdometry::LidarOdometry(const ros::NodeHandle& nh) :
  nh_(nh),
  min_range_(3.0),
  max_range_(90.0),
  lidar_type_(0),
  scan_lines_(64),
  scan_regions_(8),
  edges_per_region_(10),
  min_points_per_scan_(scan_regions_ * edges_per_region_ + 10),
  init_(false),
  prev_odom_(Eigen::Isometry3d::Identity()),
  odom_(Eigen::Isometry3d::Identity()),
  q_curr(param_q),
  t_curr(param_t) {
}

LidarOdometry::~LidarOdometry() {
}

void LidarOdometry::initialize() {
  
  // Reading parameters
  // Minimum range in meters
  nh_.param("min_range", min_range_, 3.0);
  ROS_INFO("Minimum range: %.2f", min_range_);

  // Maximum range in meters
  nh_.param("max_range", max_range_, 35.0);
  ROS_INFO("Maximum range: %.2f", max_range_);

  // Lidar model: 0 for Velodyne, 1 for Ouster
  nh_.param("lidar_type", lidar_type_, 0);
  ROS_INFO("Lidar type: %i", lidar_type_);

  // Horizontal scan lines
  nh_.param("scan_lines", scan_lines_, 64);
  ROS_INFO("Scan lines: %i", scan_lines_);

  // Scan regions
  nh_.param("scan_regions", scan_regions_, 8);
  ROS_INFO("Scan regions: %i", scan_regions_);

  // Edges per region
  nh_.param("edges_per_region", edges_per_region_, 10);
  ROS_INFO("Edges per region: %i", edges_per_region_);

  min_points_per_scan_ = scan_regions_ * edges_per_region_ + 10;

  // Publishers
  pc_edges_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("edges", 1000);
  laser_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1000);

  // Subscribers
  pc_subs_ = nh_.subscribe("points", 1000, &LidarOdometry::lidarClb, this);
}

void LidarOdometry::lidarClb(const sensor_msgs::PointCloud2ConstPtr& lidar_msg) {
  
  // Converting ROS message to PCL
  PointCloud::Ptr pc_curr(new PointCloud);
  pcl::fromROSMsg(*lidar_msg, *pc_curr);
  
  ROS_DEBUG("---");
  ROS_DEBUG("Initial cloud: %lu points", pc_curr->points.size());

  // Split the pointcloud into scan_lines_ scans
  std::vector<PointCloud::Ptr> scans;
  splitPointCloud(pc_curr, scans);

  // Extracting features from each scan
  PointCloud::Ptr pc_edges(new PointCloud);
  extractFeatures(scans, pc_edges);
  ROS_DEBUG("Feature extraction: %lu edges", pc_edges->points.size());

  // Publishing edges if there is someone listening
  if (pc_edges_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 edges_msg;
    pcl::toROSMsg(*pc_edges, edges_msg);
    edges_msg.header = lidar_msg->header;        
    pc_edges_pub_.publish(edges_msg);
  }

  estimatePose(pc_curr, pc_edges);
}

void LidarOdometry::filter(const PointCloud::Ptr& pc_in, PointCloud::Ptr& pc_out) {

  // If the clouds are not the same, prepare the output
  if (pc_in != pc_out) {
    pc_out->header = pc_in->header;
    pc_out->sensor_origin_ = pc_in->sensor_origin_;
    pc_out->sensor_orientation_ = pc_in->sensor_orientation_;
  }

  for (std::size_t i = 0; i < pc_in->points.size(); i++) {
    double x = pc_in->points[i].x;
    double y = pc_in->points[i].y;
    double z = pc_in->points[i].z;

    // Remove the point if it is NaN
    if (!pc_in->is_dense && (!std::isfinite(x) ||
                             !std::isfinite(y) ||
                             !std::isfinite(z))) {
      continue;
    }

    // Filter the point by its distance
    double distance = std::sqrt(x * x + y * y);
    if (distance > max_range_ || distance < min_range_) {
      continue;
    }

    // Adding the point to the final cloud
    pc_out->push_back(pc_in->points[i]);
  }

  pc_out->width = static_cast<std::uint32_t>(pc_out->points.size());
  pc_out->height = 1;
  pc_out->is_dense = true;
}

void LidarOdometry::computeNormals(const PointCloud::Ptr& pc_in, const PointCloud::Ptr& pc_search, PointCloudNormal::Ptr& pc_normals) {
  
  // Computing the normals for each point
  // TODO Use integral images if it is an organized PC.
  pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
  ne.setInputCloud(pc_in);
  if (pc_in != pc_search) {
    ne.setSearchSurface(pc_search);
  }
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  ne.setSearchMethod(tree);  
  ne.setRadiusSearch(0.3);
  PointCloudNormal::Ptr normals(new PointCloudNormal);
  ne.compute(*normals);
  
  // Validating if there is any NaN normal
  int normals_ok = 0;
  for (size_t i = 0; i < normals->size(); i++) {
    if (pcl::isFinite<pcl::Normal>((*normals)[i])) {
      normals_ok++;
    }
  }
  ROS_DEBUG("Normal extraction: %d normals", normals_ok);

  pc_normals = normals;
}

void LidarOdometry::computeDescriptors(const PointCloud::Ptr& pc_in, const PointCloudNormal::Ptr& pc_normals, PointCloudFPFH::Ptr& descriptors) {
  
  // Computing features for each edge
  pcl::FPFHEstimationOMP<Point, pcl::Normal, pcl::FPFHSignature33> fest;
  fest.setInputCloud(pc_in);
  // if (pc_in != pc_search) {
  //   fest.setSearchSurface(pc_search);
  // }
  fest.setInputNormals(pc_normals);
  fest.setRadiusSearch(0.5);  
  PointCloudFPFH::Ptr fpfhs(new PointCloudFPFH);
  fest.compute(*fpfhs);
  
  descriptors = fpfhs;
}

bool LidarOdometry::isValidPoint(const double x, const double y, const double z, double* dist) {

  bool valid = true;
  
  // Ignore the point if it is NaN
  if ((!std::isfinite(x) ||
       !std::isfinite(y) ||
       !std::isfinite(z))) {
    valid = false;
  }

  // Filter the point by its distance in XY
  *dist = std::sqrt(x * x + y * y);
  if (*dist > max_range_ || *dist < min_range_) {
    valid = false;
  }

  return valid;
}

void LidarOdometry::splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans) {

  // Initialize the vector of scans
  scans.clear();  
  for (int i = 0; i < scan_lines_; i++) {
    scans.push_back(PointCloud::Ptr(new PointCloud));
  }

  // Processing the scan according to the model  
  if (lidar_type_ == 0) {  // Velodyne
    // Assigning each point to its corresponding scan line
    for (size_t i = 0; i < pc_in->points.size(); i++) {
      double x = pc_in->points[i].x;
      double y = pc_in->points[i].y;
      double z = pc_in->points[i].z;

      // Check if the point is valid
      double distance;
      if (!isValidPoint(x, y, z, &distance)) {
        continue;
      }

      // Code adapted from FLOAM (https://github.com/wh200720041/floam)
      int scan_id = -1;
      double angle = atan(z / distance) * 180 / M_PI;

      if (scan_lines_ == 16) {
        scan_id = int((angle + 15) / 2 + 0.5);
        if (scan_id > (scan_lines_ - 1) || scan_id < 0) {
          continue;
        }
      } else if (scan_lines_ == 32) {
        scan_id = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
        if (scan_id > (scan_lines_ - 1) || scan_id < 0) {
          continue;
        }
      } else if (scan_lines_ == 64) {
        if (angle >= -8.83)
          scan_id = int((2 - angle) * 3.0 + 0.5);
        else
          scan_id = scan_lines_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

        if (angle > 2 || angle < -24.33 || scan_id > 63 || scan_id < 0) {
          continue;
        }              
      } else {
        ROS_ERROR_ONCE("Invalid scan lines: %i", scan_lines_);
      }

      // Adding the point to the corresponding scan
      if (scan_id != -1) {
        scans[scan_id]->push_back(pc_in->points[i]);
      }      
    }
  } else {
    ROS_ERROR_ONCE("Incorrect Lidar type");
  }
}

void LidarOdometry::extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges) {
  
  // Code adapted from FLOAM (https://github.com/wh200720041/floam)

  // Each scan is processed individually  
  for (int i = 0; i < scan_lines_; i++) {
    // Check that at least a minimum number of points can be extracted as edges
    if (scans[i]->points.size() < min_points_per_scan_) {
      continue;
    }

    // Compute the smoothness of each point in the scan
    std::vector<SmoothnessItem> smooths;
    for(size_t j = 5; j < scans[i]->points.size() - 5; j++) {
      double diff_x = scans[i]->points[j - 5].x + 
                      scans[i]->points[j - 4].x + 
                      scans[i]->points[j - 3].x + 
                      scans[i]->points[j - 2].x + 
                      scans[i]->points[j - 1].x - 
                      10 * scans[i]->points[j].x + 
                      scans[i]->points[j + 1].x + 
                      scans[i]->points[j + 2].x + 
                      scans[i]->points[j + 3].x + 
                      scans[i]->points[j + 4].x + 
                      scans[i]->points[j + 5].x;
      double diff_y = scans[i]->points[j - 5].y + 
                      scans[i]->points[j - 4].y + 
                      scans[i]->points[j - 3].y + 
                      scans[i]->points[j - 2].y + 
                      scans[i]->points[j - 1].y - 
                      10 * scans[i]->points[j].y + 
                      scans[i]->points[j + 1].y + 
                      scans[i]->points[j + 2].y + 
                      scans[i]->points[j + 3].y + 
                      scans[i]->points[j + 4].y + 
                      scans[i]->points[j + 5].y;
      double diff_z = scans[i]->points[j - 5].z + 
                      scans[i]->points[j - 4].z + 
                      scans[i]->points[j - 3].z + 
                      scans[i]->points[j - 2].z + 
                      scans[i]->points[j - 1].z - 
                      10 * scans[i]->points[j].z + 
                      scans[i]->points[j + 1].z + 
                      scans[i]->points[j + 2].z + 
                      scans[i]->points[j + 3].z + 
                      scans[i]->points[j + 4].z + 
                      scans[i]->points[j + 5].z;
      SmoothnessItem item(j, diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
      picked_[j] = false;
      smooths.push_back(item);
    }    

    // Extractg edges from each region
    int total_points = scans[i]->points.size() - 10;
    int sector_length = (int)(total_points / scan_regions_);
    for(int j = 0; j < scan_regions_; j++) {
      // Compute the bounds of each region
      int region_start = sector_length * j;
      int region_end = sector_length * (j + 1);
      if (j == scan_regions_ - 1) {
        // Remaining points are considered in the last region
        region_end = total_points;
      }

      std::vector<SmoothnessItem> smooths_sub(smooths.begin() + region_start,
                                              smooths.begin() + region_end);
      extractFeaturesFromRegion(scans[i], smooths_sub, pc_edges);            
    }
  }
}

void LidarOdometry::extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges) {

  // Code adapted from FLOAM (https://github.com/wh200720041/floam)
  
  // Sorting smoothness in decreasing order
  std::sort(smooths.begin(), smooths.end());

  // Extracting edges
  int picked_edges = 0;
  for (size_t i = 0; i < smooths.size(); i++) {
    int point_index = smooths[i].point_index;
    
    if (!picked_[point_index]) { // Not picked yet
      // Checking distance / max per region
      if (smooths[i].smoothness < 0.1 || picked_edges > edges_per_region_) {
        break;
      }

      // Picking the point as an edge
      pc_edges->push_back(pc_in->points[point_index]);
      picked_edges++;
      picked_[point_index] = true;

      // Mark neighbors in S as picked
      for (int l = 1; l <= 5; l++) {
        double diff_x = pc_in->points[point_index + l].x - 
                        pc_in->points[point_index + l - 1].x;
        double diff_y = pc_in->points[point_index + l].y - 
                        pc_in->points[point_index + l - 1].y;
        double diff_z = pc_in->points[point_index + l].z - 
                        pc_in->points[point_index + l - 1].z;
        
        // Checking if the distance between two consecutive points is high
        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
          break;
        }

        picked_[point_index + l] = true;
      }

      for (int l = -1; l >= -5; l--) {
        double diff_x = pc_in->points[point_index + l].x - 
                        pc_in->points[point_index + l + 1].x;        
        double diff_y = pc_in->points[point_index + l].y - 
                        pc_in->points[point_index + l + 1].y;
        double diff_z = pc_in->points[point_index + l].z - 
                        pc_in->points[point_index + l + 1].z;
        
        // Checking if the distance between two consecutive points is high
        if (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 0.05) {
          break;
        }

        picked_[point_index + l] = true;
      }      
    }
  }
}

void LidarOdometry::estimatePose(const PointCloud::Ptr& pc_in, const PointCloud::Ptr& pc_edges) {

  // If it is the first pointcloud, we just save the edges
  if (!init_) {
    prev_edges_.push_back(pc_edges);
    init_ = true;
  } else {
    // Computing local map    
    PointCloud::Ptr total_points(new PointCloud);
    for (size_t i = 0; i < prev_edges_.size(); i++) {
      *total_points += *(prev_edges_[i]);
    }

    ROS_DEBUG("Total points: %lu", total_points->size());

    PointCloud::Ptr local_map(new PointCloud);
    if (prev_edges_.size() == 5) {
      // Voxelize the points
      pcl::VoxelGrid<Point> voxel_filter;
      voxel_filter.setLeafSize(0.15, 0.15, 0.15);
      voxel_filter.setInputCloud(total_points);
      voxel_filter.filter(*local_map);
    } else {
      local_map = total_points;
    }
    ROS_DEBUG("Local Map points: %lu", local_map->size());

    // Creating a tree to search for correspondences
    pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
    tree->setInputCloud(local_map);

    // Predict the current pose
    Eigen::Isometry3d pred_odom = odom_ * (prev_odom_.inverse() * odom_);
    prev_odom_ = odom_;
    odom_ = pred_odom;
    q_curr = Eigen::Quaterniond(odom_.rotation());
    t_curr = odom_.translation();

    // Translated edges
    PointCloud::Ptr pc_edges_w(new PointCloud);

    // We perform a short ICP algorithm assuming small movement
    for (int optim_it = 0; optim_it < 2; optim_it++) {

      // Transform the current edges to the map according to the current pose
      pc_edges_w->clear();
      pcl::transformPointCloud(*pc_edges, *pc_edges_w, odom_.matrix());

      // Compute edge correspondences
      // std::vector<Match> matchings;
      std::vector<std::vector<Eigen::Vector3d>> matchings;
      for (size_t i = 0; i < pc_edges_w->points.size(); i++) {
        std::vector<int> indices;
        std::vector<float> sq_dist;
        tree->nearestKSearch(pc_edges_w->points[i], 5, indices, sq_dist);
        if (sq_dist[4] < 1.0) {
          // Computing the center of mass
          // Eigen::Vector3d center(0, 0, 0);
          // for (int j = 0; j < 5; j++) {
          //   Eigen::Vector3d tmp(local_map->points[indices[j]].x,
          //                       local_map->points[indices[j]].y,
          //                       local_map->points[indices[j]].z);
          //   center = center + tmp;
          // }
          // center = center / 5.0;

          // // Creating a match between the current point and the center of mass
          // Eigen::Vector3d curr_point(pc_edges_w->points[i].x,
          //                            pc_edges_w->points[i].y,
          //                            pc_edges_w->points[i].z);
          // Match m = std::make_pair(curr_point, center);
          // matchings.push_back(m);

          std::vector<Eigen::Vector3d> match;
          Eigen::Vector3d curr_point(pc_edges_w->points[i].x,
                                     pc_edges_w->points[i].y,
                                     pc_edges_w->points[i].z);
          match.push_back(curr_point);

          Eigen::Vector3d pta(local_map->points[indices[0]].x,
                              local_map->points[indices[0]].y,
                              local_map->points[indices[0]].z);
          match.push_back(pta);

          Eigen::Vector3d ptb(local_map->points[indices[1]].x,
                              local_map->points[indices[1]].y,
                              local_map->points[indices[1]].z);
          match.push_back(ptb);

          matchings.push_back(match);
        }
      }

      ROS_DEBUG("Matchings: %lu", matchings.size());

      // Optimizing the current pose
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(param_q, 4, q_parameterization);
      problem.AddParameterBlock(param_t, 3);

      // Adding constraints
      for (size_t j = 0; j < matchings.size(); j++) {
        // ceres::CostFunction* cost_function = Point2PointFactor::create(matchings[j].first, matchings[j].second);
        ceres::CostFunction* cost_function = Point2LineFactor::create(matchings[j][0], matchings[j][1], matchings[j][2]);
        problem.AddResidualBlock(cost_function, loss_function, param_q, param_t);
      }

      // Solving the optimization problem
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 5;
      options.minimizer_progress_to_stdout = false;
      options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );
      options.num_linear_solver_threads = sysconf( _SC_NPROCESSORS_ONLN );
      // options.initial_trust_region_radius = 1e14;
      // options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      std::cout << summary.BriefReport() << "\n";

      odom_ = Eigen::Isometry3d::Identity();
      odom_.linear() = q_curr.toRotationMatrix();
      odom_.translation() = t_curr;
    }

    // Compute the position of the detectd edges according to the final estimate position
    pc_edges_w->clear();
    pcl::transformPointCloud(*pc_edges, *pc_edges_w, odom_.matrix());

    // Move edges using the updated pose
    prev_edges_.push_back(pc_edges_w);
    if (prev_edges_.size() > 5) {
      prev_edges_.erase(prev_edges_.begin());
    }

    // Publishing odometry
    Eigen::Quaterniond q_current(odom_.rotation());
    Eigen::Vector3d t_current = odom_.translation();

    nav_msgs::Odometry laser_odom_msg;
    laser_odom_msg.header.frame_id = "world";
    laser_odom_msg.child_frame_id = "base_link";
    laser_odom_msg.header.stamp = ros::Time::now();
    laser_odom_msg.pose.pose.orientation.x = q_current.x();
    laser_odom_msg.pose.pose.orientation.y = q_current.y();
    laser_odom_msg.pose.pose.orientation.z = q_current.z();
    laser_odom_msg.pose.pose.orientation.w = q_current.w();
    laser_odom_msg.pose.pose.position.x = t_current.x();
    laser_odom_msg.pose.pose.position.y = t_current.y();
    laser_odom_msg.pose.pose.position.z = t_current.z();
    laser_odom_pub_.publish(laser_odom_msg);
  }  
}

}  // namespace liodom
