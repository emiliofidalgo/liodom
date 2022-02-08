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

#include "liodom/feature_extractor.h"

namespace liodom {

FeatureExtractor::FeatureExtractor(const ros::NodeHandle& nh) :
  nh_(nh),
  sdata(SharedData::getInstance()),
  stats(Stats::getInstance()),
  params(Params::getInstance()),
  ncores_(2) {
    // Updating the available number of cores for processing
    int max_ncores = omp_get_max_threads() - 5;
    if (max_ncores > 1) {
      ncores_ = max_ncores;
    }    
}

FeatureExtractor::~FeatureExtractor() {  
}

void FeatureExtractor::operator()(std::atomic<bool>& running) {
  
  while(running) {

    PointCloud::Ptr pc_curr;
    std_msgs::Header pc_header;
    
    if (sdata->popPointCloud(pc_curr, pc_header)) {      

      // Split the pointcloud into scan_lines_ scans
      std::vector<PointCloud::Ptr> scans;
      splitPointCloud(pc_curr, scans);

      auto start_t = Clock::now();

      // Extracting features from each scan
      PointCloud::Ptr pc_edges(new PointCloud);
      extractFeatures(scans, pc_edges);
      auto end_t = Clock::now();

      ROS_DEBUG("Feature extraction: %lu edges", pc_edges->points.size());

      // Register stats
      if (params->save_results_) {
        stats->addFeatureExtractionTime(start_t, end_t);
        stats->addNumOfFeats(pc_edges->points.size());
      }

      // Send extracted features for laser odometry
      sdata->pushFeatures(pc_edges, pc_header);      
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

bool FeatureExtractor::isValidPoint(const double x, const double y, const double z, double* dist) {

  bool valid = true;
  
  // Ignore the point if it is NaN
  if ((!std::isfinite(x) ||
       !std::isfinite(y) ||
       !std::isfinite(z))) {
    valid = false;
  }

  // Filter the point by its distance in XY
  *dist = std::sqrt(x * x + y * y);
  if (*dist > params->max_range_ || *dist < params->min_range_) {
    valid = false;
  }

  return valid;
}

void FeatureExtractor::splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans) {

  // Initialize the vector of scans
  scans.clear();
  for (int i = 0; i < params->scan_lines_; i++) {
    scans.push_back(PointCloud::Ptr(new PointCloud));
  }

  // Processing the scan according to the Lidar model
  if (params->lidar_type_ == 0) {  // Velodyne
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

      if (params->scan_lines_ == 64) {
        if (angle >= -8.83)
          scan_id = int((2 - angle) * 3.0 + 0.5);
        else
          scan_id = params->scan_lines_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

        if (angle > 2 || angle < -24.33 || scan_id > 63 || scan_id < 0) {
          continue;
        }        
      } else if (params->scan_lines_ == 32) {
        scan_id = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
        if (scan_id > (params->scan_lines_ - 1) || scan_id < 0) {
          continue;
        }
      } else if (params->scan_lines_ == 16) {
        scan_id = int((angle + 15) / 2 + 0.5);
        if (scan_id > (params->scan_lines_ - 1) || scan_id < 0) {
          continue;
        }                      
      } else {
        ROS_ERROR_ONCE("Invalid scan lines: %i", params->scan_lines_);
      }

      // Adding the point to the corresponding scan
      if (scan_id != -1) {
        scans[scan_id]->push_back(pc_in->points[i]);
      }      
    }
  } else if (params->lidar_type_ == 1) { // Ouster
    // Assigning each point to its corresponding scan line
    for (size_t row = 0; row < pc_in->height; row++) {
      for (size_t col = 0; col < pc_in->width; col++) {
        double x = pc_in->at(col, row).x;
        double y = pc_in->at(col, row).y;
        double z = pc_in->at(col, row).z;
        
        // Check if the point is valid
        double distance;
        if (!isValidPoint(x, y, z, &distance)) {
          continue;
        }

        // Adding the point to the corresponding scan        
        scans[row]->push_back(pc_in->at(col, row));
      }
    }
  } else {
    ROS_ERROR_ONCE("Incorrect Lidar type");
  }
}

void FeatureExtractor::extractFeatures(const std::vector<PointCloud::Ptr>& scans, PointCloud::Ptr& pc_edges) {
  
  // Code adapted from FLOAM (https://github.com/wh200720041/floam)

  // Each scan is processed individually  
  for (int i = 0; i < params->scan_lines_; i++) {
    // Check that at least a minimum number of points can be extracted as edges
    if (scans[i]->points.size() < params->min_points_per_scan_) {
      continue;
    }

    // Compute the smoothness of each point in the scan
    std::vector<SmoothnessItem> smooths_aux(scans[i]->points.size(), SmoothnessItem());
    #pragma omp parallel for num_threads(ncores_)
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
      smooths_aux[j] = item;
    }

    // Extracting subvector
    std::vector<SmoothnessItem> smooths = {smooths_aux.begin() + 5, smooths_aux.end() - 5};

    // Extractg edges from each region
    int total_points = scans[i]->points.size() - 10;
    int sector_length = (int)(total_points / params->scan_regions_);
    for(int j = 0; j < params->scan_regions_; j++) {
      // Compute the bounds of each region
      int region_start = sector_length * j;
      int region_end = sector_length * (j + 1);
      if (j == params->scan_regions_ - 1) {
        // Remaining points are considered in the last region
        region_end = total_points;
      }

      std::vector<SmoothnessItem> smooths_sub(smooths.begin() + region_start,
                                              smooths.begin() + region_end);
      extractFeaturesFromRegion(scans[i], smooths_sub, pc_edges);
    }
  }
}

void FeatureExtractor::extractFeaturesFromRegion(const PointCloud::Ptr& pc_in, std::vector<SmoothnessItem>& smooths, PointCloud::Ptr& pc_edges) {

  // Code adapted from FLOAM (https://github.com/wh200720041/floam)
  
  // Sorting smoothness in decreasing order
  std::sort(smooths.begin(), smooths.end());

  // Extracting edges
  int picked_edges = 0;
  for (size_t i = 0; i < smooths.size(); i++) {
    int point_index = smooths[i].point_index;
    
    if (!picked_[point_index]) { // Not picked yet
      // Checking distance / max per region
      if (smooths[i].smoothness < 0.1 || picked_edges > params->edges_per_region_) {
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
    
} // namespace liodom