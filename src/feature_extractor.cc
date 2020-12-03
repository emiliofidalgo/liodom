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
  min_range_(3.0),
  max_range_(75.0),
  lidar_type_(0),
  scan_lines_(64),
  scan_regions_(8),
  edges_per_region_(10),
  min_points_per_scan_(scan_regions_ * edges_per_region_ + 10),
  save_results_(false),
  sdata(SharedData::getInstance()),
  stats(Stats::getInstance()),
  ncores_(2)
  {
    // Updating the available number of cores for processing
    int max_ncores = omp_get_max_threads() - 5;
    if (max_ncores > 1) {
      ncores_ = max_ncores;
    }
}

FeatureExtractor::~FeatureExtractor() {  
}

void FeatureExtractor::initialize() {
  
  // Reading parameters
  // Minimum range in meters
  nh_.param("min_range", min_range_, 3.0);
  ROS_INFO("Minimum range: %.2f", min_range_);

  // Maximum range in meters
  nh_.param("max_range", max_range_, 75.0);
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

  // Save results
  nh_.param("save_results", save_results_, false);
  ROS_INFO("Save results: %s", save_results_ ? "Yes" : "No");

  // Publishers
  pc_edges_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("edges", 10);
}

void FeatureExtractor::operator()(std::atomic<bool>& running) {
  
  while(running) {

    PointCloud::Ptr pc_curr;
    std_msgs::Header pc_header;
    
    if (sdata->popPointCloud(pc_curr, pc_header)) {      

      auto start_t = Clock::now();

      // TODO
      std::vector<Plane> planes;
      extractPlanes(pc_curr, planes);

      // // Split the pointcloud into scan_lines_ scans
      // std::vector<PointCloud::Ptr> scans;
      // splitPointCloud(pc_curr, scans);

      // // Extracting features from each scan
      // PointCloud::Ptr pc_edges(new PointCloud);
      // extractFeatures(scans, pc_edges);

      auto end_t = Clock::now();

      // ROS_DEBUG("Feature extraction: %lu edges", pc_edges->points.size());

      // Register stats
      if (save_results_) {
        stats->addFeatureExtractionTime(start_t, end_t);
      }

      // Send extracted features for laser odometry
      // sdata->pushFeatures(pc_edges);

      // Publishing edges if there is someone listening
      // if (pc_edges_pub_.getNumSubscribers() > 0) {
      //   sensor_msgs::PointCloud2 edges_msg;
      //   pcl::toROSMsg(*pc_edges, edges_msg);
      //   edges_msg.header = pc_header;
      //   pc_edges_pub_.publish(edges_msg);
      // }
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
  if (*dist > max_range_ || *dist < min_range_) {
    valid = false;
  }

  return valid;
}

void FeatureExtractor::splitPointCloud(const PointCloud::Ptr& pc_in, std::vector<PointCloud::Ptr>& scans) {

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

      if (scan_lines_ == 64) {
        if (angle >= -8.83)
          scan_id = int((2 - angle) * 3.0 + 0.5);
        else
          scan_id = scan_lines_ / 2 + int((-8.83 - angle) * 2.0 + 0.5);

        if (angle > 2 || angle < -24.33 || scan_id > 63 || scan_id < 0) {
          continue;
        }        
      } else if (scan_lines_ == 32) {
        scan_id = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
        if (scan_id > (scan_lines_ - 1) || scan_id < 0) {
          continue;
        }
      } else if (scan_lines_ == 16) {
        scan_id = int((angle + 15) / 2 + 0.5);
        if (scan_id > (scan_lines_ - 1) || scan_id < 0) {
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
  } else if (lidar_type_ == 1) { // Ouster
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
  for (int i = 0; i < scan_lines_; i++) {
    // Check that at least a minimum number of points can be extracted as edges
    if (scans[i]->points.size() < min_points_per_scan_) {
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

void Plane::computeStats() {

  // Compute mean vector of the knns
  Eigen::Vector3d P_mean(0, 0, 0);
  for (size_t i = 0; i < points.size(); i++) {
    P_mean = P_mean + points[i].coords;
  }
  P_mean /= double(points.size());

  // Compute the covariance matrix
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < points.size(); i++) {
    Eigen::Matrix<double, 3, 1> tmp = points[i].coords - P_mean;
    cov = cov + (tmp * tmp.transpose());

    // We take profit of this loop to add scales
    scale += points[i].scale;
  }
  cov /= double(points.size());
  scale /= double(points.size());

  // Computing eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);

  // Set normal and curvature of the plane
  normal = saes.eigenvectors().col(0);
  curvature = saes.eigenvalues()[0] / 
                              (saes.eigenvalues()[0] + 
                               saes.eigenvalues()[1] + 
                               saes.eigenvalues()[2] +
                               (rand() % 10 + 1) * 1e-7);

  // Set the final scale (Check original implementation, why 5?)
  scale *= 5.0;
}


void FeatureExtractor::extractPlanes(const PointCloud::Ptr& pc_in, std::vector<Plane>& planes) {
  // Extracting point information
  std::vector<PointInfo> point_info(pc_in->size(), PointInfo());
  computePointInfo(pc_in, point_info);

  std::vector<Plane> planes_init;
  regionGrowing(point_info, planes_init);

  regionMerging(point_info, planes_init, planes);

  // for (int i = 0; i < planes.size(); i++) {
  //   std::cout << planes[i] << std::endl;
  // }

  // std::cout << "---" << std::endl;
}

void FeatureExtractor::computePointInfo(const PointCloud::Ptr& pc_in, std::vector<PointInfo>& point_info) {

  // Creating a tree to search for neigbours
  pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
  tree->setInputCloud(pc_in);

  int k = 25;

  #pragma omp parallel for num_threads(ncores_)
  for (size_t i = 0; i < pc_in->points.size(); i++) {
    // Set the point index
    point_info[i].point_index = i;

    // Set the point coordinates
    double x = pc_in->points[i].x;
    double y = pc_in->points[i].y;
    double z = pc_in->points[i].z;    
    point_info[i].coords = Eigen::Vector3d(x, y, z);

    // Computing point information
    // Check if the point is valid
    double distance;
    if (!isValidPoint(x, y, z, &distance)) {
      continue;
    }   

    // Set the point as valid
    point_info[i].valid = true;

    // Searching for the k NNs of the point
    std::vector<float> sq_dist;
    tree->nearestKSearch(pc_in->points[i], k, point_info[i].knns, sq_dist);

    // Compute mean vector of the knns
    Eigen::Vector3d P_mean(0, 0, 0);
    std::vector<Eigen::Vector3d> near_points;
    for (int j = 0; j < k; j++) {
      Eigen::Vector3d tmp(pc_in->points[point_info[i].knns[j]].x,
                          pc_in->points[point_info[i].knns[j]].y,
                          pc_in->points[point_info[i].knns[j]].z);
      P_mean = P_mean + tmp;
      near_points.push_back(tmp);
    }
    P_mean /= double(k);

    // Compute the covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int j = 0; j < k; j++) {
      Eigen::Matrix<double, 3, 1> tmp = near_points[j] - P_mean;
      cov = cov + (tmp * tmp.transpose());
    }
    cov /= double(k);

    // Computing eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov);

    // Set normal and curvature of the point
    point_info[i].normal = saes.eigenvectors().col(0);
    point_info[i].curvature = saes.eigenvalues()[0] / 
                              (saes.eigenvalues()[0] + 
                               saes.eigenvalues()[1] + 
                               saes.eigenvalues()[2] +
                               (rand() % 10 + 1) * 1e-7);
    
    // Set the scale of the point
    Eigen::Matrix<double, 3, 1> tmp = near_points[2] - point_info[i].coords;
    point_info[i].scale = std::sqrt(tmp.transpose() * tmp);    
  }
}

void FeatureExtractor::regionGrowing(const std::vector<PointInfo>& point_info, std::vector<Plane>& planes) {

  // Sorting points according to its curvature
  std::vector<std::pair<double, int>> idx_sorted(point_info.size());
  for (size_t i = 0; i < point_info.size(); i++) {
    idx_sorted[i].first = point_info[i].curvature;
		idx_sorted[i].second = i;
  }
  std::sort(idx_sorted.begin(), idx_sorted.end());

  double th_angle = 15.0 / 180.0 * M_PI;
  double th_normal = std::cos(th_angle);
  
  // Iterating through points in increasing order of curvature
	double percent = 0.9;
	int max_idx = int(point_info.size() * percent);
	std::vector<int> is_used(point_info.size(), 0);

	for (int i = 0; i < max_idx; i++) {
		int idx_s = idx_sorted[i].second;
    PointInfo info_s = point_info[idx_s];

    assert(idx_s == info_s.point_index);

    // Check if it is used
		if (is_used[idx_s]) {
      continue;
    }

    // Check if it is a valid point
    if (!info_s.isValid()) {
      is_used[idx_s] = 1;
      continue;
    }

    Eigen::Vector3d P_s = info_s.coords;
    Eigen::Vector3d n_ps = info_s.normal;		
    double th_o = info_s.scale;
		double th_p = std::pow(50.0 * info_s.scale, 2);

		Plane curr_plane;
		curr_plane.addPoint(info_s);
		size_t nprocessed = 0;    

		while(nprocessed < curr_plane.size()) {
      // Get the current unprocessed point
			PointInfo curr_point = curr_plane.points[nprocessed];

      // Process their knns neighbours
      std::vector<int> to_add(curr_point.knns.size(), 0);
      #pragma omp parallel for num_threads(ncores_)
      for (size_t j = 0; j < curr_point.knns.size(); j++) {
        // Get the point
        PointInfo curr_n = point_info[curr_point.knns[j]];

        // Check if it is a valid point
        if (!curr_n.isValid()) {
          is_used[curr_n.point_index] = 1;
          continue;
        }

        // Check if it is used
        if (is_used[curr_n.point_index]) {
          continue;
        }

        // Judgement1: Normal deviation
        Eigen::Vector3d n_pj = curr_n.normal;
        double ndev = std::abs(n_ps.adjoint() * n_pj); // Dot product
				if (ndev < th_normal) {
					continue;
				}

        // Judgement2: Orthogonal distance
        Eigen::Vector3d P_j = curr_n.coords;
        Eigen::Vector3d P_d = P_s - P_j;
        double odist = std::abs(n_ps.adjoint() * P_d); // Dot product
				if (odist > th_o) {
					continue;
				}

        // Judgement3: Parallel distance
				double pdist = P_d.transpose() * P_d;
				if (pdist > th_p) {
					continue;
				}

        // Add the neighbor to the current plane
        to_add[j] = 1;
        is_used[curr_n.point_index] = 1;
      }

      // Adding neighbors to the plane
      for (size_t j = 0; j < curr_point.knns.size(); j++) {
        if (to_add[j]) {
          PointInfo curr_n = point_info[curr_point.knns[j]];
          curr_plane.points.push_back(curr_n);
        }
      }      

      // Mark this point as processed
      nprocessed++;
      is_used[curr_point.point_index] = 1;
		}

    // Adding the current plane to the list if required
		if (curr_plane.size() > 30) {
      curr_plane.computeStats();
			planes.push_back(curr_plane);
		} else {
      for (size_t j = 0; j < curr_plane.size(); j++) {
        // Freeing points as possible neighbors
				is_used[curr_plane.points[j].point_index] = 0;
			}
		}
	}
}

void FeatureExtractor::regionMerging(const std::vector<PointInfo>& point_info, const std::vector<Plane>& planes_in, std::vector<Plane>& planes_out) {
  
  // Setting the initial plane label for each point
  std::vector<int> point_label(point_info.size(), -1);
  #pragma omp parallel for num_threads(ncores_)
	for (size_t i = 0; i < planes_in.size(); i++) {
    for (size_t j = 0; j < planes_in[i].points.size(); j++) {
      point_label[planes_in[i].points[j].point_index] = i;
		}
	}

  // Finding adjacent planes
  std::vector<std::set<int>> adjacent_planes(planes_in.size());
  #pragma omp parallel for num_threads(ncores_)
  for (size_t i = 0; i < planes_in.size(); i++) { // i is the plane ID
    for (size_t j = 0; j < planes_in[i].points.size(); j++) { // j is the point id in plane i
      for (size_t k = 0; k < planes_in[i].points[j].knns.size(); k++) { // k is the neighboring point
        int plane_nn = point_label[planes_in[i].points[j].knns[k]];
        if (plane_nn > 0 && plane_nn != i) {
          adjacent_planes[i].insert(plane_nn);
        }
      }        
    }
  }

  pcl::RGB rgb;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (size_t i = 0; i < planes_in.size(); i++) {
      // Generate unique colour
      pcl::visualization::getRandomColors(rgb, 0, 255);      
      for (size_t j = 0; j < planes_in[i].points.size(); j++) { // j is the point id in plane i
        pcl::PointXYZRGB p(          
          (std::uint8_t)rgb.r,
          (std::uint8_t)rgb.g,
          (std::uint8_t)rgb.b
        );
        p.x = (float)planes_in[i].points[j].coords(0);
        p.y = (float)planes_in[i].points[j].coords(1);
        p.z = (float)planes_in[i].points[j].coords(2);
        pc_rgb->push_back(p);
      }    
  }

   // Create viewer
  pcl::visualization::PCLVisualizer viewer;
  viewer.setBackgroundColor (0, 0, 0);

  viewer.addPointCloud<pcl::PointXYZRGB>(pc_rgb, "PC");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "PC");

  // Keep viewer running
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


  // for (int i = 0; i < adjacent_planes.size(); i++) {
  //   std::cout << "Plane " << i << std::endl;
  //   for (auto elem : adjacent_planes[i]) {
  //       std::cout << elem << " , ";
  //   }
  //   std::cout << "---" << std::endl;
  // }
  // exit(0);
}

} // namespace liodom