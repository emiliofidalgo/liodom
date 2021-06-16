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

#include <liodom/map.h>

namespace liodom {

Cell::Cell() :
  points_(new PointCloud),
  modified_(false) {
    // TODO Prereserve memory according to a maximum number of points.
}

Cell::Cell(const PointCloud::Ptr& pc) :
  points_(new PointCloud),
  modified_(true) {
    // TODO Prereserve memory according to a maximum number of points.
    pcl::copyPointCloud(*pc, *points_);
}

Cell::~Cell() {
  points_->clear();
}

void Cell::addPoint(const Point& p) {
  points_->push_back(p);
  modified_ = true;
}

void Cell::addPoints(const PointCloud::Ptr& pc) {
  *points_ += *pc;
  modified_ = true;
}

void Cell::clear() {
  points_->clear();
  modified_ = true;
}

void Cell::filter(pcl::VoxelGrid<Point>& filt) {
  filt.setInputCloud(points_);
  filt.filter(*points_);
  modified_ = false;
}

bool Cell::modified() {
  return modified_;
}

PointCloud::Ptr Cell::getPoints() {
  return points_;
}

Map::Map(const double xy_size, const double z_size, const double res) :
  voxel_xysize_(xy_size),
  inv_voxel_xysize_(1.0 / xy_size),
  voxel_xysize_half_(xy_size / 2.0),
  voxel_zsize_(z_size),
  inv_voxel_zsize_(1.0 / z_size),
  voxel_zsize_half_(z_size / 2.0),
  resolution_(res) {

    // VoxelGrid filter
	  vgrid_filter_.setLeafSize(resolution_, resolution_, resolution_);
}

Map::~Map() {
  // Delete memory allocated dynamically
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    delete cells_vector_[i];
  }
}

void Map::updateMap(const PointCloud::Ptr& pc_in, const Eigen::Isometry3d& pose) {

  // Compute world coordinates
  PointCloud::Ptr points_map(new PointCloud);
  pcl::transformPointCloud(*pc_in, *points_map, pose.matrix());

  // Process each point
  for (size_t i = 0; i < points_map->points.size(); i++) {

    // Get the current point
		Point point = points_map->points[i];

		// Compute the corresponding cell
    int voxel_x = int(std::floor(point.x * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));
    int voxel_y = int(std::floor(point.y * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));
    int voxel_z = int(std::floor(point.z * inv_voxel_zsize_) * voxel_zsize_ + (voxel_zsize_half_));
    
    // Check if the voxel is already in the map
    HashKey key(voxel_x, voxel_y, voxel_z);
    Cell* cell;
    if (cells_.find(key) == cells_.end()) {
      // Cell is not in the map, so it is added
      cell = new Cell();
      cells_[key] = cell;
      cells_vector_.push_back(cell);
    } else {
      // Otherwise we get the corresponding cell
      cell = cells_[key];
    }

    cell->addPoint(point);
  }

  // Update every modified cell
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    if (cells_vector_[i]->modified()) {
      cells_vector_[i]->filter(vgrid_filter_);
    }
  }
}

PointCloud::Ptr Map::getMap() {
  PointCloud::Ptr total_points(new PointCloud);
  // Get PCs from every cell and concatenate them
  for (size_t i = 0; i < cells_vector_.size(); i++) {
    *total_points += *(cells_vector_[i]->getPoints());
  }

  return total_points;
}

PointCloud::Ptr Map::getLocalMap(const Eigen::Isometry3d& pose, int cells_xy, int cells_z) {
  // Computing the voxel corresponding to the current position
  // Compute the corresponding cell
  int x = pose.translation().x();
  int voxel_x = int(std::floor(x * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));

  int y = pose.translation().y();
  int voxel_y = int(std::floor(y * inv_voxel_xysize_) * voxel_xysize_ + (voxel_xysize_half_));

  int z = pose.translation().z();
  int voxel_z = int(std::floor(z * inv_voxel_zsize_) * voxel_zsize_ + (voxel_zsize_half_));

  // Final local map
  PointCloud::Ptr total_points(new PointCloud);

  // Getting points on neighbouring voxels of x and y
  int init_x = voxel_x - cells_xy * voxel_xysize_;
  int end_x =  voxel_x + cells_xy * voxel_xysize_;
  int init_y = voxel_y - cells_xy * voxel_xysize_;
  int end_y =  voxel_y + cells_xy * voxel_xysize_;
  // Get PCs from every cell on x axis
  for (int i = init_x; i <= end_x; i += voxel_xysize_) {
    for (int j = init_y; j <= end_y; j += voxel_xysize_) {
      // Check if the voxel is already in the map
      HashKey key(i, j, voxel_z);
      Cell* cell;
      if (cells_.find(key) != cells_.end()) {
        cell = cells_[key];
        *total_points += *(cell->getPoints());
      }
    }        
  }
    
  // Getting points on neighbouring voxels of z
  int init_z = voxel_z - cells_z * voxel_xysize_;
  int end_z  =  voxel_z + cells_z * voxel_xysize_;
  // Get PCs from every cell on x axis
  for (int i = init_z; i <= end_z; i += voxel_zsize_) {
    // Check if the voxel is already in the map
    HashKey key(voxel_x, voxel_y, i);
    Cell* cell;
    if (cells_.find(key) != cells_.end()) {
      cell = cells_[key];
      *total_points += *(cell->getPoints());
    }    
  }

  return total_points;
}

}  // namespace liodom
