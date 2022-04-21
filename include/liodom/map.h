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

#ifndef INCLUDE_LIODOM_MAP_H
#define INCLUDE_LIODOM_MAP_H

#include <unordered_map>
#include <vector>

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// Liodom
#include <liodom/defs.h>

namespace liodom {

// Cell
class Cell {
 public:
  Cell();
  explicit Cell(const PointCloud::Ptr& pc);
  virtual ~Cell();
  
  void addPoint(const Point& p);
  void addPoints(const PointCloud::Ptr& pc);
  void clear();
  void filter(pcl::VoxelGrid<Point>& filt);
  bool modified();
  PointCloud::Ptr getPoints();

 private:
  PointCloud::Ptr points_;
  bool modified_;
};

// HashKey
struct HashKey {
  HashKey() {}
  explicit HashKey(int x_, int y_, int z_) : 
    x(x_),
    y(y_),
    z(z_) {}

  bool operator==(const HashKey& other) const {
    if (this->x == other.x && 
        this->y == other.y &&
        this->z == other.z)
        return true;
    else
      return false;
  }

  struct HashFunc {
    size_t operator()(const HashKey& k) const {
      size_t h1 = std::hash<int>()(k.x);
      size_t h2 = std::hash<int>()(k.y);
      size_t h3 = std::hash<int>()(k.z);      
      
      return (h1 ^ (h2 << 1)) ^ (h3 << 2);
      //return h1 + h2 + h3;
    }
  };

  int x;
  int y;
  int z;
};

// Defining a map of cells referred using its corresponding HashKey
typedef std::unordered_map<HashKey, Cell*, HashKey::HashFunc> HashMap;

// Map
class Map {
 public:
  explicit Map(const double xy_size, const double z_size, const double res);
  virtual ~Map();

  void updateMap(const PointCloud::Ptr& pc_in, const Eigen::Isometry3d& pose);
  PointCloud::Ptr getMap();
  PointCloud::Ptr getLocalMap(const Eigen::Isometry3d& pose, int cells_xy = 2, int cells_z = 1);
  double getMapEntropy();
 private:
  double voxel_xysize_; // Assume the same size in X and Y dimensions
  double inv_voxel_xysize_;
  double voxel_xysize_half_;
  double voxel_zsize_;
  double inv_voxel_zsize_;
  double voxel_zsize_half_;
  double resolution_;
  pcl::VoxelGrid<Point> vgrid_filter_;

  // Hash table with Cells
  HashMap cells_;
  std::vector<Cell*> cells_vector_; // For a faster iteration
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_MAP_H