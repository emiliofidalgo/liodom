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

#ifndef INCLUDE_LIODOM_DEFS_H
#define INCLUDE_LIODOM_DEFS_H

#include <chrono>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace liodom {

// PCL typedefs
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> PointCloud;

// Chrono typedefs
typedef std::chrono::high_resolution_clock Clock;

} // namespace liodom

#endif // INCLUDE_LIODOM_DEFS_H