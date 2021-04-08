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

#ifndef INCLUDE_LIODOM_PARAMS_H
#define INCLUDE_LIODOM_PARAMS_H

#include <mutex>

#include <ros/ros.h>

namespace liodom {

// Singleton class to manage params through the whole app
class Params {
  public:
    // Parameters
    double min_range_;
    double max_range_;
    int lidar_type_;
    int scan_lines_;
    int scan_regions_;
    int edges_per_region_;
    size_t min_points_per_scan_;
    size_t local_map_size_;
    bool save_results_;
    std::string results_dir_;
    std::string fixed_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    bool use_imu_;

    // Methods
    static Params* getInstance();
    Params(Params const&) = delete;
    void operator=(Params const&) = delete;

    void readParams(const ros::NodeHandle& nh);

  private:
    // Controlling the singleton
    static Params* pinstance_;
    static std::mutex params_mutex_;    

  protected:
    Params() {};
    ~Params() {
      if (pinstance_ != nullptr) {
        delete pinstance_;
      }
    };
};

} // namespace liodom

#endif // INCLUDE_LIODOM_PARAMS_H
