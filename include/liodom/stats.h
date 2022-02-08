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

#ifndef INCLUDE_LIODOM_STATS_H
#define INCLUDE_LIODOM_STATS_H

#include <fstream>
#include <iostream>
#include <queue>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <liodom/defs.h>

#define SSTR( x ) dynamic_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

namespace liodom {

// Singleton class to manage shared data
class Stats {
  public:
    static Stats* getInstance();
    Stats(Stats const&) = delete;
    void operator=(Stats const&) = delete;

    // Public methods
    void addPose(const Eigen::Matrix4d& pose);
    void addFeatureExtractionTime(const Clock::time_point& start, const Clock::time_point& end);
    void addLaserOdometryTime(const Clock::time_point& start, const Clock::time_point& end);
    void addNumOfFeats(const size_t& nfeats);
    void startFrame(const Clock::time_point& start);
    void stopFrame(const Clock::time_point& stop);
    void writeResults(const std::string& dir);

  private:
    // Controlling the singleton
    static Stats* pinstance_;   
    static std::mutex sdata_mutex_;

    // Members
    std::vector<Eigen::Matrix4d> poses_;
    std::vector<double> feat_extr_;
    std::vector<double> laser_odom_;
    std::vector<size_t> num_of_features_;

    // Total times
    std::mutex frame_mutex_;
    std::queue<Clock::time_point> start_times_;
    std::vector<double> frame_times_;

  protected:
    Stats() {};
    ~Stats() {
      if (pinstance_ != nullptr) {
        delete pinstance_;
      }
    };
};

} // namespace liodom

#endif // INCLUDE_LIODOM_STATS_H
