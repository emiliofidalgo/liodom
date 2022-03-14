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

#ifndef INCLUDE_LIODOM_SHARED_DATA_H
#define INCLUDE_LIODOM_SHARED_DATA_H

#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <pcl/common/io.h>

#include <liodom/defs.h>

namespace liodom {

// Singleton class to manage shared data
class SharedData {
  public:
    static SharedData* getInstance();
    SharedData(SharedData const&) = delete;
    void operator=(SharedData const&) = delete;

    void pushPointCloud(const PointCloud::Ptr& pc_in, const std_msgs::Header& header);
    bool popPointCloud(PointCloud::Ptr& pc_out, std_msgs::Header& header);

    void pushFeatures(const PointCloud::Ptr& feat_in, std_msgs::Header& header);
    bool popFeatures(PointCloud::Ptr& feat_out, std_msgs::Header& header);

    void setLocalMap(const PointCloud::Ptr& map_in);
    void getLocalMap(PointCloud::Ptr& map_out);

    void setLastIMUOri(Eigen::Quaterniond& imu_ori);
    void getLastIMUOri(Eigen::Quaterniond& imu_ori);

    void setLastZ(double& z);
    void getLastZ(double& z);

  private:
    // Controlling the singleton
    static SharedData* pinstance_;
    static std::mutex sdata_mutex_;

    // PointCloud control
    std::mutex pc_mutex_;
    std::queue<PointCloud::Ptr> pc_buf_;
    std::queue<std_msgs::Header> pc_header_;

    // Feature control
    std::mutex feat_mutex_;
    std::queue<PointCloud::Ptr> feat_buf_;
    std::queue<std_msgs::Header> feat_header_;

    // Local Map control
    std::mutex map_mutex_;
    PointCloud::Ptr local_map_;

    // Last IMU control
    std::mutex imu_mutex_;
    Eigen::Quaterniond last_IMU_ori_;

    // Last Z control
    std::mutex z_mutex_;
    double last_Z_;
    

  protected:
    SharedData() : local_map_(new PointCloud) {};
    ~SharedData() {
      if (pinstance_ != nullptr) {
        delete pinstance_;
      }
    };
};

} // namespace liodom

#endif // INCLUDE_LIODOM_SHARED_DATA_H
