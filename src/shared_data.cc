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

#include "liodom/shared_data.h"

namespace liodom {

SharedData* SharedData::pinstance_{nullptr};
std::mutex SharedData::sdata_mutex_;

SharedData* SharedData::getInstance() {
  std::lock_guard<std::mutex> lock(sdata_mutex_);
  
  if (pinstance_ == nullptr) {
    pinstance_ = new SharedData();
  }
  
  return pinstance_;
}

void SharedData::pushPointCloud(const PointCloud::Ptr& pc_in, const std_msgs::Header& header) {
  pc_mutex_.lock();
  pc_buf_.push(pc_in);
  pc_header_.push(header);
  pc_mutex_.unlock();
}

bool SharedData::popPointCloud(PointCloud::Ptr& pc_out, std_msgs::Header& header) {
  bool response = false;

  pc_mutex_.lock();
  if (!pc_buf_.empty()) {
    // Get PC
    pc_out = pc_buf_.front();
    pc_buf_.pop();

    // Get PC time
    header = pc_header_.front();
    pc_header_.pop();

    response = true;
  }
  pc_mutex_.unlock();
  
  return response;
}

void SharedData::pushFeatures(const PointCloud::Ptr& feat_in, std_msgs::Header& header) {
  feat_mutex_.lock();
  feat_buf_.push(feat_in);
  feat_header_.push(header);
  feat_mutex_.unlock();
}

bool SharedData::popFeatures(PointCloud::Ptr& feat_out, std_msgs::Header& header) {
  bool response = false;

  feat_mutex_.lock();
  if (!feat_buf_.empty()) {
    // Get features
    feat_out = feat_buf_.front();
    feat_buf_.pop();

    // Get PC time
    header = feat_header_.front();
    feat_header_.pop();

    response = true;
  }
  feat_mutex_.unlock();
  
  return response;
}

void SharedData::setLocalMap(const PointCloud::Ptr& map_in) {
  map_mutex_.lock();
  local_map_->clear();
  pcl::copyPointCloud(*map_in, *local_map_);
  map_mutex_.unlock();
}

void SharedData::getLocalMap(PointCloud::Ptr& map_out) {
  map_mutex_.lock();
  if (local_map_->size() > 0) {
    map_out->clear();
    pcl::copyPointCloud(*local_map_, *map_out);
  }
  map_mutex_.unlock();  
}

}  // namespace liodom