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

#include <liodom/stats.h>

namespace liodom {

Stats* Stats::pinstance_{nullptr};
std::mutex Stats::sdata_mutex_;

Stats* Stats::getInstance() {
  std::lock_guard<std::mutex> lock(sdata_mutex_);
  
  if (pinstance_ == nullptr) {
    pinstance_ = new Stats();
  }
  
  return pinstance_;
}

void Stats::addPose(const Eigen::Matrix4d& pose) {
  poses_.push_back(pose);
}

void Stats::addFeatureExtractionTime(const Clock::time_point& start, const Clock::time_point& end) {
  double diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  feat_extr_.push_back(diff);
}

void Stats::addLaserOdometryTime(const Clock::time_point& start, const Clock::time_point& end) {
  double diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  laser_odom_.push_back(diff);
}

void Stats::addNumOfFeats(const size_t& nfeats) {
  num_of_features_.push_back(nfeats);
}

void Stats::startFrame(const Clock::time_point& start) {
  frame_mutex_.lock();
  start_times_.push(start);
  frame_mutex_.unlock();
}	

void Stats::stopFrame(const Clock::time_point& stop) {
  frame_mutex_.lock();
  if (!start_times_.empty()) {
    Clock::time_point start = start_times_.front();
    start_times_.pop();

    double diff = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
    frame_times_.push_back(diff);
  }
  frame_mutex_.unlock();
}

void Stats::writeResults(const std::string& dir) {
  // Poses
  std::string poses_filename = dir + "poses.txt";
  std::ofstream poses_file;
  poses_file.open(poses_filename.c_str(), std::ios::out | std::ios::trunc);
  for (size_t pose_ind = 0; pose_ind < poses_.size(); pose_ind++) {
    for (int i = 0; i < 3; i++) { // Rows
      for (int j = 0; j < 4; j++) { // Cols
        poses_file << poses_[pose_ind](i, j);

        if (j != 3) {
          poses_file << " ";
        }
      }

      if (i == 2) {
        poses_file << std::endl;
      } else {
        poses_file << " ";
      }
    }
  }
  poses_file.close();

  // Feature extraction times
  std::string fext_filename = dir + "feat_ext_times.txt";
  std::ofstream fext_file;
  fext_file.open(fext_filename.c_str(), std::ios::out | std::ios::trunc);
  for (size_t fext_ind = 0; fext_ind < feat_extr_.size(); fext_ind++) {
    fext_file << feat_extr_[fext_ind] << std::endl;
  }
  fext_file.close();

  // Laser odometer times
  std::string lodom_filename = dir + "laser_odom_times.txt";
  std::ofstream lodom_file;
  lodom_file.open(lodom_filename.c_str(), std::ios::out | std::ios::trunc);
  for (size_t lodom_ind = 0; lodom_ind < laser_odom_.size(); lodom_ind++) {
    lodom_file << laser_odom_[lodom_ind] << std::endl;
  }
  lodom_file.close();

  // Number of features
  std::string nfeats_filename = dir + "nfeats.txt";
  std::ofstream nfeats_file;
  nfeats_file.open(nfeats_filename.c_str(), std::ios::out | std::ios::trunc);
  for (size_t nfeats_ind = 0; nfeats_ind < num_of_features_.size(); nfeats_ind++) {
    nfeats_file << num_of_features_[nfeats_ind] << std::endl;
  }
  nfeats_file.close();
  
  // Total times
  std::string total_filename = dir + "frame_times.txt";
  std::ofstream total_file;
  total_file.open(total_filename.c_str(), std::ios::out | std::ios::trunc);
  for (size_t total_ind = 0; total_ind < frame_times_.size(); total_ind++) {
    total_file << frame_times_[total_ind] << std::endl;
  }
  total_file.close();
}

}  // namespace liodom
