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

#include <liodom/laser_odometry.h>

namespace liodom {

LocalMapManager::LocalMapManager(const size_t max_frames) :  
  total_points_(new PointCloud),
  nframes_(0),
  max_nframes_(max_frames)
  {
}

LocalMapManager::~LocalMapManager() {
}

void LocalMapManager::addPointCloud(const PointCloud::Ptr& pc) {
  // Adding the current frame to the local map
  *total_points_ += *pc;
  nframes_++;
  sizes_.push(pc->size());

  // Removing frames at the beginning if required
  if (nframes_ > max_nframes_) {
    // Get the size of the first frame
    size_t pc_size = sizes_.front();
    sizes_.pop();

    // Remove the points corresponding to the first frame
    std::vector<int> ind(pc_size);
    std::iota(std::begin(ind), std::end(ind), 0);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    indices->indices = ind;
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(total_points_);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*total_points_);

    // Reducing the number of frames
    nframes_--;
  }
}

size_t LocalMapManager::getLocalMap(PointCloud::Ptr& map) {
  map = total_points_;
  return nframes_;
}

void LocalMapManager::setMaxFrames(const size_t max_nframes) {
  max_nframes_ = max_nframes;
}

LaserOdometer::LaserOdometer(const ros::NodeHandle& nh) :
  nh_(nh),
  min_range_(3.0),
  max_range_(75.0),
  prev_frames_(5),
  save_results_(false),
  results_dir_("~/"),
  init_(false),
  prev_odom_(Eigen::Isometry3d::Identity()),
  odom_(Eigen::Isometry3d::Identity()),
  lmap_manager(prev_frames_),
  // q_curr(param_q),
  // t_curr(param_t),
  sdata(SharedData::getInstance()),
  stats(Stats::getInstance()) {
}

LaserOdometer::~LaserOdometer() {
}

void LaserOdometer::initialize() {
  
  // Reading parameters
  // Minimum range in meters
  nh_.param("min_range", min_range_, 3.0);  

  // Maximum range in meters
  nh_.param("max_range", max_range_, 75.0);

  // Save results
  nh_.param("save_results", save_results_, false);

  // Results directory
  nh_.param<std::string>("save_results_dir", results_dir_, "~/");
  ROS_INFO("Results directory: %s", results_dir_.c_str());

  // Fixed frame
  nh_.param<std::string>("fixed_frame", fixed_frame_, "odom");
  ROS_INFO("Fixed frame: %s", fixed_frame_.c_str());

  // Base frame
  nh_.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("Fixed frame: %s", base_frame_.c_str());

  // Number of previous frames to create a local map
  int pframes = 5;
  nh_.param("prev_frames", pframes, 5);  
  ROS_INFO("Previous frames: %i", pframes);
  prev_frames_ = (size_t)pframes;
  lmap_manager.setMaxFrames(prev_frames_);

  // Publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
}

void LaserOdometer::operator()(std::atomic<bool>& running) {
  
  while(running) {   

    PointCloud::Ptr feats(new PointCloud);
    std_msgs::Header feat_header;
    
    if (sdata->popFeatures(feats, feat_header)) {
      if (!init_) {
        lmap_manager.addPointCloud(feats);
        init_ = true;

        // Register stats
        if (save_results_) {
          stats->addPose(odom_.matrix());
        }
      } else {

        auto start_t = Clock::now();

        // Computing local map
        PointCloud::Ptr local_map(new PointCloud);
        computeLocalMap(local_map);

        // Creating a tree to search for correspondences
        pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
        tree->setInputCloud(local_map);

        // Predict the current pose
        Eigen::Isometry3d pred_odom = odom_ * (prev_odom_.inverse() * odom_);
        prev_odom_ = odom_;
        odom_ = pred_odom;
        // q_curr = Eigen::Quaterniond(odom_.rotation());
        // t_curr = odom_.translation();

        // Updating the initial guess
        Eigen::Quaterniond q_curr(odom_.rotation());
        param_q[0] = q_curr.x();
        param_q[1] = q_curr.y();
        param_q[2] = q_curr.z();
        param_q[3] = q_curr.w();

        Eigen::Vector3d t_curr = odom_.translation();
        param_t[0] = t_curr.x();
        param_t[1] = t_curr.y();
        param_t[2] = t_curr.z();

        // Optimize the current pose
        for (int optim_it = 0; optim_it < 2; optim_it++) {
          
          // Define the optimization problem
          ceres::LossFunction* loss_function = new ceres::HuberLoss(0.2);
          ceres::LocalParameterization* q_parameterization = new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options problem_options;
          ceres::Problem problem(problem_options);
          problem.AddParameterBlock(param_q, 4, q_parameterization);
          problem.AddParameterBlock(param_t, 3);

          // Adding constraints
          addEdgeConstraints(feats, local_map, tree, odom_, &problem, loss_function);

          // Solving the optimization problem
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          options.max_num_iterations = 4;
          options.minimizer_progress_to_stdout = false;
          options.num_threads = sysconf( _SC_NPROCESSORS_ONLN );          
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          
          // std::cout << summary.BriefReport() << "\n";

          odom_ = Eigen::Isometry3d::Identity();
          // odom_.linear() = q_curr.toRotationMatrix();
          // odom_.translation() = t_curr;
          Eigen::Quaterniond q_new(param_q[3], param_q[0], param_q[1], param_q[2]);
          odom_.linear() = q_new.toRotationMatrix();
          odom_.translation() = Eigen::Vector3d(param_t[0], param_t[1], param_t[2]);
        }        

        // Compute the position of the detectd edges according to the final estimate position
        PointCloud::Ptr edges_map(new PointCloud);
        pcl::transformPointCloud(*feats, *edges_map, odom_.matrix());

        // Save edges and update odometry window
        lmap_manager.addPointCloud(edges_map);

        auto end_t = Clock::now();

        // Register stats
        if (save_results_) {
          stats->addPose(odom_.matrix());
          stats->addLaserOdometryTime(start_t, end_t);
        }

        // Publishing odometry
        Eigen::Quaterniond q_current(odom_.rotation());
        Eigen::Vector3d t_current = odom_.translation();

        nav_msgs::Odometry laser_odom_msg;
        laser_odom_msg.header.frame_id = fixed_frame_;
        laser_odom_msg.child_frame_id = base_frame_;
        laser_odom_msg.header.stamp = feat_header.stamp;
        laser_odom_msg.pose.pose.orientation.x = q_current.x();
        laser_odom_msg.pose.pose.orientation.y = q_current.y();
        laser_odom_msg.pose.pose.orientation.z = q_current.z();
        laser_odom_msg.pose.pose.orientation.w = q_current.w();
        laser_odom_msg.pose.pose.position.x = t_current.x();
        laser_odom_msg.pose.pose.position.y = t_current.y();
        laser_odom_msg.pose.pose.position.z = t_current.z();
        odom_pub_.publish(laser_odom_msg);

        //Publishing TF
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
        tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
        transform.setRotation(q);
        t_br_.sendTransform(tf::StampedTransform(transform, feat_header.stamp, fixed_frame_, base_frame_));
      }
    } 

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LaserOdometer::computeLocalMap(PointCloud::Ptr& local_map) {
  
  PointCloud::Ptr total_points;
  size_t nframes = lmap_manager.getLocalMap(total_points);

  ROS_DEBUG("Total points: %lu", total_points->size());

  if (nframes == prev_frames_) {
      // Voxelize the points
      pcl::VoxelGrid<Point> voxel_filter;
      voxel_filter.setLeafSize(0.15, 0.15, 0.15);
      voxel_filter.setInputCloud(total_points);
      voxel_filter.filter(*local_map);
    } else {
      local_map = total_points;
    }

    ROS_DEBUG("Local Map points: %lu", local_map->size());
}

void LaserOdometer::addEdgeConstraints(const PointCloud::Ptr& edges,
                        const PointCloud::Ptr& local_map,
                        const pcl::KdTreeFLANN<Point>::Ptr& tree,
                        const Eigen::Isometry3d& pose,
                        ceres::Problem* problem,
                        ceres::LossFunction* loss) {
  // Translate edges
  PointCloud::Ptr edges_map(new PointCloud);
  pcl::transformPointCloud(*edges, *edges_map, pose.matrix());

  int correct_matches = 0;

  for (size_t i = 0; i < edges_map->points.size(); i++) {
    std::vector<int> indices;
    std::vector<float> sq_dist;
    tree->nearestKSearch(edges_map->points[i], 5, indices, sq_dist);
    if (sq_dist[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(local_map->points[indices[j]].x,
                            local_map->points[indices[j]].y,
                            local_map->points[indices[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
      
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        // Set a correct match
        correct_matches++;
        Eigen::Vector3d curr_point(edges->points[i].x,
                                   edges->points[i].y,
                                   edges->points[i].z);

        Eigen::Vector3d pt_a(local_map->points[indices[0]].x,
                             local_map->points[indices[0]].y,
                             local_map->points[indices[0]].z);
      
        Eigen::Vector3d pt_b(local_map->points[indices[1]].x,
                             local_map->points[indices[1]].y,
                             local_map->points[indices[1]].z);

        ceres::CostFunction* cost_function = Point2LineFactor::create(curr_point, pt_a, pt_b, min_range_, max_range_);
        problem->AddResidualBlock(cost_function, loss, param_q, param_t);
      }
    }
  }

  ROS_DEBUG("Correct matchings: %i", correct_matches);
}

}  // namespace liodom
