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
  init_(false),
  prev_odom_(Eigen::Isometry3d::Identity()),
  odom_(Eigen::Isometry3d::Identity()),
  prev_stamp_(0.0),
  sdata(SharedData::getInstance()),
  stats(Stats::getInstance()),
  params(Params::getInstance()),
  lmap_manager(params->local_map_size_),
  num_freqs_(0) {

  for (int i = 0; i < 5; i++) {
    in_freqs_[i] = 20.0; // 100/5
    out_freqs_[i] = 20.0; // 100/5
  }
  mean_in_freq_ = 100.0; // high freq.
  mean_out_freq_ = 100.0; // high freq.
  last_in_time_secs_ = ros::Time::now().toSec();
  last_out_time_secs_ = last_in_time_secs_;

  // Publishers
  pc_edges_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("edges", 10);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist", 10);
}

LaserOdometer::~LaserOdometer() {
}

void LaserOdometer::operator()(std::atomic<bool>& running) {
  
  while(running) {   

    PointCloud::Ptr feats(new PointCloud);
    std_msgs::Header feat_header;
    
    if (sdata->popFeatures(feats, feat_header)) {    
      if (!init_) {
      
        auto start_t = Clock::now();

        // cache the static tf from base to laser
        if (params->laser_frame_ == "") {
          params->laser_frame_ = feat_header.frame_id;
        }

        if (!getBaseToLaserTf(params->laser_frame_))
        {
          ROS_WARN("Skipping point_cloud");
          return;
        }

        lmap_manager.addPointCloud(feats);
        init_ = true;
        prev_stamp_ = feat_header.stamp.toSec();
        
        auto end_t = Clock::now();

        // Register stats
        if (params->save_results_) {
          stats->addPose(odom_.matrix());
          stats->addLaserOdometryTime(start_t, end_t);
          stats->stopFrame(end_t);          
        }

      } else {

        auto start_t = Clock::now();

        // Computing local map
        PointCloud::Ptr local_map_rec(new PointCloud);
        PointCloud::Ptr local_map_gen(new PointCloud);
        computeLocalMap(local_map_gen, local_map_rec);        

        // Predict the current pose
        Eigen::Isometry3d pred_odom = odom_ * (prev_odom_.inverse() * odom_);
        prev_odom_ = odom_;
        odom_ = pred_odom;

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
          addEdgeConstraints(feats, local_map_gen, local_map_rec, odom_, &problem, loss_function);

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

        // Measuring the processing frequency
        // ros::Time end = ros::Time::now();
        // ros::Duration diff = end - feat_header.stamp;
        // auto hz = 1.0 / diff.toSec();
        // ROS_DEBUG("Processing frequency: %f", hz);

        // if (hz < 15.0) {
        //   ROS_WARN("Processing frequency lower than expected: %f", hz);
        // }
        double now_secs = ros::Time::now().toSec();
        mean_in_freq_ -= in_freqs_[num_freqs_];
        in_freqs_[num_freqs_] = (1.0/(feat_header.stamp.toSec() - last_in_time_secs_))/5.0;
        mean_in_freq_ += in_freqs_[num_freqs_];
        last_in_time_secs_ = feat_header.stamp.toSec();

        mean_out_freq_ -= out_freqs_[num_freqs_];
        out_freqs_[num_freqs_] = (1.0/(now_secs - last_out_time_secs_))/5.0;
        mean_out_freq_ += out_freqs_[num_freqs_];
        last_out_time_secs_ = now_secs;

        num_freqs_ ++;
        num_freqs_ = num_freqs_ % 5;

        ROS_DEBUG("Output frequency: %2.2f", mean_out_freq_);
        if (mean_out_freq_ < mean_in_freq_ * 0.8) {
          ROS_WARN("Output frequency too low: %2.2f << %2.2f", mean_out_freq_, mean_in_freq_);
        }

        // Register stats
        if (params->save_results_) {
          stats->addPose(odom_.matrix());
          stats->addLaserOdometryTime(start_t, end_t);
          stats->stopFrame(end_t);
        }

        // Transform to base_link frame before publication
        Eigen::Isometry3d odom_base_link = odom_ * laser_to_base_;
        Eigen::Quaterniond q_current(odom_base_link.rotation());
        Eigen::Vector3d t_current = odom_base_link.translation();

        // Publishing odometry
        nav_msgs::Odometry laser_odom_msg;
        laser_odom_msg.header.frame_id = params->fixed_frame_;
        laser_odom_msg.child_frame_id = params->base_frame_;
        laser_odom_msg.header.stamp = feat_header.stamp;
        //Filling pose
        laser_odom_msg.pose.pose.orientation.x = q_current.x();
        laser_odom_msg.pose.pose.orientation.y = q_current.y();
        laser_odom_msg.pose.pose.orientation.z = q_current.z();
        laser_odom_msg.pose.pose.orientation.w = q_current.w();
        laser_odom_msg.pose.pose.position.x = t_current.x();
        laser_odom_msg.pose.pose.position.y = t_current.y();
        laser_odom_msg.pose.pose.position.z = t_current.z();
        //Filling twist
        double delta_time = feat_header.stamp.toSec() - prev_stamp_;
        Eigen::Isometry3d delta_odom = ((prev_odom_ * laser_to_base_).inverse() * odom_base_link);
        Eigen::Vector3d t_delta = delta_odom.translation();
        laser_odom_msg.twist.twist.linear.x = t_delta.x() / delta_time;
        laser_odom_msg.twist.twist.linear.y = t_delta.y() / delta_time;
        laser_odom_msg.twist.twist.linear.z = t_delta.z() / delta_time;
        Eigen::Quaterniond q_delta(delta_odom.rotation());
        // we use tf because euler angles in Eigen present singularity problems
        tf::Quaternion quat(q_delta.x(), q_delta.y(), q_delta.z(), q_delta.w());
        tf::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        laser_odom_msg.twist.twist.angular.x = roll / delta_time;
        laser_odom_msg.twist.twist.angular.y = pitch / delta_time;
        laser_odom_msg.twist.twist.angular.z = yaw / delta_time;
        prev_stamp_ = feat_header.stamp.toSec();
        odom_pub_.publish(laser_odom_msg);

        // Publishing twist
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.frame_id = params->base_frame_;
        twist_msg.header.stamp = feat_header.stamp;
        twist_msg.twist = laser_odom_msg.twist.twist;
        twist_pub_.publish(twist_msg);

        // Publishing edges
        // Publishing edges if there is someone listening
        if (pc_edges_pub_.getNumSubscribers() > 0) {
          sensor_msgs::PointCloud2 edges_msg;
          pcl::toROSMsg(*feats, edges_msg);
          edges_msg.header = feat_header;
          pc_edges_pub_.publish(edges_msg);
        }

        //Publishing TF
        if (params->publish_tf_) {
          tf::Transform transform;
          transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
          tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
          transform.setRotation(q);
          tf_broadcaster_.sendTransform(tf::StampedTransform(transform, feat_header.stamp, params->fixed_frame_, params->base_frame_));
        }
      }
    } 

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void LaserOdometer::computeLocalMap(PointCloud::Ptr& local_map_gen, PointCloud::Ptr& local_map_rec) {
  
  PointCloud::Ptr rec_local_map_(new PointCloud);
  sdata->getLocalMap(rec_local_map_);
  local_map_rec = rec_local_map_;
  ROS_DEBUG("Local Map points - Received: %lu", rec_local_map_->size());

  PointCloud::Ptr total_points;
  size_t nframes = lmap_manager.getLocalMap(total_points);
  // ROS_DEBUG("Total points: %lu", total_points->size());

  PointCloud::Ptr gen_local_map_(new PointCloud);
  if (nframes == params->local_map_size_ && !params->mapping_) {
    // Voxelize the points
    pcl::VoxelGrid<Point> voxel_filter;
    // voxel_filter.setLeafSize(0.15, 0.15, 0.15);
    voxel_filter.setLeafSize(0.4, 0.4, 0.4);
    voxel_filter.setInputCloud(total_points);
    voxel_filter.filter(*gen_local_map_);
  } else {
    gen_local_map_ = total_points;
  }
  local_map_gen = gen_local_map_;
  ROS_DEBUG("Local Map points - Gen: %lu", gen_local_map_->size());  
}

void LaserOdometer::addEdgeConstraints(const PointCloud::Ptr& edges,
                        const PointCloud::Ptr& local_map_gen,
                        const PointCloud::Ptr& local_map_rec,
                        const Eigen::Isometry3d& pose,
                        ceres::Problem* problem,
                        ceres::LossFunction* loss) {
  // Translate edges
  PointCloud::Ptr edges_map(new PointCloud);
  pcl::transformPointCloud(*edges, *edges_map, pose.matrix());  

  PointCloud::Ptr local_map = local_map_rec;  
  *local_map += *local_map_gen;

  pcl::VoxelGrid<Point> voxel_filter;
  voxel_filter.setLeafSize(0.4, 0.4, 0.4);
    // voxel_filter.setLeafSize(0.4, 0.4, 0.4);
  voxel_filter.setInputCloud(local_map);
  voxel_filter.filter(*local_map);
  
  // Trying to match against the received local map
  int correct_matches = 0;
  pcl::KdTreeFLANN<Point>::Ptr tree(new pcl::KdTreeFLANN<Point>);
  tree->setInputCloud(local_map);
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

        ceres::CostFunction* cost_function = Point2LineFactor::create(curr_point, pt_a, pt_b, params->min_range_, params->max_range_);
        problem->AddResidualBlock(cost_function, loss, param_q, param_t);
      }
    }
  }

  ROS_DEBUG("Correct matchings: %i", correct_matches);
}

bool LaserOdometer::getBaseToLaserTf (const std::string& frame_id) {

  tf::TransformListener tf_listener;
  tf::StampedTransform laser_to_base_tf;

  try  {
    tf_listener.waitForTransform(
      frame_id, params->base_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform (
      frame_id, params->base_frame_, ros::Time(0), laser_to_base_tf);
  } catch (tf::TransformException &ex) {
    ROS_WARN("Could not get initial transform from base to laser frame, %s", ex.what());
    return false;
  }

  laser_to_base_.translation() = Eigen::Vector3d(laser_to_base_tf.getOrigin().x(),
                                                 laser_to_base_tf.getOrigin().y(),
                                                 laser_to_base_tf.getOrigin().z());
  Eigen::Quaterniond q_aux(laser_to_base_tf.getRotation().w(),
                           laser_to_base_tf.getRotation().x(),
                           laser_to_base_tf.getRotation().y(),
                           laser_to_base_tf.getRotation().z());
  laser_to_base_.linear() = q_aux.toRotationMatrix();

  return true;
}

}  // namespace liodom
