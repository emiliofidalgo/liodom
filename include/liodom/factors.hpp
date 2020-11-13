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

#ifndef INCLUDE_LIODOM_FACTORS_HPP
#define INCLUDE_LIODOM_FACTORS_HPP

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace liodom {

struct Point2PointFactor {

  Point2PointFactor(Eigen::Vector3d curr_point, Eigen::Vector3d map_point) :
    curr_point_(curr_point), map_point_(map_point) {};

  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {

    Eigen::Matrix<T, 3, 1> cp{T(curr_point_.x()), T(curr_point_.y()), T(curr_point_.z())};
    Eigen::Matrix<T, 3, 1> mp{T(map_point_.x()), T(map_point_.y()), T(map_point_.z())};

		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};		
		Eigen::Matrix<T, 3, 1> t_last_curr{t[0], t[1], t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

    Eigen::Matrix<T, 3, 1> error = lp - mp;

		residual[0] = T(error.x());
		residual[1] = T(error.y());
		residual[2] = T(error.z());
    
    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3d curr_point, const Eigen::Vector3d map_point) {
    return new ceres::AutoDiffCostFunction<Point2PointFactor, 3, 4, 3>(new Point2PointFactor(curr_point, map_point));
	}
  
  Eigen::Vector3d curr_point_;
  Eigen::Vector3d map_point_;
};


struct Point2LineFactor {

	Point2LineFactor(Eigen::Vector3d curr_point, Eigen::Vector3d map_point_a, Eigen::Vector3d map_point_b) :
    curr_point_(curr_point), map_point_a_(map_point_a), map_point_b_(map_point_b) {};

	template <typename T>
	bool operator()(const T* q, const T* t, T* residual) const {
    
		Eigen::Matrix<T, 3, 1> cp{T(curr_point_.x()), T(curr_point_.y()), T(curr_point_.z())};
		Eigen::Matrix<T, 3, 1> lpa{T(map_point_a_.x()), T(map_point_a_.y()), T(map_point_a_.z())};
		Eigen::Matrix<T, 3, 1> lpb{T(map_point_b_.x()), T(map_point_b_.y()), T(map_point_b_.z())};
		
		Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
		Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
		q_last_curr = q_identity.slerp(T(1), q_last_curr);
		Eigen::Matrix<T, 3, 1> t_last_curr{T(1) * t[0], T(1) * t[1], T(1) * t[2]};

		Eigen::Matrix<T, 3, 1> lp;
		lp = q_last_curr * cp + t_last_curr;

		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
		residual[2] = nu.z() / de.norm();

		return true;
	}

	static ceres::CostFunction* create(const Eigen::Vector3d curr_point, 
                                     const Eigen::Vector3d map_point_a,
                                     const Eigen::Vector3d map_point_b) {
		return new ceres::AutoDiffCostFunction<Point2LineFactor, 3, 4, 3>(new Point2LineFactor(curr_point, map_point_a, map_point_b));
	}

	Eigen::Vector3d curr_point_;
  Eigen::Vector3d map_point_a_;
  Eigen::Vector3d map_point_b_;
};

}  // namespace liodom

#endif // INCLUDE_LIODOM_FACTORS_HPP