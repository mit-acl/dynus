
/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <dynus/initial_guess.hpp>
#include <limits>
#include <queue>
#include <iostream>

#include "initial_guess.hpp"
#include <limits>
#include <queue>
#include <iostream>

InitialGuessGenerator::InitialGuessGenerator(const std::vector<LinearConstraint3D>& polytopes,
                                             const state& initial_state,
                                             const state& final_state,
                                             double h_weight,
                                             const Eigen::Vector3d& max_velocity,
                                             const Eigen::Vector3d& max_acceleration,
                                             double control_point_resolution)
    : polytopes_(polytopes),
      h_weight_(h_weight),
      dist_to_nearest_obstacle_weight_(dist_to_nearest_obstacle_weight),
      trajs_(trajs),
      current_time_(current_time),
      max_velocity_(max_velocity),
      max_acceleration_(max_acceleration),
      control_point_resolution_(control_point_resolution) 
      
{

    // Set up q0, q1, q2, q_final_m_2, q_final_m_1, q_final
    q0_ = initial_state.pos;
    q1_ = q0_ + initial_state.vel / 3.0;
    q2_ = q0_ + initial_state.vel * 2.0 / 3.0 + initial_state.accel / 6.0;
    q_final_m_2_ = final_state.pos - final_state.vel * 2.0 / 3.0 + final_state.accel / 6.0;
    q_final_m_1_ = final_state.pos - final_state.vel / 3.0;
    q_final_ = final_state.pos;
      
}

double InitialGuessGenerator::computeCost(const Eigen::Vector3d& point) 
{

    // distance from initial point
    double g = (point - q0_).norm();

    // distance to final point
    double h = (q_final_ - point).norm();

    // distance to nearest obstacle
    double dist_to_nearest_obstacle = std::numeric_limits<double>::max();
    for (const auto& traj : trajs_)
    {
        double dist = (traj.pwp.eval(current_time_) - point).norm();
        if (dist < dist_to_nearest_obstacle)
        {
            dist_to_nearest_obstacle = dist;
        }
    }

    return g + h_weight_ * h + dist_to_nearest_obstacle_weight_ * dist_to_nearest_obstacle;

}


void InitialGuessGenerator::setMaxValuesAndSamples(Eigen::Vector3d& v_max, Eigen::Vector3d& a_max, int num_samples_x,
                                           int num_samples_y, int num_samples_z, double fraction_voxel_size)
{
  all_combinations_.clear();
  indexes_samples_x_.clear();
  indexes_samples_y_.clear();
  indexes_samples_z_.clear();

  v_max_ = v_max;
  a_max_ = a_max;

  // check if num samples are correct
  // std::cout << "num sample x is: " << num_samples_x << std::endl;
  // std::cout << "num sample y is: " << num_samples_y << std::endl;
  // std::cout << "num sample z is: " << num_samples_z << std::endl;

  // ensure they are odd numbers (so that vx=0 is included in the samples)
  num_samples_x_ = (num_samples_x % 2 == 0) ? ceil(num_samples_x) : num_samples_x;
  num_samples_y_ = (num_samples_y % 2 == 0) ? ceil(num_samples_y) : num_samples_y;
  num_samples_z_ = (num_samples_z % 2 == 0) ? ceil(num_samples_z) : num_samples_z;

  for (int i = 0; i < num_samples_x; i++)
  {
    indexes_samples_x_.push_back(i);
  }

  for (int i = 0; i < num_samples_y; i++)
  {
    indexes_samples_y_.push_back(i);
  }

  for (int i = 0; i < num_samples_z; i++)
  {
    indexes_samples_z_.push_back(i);
  }

  for (int jx : indexes_samples_x_)
  {
    for (int jy : indexes_samples_y_)
    {
      for (int jz : indexes_samples_z_)
      {
        // std::cout << "Pushing combination " << jx << ", " << jy << ", " << jz << std::endl;
        all_combinations_.push_back(std::tuple<int, int, int>(jx, jy, jz));
      }
    }
  }

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  shuffle(all_combinations_.begin(), all_combinations_.end(), std::default_random_engine(seed));

  double min_voxel_size;
  double max_voxel_size;
  computeLimitsVoxelSize(min_voxel_size, max_voxel_size);

  // Ensure  fraction_voxel_size is in [0,1]
  fraction_voxel_size = (fraction_voxel_size > 1) ? 1 : fraction_voxel_size;
  fraction_voxel_size = (fraction_voxel_size < 0) ? 0 : fraction_voxel_size;

  voxel_size_ = min_voxel_size + fraction_voxel_size * (max_voxel_size - min_voxel_size);

  orig_ = q2_ - Eigen::Vector3d(bbox_x_ / 2.0, bbox_y_ / 2.0, bbox_z_ / 2.0);
}

bool InitialGuessGenerator::checkIfPointValid(const Node& current_node) 
{

    int polytope_idx = floor(current_node.index / 4.0); // 4 control points per polytope

    // Check for polytope constraint: Ax <= b
    Eigen::MatrixXd A = polytopes_[polytope_idx].A();
    auto bb = polytopes_[polytope_idx].b();

    for (int i = 0; i < bb.rows(); i++)
    {
        if (!(A.row(i) * current_node.position <= bb[i]))
        {
            return false;
        }
    }

    if ((current_node).position.x() > x_max_ || (current_node).position.x() < x_min_ ||  // Outside the limits
        (current_node).position.y() > y_max_ || (current_node).position.y() < y_min_ || // Outside the limits
        (current_node).position.z() > z_max_ || (current_node).position.z() < z_min_ || // Outside the limits
        ((current_node).position - q0_).norm() >= Ra_)// Outside the radius
    {
        return false;
    }

    return true;

}
        

bool InitialGuessGenerator::generateInitialGuess() 
{
    // Initialization
    std::priority_queue<Node> search_queue; // Min-heap based on cost
    map_open_list_.clear();
    Node node_q2(q2_, 2, computeCost(q2_), nullptr); // Very first node
    search_queue.push(node_q2);
    Node* current_node_ptr = &node_q2;
    closest_dist_so_far_ = std::numeric_limits<double>::max();
    closest_result_so_far_ptr_ = nullptr;

    // Search
    while (!search_queue.empty()) 
    {

        // Get the current node
        *current_node_ptr = search_queue.top();
        search_queue.pop();

        // For goal check
        double dist = ((*current_ptr).position - goal_).norm();
        if (closest_result_so_far_ptr_ == nullptr)
        {
            closest_dist_so_far_ = dist;
            closest_result_so_far_ptr_ = current_ptr;
        }

        // Check if the current node has already been expanded
        unsigned int ix, iy, iz;
        ix = round(((*current_ptr).position.x() - orig_.x()) / voxel_size_);
        iy = round(((*current_ptr).position.y() - orig_.y()) / voxel_size_);
        iz = round(((*current_ptr).position.z() - orig_.z()) / voxel_size_);
        auto ptr_to_voxel = map_open_list_.find(Eigen::Vector3i(ix, iy, iz));
        bool already_exist = (ptr_to_voxel != map_open_list_.end());
        if (already_exist || checkIfPointValid(*current_ptr)) continue;














        // Check if the current node is a control point
        if (current_node.index == 2) 
        {
            control_points_.push_back(current_node.position);
            if (control_points_.size() == numControlPoints_) 
            {
                return true;
            }
        }

        // Expand the current node
        expandNode(current_node, search_queue);
    }


    const Polyhedron3D& polytope = polytopes_[polytope_idx];


    Eigen::Vector3d initial_point = (polytope.vertices.rowwise().mean());
    search_queue.push({initial_point, 0.0, nullptr});

    while (!search_queue.empty()) {
        Node current = search_queue.top();
        search_queue.pop();

        if (isFeasibleControlPoint(current.position, polytope_idx)) {
            control_points.push_back(current.position);
            if (control_points.size() == 4) {
                return true;
            }
        }

        for (const Eigen::Vector3d& neighbor : generateNeighbors(current.position)) {
            if (isFeasibleControlPoint(neighbor, polytope_idx)) {
                double cost = current.cost + (neighbor - current.position).norm();
                search_queue.push({neighbor, cost, std::make_shared<Node>(current)});
            }
        }
    }

    return false;
}

std::vector<Eigen::Vector3d> InitialGuessGenerator::generateNeighbors(const Eigen::Vector3d& point) {
    std::vector<Eigen::Vector3d> neighbors;
    for (double dx = -control_point_resolution_; dx <= control_point_resolution_; dx += control_point_resolution_) {
        for (double dy = -control_point_resolution_; dy <= control_point_resolution_; dy += control_point_resolution_) {
            for (double dz = -control_point_resolution_; dz <= control_point_resolution_; dz += control_point_resolution_) {
                if (dx == 0.0 && dy == 0.0 && dz == 0.0) continue;
                neighbors.emplace_back(point + Eigen::Vector3d(dx, dy, dz));
            }
        }
    }
    return neighbors;
}

bool InitialGuessGenerator::isFeasibleControlPoint(const Eigen::Vector3d& point, size_t polytope_idx) const {
    return isPointInPolytope(point, polytopes_[polytope_idx]);
}

bool InitialGuessGenerator::isPointInPolytope(const Eigen::Vector3d& point, const Polyhedron3D& polytope) const {
    for (size_t i = 0; i < polytope.normals.rows(); ++i) {
        if (polytope.normals.row(i).dot(point) > polytope.offsets[i]) {
            return false;
        }
    }
    return true;
}

bool InitialGuessGenerator::validateContinuity(const std::vector<std::vector<Eigen::Vector3d>>& control_points) const {
    for (size_t i = 1; i < control_points.size(); ++i) {
        if ((control_points[i][0] - control_points[i - 1].back()).norm() > control_point_resolution_) {
            return false;
        }
    }
    return true;
}

bool InitialGuessGenerator::validateDynamicConstraints(const std::vector<std::vector<Eigen::Vector3d>>& control_points) const {
    for (const auto& segment : control_points) {
        for (size_t i = 1; i < segment.size(); ++i) {
            Eigen::Vector3d velocity = segment[i] - segment[i - 1];
            if (velocity.cwiseAbs().maxCoeff() > max_velocity_.maxCoeff()) {
                return false;
            }
            if (i > 1) {
                Eigen::Vector3d acceleration = velocity - (segment[i - 1] - segment[i - 2]);
                if (acceleration.cwiseAbs().maxCoeff() > max_acceleration_.maxCoeff()) {
                    return false;
                }
            }
        }
    }
    return true;
}
