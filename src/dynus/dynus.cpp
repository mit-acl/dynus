/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "dynus/dynus.hpp"

using namespace dynus;
using namespace termcolor;

typedef timer::Timer MyTimer;

// ----------------------------------------------------------------------------

/**
 * @brief Constructor for DYNUS.
 * @param parameters par: Input configuration parameters.
 */
DYNUS::DYNUS(parameters par) : par_(par)
{

  // Set up dgp_manager
  dgp_manager_.setParameters(par_);

  // Compute factors_ for time allocation
  if (par_.use_dynamic_factor)
  {
    // Dynamic factor search
    num_dynamic_factors_ = static_cast<int>((2 * par_.dynamic_factor_k_radius) / par_.factor_constant_step_size) + 1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++)
    {
      double factor = par_.dynamic_factor_initial_mean - par_.dynamic_factor_k_radius + i * par_.factor_constant_step_size;
      if (factor >= 1.0)
        factors_.push_back(factor);
    }
  }
  else
  {
    // Constant factor search
    num_dynamic_factors_ = static_cast<int>((par_.factor_final - par_.factor_initial) / par_.factor_constant_step_size) + 1;
    factors_.reserve(num_dynamic_factors_);
    for (int i = 0; i < num_dynamic_factors_; i++)
    {
      double factor = par_.factor_initial + i * par_.factor_constant_step_size;
      factors_.push_back(factor);
    }
  }

  // Set up unconstrained optimization solver for whole trajectory
  whole_traj_solver_ptrs_.reserve(num_dynamic_factors_);
  for (int i = 0; i < num_dynamic_factors_; i++)
  {
    whole_traj_solver_ptrs_.push_back(std::make_shared<SolverGurobi>());
    whole_traj_solver_ptrs_[i]->initializeSolver(par_);
  }

  // Set up decomp ellip workers for each thread
  ellip_workers_.resize(whole_traj_solver_ptrs_.size());

  // Pre-compute the worst initial_dt * par_.num_N (this is the worst case time allocated for the whole trajectory)
  auto tmp_traj_solver_ptr = std::make_shared<SolverGurobi>();
  tmp_traj_solver_ptr->initializeSolver(par_);
  tmp_traj_solver_ptr->resetToNominalState();
  state tmp_start_state, tmp_end_state;
  tmp_end_state.setPos(par_.num_P * par_.max_dist_vertexes, 0.0, 0.0);
  tmp_traj_solver_ptr->setX0(tmp_start_state);
  tmp_traj_solver_ptr->setXf(tmp_end_state);
  worst_traj_time_ = tmp_traj_solver_ptr->getInitialDt() * par_.num_N;

  std::cout << bold << green << "[DYNUS] Worst case trajectory time for pre-computation: " << worst_traj_time_ << " [s]" << reset << std::endl;

  // Set up basis converter
  BasisConverter basis_converter;
  A_rest_pos_basis_ = basis_converter.getArestMinvo(); // Use Minvo basis
  A_rest_pos_basis_inverse_ = A_rest_pos_basis_.inverse();

  // Parameters
  v_max_3d_ = Eigen::Vector3d(par_.v_max, par_.v_max, par_.v_max);
  v_max_ = par_.v_max;
  a_max_3d_ = Eigen::Vector3d(par_.a_max, par_.a_max, par_.a_max);
  j_max_3d_ = Eigen::Vector3d(par_.j_max, par_.j_max, par_.j_max);

  // Initialize the state
  changeDroneStatus(DroneStatus::GOAL_REACHED);

  // Initialize the map size
  wdx_ = par_.initial_wdx;
  wdy_ = par_.initial_wdy;
  wdz_ = par_.initial_wdz;

  // Map resolution
  map_res_ = par_.res;
}

// ----------------------------------------------------------------------------

/**
 * @brief Starts adaptive k-value.
 */
void DYNUS::startAdaptKValue()
{

  // Compute the average computation time
  const size_t num_samples = store_computation_times_.size();
  for (const auto &comp_time : store_computation_times_)
  {
    est_comp_time_ += comp_time;
  }
  est_comp_time_ = est_comp_time_ / num_samples;

  // Start k_value adaptation
  use_adapt_k_value_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the subgoal.
 * @param const state &A: starting state.
 * @param const state &G_term: goal state.
 * @return bool
 */
void DYNUS::computeG(const state &A, const state &G_term, double horizon)
{
  // Initialize the result
  state local_G;

  // Compute pos for G
  local_G.pos = dynus_utils::projectPointToSphere(A.pos, G_term.pos, horizon);

  // Compute yaw for G
  Eigen::Vector3d dir = (G_term.pos - local_G.pos).normalized();
  local_G.yaw = atan2(dir[1], dir[0]);

  // Set G
  setG(local_G);
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if we need to replan.
 * @return bool
 */

bool DYNUS::needReplan(const state &local_state, const state &local_G_term, const state &last_plan_state)
{

  // Compute the distance to the terminal goal
  double dist_to_term_G = (local_state.pos - local_G_term.pos).norm();
  double dist_from_last_plan_state_to_term_G = (last_plan_state.pos - local_G_term.pos).norm();

  if (dist_to_term_G < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    return false;
  }

  if (dist_to_term_G < par_.goal_seen_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN); // This triggers to use the hard final state constraint
  }

  if (drone_status_ == DroneStatus::GOAL_SEEN && dist_from_last_plan_state_to_term_G < par_.goal_radius)
  {
    return false;
  }

  // Don't plan if drone is not traveling
  if (drone_status_ == DroneStatus::GOAL_REACHED || (drone_status_ == DroneStatus::YAWING))
    return false;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::findAandAtime(state &A, double &A_time, double current_time, double last_replaning_computation_time)
{

  mtx_plan_.lock();
  int plan_size = plan_.size();
  mtx_plan_.unlock();

  if (plan_size == 0)
  {
    std::cout << bold << red << "plan_size == 0" << reset << std::endl;
    return false;
  }

  if (par_.use_state_update)
  {
    // Change k_value dynamically
    // To get stable results, we will use a default value of k_value until we have enough computation time
    if (!use_adapt_k_value_)
    {
      // Use default k_value
      k_value_ = std::max((int)plan_size - par_.default_k_value, 0);

      // Store computation times
      if (num_replanning_ != 1) // Don't store the very first computation time (because we don't have a previous computation time)
        store_computation_times_.push_back(last_replaning_computation_time);
    }
    else
    {

      // Computation time filtering
      est_comp_time_ = par_.alpha_k_value_filtering * last_replaning_computation_time + (1 - par_.alpha_k_value_filtering) * est_comp_time_;

      // Get state number based on est_comp_time_ and dc
      k_value_ = std::max((int)plan_size - (int)(par_.k_value_factor * est_comp_time_ / par_.dc), 0);
    }

    // Check if k_value_ is valid
    if (plan_size - 1 - k_value_ < 0 || plan_size - 1 - k_value_ >= plan_size)
    {
      k_value_ = plan_size - 1; // If k_value_ is larger than the plan size, we set it to the last state
    }

    // Get A
    mtx_plan_.lock();
    A = plan_[plan_size - 1 - k_value_];
    mtx_plan_.unlock();

    // Get A_time
    A_time = current_time + (plan_size - 1 - k_value_) * par_.dc; // time to A from current_pos is (plan_size - 1 - k_value_) * par_.dc;
  }
  else // If we don't update state - this is for global planner benchmarking purposes
  {
    // Get state
    getState(A);
    A_time = current_time;
  }

  // Check if A is within the map (especially for z)
  if ((A.pos[2] < par_.z_min || A.pos[2] > par_.z_max) ||
      (A.pos[0] < par_.x_min || A.pos[0] > par_.x_max) ||
      (A.pos[1] < par_.y_min || A.pos[1] > par_.y_max))
  {
    printf("A (%f, %f, %f) is out of the map\n", A.pos[0], A.pos[1], A.pos[2]);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointOccupied(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointFree(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointFree(point);
}

// ----------------------------------------------------------------------------

void DYNUS::findSafeSubGoal(vec_Vecf<3> &global_path)
{
  // Keep the original global path
  vec_Vecf<3> original_global_path = global_path;

  // Reset goal path
  global_path.clear();

  if (original_global_path.empty())
    return;

  // Initialize it with the start point
  global_path.push_back(original_global_path[0]);

  // Kd-tree search parameters
  const int k = 1; // nearest neighbor
  std::vector<int> pointIdxNKNSearch(k);
  std::vector<float> pointNKNSquaredDistance(k);

  // Sampling parameters (TODO: make these parameters configurable)
  const double sample_dist = 0.1; // [m] distance between two samples along the trajectory

  // Inflation radius for unknown space (max extent)
  const double r_inflate = par_.obst_max_vel * traj_max_time_; // [m]
  const double thr_orig = par_.drone_radius;                   // [m]
  const double thr_infl = par_.drone_radius + r_inflate;       // [m]
  const double thr_orig2 = thr_orig * thr_orig;
  const double thr_infl2 = thr_infl * thr_infl;

  // Mutex lock (KD-tree shared)
  std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);

  // Helper: returns true if pt is within (unknown KD-tree distance) <= threshold^2.
  auto isWithinUnknown = [&](const Eigen::Vector3d &pt, double thr2) -> bool
  {
    pcl::PointXYZ searchPoint(pt(0), pt(1), pt(2));
    if (kdtree_unk_.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      return static_cast<double>(pointNKNSquaredDistance[0]) < thr2;
    }
    return false;
  };

  // Helper: backtrack from a hit location (segment i, arc-length s_hit along that segment)
  // until outside inflated unknown. Returns the backtracked safe point.
  auto backtrackToOutsideInflated = [&](int seg_i, double s_hit) -> Eigen::Vector3d
  {
    // We will walk backward in steps of sample_dist along the polyline.
    int i = seg_i;
    if (i < 0)
      i = 0;
    if (i >= static_cast<int>(original_global_path.size()) - 1)
      i = static_cast<int>(original_global_path.size()) - 2;

    Eigen::Vector3d A = original_global_path[i];
    Eigen::Vector3d B = original_global_path[i + 1];

    Eigen::Vector3d d = B - A;
    double L = d.norm();
    if (L < 1e-9)
      return A; // degenerate segment

    Eigen::Vector3d dir = d / L;

    // Clamp s to [0, L]
    double s = std::min(std::max(0.0, s_hit), L);

    // Start from the hit point
    Eigen::Vector3d pt = A + dir * s;

    // If we're already outside inflated unknown, keep it (shouldn't happen in your described flow)
    if (!isWithinUnknown(pt, thr_infl2))
      return pt;

    // Walk backward until outside inflated unknown or we reach the start.
    // This can cross segment boundaries if inflation is large.
    while (true)
    {
      // Step backward on current segment
      s -= sample_dist;

      if (s >= 0.0)
      {
        pt = A + dir * s;
      }
      else
      {
        // Need to go to previous segment
        i -= 1;
        if (i < 0)
        {
          // We reached the very beginning; return the start point (best we can do)
          return original_global_path.front();
        }

        // New segment [i, i+1]
        A = original_global_path[i];
        B = original_global_path[i + 1];
        d = B - A;
        L = d.norm();
        if (L < 1e-9)
        {
          // Skip degenerate segment
          s = 0.0;
          pt = A;
          continue;
        }
        dir = d / L;

        // We crossed into previous segment: set s at its end (B) plus leftover negative s
        // Example: if s was -0.03, we start at L - 0.03 on the previous segment.
        s = L + s; // s is negative here
        if (s < 0.0)
          s = 0.0;
        if (s > L)
          s = L;

        pt = A + dir * s;
      }

      // Check inflated condition
      if (!isWithinUnknown(pt, thr_infl2))
        return pt;
    }
  };

  // Loop through the global path and check for intersection with original unknown (NOT inflated)
  const int M = static_cast<int>(original_global_path.size());
  for (int i = 0; i < M - 1; i++)
  {
    Eigen::Vector3d current_gp = original_global_path[i];
    Eigen::Vector3d next_gp = original_global_path[i + 1];

    Eigen::Vector3d dir = next_gp - current_gp;
    double dist = dir.norm();
    if (dist < 1e-9)
    {
      // Degenerate; just continue
      continue;
    }
    dir /= dist;

    // Sample points along the line segment
    const int num_samples = static_cast<int>(dist / sample_dist);

    for (int j = 0; j <= num_samples; j++)
    {
      Eigen::Vector3d sample_point = current_gp + dir * (sample_dist * j);

      // Detect intersection with original unknown (same as before, but squared distance)
      if (isWithinUnknown(sample_point, thr_orig2))
      {
        // Found first contact with original unknown -> now backtrack until outside inflated unknown
        const double s_hit = sample_dist * j;
        Eigen::Vector3d safe_pt = backtrackToOutsideInflated(i, s_hit);

        // Ensure we don't add duplicates
        if ((safe_pt - global_path.back()).norm() > 1e-6)
          global_path.push_back(safe_pt);

        return; // Stop: this is the new last global path point
      }
    }

    // No unknown intersection on this segment; keep the next waypoint
    global_path.push_back(next_gp);
  }
}

// ----------------------------------------------------------------------------

void DYNUS::computeMapSize(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
{

  // Get local_A
  state local_A;
  getA(local_A);

  // Increase the effective buffer size based on the number of DGP failures.
  double dynamic_buffer = par_.map_buffer;

  // Increase the effective buffer size based on velocity.
  double dynamic_buffer_x = dynamic_buffer;
  double dynamic_buffer_y = dynamic_buffer;
  double dynamic_buffer_z = dynamic_buffer;

  // Compute the distance to the terminal goal for each axis.
  double dist_x = std::abs(min_pos[0] - max_pos[0]);
  double dist_y = std::abs(min_pos[1] - max_pos[1]);
  double dist_z = std::abs(min_pos[2] - max_pos[2]);

  // Update the map size based on the min and max positions.
  wdx_ = std::max(dist_x + 2 * dynamic_buffer_x, par_.min_wdx);
  wdy_ = std::max(dist_y + 2 * dynamic_buffer_y, par_.min_wdy);
  wdz_ = std::max(dist_z + 2 * dynamic_buffer_z, par_.min_wdz);

  // Compute the base map center as the midpoint between the min and max positions.
  map_center_ = (min_pos + max_pos) / 2.0;
}

// ----------------------------------------------------------------------------

bool DYNUS::checkPointWithinMap(const Eigen::Vector3d &point) const
{
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) && (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) && (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void DYNUS::getStaticPushPoints(vec_Vecf<3> &static_push_points)
{
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void DYNUS::getLocalGlobalPath(vec_Vecf<3> &local_global_path, vec_Vecf<3> &local_global_path_after_push)
{
  local_global_path = local_global_path_;
  local_global_path_after_push = local_global_path_after_push_;
}

// ----------------------------------------------------------------------------

void DYNUS::getGlobalPath(vec_Vecf<3> &global_path)
{
  mtx_global_path_.lock();
  global_path = global_path_;
  mtx_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getOriginalGlobalPath(vec_Vecf<3> &original_global_path)
{
  mtx_original_global_path_.lock();
  original_global_path = original_global_path_;
  mtx_original_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getFreeGlobalPath(vec_Vecf<3> &free_global_path)
{
  free_global_path = free_global_path_;
}

// ----------------------------------------------------------------------------

void DYNUS::resetData()
{

  final_g_ = 0.0;
  global_planning_time_ = 0.0;
  dgp_static_jps_time_ = 0.0;
  dgp_check_path_time_ = 0.0;
  dgp_dynamic_astar_time_ = 0.0;
  dgp_recover_path_time_ = 0.0;
  cvx_decomp_time_ = 0.0;
  local_traj_computation_time_ = 0.0;
  safe_paths_time_ = 0.0;
  safety_check_time_ = 0.0;
  yaw_sequence_time_ = 0.0;
  yaw_fitting_time_ = 0.0;

  poly_out_whole_.clear();
  poly_out_safe_.clear();
  goal_setpoints_.clear();
  pwp_to_share_.clear();
  optimal_yaw_sequence_.clear();
  yaw_control_points_.clear();
  yaw_knots_.clear();
  cps_.clear();
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveData(double &final_g,
                          double &global_planning_time,
                          double &dgp_static_jps_time,
                          double &dgp_check_path_time,
                          double &dgp_dynamic_astar_time,
                          double &dgp_recover_path_time,
                          double &cvx_decomp_time,
                          double &local_traj_computatoin_time,
                          double &safety_check_time,
                          double &safe_paths_time,
                          double &yaw_sequence_time,
                          double &yaw_fitting_time)
{
  final_g = final_g_;
  global_planning_time = global_planning_time_;
  dgp_static_jps_time = dgp_static_jps_time_;
  dgp_check_path_time = dgp_check_path_time_;
  dgp_dynamic_astar_time = dgp_dynamic_astar_time_;
  dgp_recover_path_time = dgp_recover_path_time_;
  cvx_decomp_time = cvx_decomp_time_;
  local_traj_computatoin_time = local_traj_computation_time_;
  safe_paths_time = safe_paths_time_;
  safety_check_time = safety_check_time_;
  yaw_sequence_time = yaw_sequence_time_;
  yaw_fitting_time = yaw_fitting_time_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrievePolytopes(vec_E<Polyhedron<3>> &poly_out_whole, vec_E<Polyhedron<3>> &poly_out_safe)
{
  poly_out_whole = poly_out_whole_;
  poly_out_safe = poly_out_safe_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveGoalSetpoints(std::vector<state> &goal_setpoints)
{
  goal_setpoints = goal_setpoints_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveListSubOptGoalSetpoints(std::vector<std::vector<state>> &list_subopt_goal_setpoints)
{
  list_subopt_goal_setpoints = list_subopt_goal_setpoints_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveCPs(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  cps = cps_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Replans the trajectory.
 * @param double last_replaning_computation_time: Last replanning computation time.
 * @param double current_time: Current timestamp.
 */
std::tuple<bool, bool> DYNUS::replan(double last_replaning_computation_time, double current_time)
{

  /* -------------------- Housekeeping -------------------- */

  MyTimer timer_housekeeping(true);

  // Reset Data
  resetData();

  // Check if we need to replan
  if (!checkReadyToReplan())
  {
    std::cout << bold << red << "Planner is not ready to replan" << reset << std::endl;
    return std::make_tuple(false, false);
  }

  // Get states we need
  state local_state, local_G_term, last_plan_state;
  getState(local_state);
  getGterm(local_G_term);
  getLastPlanState(last_plan_state);

  // Check if we need to replan based on the distance to the terminal goal
  if (!needReplan(local_state, local_G_term, last_plan_state))
    return std::make_tuple(false, false);

  if (par_.debug_verbose)
    std::cout << "Housekeeping: " << timer_housekeeping.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Global Planning -------------------- */

  MyTimer timer_global(true);
  vec_Vecf<3> global_path;
  if (!generateGlobalPath(global_path, current_time, last_replaning_computation_time))
  {
    if (par_.debug_verbose)
      std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, false);
  }
  if (par_.debug_verbose)
    std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Local Trajectory Optimization -------------------- */

  MyTimer timer_local(true);
  if (!planLocalTrajectory(global_path, last_replaning_computation_time))
  {
    if (par_.debug_verbose)
      std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Append to Plan -------------------- */

  MyTimer timer_append(true);
  if (!appendToPlan())
  {
    if (par_.debug_verbose)
      std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true);
  }
  if (par_.debug_verbose)
    std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Final Housekeeping -------------------- */

  MyTimer timer_final(true);

  if (par_.debug_verbose)
    std::cout << bold << green << "Replanning succeeded" << reset << std::endl;

  // Reset the replanning failure count
  replanning_failure_count_ = 0;
  if (par_.debug_verbose)
    std::cout << "Final Housekeeping: " << timer_final.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  return std::make_tuple(true, true);
}

// ----------------------------------------------------------------------------

bool DYNUS::generateGlobalPath(vec_Vecf<3> &global_path, double current_time, double last_replaning_computation_time)
{

  // Get G and G_term
  state local_G, local_G_term;
  getG(local_G);
  getGterm(local_G_term);

  // Declare local variables
  state local_A;
  double A_time;

  // Find A and A_time
  if (!findAandAtime(local_A, A_time, current_time, last_replaning_computation_time))
  {
    replanning_failure_count_++;
    return false;
  }

  // Set A and A_time
  setA(local_A);
  setA_time(A_time);

  // Compute G
  computeG(local_A, local_G_term, par_.horizon);

  // Update Map
  if (par_.sim_env == "fake_sim")
  {
    updateOccupancyMap(current_time);
  }
  else
  {
    updateMap(current_time);
  }

  // Set up the DGP planner (since updateVmax() needs to be called after setupDGPPlanner, we use v_max_ from the last replan)
  dgp_manager_.setupDGPPlanner(par_.global_planner, par_.global_planner_verbose, map_res_, v_max_, par_.a_max, par_.j_max, par_.dgp_timeout_duration_ms, par_.w_unknown, par_.w_align, par_.decay_len_cells, par_.w_side, par_.los_cells, par_.min_len, par_.min_turn);

  // Free start and goal if necessary
  if (par_.use_free_start)
    dgp_manager_.freeStart(local_A.pos, par_.free_start_factor);
  if (par_.use_free_goal)
    dgp_manager_.freeGoal(local_G.pos, par_.free_goal_factor);

  // Debug
  if (par_.debug_verbose)
    std::cout << "Solving DGP" << std::endl;

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
    local_G.pos[2] = 1.0;
  }

  // 1) Build a direction hint from the *previous* global path
  vec_Vecf<3> prev_global;
  getGlobalPath(prev_global); // last successful global path

  Eigen::Vector3d dir_hint = (local_G.pos - local_A.pos).normalized();
  if (prev_global.size() >= 2)
  {
    Eigen::Vector3d s0 = prev_global[0];
    Eigen::Vector3d s1 = prev_global[1];
    Eigen::Vector3d seg = s1 - s0;
    if (seg.norm() > 1e-8)
    {
      dir_hint = seg.normalized();
    }
  }
  else
  {
    dir_hint = local_G.pos - local_A.pos;
    if (dir_hint.norm() > 1e-8)
      dir_hint.normalize();
  }

  // Keep ground robots planar
  if (par_.vehicle_type != "uav")
    dir_hint[2] = 0.0;

  // 2) Use this as the "start_vel" argument (magnitude doesn't matter; we use the direction)
  Vec3f start_dir_hint(dir_hint.x(), dir_hint.y(), dir_hint.z());

  // Solve DGP
  vec_Vecf<3> raw_global_path;
  if (!dgp_manager_.solveDGP(local_A.pos, start_dir_hint, local_G.pos, final_g_, par_.global_planner_huristic_weight, A_time, global_path, raw_global_path))
  {
    if (par_.debug_verbose)
      std::cout << bold << red << "DGP did not find a solution" << reset << std::endl;
    dgp_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // use this for map resizing
  mtx_global_path_.lock();
  global_path_ = global_path;
  mtx_global_path_.unlock();

  // For visualization
  mtx_original_global_path_.lock();
  original_global_path_ = raw_global_path;
  mtx_original_global_path_.unlock();

  // Make sure global path does not exceed (num_P + 1)
  if (global_path.size() > par_.num_P + 1)
  {
    // Trim the global path
    global_path.resize(par_.num_P + 1);
  }

  // Find global path with safe sub goal
  findSafeSubGoal(global_path);

  // Debug
  if (par_.debug_verbose)
    std::cout << "global_path.size(): " << global_path.size() << std::endl;

  // Get computation time
  dgp_manager_.getComputationTime(global_planning_time_, dgp_static_jps_time_, dgp_check_path_time_, dgp_dynamic_astar_time_, dgp_recover_path_time_);

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::planLocalTrajectory(vec_Vecf<3> &global_path, double last_replaning_computation_time)
{

  // Get local_A, local_G and A_time
  state local_A, local_G, local_E;
  double A_time;
  getA(local_A);
  getG(local_G);
  getA_time(A_time);

  // If the global path's size is < 3 after trimming, we cannot proceed
  if (global_path.empty() || global_path.size() < 3)
  {
    // std::cout << bold << red << "Global path's size is < 3 after trimming" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // Initialize flag
  bool optimization_succeeded = false;

  // Set local_E
  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::GOAL_SEEN)
    local_E = local_G;
  else
    local_E.pos = global_path.back();

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
    local_E.pos[2] = 1.0;
  }

  /*
   * Parallelized Local Trajectory Optimization
   */

  // Reset whole trajectory planners to nominal state
  for (auto &solver : whole_traj_solver_ptrs_)
    solver->resetToNominalState();

  // Get the base map vector
  vec_Vec3f base_map;
  if (par_.sim_env == "gazebo")
  {
    dgp_manager_.getVecUnknownOccupied(base_map);
  }
  else if (par_.sim_env == "fake_sim")
  {
    dgp_manager_.getVecOccupied(base_map);
  }

  // Get obst_pos
  vec_Vecf<3> obst_pos;
  {
    std::lock_guard<std::mutex> lk(mtx_obst_pos_);
    obst_pos = obst_pos_;
  }

  // Compute an initial dt for the local trajectory optimization
  whole_traj_solver_ptrs_[0]->setX0(local_A);
  whole_traj_solver_ptrs_[0]->setXf(local_E);
  double initial_dt = whole_traj_solver_ptrs_[0]->getInitialDt();

  // Compute sub goal vector once
  std::vector<double> sub_goal;
  sub_goal.push_back(local_G.pos[0]);
  sub_goal.push_back(local_G.pos[1]);
  sub_goal.push_back(local_G.pos[2]);

  // Pre-compute convex decomposition if environment is static
  std::vector<LinearConstraint3D> shared_spatial_constraints;
  vec_E<Polyhedron<3>> shared_spatial_poly_out;
  bool use_precomputed_constraints = (par_.environment_assumption == "static");

  if (use_precomputed_constraints)
  {
    // For static environment, use spatial-only decomposition (not time-layered)
    // Compute seg_end_times based on worst-case trajectory time per spatial segment
    const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
    std::vector<double> seg_end_times = computeWorstSegEndTimesPoly(initial_dt, factors_[0], P);

    // Run spatial convex decomposition once before threading
    if (!dgp_manager_.cvxEllipsoidDecomp(
            ellip_workers_[0],
            global_path,
            base_map,
            obst_pos,
            seg_end_times,
            shared_spatial_constraints,
            shared_spatial_poly_out))
    {
      std::cout << bold << red << "Precomputed spatial convex decomposition failed for static environment" << reset << std::endl;
      return false;
    }
  }

  std::vector<std::future<std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>>> futures;
  futures.reserve(factors_.size());

  for (size_t i = 0; i < factors_.size(); ++i)
  {
    const double factor = factors_[i]; // corresponding factor for solver i

    futures.push_back(std::async(std::launch::async,
                                 [this, i, factor, &global_path, local_A, local_E, sub_goal, A_time,
                                  initial_dt, &obst_pos, &base_map, use_precomputed_constraints,
                                  &shared_spatial_constraints, &shared_spatial_poly_out]()
                                     -> std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>
                                 {
                                   try
                                   {
                                     double thread_gurobi_time = 0.0;
                                     double thread_convx_decomp_time = 0.0;
                                     vec_E<Polyhedron<3>> thread_poly_out_safe;

                                     // Per-worker decomp util (no sharing across worker index)
                                     EllipsoidDecomp3D &ellip = this->ellip_workers_[i];

                                     const bool result = generateLocalTrajectory(
                                         ellip,
                                         global_path,
                                         local_A, local_E, sub_goal, A_time,
                                         thread_gurobi_time,
                                         thread_convx_decomp_time,
                                         whole_traj_solver_ptrs_[i],
                                         factor,
                                         initial_dt,
                                         obst_pos,
                                         base_map, // base_uo snapshot
                                         thread_poly_out_safe,
                                         use_precomputed_constraints ? &shared_spatial_constraints : nullptr,
                                         use_precomputed_constraints ? &shared_spatial_poly_out : nullptr);

                                     return {result, thread_gurobi_time, thread_convx_decomp_time, factor, thread_poly_out_safe};
                                   }
                                   catch (const std::exception &ex)
                                   {
                                     std::cerr << "Exception in async task with factor " << factor
                                               << ": " << ex.what() << std::endl;
                                     return {false, 0.0, 0.0, factor, vec_E<Polyhedron<3>>{}};
                                   }
                                 }));
  }

  // Wait for any task to succeed.
  std::vector<bool> vec_optimization_succeeded;
  std::vector<std::vector<state>> vec_goal_setpoints;
  std::vector<PieceWisePol> vec_pwp_to_share;
  std::vector<std::vector<Eigen::Matrix<double, 3, 4>>> vec_cps;
  std::vector<double> vec_gurobi_times;
  std::vector<double> vec_convx_decomp_times;
  std::vector<vec_E<Polyhedron<3>>> vec_poly_out_safe;

  vec_optimization_succeeded.resize(factors_.size(), false);
  vec_goal_setpoints.resize(factors_.size());
  vec_pwp_to_share.resize(factors_.size());
  vec_cps.resize(factors_.size());
  vec_gurobi_times.resize(factors_.size(), 0.0);
  vec_convx_decomp_times.resize(factors_.size(), 0.0);
  vec_poly_out_safe.resize(factors_.size());

  for (size_t i = 0; i < futures.size(); ++i)
  {
    auto [result, thread_gurobi_time, thread_convx_decomp_time, thread_factor, thread_poly_out_safe] = futures[i].get();

    if (!result)
      continue;

    // One thread succeeded. Stop all the other solver instances.
    for (size_t j = 0; j < factors_.size(); ++j)
    {
      if (j == i)
        continue;

      try
      {
        whole_traj_solver_ptrs_[j]->stopExecution();
      }
      catch (const std::exception &e)
      {
        std::cout << "it's likely that the solver has gurobi error and already released the gurobi environment" << std::endl;
        std::cerr << e.what() << '\n';
      }
    }

    // Get Results from the successful solver.
    whole_traj_solver_ptrs_[i]->fillGoalSetPoints();
    whole_traj_solver_ptrs_[i]->getGoalSetpoints(vec_goal_setpoints[i]);
    whole_traj_solver_ptrs_[i]->getPieceWisePol(vec_pwp_to_share[i]);
    whole_traj_solver_ptrs_[i]->getControlPoints(vec_cps[i]); // Bezier control points
    vec_gurobi_times[i] = thread_gurobi_time;
    vec_convx_decomp_times[i] = thread_convx_decomp_time;
    vec_poly_out_safe[i] = thread_poly_out_safe;

    vec_optimization_succeeded[i] = true;
    // break; // Exit the loop after the first success
  }

  // Find the first successful optimization
  int successful_index = -1;
  for (size_t i = 0; i < vec_optimization_succeeded.size(); ++i)
  {
    if (vec_optimization_succeeded[i])
    {
      optimization_succeeded = true;
      goal_setpoints_ = vec_goal_setpoints[i];
      pwp_to_share_ = vec_pwp_to_share[i];
      cps_ = vec_cps[i];
      local_traj_computation_time_ = vec_gurobi_times[i];
      cvx_decomp_time_ = vec_convx_decomp_times[i];
      poly_out_safe_ = vec_poly_out_safe[i];
      successful_index = i;
      break; // Exit the loop after the first success
    }
  }

  if (optimization_succeeded)
  {
    // update list_subopt_goal_setpoints_ (vec_goal_setpoints without the successful one)
    list_subopt_goal_setpoints_.clear();
    list_subopt_goal_setpoints_.reserve(vec_goal_setpoints.size() - 1);
    for (size_t i = 0; i < vec_goal_setpoints.size(); ++i)
    {
      if (i != successful_index && !vec_goal_setpoints[i].empty())
      {
        list_subopt_goal_setpoints_.push_back(std::move(vec_goal_setpoints[i]));
      }
    }

    // update the factors_ vector
    if (par_.use_dynamic_factor)
    {
      // Save the successful factor BEFORE clearing
      double successful_factor = factors_[successful_index];

      // clear factors_ first
      factors_.clear();
      factors_.reserve(num_dynamic_factors_);

      // Set the successful factor to be the mean of k-radius factors
      for (int i = 0; i < num_dynamic_factors_; i++)
      {
        double factor = successful_factor - par_.dynamic_factor_k_radius + i * par_.factor_constant_step_size;
        if (factor >= 1.0)
          factors_.push_back(factor);
      }

      if (!dynamic_factor_inital_sucess_)
        dynamic_factor_inital_sucess_ = true;
    }
  }
  else
  {
    // if the optimization failed, we increase the factors_ for next replanning
    if (par_.use_dynamic_factor)
    {
      if (!dynamic_factor_inital_sucess_)
      {
        // shift all the factors in factors_ by factor_constant_step_size
        for (size_t i = 0; i < factors_.size(); i++)
        {
          factors_[i] = factors_[i] + par_.factor_constant_step_size;
        }
      }
    }
  }

  return optimization_succeeded;
}

// ----------------------------------------------------------------------------

void DYNUS::getPieceWisePol(PieceWisePol &pwp)
{
  pwp = pwp_to_share_;
}

// ----------------------------------------------------------------------------

// Computes worst-case segment end times (cumulative) for corridor inflation,
// while respecting the "worst assignment" idea over polytopes:
//
// - You have num_seg segments (from global_path.size()-1).
// - You have P polytopes (par_.num_P).
// - Worst assignment rule:
//     * Give 1 segment to as many of the last (P-1) polytopes as possible
//     * First polytope gets the remaining segments
//
// Returns seg_end_times with size = num_seg,
// where seg_end_times[i] is cumulative end time at end of segment i.
std::vector<double> DYNUS::computeWorstSegEndTimesPoly(
    double initial_dt, double factor, size_t num_seg)
{
  std::vector<double> seg_end_times;
  seg_end_times.reserve(num_seg);

  if (num_seg == 0)
    return seg_end_times;

  const int P = std::max(0, par_.num_P);
  if (P <= 0)
  {
    // Fallback: still produce valid per-segment times
    const double dt = initial_dt * factor;
    double t_acc = 0.0;
    for (size_t i = 0; i < num_seg; ++i)
    {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
    }
    return seg_end_times;
  }

  // Assign segments to polytopes under your rule, but using num_seg (NOT par_.num_N-1).
  // Number of last polytopes that can be guaranteed 1 segment:
  const int max_last_ones = P - 1;
  const int min_one = std::min<int>(max_last_ones, static_cast<int>(num_seg));

  std::vector<int> segments_per_poly(P, 0);

  // Last min_one polytopes get 1 segment each
  for (int k = 0; k < min_one; ++k)
  {
    const int p = (P - 1) - k;
    segments_per_poly[p] = 1;
  }

  // First polytope gets the remainder
  const int assigned_to_last = min_one;
  const int first_segments = static_cast<int>(num_seg) - assigned_to_last;
  if (first_segments > 0)
    segments_per_poly[0] += first_segments;

  // Convert to per-segment cumulative end times
  const double dt = initial_dt * factor;
  double t_acc = 0.0;

  size_t produced = 0;
  for (int p = 0; p < P && produced < num_seg; ++p)
  {
    const int k = segments_per_poly[p];
    for (int s = 0; s < k && produced < num_seg; ++s)
    {
      t_acc += dt;
      seg_end_times.push_back(t_acc);
      ++produced;
    }
  }

  // Safety fallback: if anything went odd, pad to length num_seg
  while (seg_end_times.size() < num_seg)
  {
    t_acc += dt;
    seg_end_times.push_back(t_acc);
  }

  return seg_end_times;
}

// ----------------------------------------------------------------------------

bool DYNUS::generateLocalTrajectory(
    EllipsoidDecomp3D &ellip,
    const vec_Vecf<3> &global_path,
    const state &local_A, const state &local_E, const std::vector<double> &sub_goal, double A_time,
    double &gurobi_computation_time,
    double &cvx_decomp_time,
    std::shared_ptr<SolverGurobi> &whole_traj_solver_ptr,
    double factor,
    double initial_dt,
    const vec_Vecf<3> &obst_pos,
    const vec_Vec3f &base_uo,
    vec_E<Polyhedron<3>> &poly_out_safe,
    const std::vector<LinearConstraint3D>* precomputed_spatial_constraints,
    const vec_E<Polyhedron<3>>* precomputed_spatial_poly_out)
{

  // P: spatial corridor pieces (global segments)
  const size_t P = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
  if (P == 0)
    return false;

  // N: local trajectory segments (time layers)
  const size_t N = static_cast<size_t>(par_.num_N);
  if (N == 0)
    return false;

  // Local time layers: end time of local segment n
  // NOTE: this matches your solver's uniform dt assumption (dt = initial_dt * factor).
  const double dt_layer = initial_dt * factor;
  std::vector<double> time_end_times;
  time_end_times.reserve(N);
  for (size_t n = 0; n < N; ++n)
    time_end_times.push_back((static_cast<double>(n) + 1.0) * dt_layer);

  // For choosing a representative safe corridor for visualization, keep your existing "worst case per spatial segment"
  // This gives seg_end_times size = P (global segments)
  std::vector<double> seg_end_times = computeWorstSegEndTimesPoly(initial_dt, factor, P);

  // Timer for computing the safe corridor
  MyTimer cvx_decomp_timer(true);

  // Check if we have precomputed spatial constraints (static environment)
  bool use_spatial_only = (precomputed_spatial_constraints != nullptr && precomputed_spatial_poly_out != nullptr);

  // Declare constraints outside if block so they're available for solver setup
  std::vector<std::vector<LinearConstraint3D>> l_constraints_by_time;
  std::vector<vec_E<Polyhedron<3>>> poly_out_by_time; // [N][P]

  if (use_spatial_only)
  {
    // Static environment: use precomputed spatial-only constraints
    // Copy the spatial polytopes for visualization
    poly_out_safe = *precomputed_spatial_poly_out;
    cvx_decomp_time = 0.0; // No decomposition time since we're using precomputed
  }
  else
  {
    // Dynamic environment: compute time-layered constraints for this thread
    if (!dgp_manager_.cvxEllipsoidDecompTimeLayered(
            ellip,
            global_path,
            base_uo,
            obst_pos,
            time_end_times,
            l_constraints_by_time,
            poly_out_by_time))
    {
      std::cout << bold << red << "Time-layered convex decomposition failed" << reset << std::endl;
      poly_out_safe.clear();
      return false;
    }

    cvx_decomp_time = cvx_decomp_timer.getElapsedMicros() / 1000.0;

    // Build poly_out_safe from all time layers for visualization
    poly_out_safe.clear();
    poly_out_safe.reserve(N * P);
    for (size_t n = 0; n < N; ++n)
    {
      for (size_t p = 0; p < P; ++p)
      {
        poly_out_safe.emplace_back(poly_out_by_time[n][p]);
      }
    }
  }

  // Initialize the solver.
  whole_traj_solver_ptr->setX0(local_A);                                 // Initial condition
  whole_traj_solver_ptr->setXf(local_E);                                 // Final condition

  // Set polytopes based on environment type
  if (use_spatial_only)
  {
    // Static environment: use spatial-only polytopes
    whole_traj_solver_ptr->setPolytopes(*precomputed_spatial_constraints);
  }
  else
  {
    // Dynamic environment: use time-layered polytopes
    whole_traj_solver_ptr->setPolytopesTimeLayered(l_constraints_by_time);
  }
  whole_traj_solver_ptr->setT0(A_time);                                  // Initial time (kept as-is)
  whole_traj_solver_ptr->setInitialDt(initial_dt);                       // Initial dt

  // Solve the optimization problem.
  bool gurobi_error_detected = false;
  bool gurobi_result = whole_traj_solver_ptr->generateNewTrajectory(gurobi_error_detected, gurobi_computation_time, factor);

  // If a Gurobi error occurred, reset the solver and return.
  if (gurobi_error_detected)
  {
    whole_traj_solver_ptr = std::make_shared<SolverGurobi>();
    whole_traj_solver_ptr->initializeSolver(par_);
    return false;
  }

  // If no solution is found, return.
  if (!gurobi_result)
    return false;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::appendToPlan()
{

  if (par_.debug_verbose)
    std::cout << "goal_setpoints_.size(): " << goal_setpoints_.size() << std::endl;

  // mutex lock
  mtx_plan_.lock();

  // get the size of the plan and plan_safe_paths
  int plan_size = plan_.size();

  // If the plan size is less than k_value_, which means we already passed point A, we cannot use this plan
  if (plan_size < k_value_)
  {
    if (par_.debug_verbose)
      std::cout << bold << red << "(plan_size - k_value_) = " << (plan_size - k_value_) << " < 0" << reset << std::endl;
    k_value_ = std::max(1, plan_size - 1); // Decrease k_value_ to plan_size - 1 but at least 1
  }
  else // If the plan size is greater than k_value_, which means we haven't passed point A yet, we can use this plan
  {
    plan_.erase(plan_.end() - k_value_, plan_.end());
    plan_.insert(plan_.end(), goal_setpoints_.begin(), goal_setpoints_.end());
  }

  // mutex unlock
  mtx_plan_.unlock();

  // k_value adaptation initialization
  if (!got_enough_replanning_)
  {
    if (store_computation_times_.size() < par_.num_replanning_before_adapt)
    {
      num_replanning_++;
    }
    else
    {
      startAdaptKValue();
      got_enough_replanning_ = true;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the terminal goal state.
 * @param state &G_term: Output terminal goal state.
 */
void DYNUS::getGterm(state &G_term)
{
  mtx_G_term_.lock();
  G_term = G_term_;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the terminal goal state.
 * @param state G_term: Terminal goal state to set.
 */
void DYNUS::setGterm(const state &G_term)
{
  mtx_G_term_.lock();
  G_term_ = G_term;
  mtx_G_term_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the subgoal.
 * @param state &G: Output subgoal.
 */
void DYNUS::getG(state &G)
{
  mtx_G_.lock();
  G = G_;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets point E
 * @param state &G: Output point E
 */
void DYNUS::getE(state &E)
{
  mtx_E_.lock();
  E = E_;
  mtx_E_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the subgoal.
 * @param state G: Subgoal to set.
 */
void DYNUS::setG(const state &G)
{
  mtx_G_.lock();
  G_ = G;
  mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets A (starting point for global planning).
 * @param state &G: Output A.
 */
void DYNUS::getA(state &A)
{
  mtx_A_.lock();
  A = A_;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets A (starting point for global planning).
 * @param state &G: Input A.
 */
void DYNUS::setA(const state &A)
{
  mtx_A_.lock();
  A_ = A;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void DYNUS::getA_time(double &A_time)
{
  mtx_A_time_.lock();
  A_time = A_time_;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets A (starting point for global planning)'s time
 * @param state &G: Input A time
 */
void DYNUS::setA_time(double A_time)
{
  mtx_A_time_.lock();
  A_time_ = A_time;
  mtx_A_time_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the current state.
 * @param state &state: Output current state.
 */
void DYNUS::getState(state &state)
{
  mtx_state_.lock();
  state = state_;
  mtx_state_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets the last plan state
 * @param state &state: Output last plan state
 */
void DYNUS::getLastPlanState(state &state)
{
  mtx_plan_.lock();
  state = plan_.back();
  mtx_plan_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets trajs_
 * @param std::vector<std::shared_ptr<dynTraj>> &trajs: Output trajs_
 */
void DYNUS::getTrajs(std::vector<std::shared_ptr<dynTraj>> &out)
{
  std::lock_guard<std::mutex> lock(mtx_trajs_);
  out = trajs_; // copies shared_ptr only, not expressions
}

// ----------------------------------------------------------------------------

/**
 * @brief Cleans up old trajectories.
 * @param double current_time: Current timestamp.
 */
void DYNUS::cleanUpOldTrajs(double current_time)
{
  std::lock_guard<std::mutex> lock(mtx_trajs_);

  // remove_if moves all “expired” to the end, then erase() chops them off
  trajs_.erase(
      std::remove_if(
          trajs_.begin(),
          trajs_.end(),
          [&](const std::shared_ptr<dynTraj> &t)
          {
            return (current_time - t->time_received) > par_.traj_lifetime;
          }),
      trajs_.end());
}

// ----------------------------------------------------------------------------

/**
 * @brief Adds or updates a trajectory.
 * @param dynTraj new_traj: New trajectory to add.
 * @param double current_time: Current timestamp.
 */
void DYNUS::addTraj(std::shared_ptr<dynTraj> new_traj, double current_time)
{

  // Evaluate
  Eigen::Vector3d p = new_traj->eval(current_time);
  if (!checkPointWithinMap(p))
    return;
  if ((p - state_.pos).norm() > par_.horizon)
    return;

  {
    std::lock_guard<std::mutex> lock(mtx_trajs_);
    auto it = std::find_if(trajs_.begin(), trajs_.end(),
                           [&](const std::shared_ptr<dynTraj> &t)
                           { return t && t->id == new_traj->id; });

    if (it != trajs_.end())
      *it = new_traj; // replace pointer
    else
      trajs_.push_back(new_traj);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Updates the current state.
 * @param state data: New state data.
 */
void DYNUS::updateState(state data)
{

  // If we are doing hardware and provide goal in global frame (e.g. vicon), we need to transform the goal to the local frame

  if (par_.use_hardware && par_.provide_goal_in_global_frame)
  {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(data.pos[0], data.pos[1], data.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_ * homo_pos;
    data.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);

    // Apply rotation to velocity
    data.vel = init_pose_transform_rotation_ * data.vel;

    // Apply rotation to accel
    data.accel = init_pose_transform_rotation_ * data.accel;

    // Apply rotation to jerk
    data.jerk = init_pose_transform_rotation_ * data.jerk;

    // Apply yaw
    data.yaw += yaw_init_offset_;
  }

  mtx_state_.lock();
  state_ = data;
  mtx_state_.unlock();

  if (state_initialized_ == false || drone_status_ == DroneStatus::YAWING)
  {

    // create temporary state
    state tmp;
    tmp.pos = data.pos;
    tmp.yaw = data.yaw;
    previous_yaw_ = data.yaw;

    // Push the state to the plan
    mtx_plan_.lock();
    plan_.clear();
    plan_.push_back(tmp);
    mtx_plan_.unlock();

    // Update Point A
    setA(tmp);

    // Update Point G
    setG(tmp);

    // Update the flag
    state_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the next goal (setpoint) from the plan.
 * @param state &next_goal: Output next goal state.
 * @return bool
 */
bool DYNUS::getNextGoal(state &next_goal)
{

  // Check if the planner is initialized
  if (!checkReadyToReplan())
  {
    return false;
  }

  // Pop the front of the plan
  next_goal.setZero();

  // If the plan is empty, return false
  mtx_plan_.lock(); // Lock the mutex
  auto local_plan = plan_;
  mtx_plan_.unlock(); // Unlock the mutex

  // Get the next goal
  next_goal = local_plan.front();

  // If there's more than one goal setpoint, pop the front
  if (local_plan.size() > 1)
  {
    mtx_plan_.lock();
    plan_.pop_front();
    mtx_plan_.unlock();
  }

  if (par_.use_hardware && par_.provide_goal_in_global_frame)
  {
    // Apply transformation to position
    Eigen::Vector4d homo_pos(next_goal.pos[0], next_goal.pos[1], next_goal.pos[2], 1.0);
    Eigen::Vector4d global_pos = init_pose_transform_inv_ * homo_pos;

    // Apply transformation to velocity
    Eigen::Vector3d global_vel = init_pose_transform_rotation_inv_ * next_goal.vel;

    // Apply transformation to accel
    Eigen::Vector3d global_accel = init_pose_transform_rotation_inv_ * next_goal.accel;

    // Apply transformation to jerk
    Eigen::Vector3d global_jerk = init_pose_transform_rotation_inv_ * next_goal.jerk;

    next_goal.pos = Eigen::Vector3d(global_pos[0], global_pos[1], global_pos[2]);
    next_goal.vel = global_vel;
    next_goal.accel = global_accel;
    next_goal.jerk = global_jerk;
  }

  if (!(drone_status_ == DroneStatus::GOAL_REACHED))
  {
    // Get the desired yaw
    // If the planner keeps failing, just keep spinning
    if (replanning_failure_count_ > par_.yaw_spinning_threshold)
    {
      next_goal.yaw = previous_yaw_ + par_.yaw_spinning_dyaw * par_.dc;
      next_goal.dyaw = par_.yaw_spinning_dyaw;
      previous_yaw_ = next_goal.yaw;
    }
    else
    {
      // If the local_plan is small just use the previous yaw with no dyaw
      if (local_plan.size() < 5)
      {
        next_goal.yaw = previous_yaw_;
        next_goal.dyaw = 0.0;
      }
      else
      {
        getDesiredYaw(next_goal);
      }
    }

    if (par_.use_hardware && par_.provide_goal_in_global_frame)
    {
      next_goal.yaw -= yaw_init_offset_;
    }

    next_goal.dyaw = std::clamp(next_goal.dyaw, -par_.w_max, par_.w_max);
  }
  else
  {
    next_goal.yaw = previous_yaw_;
    next_goal.dyaw = 0.0;
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the desired yaw for the next goal.
 * @param state &next_goal: Next goal state to update with desired yaw.
 */
void DYNUS::getDesiredYaw(state &next_goal)
{

  double diff = 0.0;
  double desired_yaw = 0.0;

  // Get state
  state local_state;
  getState(local_state);

  // Get G_term
  mtx_G_term_.lock();
  state G_term = G_term_;
  mtx_G_term_.unlock();

  switch (drone_status_)
  {
  case DroneStatus::YAWING:
    desired_yaw = atan2(G_term.pos[1] - next_goal.pos[1], G_term.pos[0] - next_goal.pos[0]);
    diff = desired_yaw - local_state.yaw;
    // std::cout << "diff1= " << diff << std::endl;
    break;
  case DroneStatus::TRAVELING:
  case DroneStatus::GOAL_SEEN:
    desired_yaw = atan2(next_goal.pos[1] - local_state.pos.y(), next_goal.pos[0] - local_state.pos.x());
    diff = desired_yaw - local_state.yaw;
    next_goal.yaw = desired_yaw;
    break;
  case DroneStatus::GOAL_REACHED:
    next_goal.dyaw = 0.0;
    next_goal.yaw = previous_yaw_;
    return;
  }

  dynus_utils::angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  yaw(diff, next_goal);
}

// ----------------------------------------------------------------------------

void DYNUS::yaw(double diff, state &next_goal)
{
  saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);
  dyaw_filtered_ = (1 - par_.alpha_filter_dyaw) * (copysign(1, diff) * par_.w_max) + par_.alpha_filter_dyaw * dyaw_filtered_;
  next_goal.dyaw = dyaw_filtered_;
  next_goal.yaw = previous_yaw_ + dyaw_filtered_ * par_.dc;
  previous_yaw_ = next_goal.yaw;
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the terminal goal.
 * @param const state &term_goal: Desired terminal goal state.
 */
void DYNUS::setTerminalGoal(const state &term_goal)
{

  // Get the state
  state local_state;
  getState(local_state);

  // Set the terminal goal
  setGterm(term_goal);

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = dynus_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
  mtx_G_.unlock();

  changeDroneStatus(DroneStatus::TRAVELING);

  if (!terminal_goal_initialized_)
    terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Changes the drone's status (YAWING, TRAVELING, GOAL_SEEN, GOAL_REACHED).
 * @param int new_status: New status value.
 */
void DYNUS::changeDroneStatus(int new_status)
{
  if (new_status == drone_status_)
    return;

  std::cout << "Changing DroneStatus from ";

  switch (drone_status_)
  {
  case DroneStatus::YAWING:
    std::cout << bold << "status_=YAWING" << reset;
    break;
  case DroneStatus::TRAVELING:
    std::cout << bold << "status_=TRAVELING" << reset;
    break;
  case DroneStatus::GOAL_SEEN:
    std::cout << bold << "status_=GOAL_SEEN" << reset;
    break;
  case DroneStatus::GOAL_REACHED:
    std::cout << bold << "status_=GOAL_REACHED" << reset;
    break;
  }

  std::cout << " to ";

  switch (new_status)
  {
  case DroneStatus::YAWING:
    std::cout << bold << "status_=YAWING" << reset;
    break;
  case DroneStatus::TRAVELING:
    std::cout << bold << "status_=TRAVELING" << reset;
    break;
  case DroneStatus::GOAL_SEEN:
    std::cout << bold << "status_=GOAL_SEEN" << reset;
    break;
  case DroneStatus::GOAL_REACHED:
    std::cout << bold << "status_=GOAL_REACHED" << reset;
    break;
  }

  std::cout << std::endl;

  drone_status_ = new_status;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if all necessary components are initialized.
 * @return bool
 */
bool DYNUS::checkReadyToReplan()
{
  return state_initialized_ &&
         terminal_goal_initialized_ &&
         dgp_manager_.isMapInitialized() &&
         (!par_.use_hardware || (kdtree_map_initialized_
                                 // && kdtree_unk_initialized_
                                 ));

  // if (!is_ready) printf("\033[1;31mNot ready to replan: state_initialized_=%d, terminal_goal_initialized_=%d, map_initialized_=%d, kdtree_map_initialized_=%d\033[0m\n",
  //                            state_initialized_, terminal_goal_initialized_,
  //                            dgp_manager_.isMapInitialized(),
  //                            kdtree_map_initialized_ /*, kdtree_unk_initialized_*/);
}

// ----------------------------------------------------------------------------

void DYNUS::updateMapPtr(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_unk)
{
  // 1) Atomically store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    pclptr_map_ = pclptr_map;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    pclptr_unk_ = pclptr_unk;
  }

  if (!dgp_manager_.isMapInitialized())
  {
    updateMap(0.0);
  }
}

// ----------------------------------------------------------------------------

void DYNUS::updateMap(double current_time)
{
  // Update the map size
  state local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions and traj_max_time
  vec_Vecf<3> obst_pos;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;

  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, pred_samples, pred_times, current_time);

  dgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // time the map update
  MyTimer timer_map(true);

  // 2) map update
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    std::lock_guard<std::mutex> lk2(mtx_pclptr_unk_);

    dgp_manager_.updateMap(wdx_, wdy_, wdz_, map_center_, pclptr_map_, pclptr_unk_, obst_pos, traj_max_time_);

    if (par_.debug_verbose)
      std::cout << "Map update time: " << timer_map.getElapsedMicros() / 1000.0 << " ms" << std::endl;

    // 3) Known‐space KD‐tree
    if (pclptr_map_ && !pclptr_map_->points.empty())
    {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      dgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
    }
    else
    {
      RCLCPP_WARN(
          rclcpp::get_logger("dynus"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }

  // 4) Unknown‐space KD‐tree
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_unk_);
    if (pclptr_unk_ && !pclptr_unk_->points.empty())
    {
      std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
      kdtree_unk_.setInputCloud(pclptr_unk_);
      kdtree_unk_initialized_ = true;
      // merge known into unknown vector
      dgp_manager_.updateVecUnknownOccupied(pclptr_to_vec(pclptr_unk_));
      dgp_manager_.insertVecOccupiedToVecUnknownOccupied();
    }
    else
    {
      RCLCPP_WARN(
          rclcpp::get_logger("dynus"),
          "updateMap: member pclptr_unk_ was null or empty; skipping KD‐tree update");
    }
  }
}

// ----------------------------------------------------------------------------

void DYNUS::updateOccupancyMapPtr(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_map)
{
  // store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);
    pclptr_map_ = pclptr_map;
  }

  if (!dgp_manager_.isMapInitialized())
  {
    updateOccupancyMap(0.0);
  }
}

// ----------------------------------------------------------------------------

void DYNUS::updateOccupancyMap(double current_time)
{

  // Update the map size
  state local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions and traj_max_time
  vec_Vecf<3> obst_pos;
  std::vector<vec_Vecf<3>> pred_samples;
  std::vector<float> pred_times;

  traj_max_time_ = computeObstPosAndTrajMaxTimeForMapUpdate(
      obst_pos, pred_samples, pred_times, current_time);

  dgp_manager_.setDynamicPredictedSamples(pred_samples, pred_times);

  // 2) map update (unlocked)
  {
    std::lock_guard<std::mutex> lk(mtx_pclptr_map_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pclptr_unk(new pcl::PointCloud<pcl::PointXYZ>());

    dgp_manager_.updateMap(wdx_, wdy_, wdz_, map_center_, pclptr_map_, empty_pclptr_unk, obst_pos, traj_max_time_);

    // 3) Known‐space KD‐tree
    if (pclptr_map_ && !pclptr_map_->points.empty())
    {
      std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
      kdtree_map_.setInputCloud(pclptr_map_);
      kdtree_map_initialized_ = true;
      dgp_manager_.updateVecOccupied(pclptr_to_vec(pclptr_map_));
    }
    else
    {
      RCLCPP_WARN(
          rclcpp::get_logger("dynus"),
          "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
    }
  }
}

// ----------------------------------------------------------------------------

double DYNUS::computeObstPosAndTrajMaxTimeForMapUpdate(
    vec_Vecf<3> &obst_pos,
    std::vector<vec_Vecf<3>> &pred_samples, // [K][M]
    std::vector<float> &pred_times,         // [M], relative times from now
    double current_time)
{
  obst_pos.clear();
  pred_samples.clear();
  pred_times.clear();

  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  // 1) Filter obstacles and build obst_pos in a consistent order
  std::vector<std::shared_ptr<dynTraj>> selected_trajs;
  selected_trajs.reserve(local_trajs.size());

  for (const auto &traj : local_trajs)
  {
    Eigen::Vector3d p = traj->eval(current_time);
    if (!checkPointWithinMap(p) || (p - state_.pos).norm() > (par_.horizon))
      continue;

    obst_pos.push_back(p);
    selected_trajs.push_back(traj);
  }

  // Update obst_pos_ (kept as you already do)
  {
    std::lock_guard<std::mutex> lock(mtx_obst_pos_);
    obst_pos_ = obst_pos;
  }

  // 2) Horizon for map update (your existing “worst possible”)
  const double Th = worst_traj_time_ * factors_.back(); // [s]
  if (!(Th > 0.0) || selected_trajs.empty())
    return Th;

  // 3) Build time samples between [0, Th]
  const double dt = 0.5; // [s]
  int M = static_cast<int>(std::ceil(Th / dt)) + 1;
  // keep it bounded for cost (heat-map build is O(#voxels * K * M))
  M = std::max(5, std::min(M, 10));

  pred_times.resize(M);
  for (int j = 0; j < M; ++j)
  {
    const double a = (M == 1) ? 0.0 : (double)j / (double)(M - 1);
    pred_times[j] = static_cast<float>(a * Th); // relative time from now
  }

  // 4) Sample each obstacle trajectory at (current_time + pred_times[j])
  pred_samples.resize(selected_trajs.size());
  for (size_t k = 0; k < selected_trajs.size(); ++k)
  {
    pred_samples[k].resize(M);
    for (int j = 0; j < M; ++j)
    {
      const double t_abs = current_time + (double)pred_times[j];
      Eigen::Vector3d pk = selected_trajs[k]->eval(t_abs);

      // NOTE: We do NOT drop samples outside the map, because the heat map
      // will simply have no effect there. But we must avoid NaNs.
      if (!std::isfinite(pk.x()) || !std::isfinite(pk.y()) || !std::isfinite(pk.z()))
      {
        // fallback: use current position (safe default)
        pk = selected_trajs[k]->eval(current_time);
      }

      pred_samples[k][j] = pk;
    }
  }

  return Th;
}

// ----------------------------------------------------------------------------

std::shared_ptr<dynus::VoxelMapUtil> DYNUS::getMapUtilSharedPtr()
{
  return dgp_manager_.getMapUtilSharedPtr();
}

// ----------------------------------------------------------------------------

/**
 * @brief Set the initial pose.
 * @param const geometry_msgs::msg::TransformStamped &init_pose: Initial pose.
 */
void DYNUS::setInitialPose(const geometry_msgs::msg::TransformStamped &init_pose)
{
  init_pose_ = init_pose;

  // First compute transformation matrix from init_pose_ (geometry_msgs::msg::TransformStamped)
  Eigen::Matrix4d init_pose_transform = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond init_pose_quat(init_pose_.transform.rotation.w, init_pose_.transform.rotation.x, init_pose_.transform.rotation.y, init_pose_.transform.rotation.z);
  Eigen::Vector3d init_pose_translation(init_pose_.transform.translation.x, init_pose_.transform.translation.y, init_pose_.transform.translation.z);
  init_pose_transform.block<3, 3>(0, 0) = init_pose_quat.toRotationMatrix();
  init_pose_transform.block<3, 1>(0, 3) = init_pose_translation;

  // Get initial pose
  init_pose_transform_ = init_pose_transform;
  init_pose_transform_rotation_ = init_pose_quat.toRotationMatrix();
  yaw_init_offset_ = std::atan2(init_pose_transform_rotation_(1, 0),
                                init_pose_transform_rotation_(0, 0));

  std::cout << bold << green << "yaw_init_offset_: " << yaw_init_offset_ << reset << std::endl;

  // Get the inverse of init_pose_ (geometry_msgs::msg::TransformStamped)
  init_pose_transform_inv_ = init_pose_transform.inverse();
  init_pose_transform_rotation_inv_ = init_pose_quat.toRotationMatrix().inverse();
  // yaw_init_offset_ = std::atan2(init_pose_transform_rotation_inv_(1, 0),
  // init_pose_transform_rotation_inv_(0, 0));
}

// ----------------------------------------------------------------------------

// Apply the initial pose transformation to the pwp
void DYNUS::applyInitiPoseTransform(PieceWisePol &pwp)
{
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++)
  {
    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++)
    {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_ * coeff;

      // cout agent frame pose
      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

// Apply the inverse of initial pose transformation to the pwp
void DYNUS::applyInitiPoseInverseTransform(PieceWisePol &pwp)
{
  // Loop thru the intervals
  for (int i = 0; i < pwp.coeff_x.size(); i++)
  {

    // Loop thru a, b, c, and d
    for (int j = 0; j < 4; j++)
    {
      Eigen::Vector4d coeff;
      coeff[0] = pwp.coeff_x[i][j];
      coeff[1] = pwp.coeff_y[i][j];
      coeff[2] = pwp.coeff_z[i][j];
      coeff[3] = 1.0;

      // Apply multiplication
      coeff = init_pose_transform_inv_ * coeff;

      pwp.coeff_x[i][j] = coeff[0];
      pwp.coeff_y[i][j] = coeff[1];
      pwp.coeff_z[i][j] = coeff[2];
    }
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if the goal is reached.
 * @return bool
 */
bool DYNUS::goalReachedCheck()
{
  if (checkReadyToReplan() && drone_status_ == DroneStatus::GOAL_REACHED)
  {
    return true;
  }
  return false;
}