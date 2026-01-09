/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "mighty/mighty.hpp"

using namespace mighty;
using namespace termcolor;

typedef timer::Timer MyTimer;

// ----------------------------------------------------------------------------

/**
 * @brief Constructor for MIGHTY.
 * @param parameters par: Input configuration parameters.
 */
MIGHTY::MIGHTY(parameters par) : par_(par)
{

  // Set up dgp_manager
  dgp_manager_.setParameters(par_);

  // Compute factors_ for time allocation
  const int num_factors = static_cast<int>((par_.factor_final - par_.factor_initial) / par_.factor_constant_step_size) + 1;
  for (int i = 0; i < num_factors; i++)
  {
    double factor = par_.factor_initial + i * par_.factor_constant_step_size;
    factors_.push_back(factor);
  }

  // Set up unconstrained optimization solver for whole trajectory
  for (int i = 0; i < num_factors; i++)
  {
    whole_traj_solver_ptrs_.push_back(std::make_shared<SolverGurobi>());
    whole_traj_solver_ptrs_[i]->initializeSolver(par_);
  }

  // Set up decomp ellip workers for each thread
  ellip_workers_.resize(whole_traj_solver_ptrs_.size());

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
void MIGHTY::startAdaptKValue()
{

  // Compute the average computation time
  for (int i = 0; i < store_computation_times_.size(); i++)
  {
    est_comp_time_ += store_computation_times_[i];
  }
  est_comp_time_ = est_comp_time_ / store_computation_times_.size();

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
void MIGHTY::computeG(const state &A, const state &G_term, double horizon)
{
  // Initialize the result
  state local_G;

  // Compute pos for G
  local_G.pos = mighty_utils::projectPointToSphere(A.pos, G_term.pos, horizon);

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

bool MIGHTY::needReplan(const state &local_state, const state &local_G_term, const state &last_plan_state)
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

bool MIGHTY::findAandAtime(state &A, double &A_time, double current_time, double last_replaning_computation_time)
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
  if (A.pos[2] < par_.z_min || A.pos[2] > par_.z_max, A.pos[0] < par_.x_min || A.pos[0] > par_.x_max, A.pos[1] < par_.y_min || A.pos[1] > par_.y_max)
  {
    printf("A (%f, %f, %f) is out of the map\n", A.pos[0], A.pos[1], A.pos[2]);
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool MIGHTY::checkIfPointOccupied(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool MIGHTY::checkIfPointFree(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointFree(point);
}

// ----------------------------------------------------------------------------

void MIGHTY::findSafeSubGoal(vec_Vecf<3> &global_path)
{

  // Keep the original global path
  vec_Vecf<3> original_global_path = global_path;

  // Reset goal path
  global_path.clear();

  // Initialize it with the start point
  global_path.push_back(original_global_path[0]);

  // Kd-tree search parameters
  int n = 1; // find one neighbour
  std::vector<int> pointIdxNKNSearch(n);
  std::vector<float> pointNKNSquaredDistance(n);

  // sample parameters (TODO: make these parameters configurable)
  double sample_dist = 0.1; // [m] distance between two samples along the trajectory

  // flag for finding unknown space
  bool found_unk = false;

  // mutex lock
  std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);

  // loop through the global path and check if the points are in unknown space
  for (int i = 0; i < original_global_path.size() - 1; i++)
  {
    // Set the current and next global path point
    Eigen::Vector3d current_gp = original_global_path[i];
    Eigen::Vector3d next_gp = original_global_path[i + 1];

    // Compute the direction and distance between the two points
    Eigen::Vector3d dir = next_gp - current_gp;
    double dist = dir.norm();
    dir.normalize();

    // Sample points along the line segment
    int num_samples = static_cast<int>(dist / sample_dist);
    for (int j = 0; j <= num_samples; j++)
    {
      Eigen::Vector3d sample_point = current_gp + dir * sample_dist * j;
      pcl::PointXYZ searchPoint(sample_point(0), sample_point(1), sample_point(2));

      // Nearest neighbor search
      if (kdtree_unk_.nearestKSearch(searchPoint, n, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        if (sqrt(pointNKNSquaredDistance[0]) < par_.drone_radius)
        {
          // Found a point in unknown space
          found_unk = true;

          // Add the point to the safe sub goal path
          if (j != 0) // avoid adding the same point twice
            global_path.push_back(sample_point);
        }
      }

      if (found_unk)
        break;
    }

    if (found_unk)
      break;

    // add the next global path point to the safe sub goal path
    global_path.push_back(next_gp);
  }
}

// ----------------------------------------------------------------------------

void MIGHTY::computeMapSize(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
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

bool MIGHTY::checkPointWithinMap(const Eigen::Vector3d &point) const
{
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) && (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) && (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void MIGHTY::getStaticPushPoints(vec_Vecf<3> &static_push_points)
{
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void MIGHTY::getLocalGlobalPath(vec_Vecf<3> &local_global_path, vec_Vecf<3> &local_global_path_after_push)
{
  local_global_path = local_global_path_;
  local_global_path_after_push = local_global_path_after_push_;
}

// ----------------------------------------------------------------------------

void MIGHTY::getGlobalPath(vec_Vecf<3> &global_path)
{
  mtx_global_path_.lock();
  global_path = global_path_;
  mtx_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void MIGHTY::getOriginalGlobalPath(vec_Vecf<3> &original_global_path)
{
  mtx_original_global_path_.lock();
  original_global_path = original_global_path_;
  mtx_original_global_path_.unlock();
}

// ----------------------------------------------------------------------------

void MIGHTY::getFreeGlobalPath(vec_Vecf<3> &free_global_path)
{
  free_global_path = free_global_path_;
}

// ----------------------------------------------------------------------------

void MIGHTY::resetData()
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

void MIGHTY::retrieveData(double &final_g,
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

void MIGHTY::retrievePolytopes(vec_E<Polyhedron<3>> &poly_out_whole, vec_E<Polyhedron<3>> &poly_out_safe)
{
  poly_out_whole = poly_out_whole_;
  poly_out_safe = poly_out_safe_;
}

// ----------------------------------------------------------------------------

void MIGHTY::retrieveGoalSetpoints(std::vector<state> &goal_setpoints)
{
  goal_setpoints = goal_setpoints_;
}

// ----------------------------------------------------------------------------

void MIGHTY::retrieveListSubOptGoalSetpoints(std::vector<std::vector<state>> &list_subopt_goal_setpoints)
{
  list_subopt_goal_setpoints = list_subopt_goal_setpoints_;
}

// ----------------------------------------------------------------------------

void MIGHTY::retrieveCPs(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  cps = cps_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Replans the trajectory.
 * @param double last_replaning_computation_time: Last replanning computation time.
 * @param double current_time: Current timestamp.
 */
std::tuple<bool, bool> MIGHTY::replan(double last_replaning_computation_time, double current_time)
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

bool MIGHTY::generateGlobalPath(vec_Vecf<3> &global_path, double current_time, double last_replaning_computation_time)
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
  // if (!dgp_manager_.solveDGP(local_A.pos, local_A.vel, local_G.pos, final_g_, par_.global_planner_huristic_weight, A_time, global_path))
  if (!dgp_manager_.solveDGP(local_A.pos, start_dir_hint, local_G.pos, final_g_, par_.global_planner_huristic_weight, A_time, global_path))
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
  original_global_path_ = global_path;
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

bool MIGHTY::planLocalTrajectory(vec_Vecf<3> &global_path, double last_replaning_computation_time)
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

  // Compute goal pull time
  const double goal_pull_time = par_.goal_pull_time_buffer * last_replaning_computation_time;

  std::vector<std::future<std::tuple<bool, double, double, double, vec_E<Polyhedron<3>>>>> futures;
  futures.reserve(whole_traj_solver_ptrs_.size());

  for (size_t i = 0; i < whole_traj_solver_ptrs_.size(); ++i)
  {
    const double factor = factors_[i]; // corresponding factor for solver i

    futures.push_back(std::async(std::launch::async,
                                 [this, i, factor, &global_path, local_A, local_E, sub_goal, A_time,
                                  initial_dt, &obst_pos, &base_map, goal_pull_time]()
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
                                         goal_pull_time);

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

  vec_optimization_succeeded.resize(whole_traj_solver_ptrs_.size(), false);
  vec_goal_setpoints.resize(whole_traj_solver_ptrs_.size());
  vec_pwp_to_share.resize(whole_traj_solver_ptrs_.size());
  vec_cps.resize(whole_traj_solver_ptrs_.size());
  vec_gurobi_times.resize(whole_traj_solver_ptrs_.size(), 0.0);
  vec_convx_decomp_times.resize(whole_traj_solver_ptrs_.size(), 0.0);
  vec_poly_out_safe.resize(whole_traj_solver_ptrs_.size());

  for (size_t i = 0; i < futures.size(); ++i)
  {
    auto [result, thread_gurobi_time, thread_convx_decomp_time, thread_factor, thread_poly_out_safe] = futures[i].get();

    if (!result)
      continue;

    // One thread succeeded. Stop all the other solver instances.
    for (size_t j = 0; j < whole_traj_solver_ptrs_.size(); ++j)
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
    for (size_t i = 0; i < vec_goal_setpoints.size(); ++i)
    {
      if (i != successful_index && !vec_goal_setpoints[i].empty())
      {
        list_subopt_goal_setpoints_.push_back(vec_goal_setpoints[i]);
      }
    }
  }

  return optimization_succeeded;
}

// ----------------------------------------------------------------------------

void MIGHTY::getPieceWisePol(PieceWisePol &pwp)
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
std::vector<double> MIGHTY::computeWorstSegEndTimesPoly(
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

bool MIGHTY::generateLocalTrajectory(
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
    double goal_pull_time)
{

  // Compute worst-case (conservative) segment end times for safe corridor generation
  const size_t num_seg = (global_path.size() >= 2) ? (global_path.size() - 1) : 0;
  std::vector<double> seg_end_times = computeWorstSegEndTimesPoly(initial_dt, factor, num_seg);

  if (seg_end_times.size() != num_seg)
  {
    std::cout << "[BUG] seg_end_times.size()=" << seg_end_times.size()
              << " num_seg=" << num_seg
              << " global_path.size()=" << global_path.size()
              << " par_.num_P=" << par_.num_P
              << std::endl;
  }

  // Timer for computing the safe corridor
  MyTimer cvx_decomp_timer(true);

  // Get safe corridor polytopes
  std::vector<LinearConstraint3D> l_constraints;

  if (!dgp_manager_.cvxEllipsoidDecomp(
          ellip,
          global_path,
          base_uo,
          obst_pos,
          seg_end_times,
          l_constraints,
          poly_out_safe))
  {
    std::cout << bold << red << "Convex decomposition failed" << reset << std::endl;
    poly_out_safe.clear();
    return false;
  }

  cvx_decomp_time = cvx_decomp_timer.getElapsedMicros() / 1000.0;

  // Initialize the solver.
  whole_traj_solver_ptr->setX0(local_A);                  // Initial condition
  whole_traj_solver_ptr->setXf(local_E);                  // Final condition
  whole_traj_solver_ptr->setPolytopes(l_constraints);     // Safe corridor polytopes
  whole_traj_solver_ptr->setT0(A_time);                   // Initial time
  whole_traj_solver_ptr->setInitialDt(initial_dt);        // Initial dt
  whole_traj_solver_ptr->setSubGoal(sub_goal);            // Subgoal for goal pulling
  whole_traj_solver_ptr->setGoalPullTime(goal_pull_time); // Goal pull time

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

bool MIGHTY::appendToPlan()
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
void MIGHTY::getGterm(state &G_term)
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
void MIGHTY::setGterm(const state &G_term)
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
void MIGHTY::getG(state &G)
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
void MIGHTY::getE(state &E)
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
void MIGHTY::setG(const state &G)
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
void MIGHTY::getA(state &A)
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
void MIGHTY::setA(const state &A)
{
  mtx_A_.lock();
  A_ = A;
  mtx_A_.unlock();
}

// ----------------------------------------------------------------------------

void MIGHTY::getA_time(double &A_time)
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
void MIGHTY::setA_time(double A_time)
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
void MIGHTY::getState(state &state)
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
void MIGHTY::getLastPlanState(state &state)
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
void MIGHTY::getTrajs(std::vector<std::shared_ptr<dynTraj>> &out)
{
  std::lock_guard<std::mutex> lock(mtx_trajs_);
  out = trajs_; // copies shared_ptr only, not expressions
}

// ----------------------------------------------------------------------------

/**
 * @brief Cleans up old trajectories.
 * @param double current_time: Current timestamp.
 */
void MIGHTY::cleanUpOldTrajs(double current_time)
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
void MIGHTY::addTraj(std::shared_ptr<dynTraj> new_traj, double current_time)
{

  // Evaluate
  // Eigen::Vector3d p = new_traj->pwp.eval(current_time);
  // if (!checkPointWithinMap(p))
  //   return;
  // if ((p - state_.pos).norm() > par_.horizon)
  //   return;

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
void MIGHTY::updateState(state data)
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
bool MIGHTY::getNextGoal(state &next_goal)
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
void MIGHTY::getDesiredYaw(state &next_goal)
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

  mighty_utils::angle_wrap(diff);
  if (fabs(diff) < 0.04 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  yaw(diff, next_goal);
}

// ----------------------------------------------------------------------------

void MIGHTY::yaw(double diff, state &next_goal)
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
void MIGHTY::setTerminalGoal(const state &term_goal)
{

  // Get the state
  state local_state;
  getState(local_state);

  // Set the terminal goal
  setGterm(term_goal);

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = mighty_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
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
void MIGHTY::changeDroneStatus(int new_status)
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
bool MIGHTY::checkReadyToReplan()
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

void MIGHTY::updateMap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_map,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_unk,
    double current_time)
{
  // 1) Atomically store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    pclptr_map_ = pclptr_map;
  }
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_unk_);
    pclptr_unk_ = pclptr_unk;
  }

  // Update the map size
  state local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions and traj_max_time
  vec_Vecf<3> obst_pos;
  double traj_max_time = computeObstPosAndTrajMaxTimeForMapUpdate(obst_pos, current_time);

  // time the map update
  MyTimer timer_map(true);

  // 2) map update (unlocked)
  dgp_manager_.updateMap(wdx_, wdy_, wdz_, map_center_, pclptr_map_, obst_pos, traj_max_time);

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
        rclcpp::get_logger("mighty"),
        "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
  }

  // 4) Unknown‐space KD‐tree
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
        rclcpp::get_logger("mighty"),
        "updateMap: member pclptr_unk_ was null or empty; skipping KD‐tree update");
  }
}

// ----------------------------------------------------------------------------

void MIGHTY::updateOccupancyMap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pclptr_map,
    double current_time)
{
  // 1) Atomically store the incoming clouds
  {
    std::lock_guard<std::mutex> lk(mtx_kdtree_map_);
    pclptr_map_ = pclptr_map;
  }

  // Update the map size
  state local_state, local_G;
  getState(local_state);
  getG(local_G);
  computeMapSize(local_state.pos, local_G.pos);

  // Get dynamic obstacles' positions and traj_max_time
  vec_Vecf<3> obst_pos;
  double traj_max_time = computeObstPosAndTrajMaxTimeForMapUpdate(obst_pos, current_time);

  // 2) map update (unlocked)
  dgp_manager_.updateMap(wdx_, wdy_, wdz_, map_center_, pclptr_map_, obst_pos, traj_max_time);

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
        rclcpp::get_logger("mighty"),
        "updateMap: member pclptr_map_ was null or empty; skipping KD-tree update");
  }
}

// ----------------------------------------------------------------------------

double MIGHTY::computeObstPosAndTrajMaxTimeForMapUpdate(vec_Vecf<3> &obst_pos, double current_time)
{
  // Get a vector of obstacles' current positions
  obst_pos.clear();

  std::vector<std::shared_ptr<dynTraj>> local_trajs;
  getTrajs(local_trajs);

  for (const auto &traj : local_trajs)
  {
    Eigen::Vector3d p = traj->eval(current_time);
    if (!checkPointWithinMap(p) || (p - state_.pos).norm() > (par_.horizon / 2.0))
      continue;
    obst_pos.push_back(p);
  }

  // update obst_pos_
  {
    std::lock_guard<std::mutex> lock(mtx_obst_pos_);
    obst_pos_ = obst_pos;
  }

  // Get the traj_max_time
  if (prev_traj_max_time_ == -1.0) // not initialized yet
  {
    return (par_.max_dist_vertexes * par_.num_P) / par_.v_max; // initial huristic value
  }

  return prev_traj_max_time_ * 1.2; // increase by 20% to be safe
}

// ----------------------------------------------------------------------------

/**
 * @brief Set the initial pose.
 * @param const geometry_msgs::msg::TransformStamped &init_pose: Initial pose.
 */
void MIGHTY::setInitialPose(const geometry_msgs::msg::TransformStamped &init_pose)
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
void MIGHTY::applyInitiPoseTransform(PieceWisePol &pwp)
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
void MIGHTY::applyInitiPoseInverseTransform(PieceWisePol &pwp)
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
bool MIGHTY::goalReachedCheck()
{
  if (checkReadyToReplan() && drone_status_ == DroneStatus::GOAL_REACHED)
  {
    return true;
  }
  return false;
}