/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
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

  // Set up gurobi solvers for whole trajectory
  for (int i = 0; i < par_.num_factors; i++)
  {
    whole_traj_solver_ptrs_.push_back(std::make_shared<SolverGurobi>());
    initializeSolver(par_.num_N, whole_traj_solver_ptrs_[i]);
  }

  // Set up gurobi solver for safe trajectory
  for (int i = 0; i < par_.num_safe_paths; i++)
  {
    safe_traj_solver_ptrs_.push_back(std::make_shared<SolverGurobi>());
    initializeSolver(3, safe_traj_solver_ptrs_[i]); // Use closed-form -> N = 3
  }

  // Set up gurobi solver for contingency trajectory
  contingency_traj_solver_ptr_ = std::make_shared<SolverGurobi>();
  initializeSolver(3, contingency_traj_solver_ptr_); // Use closed-form -> N = 3

  // Set up yaw solver
  yaw_solver_.initialize(par_);
  yaw_solver_.setThreads(par_.gurobi_threads);

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

  // Initialize frontier flag
  use_frontiers_as_G_ = par_.use_frontiers;

  // P, N adaptation parameters
  // initialize gurobi_failed_counter_ vector with the size of num_N_max
  for (int i = 0; i < par_.num_N_max + 1; i++) // +1 so that gurobi_failed_counter_[i] corresponds to the number of failures when N = i
  {
    gurobi_failed_counter_.push_back(0);
  }

  // initialize current_N_, old_N_, current_P_, old_P_
  current_N_ = par_.num_N;
  old_N_ = par_.num_N;
  current_P_ = par_.num_P_min;
  old_P_ = par_.num_P_min;

  // Initialize the map size
  wdx_ = par_.initial_wdx;
  wdy_ = par_.initial_wdy;
  wdz_ = par_.initial_wdz;

  // Initialize max_dist_vertexes_
  max_dist_vertexes_ = par_.max_dist_vertexes;

  // Map resolution
  map_res_ = par_.res;

  // initialize factor
  previous_successful_factor_ = par_.min_factor;
}

// ----------------------------------------------------------------------------

/**
 * @brief Initializes the Gurobi solver.
 * @param int num_N: Number of segments for the optimization.
 */
void DYNUS::initializeSolver(int num_N, const std::shared_ptr<SolverGurobi> &traj_solver_ptr_)
{

  traj_solver_ptr_->setN(num_N);
  traj_solver_ptr_->setUseRefPoints(par_.use_ref_points, par_.num_ref_sample_points);
  traj_solver_ptr_->setClosedFormSolutionParams(par_.closed_form_time_allocation_adj_iter_max, par_.closed_form_initial_factor, par_.closed_form_factor_increment, par_.closed_form_factor_initial_decrement);
  traj_solver_ptr_->setOptimizationType(par_.optimization_type);
  traj_solver_ptr_->setDC(par_.dc);
  traj_solver_ptr_->setMapSize(par_.x_min, par_.x_max, par_.y_min, par_.y_max, par_.z_min, par_.z_max, par_.res);
  traj_solver_ptr_->setVehicleType(par_.vehicle_type);
  traj_solver_ptr_->createVars();
  double max_values[3] = {par_.v_max, par_.a_max, par_.j_max};
  traj_solver_ptr_->setBounds(max_values);
  traj_solver_ptr_->setTimeAllocationParameters(par_.factor_initial, par_.factor_final, par_.factor_gamma_up, par_.factor_gamma_down, par_.factor_constant_step_size, par_.factor_minimum, par_.factor_delta_step_size, par_.use_constant_step_size, par_.count_to_switch_to_constant_step_size);
  traj_solver_ptr_->setVerbose(par_.gurobi_verbose);
  traj_solver_ptr_->setThreads(par_.gurobi_threads);
  traj_solver_ptr_->setWMax(par_.w_max);
  traj_solver_ptr_->setWeights(par_.control_cost_weight, par_.dynamic_weight, par_.final_pos_weight, par_.final_vel_weight, par_.final_accel_weight, par_.final_yaw_cost_weight, par_.total_yaw_diff_weight, par_.ref_point_weight);
  traj_solver_ptr_->setCollisionClearance(par_.collision_clearance);
  traj_solver_ptr_->setFlags(par_.use_hard_constr_for_final_yaw, par_.debug_verbose);
  traj_solver_ptr_->setConstraintFlags(par_.use_hard_constr_for_final_state, par_.use_hard_dynamic_constraints);
  traj_solver_ptr_->setPlannerMode(par_.planner_mode);
  traj_solver_ptr_->setNumObsDistSamples(par_.num_obs_dist_samples);
  // traj_solver_ptr_->setLocalBoxSize(par_.local_box_size); // Currently not used but may be used in the future
}

// ----------------------------------------------------------------------------

/**
 * @brief Set maximum velocity
 */
void DYNUS::setVmax(double v_max)
{
  // Update the maximum velocity
  v_max_3d_ = Eigen::Vector3d(v_max, v_max, v_max);
  v_max_ = v_max;

  // We change the maximum velocity in whole_traj_solver_ptr_ but not in safe_traj_solver_ptr_ because safe trajectory should be generated using maximum velocity to avoid collisions
  double max_values[3] = {v_max, par_.a_max, par_.j_max};

  for (int i = 0; i < whole_traj_solver_ptrs_.size(); i++)
  {
    whole_traj_solver_ptrs_[i]->setBounds(max_values);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Starts adaptive k-value.
 */
void DYNUS::startAdaptKValue()
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
bool DYNUS::computeG(const state &A, const state &G_term, double horizon)
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

  // If we further want to find a G point that is in free space

  // mtx_plan_.lock();
  // state last_plan_state = plan_.back();
  // mtx_plan_.unlock();

  // // Find the closest free point to the projected point
  // state G_closest;
  // dgp_manager_.findClosestFreePoint(G.pos, G_closest.pos);
  // mtx_G_.lock();
  // G_ = G_closest;
  // mtx_G_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves planning parameters P and N.
 * @param int &new_P: Output new P value.
 * @param int &new_N: Output new N value.
 * @param int &old_P: Output previous P value.
 * @param int &old_N: Output previous N value.
 */
void DYNUS::getPandN(int &new_P, int &new_N, int &old_P, int &old_N)
{
  new_P = current_P_;
  new_N = current_N_;
  old_P = old_P_;
  old_N = old_N_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets a new value for N.
 * @param int new_N: New number of segments.
 */
void DYNUS::setNewN(int new_N, const std::shared_ptr<SolverGurobi> &traj_solver_ptr_)
{
  traj_solver_ptr_->removeVars();
  traj_solver_ptr_->setN(new_N);
  traj_solver_ptr_->createVars();
}

// ----------------------------------------------------------------------------

/**
 * @brief Check if the plan_ (future trajectory) is safe
 */
void DYNUS::checkFuturePlanSafety()
{

  // Check if we need to replan
  if (!checkReadyToReplan())
    return;

  // get local plan
  mtx_plan_.lock();
  std::deque<state> local_plan = plan_;
  mtx_plan_.unlock();

  // If the plan's size is less than 2, we don't need to check
  if (local_plan.size() < 2)
    return;

  // get local trajs_
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // If there's no traj, we don't need to check
  if (local_trajs.empty())
    return;

  // Get the start and end time
  double start_time = local_plan.front().t;
  double end_time = local_plan.back().t;

  // initialize the result
  bool result = true;

  // Loop through the plan
  Eigen::Vector3d collision_prune_traj_pos;
  double collision_prune_traj_time;
  for (double t = start_time; t < end_time; t += par_.safety_check_dt)
  {

    // Get the index of the state at time t
    int idx = int((t - start_time) / par_.dc);

    // sanity check (t and local_plan[idx].t should be about the same)
    // printf("t: %f\n", t);
    // printf("idx: %d\n", idx);
    // printf("local_plan.size(): %d\n", local_plan.size());
    // printf("local_plan[idx].t: %f\n", local_plan[idx].t);

    // Make sure the index is within the bounds
    if (idx >= local_plan.size())
      idx = local_plan.size() - 1;

    // Get the agent position at time t
    Eigen::Vector3d agent_pos = local_plan[idx].pos;

    // Check if the state is safe
    // loop through the trajs_
    for (const auto &traj : local_trajs)
    {
      // Get the traj state at time t
      Eigen::Vector3d traj_pos = traj.pwp.eval(t);

      // Check if the state is safe
      // Check if the distance is less than the safety distance
      if ((abs(agent_pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj.bbox[0]) / 2.0) && (abs(agent_pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj.bbox[1]) / 2.0) && (abs(agent_pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj.bbox[2]) / 2.0))
      {
        std::cout << red << bold << "Future collision detected" << reset << std::endl;
        result = false;
        collision_prune_traj_pos = traj_pos;
        collision_prune_traj_time = t;
        break;
      }
    }

    if (!result)
      break;
  }

  // If the plan is safe, we continue with the plan
  if (result)
    return;

  // If the plan is not safe, we need to switch to a contingency plan
  switchToSafePath(collision_prune_traj_pos, collision_prune_traj_time);
}

// ----------------------------------------------------------------------------

/**
 * @brief switch to a safe path that starts from the point closest to the current position
 */
void DYNUS::switchToSafePath(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time)
{

  // Get the current state
  state local_state;
  getState(local_state);

  // Find the closest safe path starting point
  mtx_plan_.lock();
  mtx_plan_safe_paths_.lock();

  bool safe_path_found = false;
  int closest_safe_path_idx = 0;
  for (int i = 0; i < plan_safe_paths_.size(); i++)
  {
    if (!plan_safe_paths_[i].empty())
    {

      // get the first and last point of plan_safe_paths_[i]
      state first_point = plan_safe_paths_[i].front();
      state last_point = plan_safe_paths_[i].back();

      // get local trajs_
      std::vector<dynTraj> local_trajs;
      getTrajs(local_trajs);

      // check if this safe path is safe
      bool safe_path_is_safe = true;
      for (const auto &traj : local_trajs)
      {
        // check if the first point and last point of the safe path is safe against first and last point of the trajs_
        // Note: We only check the first and last point of the safe path because time is critical here
        for (int j = 0; j < 2; j++)
        {
          Eigen::Vector3d traj_pos;
          if (j == 0)
          {
            traj_pos = traj.pwp.eval(traj.pwp.times.front());
          }
          else
          {
            traj_pos = traj.pwp.eval(traj.pwp.times.back());
          }

          // Check if the distance is less than the safety distance
          if ((abs(first_point.pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj.bbox[0]) / 2.0) && (abs(first_point.pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj.bbox[1]) / 2.0) && (abs(first_point.pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj.bbox[2]) / 2.0) || (abs(last_point.pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj.bbox[0]) / 2.0) && (abs(last_point.pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj.bbox[1]) / 2.0) && (abs(last_point.pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj.bbox[2]) / 2.0))
          {
            safe_path_is_safe = false;
            break;
          }
        }

        if (!safe_path_is_safe)
        {
          break;
        }
      }

      if (safe_path_is_safe)
      {
        safe_path_found = true;
        closest_safe_path_idx = i;
        break;
      }
    }
  }

  if (!safe_path_found)
  {
    std::cout << red << bold << "Future collision detected but no safe path found so generate a contingency plan" << reset << std::endl;
    generateContingencyPlan(collision_prune_traj_pos, collision_prune_traj_time); // Assumes that the mutex mtx_plan_ and mtx_plan_safe_paths_ are locked
    mtx_plan_.unlock();
    mtx_plan_safe_paths_.unlock();
    return;
  }

  std::cout << green << bold << "Future collision detected so swith to safe path" << reset << std::endl;

  // Now we have the closest safe path starting point so replace the plan with the safe path
  plan_.erase(plan_.begin() + closest_safe_path_idx, plan_.end());
  plan_.insert(plan_.end(), plan_safe_paths_[closest_safe_path_idx].begin(), plan_safe_paths_[closest_safe_path_idx].end());

  // Clear the plan_safe_paths_ and resize it as the same size as the plan_
  plan_safe_paths_.clear();
  plan_safe_paths_.resize(plan_.size(), std::vector<state>());

  mtx_plan_.unlock();
  mtx_plan_safe_paths_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Generates a contingency plan - this should be quicker than par_.dc so we can change the plan immediately
 * @note Assumes that the mutex mtx_plan_ and mtx_plan_safe_paths_ are locked
 */
void DYNUS::generateContingencyPlan(const Eigen::Vector3d &collision_prune_traj_pos, double collision_prune_traj_time)
{

  // Get the next goal from plan_
  state start = plan_.front();

  // Container for potential goals
  std::vector<state> potential_goals;

  // Compute the safe path distance
  double safe_path_distance = par_.min_safe_path_distance +
                              (par_.max_safe_path_distance - par_.min_safe_path_distance) * (start.vel.norm() / par_.v_max);

  // Compute the center goal along the velocity direction
  // Assuming start.pos is of a vector type (e.g. Eigen::Vector3d)
  Eigen::Vector3d velocity_dir = start.vel.normalized();
  Eigen::Vector3d center_goal = start.pos + safe_path_distance * velocity_dir;

  // Define a plane perpendicular to velocity.
  // Use a global 'up' vector (e.g., (0,0,1)). If velocity is almost vertical, we need to choose another reference
  Eigen::Vector3d global_up(0, 0, 1);
  if (std::fabs(velocity_dir.dot(global_up)) > 0.99)
  {
    global_up = Eigen::Vector3d(1, 0, 0);
  }

  // Compute a left vector by taking the cross product of velocity and global up.
  Eigen::Vector3d left_vector = velocity_dir.cross(global_up).normalized();

  // Compute an "up" vector on the plane (perpendicular to both velocity and left)
  Eigen::Vector3d up_vector = velocity_dir.cross(left_vector).normalized();

  // Add the center goal as one potential goal
  state center_state;
  center_state.pos = center_goal;
  potential_goals.push_back(center_state);

  // Generate 8 surrounding potential goals:
  // Directions: 0° (left), 45° (up-left), 90° (up), 135° (up-right),
  // 180° (right), 225° (down-right), 270° (down), 315° (down-left)
  for (int i = 0; i < 8; ++i)
  {
    double angle = (M_PI / 4.0) * i; // angle in radians
    // Compute the offset vector in the plane using the left and up vectors.
    Eigen::Vector3d offset = par_.contingency_lateral_offset * (std::cos(angle) * left_vector + std::sin(angle) * up_vector);

    state goal_state;
    goal_state.pos = center_goal + offset;
    potential_goals.push_back(goal_state);
  }

  // Identify the potential goal that is farthest from the collision prune trajectory and try to plan to that goal
  // If not successful, try the next farthest goal

  // Order the potential goals by distance from the collision prune trajectory
  std::vector<std::pair<double, state>> potential_goals_with_distance;
  for (auto goal : potential_goals)
  {
    double distance = (goal.pos - collision_prune_traj_pos).norm();
    potential_goals_with_distance.push_back(std::make_pair(distance, goal));
  }

  // Sort the potential goals by distance
  std::sort(potential_goals_with_distance.begin(), potential_goals_with_distance.end(),
            [](const std::pair<double, state> &a, const std::pair<double, state> &b)
            { return a.first > b.first; });

  // Try to plan to the farthest goal
  bool plan_success = false;
  std::vector<state> contingency_path;

  for (auto goal : potential_goals_with_distance)
  {

    // clear the contingency path
    contingency_path.clear();

    // Plan to the goal using the closed-form solution
    contingency_traj_solver_ptr_->resetToNominalState();
    plan_success = solveClosedForm(start, goal.second, contingency_path, contingency_traj_solver_ptr_, 0.2); // can use any safe_traj_solver_ptr

    // If planning is successful, break
    if (plan_success)
      break;
  }

  // Update the plan_
  if (plan_success)
  {
    // Update the plan_
    plan_.clear();
    plan_.insert(plan_.end(), contingency_path.begin(), contingency_path.end());
  }
  else
  {
    // If planning is not successful, we will just stop the drone
    state stop_state;
    stop_state.pos = start.pos;
    stop_state.vel = Eigen::Vector3d::Zero();
    stop_state.accel = Eigen::Vector3d::Zero();
    stop_state.jerk = Eigen::Vector3d::Zero();
    stop_state.yaw = start.yaw;
    stop_state.dyaw = 0.0;
    stop_state.t = start.t;
    plan_.clear();
    plan_.push_back(stop_state);
  }

  // Clear the plan_safe_paths_ and resize it as the same size as the plan_
  plan_safe_paths_.clear();
  plan_safe_paths_.resize(plan_.size(), std::vector<state>());
}

// ----------------------------------------------------------------------------

/**
 * @brief Adapts planning parameters N.
 * @param double current_time: The current timestamp.
 * @return std::tuple<bool, bool> (N changed, result)
 */
void DYNUS::adaptN(double current_time, bool &new_P_or_N_set)
{

  // Record old N
  old_N_ = current_N_;

  // If it keeps failing, we increase N
  if (gurobi_failed_counter_[current_N_] > par_.num_replan_fail_threshold)
  {
    // Get N value (number of segments)
    int new_N = std::max(par_.num_N, std::min(par_.num_N_max, current_N_ + 1));
    if (new_N > current_N_)
    {

      // Reset the counter
      gurobi_failed_counter_[current_N_] = 0;

      // Set new N
      current_N_ = new_N;

      // Set new N in the solvers
      for (int i = 0; i < whole_traj_solver_ptrs_.size(); i++)
      {
        setNewN(current_N_, whole_traj_solver_ptrs_[i]);
      }
    }
  }
  else if (current_N_ > par_.num_N &&                                                                                                            // We don't wanna go below the initial N
           gurobi_failed_counter_[current_N_] <= par_.num_replan_fail_threshold &&                                                               // If the counter is below the threshold, we can decrease N
           current_time - current_N_time_ > (gurobi_failed_counter_[current_N_] + 1) * par_.cool_down_duration_factor * par_.cool_down_duration) // If the counter is below the threshold, we can decrease N. +1 for the case when the counter is 0
  {
    current_N_ = std::max(par_.num_N, current_N_ - 1);

    // Set new N in the solvers
    for (int i = 0; i < whole_traj_solver_ptrs_.size(); i++)
    {
      setNewN(current_N_, whole_traj_solver_ptrs_[i]);
    }
  }

  // If N is changed, we need to publish it as a topic in dynus_node
  new_P_or_N_set = old_N_ != current_N_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Adapts planning parameters P.
 * @param double current_time: The current timestamp.
 * @return std::tuple<bool, bool> (P changed, result)
 */
bool DYNUS::adaptP(double current_time, bool &new_P_or_N_set)
{

  // Record old P
  old_P_ = current_P_;

  // Uncertainty from unknown cells
  int num_unknown_cells = dgp_manager_.countUnknownCells(); // Get number of unknown cells
  int total_num_cells = dgp_manager_.getTotalNumCells();    // Get total number of cells

  // Sanity check
  if (total_num_cells == 0)
  {
    std::cout << bold << red << "total_num_cells == 0" << reset << std::endl;
    return false;
  }

  // --- Compute the maximum velocity based on uncertainty and the updated exponent ---
  double ratio = static_cast<double>(num_unknown_cells) / total_num_cells;

  // If the ratio is greater than 0.5, we use the minimum number of polytopes
  current_P_ = ratio > 0.5 ? par_.num_P_min : par_.num_P_max;

  // If P is changed, we need to publish it as a topic in dynus_node
  new_P_or_N_set = old_P_ != current_P_;

  return true;
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

  if (par_.dist_to_term_g_verbose)
    std::cout << "dist_to_term_G: " << dist_to_term_G << std::endl;

  if (dist_to_term_G < par_.goal_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_REACHED);
    // Reset the parameters
    if (par_.p_n_mode == "dynus")
    {
      for (int i = 0; i < whole_traj_solver_ptrs_.size(); i++)
      {
        setNewN(par_.num_N, whole_traj_solver_ptrs_[i]);
      }
    }
    return false;
  }

  if (dist_to_term_G < par_.goal_seen_radius)
  {
    changeDroneStatus(DroneStatus::GOAL_SEEN); // This triggers to use the hard final state constraint
    // We observed when the goal is close, the soft final state constraint fails - so we will use MIQP
    if (par_.p_n_mode == "dynus")
    {
      for (int i = 0; i < whole_traj_solver_ptrs_.size(); i++)
      {
        setNewN(par_.num_N_when_close_to_goal, whole_traj_solver_ptrs_[i]);
      }
    }
    // return false;
  }

  if (drone_status_ == DroneStatus::GOAL_SEEN && dist_from_last_plan_state_to_term_G < par_.goal_radius)
  {
    // If you try to replan when the drone is really close to the goal, it will fail. so we will just return false
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
      std::cout << bold << red << "k_value_ is invalid" << reset << std::endl;
      return false;
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

// We want to push the original global path (with very small dist between points) but we don't want to push the points all the way to the end point (because that's a lot of points). So we estimate the end of local global path and cut the path there
void DYNUS::shortenGlobalPath(vec_Vecf<3> &global_path)
{

  // First compute the estimated distance to the end of the local global path
  double dist_to_end_of_local_global_path = max_dist_vertexes_ * (current_N_ + par_.num_lookahead_global_path_for_push);

  // Loop thru the global path and find the point that is closest to the estimated end of the local global path
  double total_dist_so_far = 0.0;
  int idx = 0;
  for (int i = 0; i < global_path.size(); i++)
  {
    if (i == 0)
      continue;

    total_dist_so_far += (global_path[i] - global_path[i - 1]).norm();

    if (total_dist_so_far > dist_to_end_of_local_global_path)
    {
      idx = i;
      break;
    }
  }

  // Cut the global path
  global_path.erase(global_path.begin() + idx, global_path.end());
}

// ----------------------------------------------------------------------------

void DYNUS::filterStaticPushPoints(const Vec3f &latest_mean_push_point, const vec_Vecf<3> &global_path, bool found_static_push_points)
{

  // If there's new static push points, we cluster/filter them
  if (found_static_push_points)
  {

    // Double check
    if (!checkIfPointWithinMap(latest_mean_push_point) || !checkIfPointOccupied(latest_mean_push_point))
      return;

    // If the static_push_points_ is empty, we add the latest_mean_push_point and return
    if (static_push_points_.empty())
    {
      static_push_points_.push_back(latest_mean_push_point);
      return;
    }

    // static_push_points_ stores all the push points, so we check if any of them are close, we compute the mean and add it to static_push_points_, and remove the old points. If not, we just add the latest_mean_push_point
    bool is_close = false;
    for (size_t i = 0; i < static_push_points_.size(); i++)
    {
      // Check if the latest_mean_push_point is close to any of the points in static_push_points_
      if ((latest_mean_push_point - static_push_points_[i]).norm() < par_.static_push_clustering_threshold)
      {
        // Compute the mean and check if the mean point is not in free space, if it is, we update the point in static_push_points_
        Vec3f mean_point = (latest_mean_push_point + static_push_points_[i]) / 2.0;

        if (checkIfPointOccupied(mean_point) && checkIfPointWithinMap(mean_point))
        {
          static_push_points_[i] = mean_point;
          is_close = true;
          break;
        }
      }
    }

    // If the latest_mean_push_point is not close to any of the points in static_push_points_, we add it
    if (!is_close)
      static_push_points_.push_back(latest_mean_push_point);
  }

  // Also we go through the global path and check if any of the path points are close to points stored in static_push_points_, and if not, we remove them. This is important because we don't want to keep old points that are not close to the path
  vec_Vecf<3> new_static_push_points;

  // Loop through the static_push_points_
  for (size_t i = 0; i < static_push_points_.size(); i++)
  {
    bool is_close_to_path = false;
    for (size_t j = 0; j < global_path.size(); j++)
    {
      if ((static_push_points_[i] - global_path[j]).norm() < par_.max_dist_threshold_for_static_push)
      {
        is_close_to_path = true;
        break;
      }
    }

    // If the point is close to the path, we keep it
    if (is_close_to_path)
    {
      new_static_push_points.push_back(static_push_points_[i]);
    }
  }

  // Loop through the static push_points_ and remove the points that are not in occupied space
  static_push_points_.clear();
  for (size_t i = 0; i < new_static_push_points.size(); i++)
  {
    if (checkIfPointOccupied(new_static_push_points[i]) && checkIfPointWithinMap(new_static_push_points[i]))
    {
      static_push_points_.push_back(new_static_push_points[i]);
    }
  }
}

void DYNUS::staticPush(vec_Vecf<3> &global_path)
{
  // Get the static push vectors for each segment; if none found, return.
  Vecf<3> mean_push_point = Vecf<3>::Zero();
  bool found_static_push_points = dgp_manager_.computeStaticPushPoints(global_path, par_.dist_discretization, mean_push_point, par_.num_lookahead_global_path_for_push);

  // Filter the static push points.
  filterStaticPushPoints(mean_push_point, global_path, found_static_push_points);

  // If there are no static push points, return.
  if (static_push_points_.empty())
    return;

  // Process each global path point (skip the first point, which remains unchanged).
  for (size_t i = 1; i < global_path.size(); i++)
  {
    // Compute the cumulative push vector from all static push points.
    Eigen::Vector3d total_push_vector = Eigen::Vector3d::Zero();
    int valid_push_count = 0;

    for (size_t j = 0; j < static_push_points_.size(); j++)
    {
      Eigen::Vector3d push_vector = global_path[i] - static_push_points_[j];
      double dist = push_vector.norm();
      // Only include static push points that are within the allowed threshold.
      if (dist <= par_.max_dist_threshold_for_static_push)
      {
        total_push_vector += push_vector.normalized();
        valid_push_count++;
      }
    }

    // If no static push point contributed, skip this global path point.
    if (valid_push_count == 0)
      continue;

    // Normalize the cumulative push vector.
    total_push_vector.normalize();

    // Start with the maximum push force and decrease it individually if needed.
    double current_push = par_.push_force_static;
    double step = current_push / 10.0;
    Eigen::Vector3d candidate_point = global_path[i] + current_push * total_push_vector;

    // Decrease the push force until the candidate point is collision-free.
    while (current_push >= 0 &&
           (!checkIfPointFree(candidate_point) || !checkIfPointWithinMap(candidate_point)))
    {
      current_push -= step;
      candidate_point = global_path[i] + current_push * total_push_vector;
    }

    // Sometiems the free point a second ago becomes non-free in the next iteration, so we will decrease the push force one more step if it's not already 0
    if (current_push > 0)
    {
      current_push -= step;
      candidate_point = global_path[i] + current_push * total_push_vector;
    }

    // Update the global path point with the safe candidate.
    global_path[i] = candidate_point;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointOccupied(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointOccupied(point);
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointWithinMap(const Vec3f &point)
{
  // Check if the point is free
  return (point.x() >= par_.x_min && point.x() <= par_.x_max && point.y() >= par_.y_min && point.y() <= par_.y_max && point.z() >= par_.z_min && point.z() <= par_.z_max);
}

// ----------------------------------------------------------------------------

bool DYNUS::checkIfPointFree(const Vec3f &point)
{
  // Check if the point is free
  return dgp_manager_.checkIfPointFree(point);
}

void DYNUS::dynamicPush(vec_Vecf<3> &global_path, double current_time, std::vector<dynTraj> &local_trajs)
{

  // Initialize the adjusted path
  vec_Vec3f adjusted_path;

  // Keep the first point unchanged (initial condition)
  adjusted_path.push_back(global_path[0]);

  // Loop through the remaining points in the path
  for (int i = 1; i < global_path.size(); i++)
  {

    // Initialize the adjusted point
    Eigen::Vector3d point = global_path[i];
    Eigen::Vector3d total_repulsion = Eigen::Vector3d::Zero();

    // Accumulate repulsion from each obstacle
    for (const auto &traj : local_trajs)
    {
      // Compute push force for this obstacle
      double push_force = par_.dyn_obst_global_planner_push_k +
                          par_.dyn_obst_global_planner_push_cov_p_alpha * traj.ekf_cov_p.norm();

      // Evaluate obstacle's position at current time
      double traj_x = traj.pwp.eval(current_time).x();
      double traj_y = traj.pwp.eval(current_time).y();
      double traj_z = traj.pwp.eval(current_time).z();
      Eigen::Vector3d obstacle_position(traj_x, traj_y, traj_z);

      // Compute vector from obstacle to point
      Eigen::Vector3d vec_to_point = point - obstacle_position;
      double distance = vec_to_point.norm();

      if (distance < par_.collision_clearance)
      {
        // Avoid division by zero
        if (distance < 1e-6)
          distance = 1e-6;

        Eigen::Vector3d direction = vec_to_point.normalized();
        Eigen::Vector3d repulsion;

        if (distance < 3.0)
        {
          repulsion = par_.dyn_obst_replusion_max * direction;
        }
        else
        {
          repulsion = push_force / distance * direction;
        }

        // Accumulate the repulsion
        total_repulsion += repulsion;
      }
    } // end for each obstacle

    // Ensure the repulsion for each axis is within the maximum limit
    total_repulsion = total_repulsion.cwiseMax(-par_.dyn_obst_replusion_max)
                        .cwiseMin(par_.dyn_obst_replusion_max);

    // Apply the computed total repulsion to the point.
    Eigen::Vector3d pushed_point = point + total_repulsion;
    Eigen::Vector3d original_total_repulsion = total_repulsion;

    // Check for collisions and adjust the repulsion if needed.
    Eigen::Vector3d original_pushed_point = pushed_point;
    for (int k = 1; k <= 10; k++)
    {
      // if (checkIfPointFree(pushed_point) && checkIfPointWithinMap(pushed_point))
      if (!checkIfPointOccupied(pushed_point) && checkIfPointWithinMap(pushed_point))
        break; // Exit loop if collision-free

      // Gradually reduce the overall repulsion and update the pushed point.
      total_repulsion = original_total_repulsion * (1.0 - k / 10.0);
      pushed_point = point + total_repulsion;
    }

    adjusted_path.push_back(pushed_point);
  }

  // Update the global path with the adjusted path.
  global_path = adjusted_path;
}

// ----------------------------------------------------------------------------

void DYNUS::distributePath(vec_Vecf<3> &global_path, double distance_between_vertexes, int num_vertexes_to_keep)
{
  // createMoreVertexes in case dist between vertexes is too big
  dynus_utils::createMoreVertexes(global_path, distance_between_vertexes);

  // Force the global path to have less than max_value elements (par_.numP + 1 because the path contains points not segments)
  if (par_.use_min_dist_cutoff)
  {
    deleteVertexes(global_path, current_P_ + 1 + par_.num_lookahead_global_path_for_push, par_.min_dist_vertexes, par_.delete_too_close_points_in_global_path);
  }

  if (global_path.size() > num_vertexes_to_keep) // + 1 because path has one more points than the number of polytopes
  {
    global_path.erase(global_path.begin() + num_vertexes_to_keep, global_path.end());
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::getSafeCorridor(const vec_Vecf<3> &global_path, const vec_Vecf<3> &free_global_path, double current_time, double A_time, const state &local_G, const state &A)
{

  // Timer for computing the safe corridor
  MyTimer cvx_decomp_timer(true);

  // Check if there's any nearby dynamic trajectories
  bool check_nearby_dynamic_trajectories = checkNearbyTrajs(current_time);

  // Initialize decomposition result
  bool convex_decomp_result_whole = false;
  bool convex_decomp_result_safe = false;

  // Debug
  if (par_.debug_verbose)
    std::cout << "Convex decomposition" << std::endl;

  // Convex decomposition
  if (check_nearby_dynamic_trajectories) // if there are nearby dynamic trajectories
  {

    // Get the travel times
    std::vector<double> travel_times_whole = dynus_utils::getTravelTimes(global_path, A, par_.debug_verbose, v_max_3d_, a_max_3d_);
    convex_decomp_result_whole = dgp_manager_.cvxEllipsoidDecompTemporal(A, global_path, safe_corridor_polytopes_whole_, poly_out_whole_, A_time, travel_times_whole, false);
  }
  else // if there are no nearby dynamic trajectories
  {
    convex_decomp_result_whole = dgp_manager_.cvxEllipsoidDecomp(A, global_path, safe_corridor_polytopes_whole_, poly_out_whole_, false); // last argument indicates if we want to use this for safe paths
  }

  if (need_to_generate_safe_corridor_for_safe_path_)
  {
    // Convex decomposition
    if (check_nearby_dynamic_trajectories) // if there are nearby dynamic trajectories
    {

      // Get the travel times
      std::vector<double> travel_times_safe = dynus_utils::getTravelTimes(free_global_path, A, par_.debug_verbose, v_max_3d_, a_max_3d_);
      convex_decomp_result_safe = dgp_manager_.cvxEllipsoidDecompTemporal(A, free_global_path, safe_corridor_polytopes_safe_, poly_out_safe_, A_time, travel_times_safe, true);
    }
    else // if there are no nearby dynamic trajectories
    {
      convex_decomp_result_safe = dgp_manager_.cvxEllipsoidDecomp(A, free_global_path, safe_corridor_polytopes_safe_, poly_out_safe_, true); // last argument indicates if we want to use this for safe paths
    }
  }

  // Check if the convex decomposition failed
  if (!convex_decomp_result_whole || (need_to_generate_safe_corridor_for_safe_path_ && !convex_decomp_result_safe))
  {
    std::cout << bold << red << "Convex decomposition failed" << reset << std::endl;
    poly_out_whole_.clear();
    poly_out_safe_.clear();
    return false;
  }

  // If the safe_corridor_polytopes_ is empty, that means the convex decomposition failed
  if (safe_corridor_polytopes_whole_.empty() || (need_to_generate_safe_corridor_for_safe_path_ && safe_corridor_polytopes_safe_.empty()))
  {
    std::cout << bold << red << "safe_corridor_polytopes_whole_.size() == 0 or safe_corridor_polytopes_safe_.size() == 0" << reset << std::endl;
    poly_out_whole_.clear();
    poly_out_safe_.clear();
    return false;
  }

  // Get computation time [ms]
  cvx_decomp_time_ = cvx_decomp_timer.getElapsedMicros() / 1000.0;

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::computeMapSize(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos, const state &A, const state &G)
{

  // Increase the effective buffer size based on the number of DGP failures.
  double dynamic_buffer = par_.map_buffer + par_.failure_map_buffer_increment * dgp_failure_count_;

  // Increase the effective buffer size based on velocity.
  double dynamic_buffer_x = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(A.vel[0]);
  double dynamic_buffer_y = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(A.vel[1]);
  double dynamic_buffer_z = dynamic_buffer + par_.map_buffer_velocity_factor * std::abs(A.vel[2]);

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

  // // Compute the raw shift based on the agent's velocity.
  // Eigen::Vector3d velocity_shift = par_.center_shift_factor * A.vel;

  // // Clip the velocity shift to the maximum allowable shift.
  // Eigen::Vector3d clipped_shift = velocity_shift.cwiseMin(Eigen::Vector3d(par_.map_buffer, par_.map_buffer, par_.map_buffer)).cwiseMax(Eigen::Vector3d(-par_.map_buffer, -par_.map_buffer, -par_.map_buffer));

  // Compute the new map center by applying the clipped shift to the base center.
  // map_center_ = base_center + clipped_shift;
}

// ----------------------------------------------------------------------------

void DYNUS::getCurrentWds(double &wdx, double &wdy, double &wdz)
{
  wdx = wdx_;
  wdy = wdy_;
  wdz = wdz_;
}

// ----------------------------------------------------------------------------

bool DYNUS::checkPointWithinMap(const Eigen::Vector3d &point) const
{
  // Check if the point is within the map boundaries for each axis
  return (std::abs(point[0] - map_center_[0]) <= wdx_ / 2.0) && (std::abs(point[1] - map_center_[1]) <= wdy_ / 2.0) && (std::abs(point[2] - map_center_[2]) <= wdz_ / 2.0);
}

// ----------------------------------------------------------------------------

void DYNUS::updateMapRes()
{

  // Check if we need to adapt the map resolution
  if (replanning_failure_count_ < par_.map_res_adaptation_threshold)
  {
    // map_res_ = par_.res;
    return;
  }

  // Decrease the map resolution
  double factor = static_cast<double>(replanning_failure_count_ - par_.map_res_adaptation_threshold) / static_cast<double>(par_.map_res_adaptation_threshold);
  printf("factor: %f\n", factor);
  map_res_ = std::min(map_res_, std::max(par_.res - factor * par_.map_res_adaptation_decrement, par_.dynus_map_res_min));
  printf("map_res_: %f\n", map_res_);

  // Update map resolution in the DGP manager
  dgp_manager_.updateMapRes(map_res_);
}

// ----------------------------------------------------------------------------

void DYNUS::getStaticPushPoints(vec_Vecf<3> &static_push_points)
{
  static_push_points = static_push_points_;
}

// ----------------------------------------------------------------------------

void DYNUS::getPpoints(vec_Vecf<3> &p_points)
{
  p_points = p_points_;
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
  gurobi_computation_time_ = 0.0;
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
                         double &gurobi_computation_time,
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
  gurobi_computation_time = gurobi_computation_time_;
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

void DYNUS::retrievePwpToShare(PieceWisePol &pwp_to_share)
{
  pwp_to_share = pwp_to_share_;
}

// ----------------------------------------------------------------------------

void DYNUS::retrieveYawData(std::vector<double> &optimal_yaw_sequence,
                            std::vector<double> &yaw_control_points,
                            std::vector<double> &yaw_knots)
{
  optimal_yaw_sequence = optimal_yaw_sequence_;
  yaw_control_points = yaw_control_points_;
  yaw_knots = yaw_knots_;
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
 * @return [bool, bool, bool]: replanning_result, dgp_result, new_P_or_N_set.
 */
std::tuple<bool, bool, bool> DYNUS::replan(double last_replaning_computation_time, double current_time)
{
  /* -------------------- Housekeeping -------------------- */

  // if (par_.debug_verbose)
  MyTimer timer_housekeeping(true);

  // Reset Data
  resetData();

  // Check if we need to replan
  if (!checkReadyToReplan())
    return std::make_tuple(false, false, false);

  // P, N adaptation
  bool new_P_or_N_set = false;
  if (par_.p_n_mode == "dynus")
    adaptN(current_time, new_P_or_N_set);

  // Map resolution adaptation
  if (par_.use_map_res_adaptation)
    updateMapRes();

  // Get states we need
  state local_state, local_G_term, last_plan_state;
  getState(local_state);
  getGterm(local_G_term);
  getLastPlanState(last_plan_state);

  // Check if we need to replan based on the distance to the terminal goal
  if (!needReplan(local_state, local_G_term, last_plan_state))
    return std::make_tuple(false, false, new_P_or_N_set);

  if (par_.debug_verbose)
    std::cout << "Housekeeping: " << timer_housekeeping.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Global Planning -------------------- */

  MyTimer timer_global(true);
  vec_Vecf<3> global_path;
  if (!generateGlobalPath(global_path, current_time, last_replaning_computation_time, new_P_or_N_set))
  {
    if (par_.debug_verbose)
      std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, false, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Global Planning: " << timer_global.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Push Path -------------------- */

  MyTimer timer_push(true);
  vec_Vecf<3> free_global_path;
  need_to_generate_safe_corridor_for_safe_path_ = true; // initialize to true
  if (!pushPath(global_path, free_global_path, current_time))
  {
    if (par_.debug_verbose)
      std::cout << "Push Path: " << timer_push.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, false, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Push Path: " << timer_push.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  // If we are just visualizing the global path for debugging or visualization, return true
  if (par_.use_path_push_for_visualization)
    return std::make_tuple(true, true, new_P_or_N_set);

  /* -------------------- Local Trajectory Optimization -------------------- */

  MyTimer timer_local(true);
  if (!planLocalTrajectory(global_path, free_global_path, current_time))
  {
    if (par_.debug_verbose)
      std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Local Trajectory Optimization: " << timer_local.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Safety Check -------------------- */

  MyTimer timer_safety(true);
  if (!safetyCheck(pwp_to_share_, current_time))
  { // current_time is used to choose the right obstacle trajectory
    std::cout << bold << red << "Safety check failed" << reset << std::endl;
    replanning_failure_count_++;
    if (par_.debug_verbose)
      std::cout << "Safety Check: " << timer_safety.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Safety Check: " << timer_safety.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Safe Paths -------------------- */

  MyTimer timer_safe(true);
  std::vector<std::vector<state>> safe_path_points;
  if (!planSafePaths(safe_path_points))
  {
    if (par_.debug_verbose)
      std::cout << "Safe Paths: " << timer_safe.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Safe Paths: " << timer_safe.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Get Yaw -------------------- */

  MyTimer timer_yaw(true);
  // Get local trajectories
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);
  // Compute yaw if required
  if (par_.vehicle_type == "uav" && par_.use_yaw && local_trajs.size() > 0)
  {
    if (par_.debug_verbose)
      std::cout << "Compute yaw" << std::endl;
    // Get local A state
    state local_A;
    getA(local_A);
    // Compute yaw
    computeYaw(local_A, goal_setpoints_, pwp_to_share_, optimal_yaw_sequence_, yaw_control_points_, yaw_knots_);
  }
  if (par_.debug_verbose)
    std::cout << "Get Yaw: " << timer_yaw.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Append to Plan -------------------- */

  MyTimer timer_append(true);
  if (!appendToPlan(safe_path_points))
  {
    if (par_.debug_verbose)
      std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;
    return std::make_tuple(false, true, new_P_or_N_set);
  }
  if (par_.debug_verbose)
    std::cout << "Append to Plan: " << timer_append.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  /* -------------------- Final Housekeeping -------------------- */

  MyTimer timer_final(true);
  // For terminal goal mode with frontiers: if the terminal goal is close, then stop using frontiers.
  if (par_.flight_mode == "terminal_goal" && par_.use_frontiers && dgp_manager_.checkIfPointFree(local_G_term.pos))
    use_frontiers_as_G_ = false;

  if (par_.debug_verbose)
    std::cout << bold << green << "Replanning succeeded" << reset << std::endl;

  // Reset the replanning failure count
  replanning_failure_count_ = 0;
  if (par_.debug_verbose)
    std::cout << "Final Housekeeping: " << timer_final.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  return std::make_tuple(true, true, new_P_or_N_set);
}

// ----------------------------------------------------------------------------

bool DYNUS::generateGlobalPath(vec_Vecf<3> &global_path, double current_time, double last_replaning_computation_time, bool &new_P_or_N_set)
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
  if (par_.flight_mode == "terminal_goal")
  {
    computeG(local_A, local_G_term, par_.horizon);
  }
  else if (par_.flight_mode == "exploration" || use_frontiers_as_G_)
  {
    state frontier_G;
    getBestFrontier(frontier_G);

    mtx_G_.lock();
    G_.pos = dynus_utils::projectPointToSphere(local_A.pos, frontier_G.pos, par_.horizon);
    mtx_G_.unlock();
  }

  // Set up the DGP planner (since updateVmax() needs to be called after setupDGPPlanner, we use v_max_ from the last replan)
  dgp_manager_.setupDGPPlanner(par_.global_planner, par_.global_planner_verbose, map_res_, v_max_, par_.a_max, par_.j_max, par_.dgp_timeout_duration_ms);

  // Free start and goal if necessary
  if (par_.use_free_start)
    dgp_manager_.freeStart(local_A.pos, par_.free_start_factor);
  if (par_.use_free_goal)
    dgp_manager_.freeGoal(local_G.pos, par_.free_goal_factor);

  // Debug
  if (par_.debug_verbose)
    std::cout << "Solving DGP" << std::endl;

  // Adjust maximum velocity based on the unknown cells in the map
  // This needs to come after setupDGPPlanner
  if (par_.use_v_max_adaptation)
    updateVmax();

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
    local_G.pos[2] = 1.0;
  }

  // Solve DGP
  if (!dgp_manager_.solveDGP(local_A.pos, local_A.vel, local_G.pos, final_g_, par_.global_planner_huristic_weight, A_time, global_path))
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

  // Debug
  if (par_.debug_verbose)
    std::cout << "global_path.size(): " << global_path.size() << std::endl;

  // Get computation time
  dgp_manager_.getComputationTime(global_planning_time_, dgp_static_jps_time_, dgp_check_path_time_, dgp_dynamic_astar_time_, dgp_recover_path_time_);

  // Adjust max_dist_vertexes based on replanning_failure_count
  if (par_.use_max_dist_vertexes_adaptation)
    updateMaxDistVertexes();

  // Adjust P
  if (!adaptP(current_time, new_P_or_N_set))
  {
    replanning_failure_count_++;
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::pushPath(vec_Vecf<3> &global_path, vec_Vecf<3> &free_global_path, double current_time)
{

  // Sometimes this global path goes into unknown space, where there's known free space around it. We want to push the path into the free space
  dgp_manager_.pushPathIntoFreeSpace(global_path, free_global_path);

  // Assume we are using raw path, we first want to get rid of path that goes into unknown space
  // Check if the path is in free space
  if (!dgp_manager_.checkIfPathInFree(global_path, free_global_path) && par_.plan_only_in_free_space)
  {
    std::cout << bold << red << "Free global path only has one element (the starting pos)" << reset << std::endl;
    dgp_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // Check if the free_global_path is long enough -> original global_path has enough free points
  if (free_global_path.size() > current_P_ + 1 || !par_.plan_only_in_free_space)
  {
    // the original global path has enough free points
    need_to_generate_safe_corridor_for_safe_path_ = false;
  }

  // We want to push the original global path (with very small dist between points) but we don't want to push the points all the way to the end point (because that's a lot of points). So we estimate the end of local global path and cut the path there
  // shortenGlobalPath(global_path);
  // distributePath(global_path, max_dist_vertexes_, current_P_ + 1 + par_.num_lookahead_global_path_for_push);

  // debug visualization
  local_global_path_ = global_path;

  // For static environment we push the path from static obstacles
  staticPush(global_path);

  // Find local trajs
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // clean up the path
  dgp_manager_.cleanUpPath(global_path);
  distributePath(global_path, max_dist_vertexes_, current_P_ + 1 + par_.num_lookahead_global_path_for_push);
  if (need_to_generate_safe_corridor_for_safe_path_)
  {
    dgp_manager_.cleanUpPath(free_global_path);
    distributePath(free_global_path, max_dist_vertexes_, current_P_ + 1 + par_.num_lookahead_global_path_for_push);
  }

  // For dynamic environment we push the path from dynamic obstacles
  if (!local_trajs.empty())
  {
    dynamicPush(global_path, current_time, local_trajs);
    dgp_manager_.cleanUpPath(global_path);
    distributePath(global_path, max_dist_vertexes_, current_P_ + 1 + par_.num_lookahead_global_path_for_push);
  }


  // debug visualization
  local_global_path_after_push_ = global_path;

  // If we are just visualizing the global path for debugging or visualization, return true
  if (par_.use_path_push_for_visualization)
    return true;

  // if (!free_global_path.size() < 2)
  // {
  //   // sometimes the path ends very close to obstacles, so check if the point has enough clearance from obstacles, and if not, make the segment shorter
  //   // get the last point, the direction vector, and the distance to the last point
  //   Vec3f last_point = free_global_path.back();
  //   Vec3f second_last_point = free_global_path[free_global_path.size() - 2];
  //   Vec3f direction = last_point - second_last_point;
  //   direction.normalize();
  //   double distance = direction.norm();
  //   for (int i = 0; i < 10; i++)
  //   {
  //     if (!dgp_manager_.checkIfPointHasNonFreeNeighbour(last_point) && checkIfPointWithinMap(last_point))
  //       break; // Exit loop if collision-free

  //     // Gradually reduce the distance and update the last point
  //     distance -= 0.1;
  //     last_point = second_last_point + distance * direction;
  //   }

  //   // Update the global path with the adjusted last point
  //   free_global_path.back() = last_point;
  // }

  // Update global_path_ and free_global_path_
  mtx_global_path_.lock();
  global_path_ = global_path;
  mtx_global_path_.unlock();

  if (need_to_generate_safe_corridor_for_safe_path_)
    free_global_path_ = free_global_path;
  else
    free_global_path_.clear();

  // DGP succeeded
  dgp_failure_count_ = 0;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::planLocalTrajectory(vec_Vecf<3> &global_path, vec_Vecf<3> &free_global_path, double current_time)
{

  // Get local_A, local_G and A_time
  state local_A, local_G;
  double A_time;
  getA(local_A);
  getG(local_G);
  getA_time(A_time);

  // We will only need current_P_ polytopes from the global path
  if (global_path.size() > current_P_ + 1) // + 1 because path has one more points than the number of polytopes
  {
    global_path.erase(global_path.begin() + current_P_ + 1, global_path.end());
  }

  // We will only need current_P_ polytopes from the free global path
  if (need_to_generate_safe_corridor_for_safe_path_ && free_global_path.size() > current_P_ + 1) // + 1 because path has one more points than the number of polytopes
  {
    free_global_path.erase(free_global_path.begin() + current_P_ + 1, free_global_path.end());
  }

  // convex decomposition
  if (par_.optimization_type == "hard" || par_.optimization_type == "soft_with_check")
  {
    if (!getSafeCorridor(global_path, free_global_path, current_time, A_time, local_G, local_A))
    {
      replanning_failure_count_++;
      return false;
    }
  }

  // Get the last safe corridor polytope's mean point and make it Local_E
  state local_E;
  Vec3f mean_point;

  if (drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::GOAL_SEEN)
  {
    local_E = local_G;
  }
  else if (safe_corridor_polytopes_whole_.back().getMeanPoint(mean_point))
  {
    local_E.pos = mean_point;
  }
  else
  {
    local_E.pos = global_path.back();
  }

  // Update E_
  mtx_E_.lock();
  E_ = local_E;
  mtx_E_.unlock();

  // if using ground robot, we fix the z
  if (par_.vehicle_type != "uav")
  {
    local_A.pos[2] = 1.0;
    local_E.pos[2] = 1.0;
  }

  // Debug
  if (par_.debug_verbose)
    std::cout << "Gurobi Solver setup" << std::endl;

  /*
   * Parallelized Local Trajectory Optimization
   */

  // Reset whole trajectory planners to nominal state
  for (auto &solver : whole_traj_solver_ptrs_)
    solver->resetToNominalState();

  // Reset safe trajectory planners to nominal state
  for (auto &solver : safe_traj_solver_ptrs_)
    solver->resetToNominalState();

  // Create a cancellation flag.
  std::atomic<bool> cancel_flag(false);

  // One thread approach
  // bool optimization_succeeded = generateLocalTrajectory(
  //                                        local_A, local_E, A_time, global_path,
  //                                        gurobi_computation_time_, whole_traj_solver_ptrs_[0], previous_successful_factor_, cancel_flag);

  // Multi-threaded approach
  // Launch parallel tasks for each solver pointer/factor pair.
  // Each async task returns a tuple containing:
  //   - a bool indicating success,
  //   - its own goal setpoints, piecewise polynomial, control points,
  //   - computation time, and the factor used.

  // Get factors
  computeFactors();

  // Find initial_dt_ for all the solvers so I don't have to do it in each thread
  // Identify the axis with the largest displacement
  double max_disp = fabs(local_E.pos[0] - local_A.pos[0]);
  for (int i = 1; i < 3; i++)
  {
    double disp = fabs(local_E.pos[i] - local_A.pos[i]);
    if (disp > max_disp)
      max_disp = disp;
  }

  // Compute time based on maximum velocity for the best axis
  double initial_dt = max_disp / v_max_;

  std::vector<std::future<std::tuple<bool, double, double>>> futures;
  for (size_t i = 0; i < whole_traj_solver_ptrs_.size(); ++i)
  {
    double factor = factors_[i]; // corresponding factor for solver i

    futures.push_back(std::async(std::launch::async,
                                 [=, &cancel_flag]() -> std::tuple<bool, double, double>
                                 {
                                   try
                                   {
                                     // Check for cancellation early.
                                     if (cancel_flag.load())
                                     {
                                       return std::make_tuple(false, 0.0, factor);
                                     }
                                     double thread_gurobi_time = 0.0;
                                     bool result = generateLocalTrajectory(
                                         local_A, local_E, A_time, global_path,
                                         thread_gurobi_time, whole_traj_solver_ptrs_[i], factor, initial_dt, cancel_flag);

                                     return std::make_tuple(result, thread_gurobi_time, factor);
                                   }
                                   catch (const std::exception &ex)
                                   {
                                     std::cerr << "Exception in async task with factor " << factor << ": " << ex.what() << std::endl;
                                     return std::make_tuple(false, 0.0, factor);
                                   }
                                 }));
  }

  // Wait for any task to succeed.
  bool optimization_succeeded = false;
  double successful_factor = 0.0;
  for (size_t i = 0; i < futures.size(); ++i)
  {
    auto [result, thread_gurobi_time, thread_factor] = futures[i].get();
    if (result)
    {
      // One thread succeeded. Cancel all the other solver instances.
      for (size_t j = 0; j < whole_traj_solver_ptrs_.size(); ++j)
      {
        if (j != i) // Skip the successful one.
        {
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
      }

      // Get Results.
      whole_traj_solver_ptrs_[i]->fillGoalSetPoints();
      whole_traj_solver_ptrs_[i]->getGoalSetpoints(goal_setpoints_);
      whole_traj_solver_ptrs_[i]->getPieceWisePol(pwp_to_share_);
      whole_traj_solver_ptrs_[i]->getControlPoints(cps_); // Bezier control points

      optimization_succeeded = true;
      successful_factor = thread_factor;
      gurobi_computation_time_ = thread_gurobi_time;
      previous_successful_factor_ = thread_factor;
      break;
    }
  }

  if (!optimization_succeeded)
  {
    std::cout << bold << red << "Local trajectory optimization failed" << reset << std::endl;
    local_trajectory_failure_count_++;
    replanning_failure_count_++;
    return false;
  }

  // Single-threaded approach
  // whole_traj_solver_ptrs_[0]->fillGoalSetPoints();
  // whole_traj_solver_ptrs_[0]->getGoalSetpoints(goal_setpoints_);
  // whole_traj_solver_ptrs_[0]->getPieceWisePol(pwp_to_share_);
  // whole_traj_solver_ptrs_[0]->getControlPoints(cps_); // Bezier control points

  // Reset the local trajectory failure count
  local_trajectory_failure_count_ = 0;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::generateLocalTrajectory(const state &local_A, const state &local_E, double A_time,
                                    const vec_Vec3f &global_path,
                                    double &gurobi_computation_time, std::shared_ptr<SolverGurobi> &whole_traj_solver_ptr,
                                    double factor,
                                    double initial_dt,
                                    const std::atomic<bool> &cancel_flag)
{
  // Check cancellation before starting heavy work.
  if (cancel_flag.load())
  {
    std::cout << "Cancelled before starting generateLocalTrajectory" << std::endl;
    return false;
  }

  // Initialize the solver.
  whole_traj_solver_ptr->setX0(local_A); // Initial condition
  whole_traj_solver_ptr->setXf(local_E); // Final condition
  if (par_.optimization_type == "hard" || par_.optimization_type == "soft_with_check")
    whole_traj_solver_ptr->setPolytopes(safe_corridor_polytopes_whole_); // Safe corridor polytopes
  whole_traj_solver_ptr->setT0(A_time);                                  // Initial time
  whole_traj_solver_ptr->setInitialDt(initial_dt);                       // Initial time step
  if (current_N_ == 3)
    whole_traj_solver_ptr->setClosedFormInitialDt(0.3); // TODO maybe use travel time?

  // Check cancellation before running the solver.
  if (cancel_flag.load())
  {
    std::cout << "Cancelled before calling generateNewTrajectoryWithFactor" << std::endl;
    return false;
  }

  // Solve the optimization problem.
  bool gurobi_error_detected = false;
  bool gurobi_result = whole_traj_solver_ptr->generateNewTrajectoryWithFactor(gurobi_error_detected, global_path, gurobi_computation_time, factor);
  // bool gurobi_result = whole_traj_solver_ptr->generateNewTrajectoryWithIteration(gurobi_error_detected, global_path, gurobi_computation_time);

  // If a Gurobi error occurred, reset the solver and return.
  if (gurobi_error_detected)
  {
    std::cout << bold << red << "Gurobi error detected" << reset << std::endl;
    // whole_traj_solver_ptr.reset();
    whole_traj_solver_ptr = std::make_shared<SolverGurobi>();
    initializeSolver(current_N_, whole_traj_solver_ptr);
    return false;
  }

  // If no solution is found, return.
  if (!gurobi_result)
  {
    // std::cout << bold << red << "Local Optimization Failed" << reset << std::endl;
    gurobi_failed_counter_[current_N_]++;
    return false;
  }
  else
  {
    gurobi_failed_counter_[current_N_] = 0;
  }

  return true;
}

// ----------------------------------------------------------------------------

void DYNUS::computeFactors()
{

  // Initialize factors
  factors_.clear();
  double first_factor = par_.min_factor;

  // If the last plan failed, we start with the largest factor tried last time
  if (local_trajectory_failure_count_ > 0 && !should_reset_factors_)
  {
    first_factor = previous_largest_factor_;

    // if the last factor will be bigger than par_.max_factor, we start from par_.max_factor - par_.factor_increment * (par_.num_factors - 1)
    if (first_factor + par_.factor_increment * (par_.num_factors - 1) > par_.max_factor)
    {

      // Make sure we don't go over the maximum factor
      first_factor = par_.max_factor - par_.factor_increment * (par_.num_factors - 1);

      // if we reached the maximum factor, we start from the beginning
      should_reset_factors_ = true;
    }
  }
  else // Last replan was successful
  {
    should_reset_factors_ = false;
    // Determine the middle index (if num_factors is even, this uses the lower middle index)
    int mid_index = par_.num_factors / 2;

    // we use the previous successful factor as the middle element.
    first_factor = previous_successful_factor_ - mid_index * par_.factor_increment;
  }

  // first factor should be greater than par_.min_factor;
  if (first_factor < par_.min_factor)
    first_factor = par_.min_factor;

  // Generate factors
  for (int i = 0; i < par_.num_factors; i++)
    factors_.push_back(first_factor + i * par_.factor_increment);

  // Track previous largest factor
  previous_largest_factor_ = factors_.back();
}

// ----------------------------------------------------------------------------

bool DYNUS::planSafePaths(std::vector<std::vector<state>> &safe_path_points)
{
  MyTimer safe_paths_timer(true);

  std::map<int, std::vector<state>> safe_paths_in_whole_traj_indexed;
  if (!computeSafePaths(goal_setpoints_, safe_paths_in_whole_traj_indexed))
  {
    std::cout << bold << red << "Not enough safe paths computed" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // Get the safe paths computation time
  safe_paths_time_ = safe_paths_timer.getElapsedMicros() / 1000.0;

  // Print the time for computing safe paths
  if (par_.debug_verbose)
    std::cout << "Safe paths computation time: " << safe_paths_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  // TODO: add safe paths for visualization?

  MyTimer safe_path_points_timer(true);

  // Generate safe path points (which has the same size as goal_setpoints_) has a safe path if the goal setpoint at the same index has the safe path
  if (!generateSafePathPointsandincludeBestSafePath(goal_setpoints_, safe_path_points, safe_paths_in_whole_traj_indexed))
  {
    std::cout << bold << red << "Safe path points generation failed" << reset << std::endl;
    replanning_failure_count_++;
    return false;
  }

  // Print the time for computing safe path points
  if (par_.debug_verbose)
    std::cout << "Safe path points generation time: " << safe_path_points_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;

  return true;
}

// ----------------------------------------------------------------------------

bool DYNUS::appendToPlan(const std::vector<std::vector<state>> &safe_path_points)
{

  if (par_.debug_verbose)
    std::cout << "goal_setpoints_.size(): " << goal_setpoints_.size() << std::endl;

  // mutex lock
  mtx_plan_.lock();
  mtx_plan_safe_paths_.lock();

  // get the size of the plan and plan_safe_paths
  int plan_size = plan_.size();
  int plan_safe_paths_size = plan_safe_paths_.size();

  // make sure the size of the plan and plan_safe_paths are the same
  if (need_to_generate_safe_corridor_for_safe_path_ && plan_size != plan_safe_paths_size)
  {
    std::cout << bold << red << "plan_size != plan_safe_paths_size" << reset << std::endl;
    mtx_plan_.unlock();
    mtx_plan_safe_paths_.unlock();
    replanning_failure_count_++;
    return false;
  }

  // If the plan size is less than k_value_, which means we already passed point A, we cannot use this plan
  if ((plan_size - 1 - k_value_) < 0)
  {
    if (par_.debug_verbose)
      std::cout << bold << red << "(plan_size - 1 - k_value_) = " << (plan_size - 1 - k_value_) << " < 0" << reset << std::endl;
    mtx_plan_.unlock();
    mtx_plan_safe_paths_.unlock();
    replanning_failure_count_++;
    return false;
  }
  else // If the plan size is greater than k_value_, which means we haven't passed point A yet, we can use this plan
  {
    plan_.erase(plan_.end() - k_value_ - 1, plan_.end()); // this deletes also the initial condition
    plan_.insert(plan_.end(), goal_setpoints_.begin(), goal_setpoints_.end());
    plan_safe_paths_.erase(plan_safe_paths_.end() - k_value_ - 1, plan_safe_paths_.end());
    plan_safe_paths_.insert(plan_safe_paths_.end(), safe_path_points.begin(), safe_path_points.end());
  }

  // mutex unlock
  mtx_plan_safe_paths_.unlock();
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
 * @brief Compute safe paths using closed-form solution.
 * @param std::vector<state> &goal_setpoints: used to discretize the path.
 * @param std::vector<std::vector<state>> &safe_paths: Output safe paths.
 */
bool DYNUS::computeSafePaths(const std::vector<state> &goal_setpoints, std::map<int, std::vector<state>> &safe_paths)
{

  // Initialize the safe paths
  safe_paths.clear();

  // Compute H point in the goal setpoints
  int h_point_idx = findHPointIndex(goal_setpoints);

  // Get parameters
  int num_increment_goal_setpoints = (int)h_point_idx / par_.num_safe_paths;

  // Sanity check
  if (num_increment_goal_setpoints == 0)
  {
    std::cout << bold << red << "H point is very close" << reset << std::endl;
    return false;
  }

// Loop through discretized goal setpoints
// Note disc_idx starts from num_increment_goal_setpoints because we don't need to compute the safe path for the first goal setpoint
// Also note disc_idx <= h_point_idx because we can compute the safe path for the last goal setpoint
// for (int disc_idx = num_increment_goal_setpoints; disc_idx <= h_point_idx; disc_idx += num_increment_goal_setpoints)
#pragma omp parallel for
  for (int idx = 0; idx < par_.num_safe_paths; idx++)
  {

    // Get safe_traj_solver_ptr
    std::shared_ptr<SolverGurobi> safe_traj_solver_ptr = safe_traj_solver_ptrs_[idx];

    // Get disc_idx
    int disc_idx = idx * num_increment_goal_setpoints;

    // Get the current goal setpoint
    state current_goal_setpoint = goal_setpoints[disc_idx];

    // Compute the safe paths
    std::vector<state> current_safe_path;

    MyTimer safe_path_timer(true);

    // Generate safe paths with closed form
    if (generateSafePathsWithClosedForm(current_goal_setpoint, current_safe_path, safe_traj_solver_ptr))
    {
#pragma omp critical
      {
        safe_paths[disc_idx] = current_safe_path;
      }
    }

    // Print the time for computing the safe path
    if (par_.debug_verbose)
      std::cout << "disc_idx: " << disc_idx << ", Safe path computation time: " << safe_path_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;
  }

  // If there's enough safe paths, return true. But if we planning in only free space, we don't necessarily need to have enough safe paths
  if (!par_.plan_only_in_free_space)
  {
    if (safe_paths.size() >= par_.min_num_safe_paths)
      return true;
    else
      return false;
  }
  else
  {
    return true;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::generateSafePathPointsandincludeBestSafePath(std::vector<state> &goal_setpoints, std::vector<std::vector<state>> &safe_path_points, std::map<int, std::vector<state>> &safe_paths)
{

  // Check if the safe paths is empty
  if (safe_paths.empty())
  {
    return false;
  }

  if (!need_to_generate_safe_corridor_for_safe_path_ || !par_.plan_only_in_free_space) // If planning in unknown space, we don't need to replace the last bit with safe paths
  {
    // Initiliaze safe_path_points with zeros with the same size as goal_setpoints
    safe_path_points.clear();
    safe_path_points.resize(goal_setpoints.size(), std::vector<state>());

    // Retrieve the safe paths
    for (auto it = safe_paths.begin(); it != safe_paths.end(); it++)
    {
      // Append the safe path to safe_path_points
      safe_path_points[it->first] = it->second;
    }

    // Sanity check
    assert(goal_setpoints.size() == safe_path_points.size());

    return true;
  }
  else // If planning in unknown space, we need to replace the last bit with safe paths
  {
    // Initiliaze safe_path_points with zeros with the same size as goal_setpoints
    safe_path_points.clear();
    safe_path_points.resize(goal_setpoints.size(), std::vector<state>());

    // Get the last goal setpoint
    state last_goal_setpoint = goal_setpoints.back();

    // Get the best safe path by looping through the safe paths and find the one that ends closest to the goal.
    double min_distance = std::numeric_limits<double>::max();
    int best_safe_path_idx = -1;
    for (auto it = safe_paths.begin(); it != safe_paths.end(); it++)
    {

      // Append the safe path to safe_path_points
      safe_path_points[it->first] = it->second;

      // Get the last state of the safe path
      state last_safe_state = it->second.back();

      // Compute the distance between the last state of the safe path and the last goal setpoint
      double distance = (last_safe_state.pos - last_goal_setpoint.pos).norm();

      // Update the minimum distance and the best safe path
      if (distance < min_distance)
      {
        min_distance = distance;
        best_safe_path_idx = it->first;
      }
    }

    // Erase anything after best_safe_path_idx
    goal_setpoints.erase(goal_setpoints.begin() + best_safe_path_idx, goal_setpoints.end());
    safe_path_points.erase(safe_path_points.begin() + best_safe_path_idx, safe_path_points.end());

    // Append the best safe path to the goal setpoints
    std::vector<state> best_safe_path = safe_paths[best_safe_path_idx];

    // Append the best safe path to the goal setpoints for the plan
    goal_setpoints.insert(goal_setpoints.end(), best_safe_path.begin(), best_safe_path.end());

    // Add empty vectors to safe_path_points to match the size of goal_setpoints
    for (int i = 0; i < best_safe_path.size(); i++)
    {
      safe_path_points.push_back(std::vector<state>());
    }

    // Sanity check
    assert(goal_setpoints.size() == safe_path_points.size());

    return true;
  }
}

// ----------------------------------------------------------------------------

bool DYNUS::generateSafePathsWithClosedForm(const state &start_goal_setpoint, std::vector<state> &safe_path, const std::shared_ptr<SolverGurobi> &safe_traj_solver_ptr)
{

  // Clear the safe path
  safe_path.clear();

  // Generate potential goal for start_goal_setpoint
  state potential_goal;
  double closed_form_dt;
  generatePotentialGoal(start_goal_setpoint, potential_goal, closed_form_dt);

  // Check if the potential goal is in safe space
  if (!dgp_manager_.checkIfPointFree(potential_goal.pos))
  {
    if (par_.debug_verbose)
    {
      std::cout << "start_goal_setpoint.pos: " << start_goal_setpoint.pos.transpose() << std::endl;
      std::cout << "potential_goal.pos: " << potential_goal.pos.transpose() << std::endl;
      std::cout << bold << red << "Potential goal is not in free space" << reset << std::endl;
    }
    return false;
  }

  // If the goal is in safe space, compute the safe path
  // Compute the safe path
  return solveClosedForm(start_goal_setpoint, potential_goal, safe_path, safe_traj_solver_ptr, closed_form_dt);
}

// ----------------------------------------------------------------------------

bool DYNUS::solveClosedForm(const state &start, const state &goal, std::vector<state> &safe_path, const std::shared_ptr<SolverGurobi> &safe_traj_solver_ptr, double closed_form_dt)
{

  // If the goal is in safe space, compute the safe path
  // Compute the safe path
  safe_traj_solver_ptr->setX0(start);
  safe_traj_solver_ptr->setXf(goal);
  if (need_to_generate_safe_corridor_for_safe_path_)
    safe_traj_solver_ptr->setPolytopes(safe_corridor_polytopes_safe_, true);
  else
    safe_traj_solver_ptr->setPolytopes(safe_corridor_polytopes_whole_, true);
  safe_traj_solver_ptr->setT0(start.t);
  safe_traj_solver_ptr->setClosedFormInitialDt(closed_form_dt);

  // Debug
  // printf("start.pos: %f, %f, %f\n", start.pos[0], start.pos[1], start.pos[2]);
  // printf("goal.pos: %f, %f, %f\n", goal.pos[0], goal.pos[1], goal.pos[2]);
  // printf("start.t: %f\n", start.t);

  // Solve the optimization problem
  if (!safe_traj_solver_ptr->findClosedFormSolution())
    return false;

  // Get the safe path
  safe_traj_solver_ptr->fillGoalSetPoints();
  safe_traj_solver_ptr->getGoalSetpoints(safe_path);

  // TODO: Now we have two sets of trajectory - the first half is the whole trajectory, and the second half is the safe trajectory - I need to make sure that I publish both of them with time stamps
  // TODO: I also need to publish pwp when future collisions are detected and switch to safe trajectory
  // safe_traj_solver_ptr->getPieceWisePol(pwp);

  // Check if the safe path is valid
  if (safe_path.size() > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// ----------------------------------------------------------------------------

void DYNUS::generatePotentialGoal(const state &start_goal_setpoint, state &potential_goal, double &closed_form_dt)
{

  // Initialize the potential goal with zeros
  potential_goal.setZero();

  // Option 1: Using constant jerk model - this could generate very optimistic potential goals
  // // Using constant jerk model, compute minimum time until it stops given the current state
  // double Tx = computeMinimumTimeUntilStop(start_goal_setpoint.vel[0], start_goal_setpoint.accel[0], start_goal_setpoint.jerk[0]);
  // double Ty = computeMinimumTimeUntilStop(start_goal_setpoint.vel[1], start_goal_setpoint.accel[1], start_goal_setpoint.jerk[1]);
  // double Tz = computeMinimumTimeUntilStop(start_goal_setpoint.vel[2], start_goal_setpoint.accel[2], start_goal_setpoint.jerk[2]);

  // // Compute the minimum displacement until it stops (v0 * T + 0.5 * a0 * T^2 - 1/6 * j * T^3)
  // double delta_x = start_goal_setpoint.vel[0] * Tx + 0.5 * start_goal_setpoint.accel[0] * Tx * Tx - 1.0 / 6.0 * par_.j_max * Tx * Tx * Tx;
  // double delta_y = start_goal_setpoint.vel[1] * Ty + 0.5 * start_goal_setpoint.accel[1] * Ty * Ty - 1.0 / 6.0 * par_.j_max * Ty * Ty * Ty;
  // double delta_z = start_goal_setpoint.vel[2] * Tz + 0.5 * start_goal_setpoint.accel[2] * Tz * Tz - 1.0 / 6.0 * par_.j_max * Tz * Tz * Tz;

  // potential_goal.pos[0] = start_goal_setpoint.pos[0] + delta_x;
  // potential_goal.pos[1] = start_goal_setpoint.pos[1] + delta_y;
  // potential_goal.pos[2] = start_goal_setpoint.pos[2] + delta_z;

  // Option 2: given a distance (linearly increased by the velocity), define a stop point in the direction of the current velocity

  // Linearly increase safe_path_distance between min_safe_path_distance and max_safe_path_distance
  double safe_path_distance = par_.min_safe_path_distance + (par_.max_safe_path_distance - par_.min_safe_path_distance) * (start_goal_setpoint.vel.norm() / par_.v_max); // Use par_.v_max instead of v_max_ because this is for safe plan and we should use the maximum possible v_max - should not worry about uncertainty.

  // Estimate the time to reach the potential goal
  closed_form_dt = safe_path_distance / start_goal_setpoint.vel.norm();

  // Update the potential goal
  potential_goal.pos = start_goal_setpoint.pos + safe_path_distance * start_goal_setpoint.vel.normalized();
}

// ----------------------------------------------------------------------------

double DYNUS::computeMinimumTimeUntilStop(double v0, double a0, double j)
{
  // Compute the minimum time until it stops
  // T = (a0 + sqrt(a0^2 + 2 * j * v0)) / j
  return (a0 + std::sqrt(a0 * a0 + 2 * j * v0)) / j;
}

// ----------------------------------------------------------------------------

int DYNUS::findHPointIndex(const std::vector<state> &goal_setpoints)
{

  // Initialize the index
  int h_point_idx = 0;

  // Iterate through the goal setpoints
  for (int i = 0; i < goal_setpoints.size(); i += 10)
  {

    // Get the current goal setpoint
    state current_goal = goal_setpoints[i];

    // Check if the current goal setpoint is safe
    if (dgp_manager_.checkIfPointFree(current_goal.pos))
    {
      h_point_idx = i;
    }
    else
    {
      break;
    }
  }

  return h_point_idx;
}

// ----------------------------------------------------------------------------

void DYNUS::updateVmax()
{
  // Uncertainty from unknown cells
  int num_unknown_cells = dgp_manager_.countUnknownCells(); // Get number of unknown cells
  int total_num_cells = dgp_manager_.getTotalNumCells();    // Get total number of cells

  // Sanity check
  if (total_num_cells == 0)
    return;

  // --- Update the adaptation exponent p based on dynamic obstacles ---
  // Number of dynamic obstacles (assumed available from local_trajs)
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);
  int num_dynamic_obstacles = local_trajs.size();

  double desired_p;
  if (num_dynamic_obstacles >= par_.v_max_adaptation_obstacle_threshold)
  {
    desired_p = par_.v_max_adaptation_p_min;
  }
  else
  {
    // Compute the ratio of dynamic obstacles relative to the threshold.
    double ratio_obs = static_cast<double>(num_dynamic_obstacles) / par_.v_max_adaptation_obstacle_threshold;

    // Interpolate p in the log-domain.
    double log_p_max = std::log(par_.v_max_adaptation_p_max);
    double log_p_min = std::log(par_.v_max_adaptation_p_min);
    double log_desired_p = log_p_max + (log_p_min - log_p_max) * ratio_obs;
    desired_p = std::exp(log_desired_p);
  }

  // --- Compute the maximum velocity based on uncertainty and the updated exponent ---
  double ratio = static_cast<double>(num_unknown_cells) / total_num_cells;
  double v_max = par_.v_max_adaptation_v_min + (par_.v_max - par_.v_max_adaptation_v_min) *
                                                   (1 - std::pow(ratio, desired_p));

  // Update v_max
  setVmax(v_max);
}

// ----------------------------------------------------------------------------

void DYNUS::updateMaxDistVertexes()
{
  // Number of dynamic obstacles (assumed available from local_trajs)
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);
  int num_dynamic_obstacles = local_trajs.size();

  if (num_dynamic_obstacles >= par_.max_dist_vertexes_adaptation_obstacle_threshold)
  {
    max_dist_vertexes_ = par_.min_dist_vertexes;
  }
  else
  {
    // Compute the ratio of dynamic obstacles relative to the threshold.
    double ratio_obs = static_cast<double>(num_dynamic_obstacles) / par_.dynamic_obsts_total_max_num;
    max_dist_vertexes_ = par_.min_dist_vertexes + (par_.max_dist_vertexes - par_.min_dist_vertexes) * (1 - ratio_obs);
  }

  // If the replanning failed, we decrease the max_dist_vertexes_
  if (replanning_failure_count_ > par_.max_dist_vertexes_adaptation_threshold)
  {
    double decrease = replanning_failure_count_ / par_.max_dist_vertexes_adaptation_threshold;
    max_dist_vertexes_ = std::max(max_dist_vertexes_ - decrease, par_.min_dist_vertexes);
  }

  // Update it in DGP manager
  dgp_manager_.updateMaxDistVertexes(max_dist_vertexes_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the risk associated with planning horizon based on the uncertainty.
 * @return double: Risk value.
 */
double DYNUS::computeRisk(double current_time, const Eigen::Vector3d &A_pos)
{

  // Uncertainty from unknown cells
  int num_unknown_cells = dgp_manager_.countUnknownCells(); // Get number of unknown cells
  int total_num_cells = dgp_manager_.getTotalNumCells();    // Get total number of cells

  // Uncertainty from dynamic obstacles
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // If there are no dynamic obstacles, we just return the uncertainty from unknown cells
  if (local_trajs.empty())
    return num_unknown_cells / total_num_cells;

  // Get the closest dynamic obstacle's distance
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto &traj : local_trajs)
  {
    double distance = (traj.pwp.eval(current_time) - A_pos).norm();
    if (distance < min_distance)
      min_distance = distance;
  }

  // Compute exponential decay risk (when the obst is at min_allowable_distance, it will be 1.0)
  double min_allowable_distance = 2 * par_.drone_bbox[0]; // 2 is just a safety factor
  double risk_prox = std::min(std::exp(-(min_distance - min_allowable_distance)), 1.0);

  // Compute the risk associated with dynamic obstacles
  double risk_dyn = par_.dynamic_obsts_prox_weight * risk_prox + par_.dynamic_obsts_count_weight * std::min((double)local_trajs.size() / par_.dynamic_obsts_total_max_num, 1.0);

  // Compute the total risk
  double risk = par_.unknown_cells_weight * (num_unknown_cells / total_num_cells) + par_.dynamic_obsts_weight * risk_dyn;

  return risk;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the planning horizon
 * @return double: Planning horizon.
 */
double DYNUS::computeHorizon(double current_time, const Eigen::Vector3d &A_pos)
{

  // Get the risk associated with the planning horizon
  double risk = computeRisk(current_time, A_pos);

  // Compute the planning horizon
  double horizon = par_.max_horizon - (par_.max_horizon - par_.min_horizon) * risk;

  return horizon;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the optimal yaw sequence.
 * @param const state &A: Starting point of replanning.
 * @param std::vector<state> &goal_setpoints: Global path.
 * @param PieceWisePol &pwp: Piecewise polynomial trajectory.
 * @param std::vector<double> &optimal_yaw_sequence: Output optimal yaw sequence.
 * @param std::vector<double> &yaw_control_points: Output yaw control points.
 * @param std::vector<double> &yaw_knots: Output yaw knot vector.
 */
void DYNUS::computeYaw(const state &A, std::vector<state> &goal_setpoints, PieceWisePol &pwp,
                       std::vector<double> &optimal_yaw_sequence, std::vector<double> &yaw_control_points,
                       std::vector<double> &yaw_knots)
{

  //// Get tracking yaw

  // Get initial_pos, initial_yaw, and time_horizon
  Eigen::Vector3d initial_pos = A.pos;
  double initial_yaw = A.yaw;
  double time_horizon = pwp.times.back() - pwp.times.front();

  // Get local trajs_
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // Get G_
  mtx_G_.lock();
  state G = G_;
  mtx_G_.unlock();

  if (par_.debug_verbose)
    std::cout << "Compute optimal yaw sequence" << std::endl;

  MyTimer yaw_sequence_timer(true);

  // Find optimal yaw sequence
  optimal_yaw_sequence = yaw_solver_.findOptimalYaw(pwp, local_trajs, initial_pos, initial_yaw, G.yaw, time_horizon);

  // Get computation time [ms]
  yaw_sequence_time_ = yaw_sequence_timer.getElapsedMicros() / 1000.0;

  if (par_.debug_verbose)
  {
    std::cout << "optimal_yaw_sequence.size(): " << optimal_yaw_sequence.size() << std::endl;
    for (int i = 0; i < optimal_yaw_sequence.size(); i++)
    {
      std::cout << "optimal_yaw_sequence " << i << ": " << optimal_yaw_sequence[i] << std::endl;
    }
  }

  // For computational stability, shift pwp.times so that it starts from 0.0
  std::vector<double> times_shifted;
  for (int i = 0; i < pwp.times.size(); i++)
  {
    times_shifted.push_back(pwp.times[i] - pwp.times.front());
  }

  if (par_.debug_verbose)
    std::cout << "Bspline fitting" << std::endl;

  MyTimer yaw_fitting_timer(true);

  // Bspline fitting
  yaw_solver_.BsplineFitting(goal_setpoints, times_shifted, optimal_yaw_sequence, yaw_control_points, yaw_knots);

  // Get computation time [ms]
  yaw_fitting_time_ = yaw_fitting_timer.getElapsedMicros() / 1000.0;
}

// ----------------------------------------------------------------------------

/**
 * @brief Check if the newly optimized trajectory is safe
 * @param pwp PieceWisePol
 * @param current_time double
 */
bool DYNUS::safetyCheck(PieceWisePol &pwp, double current_time)
{

  MyTimer safety_check_timer(true);

  // Initialize the flag
  bool result = true;

  // Loop through trajs_
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  for (auto traj : local_trajs)
  {

    // identify the start time and end time
    double start_time = std::min(pwp.times.front(), traj.pwp.times.front());
    double end_time = std::max(pwp.times.back(), traj.pwp.times.back());

    // Loop through the trajectory
    for (double t = pwp.times.front(); t < pwp.times.back(); t += par_.safety_check_dt)
    {

      // Get the position
      Eigen::Vector3d agent_pos = pwp.eval(t);

      // Get the position of the trajectory
      Eigen::Vector3d traj_pos = traj.pwp.eval(t);

      // Check if the distance is less than the safety distance
      if ((abs(agent_pos[0] - traj_pos[0]) < (par_.drone_bbox[0] + traj.bbox[0]) / 2.0) && (abs(agent_pos[1] - traj_pos[1]) < (par_.drone_bbox[1] + traj.bbox[1]) / 2.0) && (abs(agent_pos[2] - traj_pos[2]) < (par_.drone_bbox[2] + traj.bbox[2]) / 2.0))
      {
        result = false;
        break;
      }

    } // end of for loop for t

  } // end of for loop for trajs_

  // Get the safety check time
  safety_check_time_ = safety_check_timer.getElapsedMicros() / 1000.0;

  // Return the result
  return result;
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the static map.
 * @param vec_Vecf<3> &occupied_cells: Output map for occupied cells.
 */
void DYNUS::getMap(vec_Vecf<3> &occupied_cells)
{
  // Get the occupied cells
  dgp_manager_.getOccupiedCells(occupied_cells);
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrieves the dynamic map (occupied, free, and unknown cells).
 * @param vec_Vecf<3> &occupied_cells: Output map for dynamic occupied cells.
 * @param vec_Vecf<3> &free_cells: Output map for dynamic free cells.
 * @param vec_Vecf<3> &unknown_cells: Output container for unknown cells.
 * @param double current_time: Current timestamp.
 */
void DYNUS::getDynamicMap(vec_Vecf<3> &occupied_cells, vec_Vecf<3> &free_cells, vec_Vecf<3> &unknown_cells, double current_time)
{
  // Get the occupied cells
  dgp_manager_.getDynamicOccupiedCellsForVis(occupied_cells, free_cells, unknown_cells, current_time);
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
 * @param std::vector<dynTraj> &trajs: Output trajs_
 */
void DYNUS::getTrajs(std::vector<dynTraj> &trajs)
{
  mtx_trajs_.lock();
  trajs = trajs_;
  mtx_trajs_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Manages communication delay and inflates the bounding box.
 * @param dynTraj &new_traj: Trajectory to update.
 * @param double current_time: Current timestamp.
 */
void DYNUS::manageCommDelayAndInflateBBox(dynTraj &new_traj, double current_time)
{

  if (comm_delay_map_.find(new_traj.id) == comm_delay_map_.end())
  {
    // If it's a new agent, just add it
    comm_delay_map_[new_traj.id] = new_traj.communication_delay;
  }
  else
  {
    // Filter and update the communication delay
    comm_delay_map_[new_traj.id] = par_.comm_delay_filter_alpha * new_traj.communication_delay + (1 - par_.comm_delay_filter_alpha) * comm_delay_map_[new_traj.id];
  }

  // Get the inflation
  double comm_delay_inflation = std::max(0.0, (std::min(par_.comm_delay_inflation_alpha * new_traj.pwp.velocity(current_time).norm() * comm_delay_map_[new_traj.id], par_.comm_delay_inflation_max)));

  // Inflate the bbox
  new_traj.bbox[0] += comm_delay_inflation;
  new_traj.bbox[1] += comm_delay_inflation;
  new_traj.bbox[2] += comm_delay_inflation;
}

// ----------------------------------------------------------------------------

/**
 * @brief Cleanup the communication delay map based on trajs_
 * @note This function assmues that mtx_trajs_ is locked
 */
void DYNUS::cleranUpCommDelayMap()
{

  // Go thru trajs_ first and get keys to retain
  std::vector<int> keys_to_retain;
  for (auto traj : trajs_)
  {
    keys_to_retain.push_back(traj.id);
  }

  // Check if comm_delay_map_'s key is in keys_to_retain - if not delete it
  std::vector<int> keys_to_delete;
  for (auto it = comm_delay_map_.begin(); it != comm_delay_map_.end(); it++)
  {
    if (std::find(keys_to_retain.begin(), keys_to_retain.end(), it->first) == keys_to_retain.end())
    {
      keys_to_delete.push_back(it->first);
    }
  }

  // Delete the keys
  for (auto key : keys_to_delete)
  {
    comm_delay_map_.erase(key);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Cleans up old trajectories.
 * @param double current_time: Current timestamp.
 */
void DYNUS::cleanUpOldTrajs(double current_time)
{
  // If trajs_ has a trajectory that is old enough, delete it
  mtx_trajs_.lock();

  if (trajs_.empty())
  {
    mtx_trajs_.unlock();
    return;
  }

  for (int i = 0; i < trajs_.size(); i++)
  {
    if (current_time - trajs_[i].time_received > par_.traj_lifetime)
    {
      trajs_.erase(trajs_.begin() + i);
    }
  }
  mtx_trajs_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Adds or updates a trajectory.
 * @param dynTraj new_traj: New trajectory to add.
 * @param double current_time: Current timestamp.
 */
void DYNUS::addTraj(dynTraj new_traj, double current_time)
{

  // Check if the trajectory is within the map
  if (!checkPointWithinMap(new_traj.pwp.eval(new_traj.pwp.times.front())) && !checkPointWithinMap(new_traj.pwp.eval(new_traj.pwp.times.back())))
    return;

  // Check if the trajectory is close enough to the drone
  if ((new_traj.pwp.eval(new_traj.pwp.times.front()) - state_.pos).norm() > par_.collision_clearance && (new_traj.pwp.eval(new_traj.pwp.times.back()) - state_.pos).norm() > par_.collision_clearance)
    return;

  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // If we track communication delays, we compute the communication delay associated with this id and inflate bbox
  if (new_traj.is_agent && par_.use_comm_delay_inflation)
    manageCommDelayAndInflateBBox(new_traj, current_time);

  // Check if there's a trajectory with the same id
  // If there is, replace it. If there isn't, add it
  std::vector<dynTraj>::iterator obs_ptr = std::find_if(local_trajs.begin(), local_trajs.end(), [=](const dynTraj &traj)
                                                        { return traj.id == new_traj.id; });

  if (obs_ptr != std::end(local_trajs)) // if the object already exists, substitute its trajectory
  {
    *obs_ptr = new_traj;
  }
  else // if it doesn't exist, add it to local_trajs
  {
    // Add control points to the trajectory
    new_traj.control_points = dynus_utils::convertCoefficients2ControlPoints(new_traj.pwp, A_rest_pos_basis_inverse_);
    local_trajs.push_back(new_traj);
  }

  if (par_.use_comm_delay_inflation)
    cleranUpCommDelayMap();

  // Update trajs_
  mtx_trajs_.lock();
  trajs_ = local_trajs;
  mtx_trajs_.unlock();
}

// ----------------------------------------------------------------------------

/**
 * @brief Updates trajectories for Tmap.
 */
void DYNUS::updateTrajsForTmap()
{

  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // Update map_util_'s local_trajs
  dgp_manager_.updateTrajs(local_trajs);
}

// ----------------------------------------------------------------------------

/**
 * @brief Shares the static map (ROI) with other agents.
 * @param std::unordered_map<int, std::shared_ptr<octomap::TimedOcTree>> &roi_octrees: Output map of ROI octrees.
 * @param double current_time: Current timestamp.
 */
void DYNUS::shareStaticMap(std::unordered_map<int, std::shared_ptr<octomap::TimedOcTree>> &roi_octrees, double current_time)
{

  // Go thru trajs_ and check if we have something useful to share for each agent
  mtx_trajs_.lock();

  if (trajs_.empty())
  {
    mtx_trajs_.unlock();
    return;
  }

  std::vector<dynTraj> local_trajs = trajs_;
  mtx_trajs_.unlock();

  // Filter the local trajs to only include the agents
  std::vector<dynTraj> agents_trajs;
  for (auto traj : local_trajs)
  {
    if (traj.is_agent)
    {
      agents_trajs.push_back(traj);
    }
  }

  // Get local octrees
  mtx_d435_octree_.lock();
  std::shared_ptr<octomap::TimedOcTree> d435_octree = std::make_shared<octomap::TimedOcTree>(*d435_octree_);
  mtx_d435_octree_.unlock();

  mtx_mid360_octree_.lock();
  std::shared_ptr<octomap::TimedOcTree> mid360_octree = std::make_shared<octomap::TimedOcTree>(*mid360_octree_);
  mtx_mid360_octree_.unlock();

  // TODO: share the ROI already shared by the other agents?

  // Check for each id
  for (const auto &agents_traj : agents_trajs)
  {

    // Initialize the roi_octree
    std::shared_ptr<octomap::TimedOcTree> roi_octree;
    roi_octree = std::make_shared<octomap::TimedOcTree>(d435_octree->getResolution(), d435_octree->getDecayDuration()); // TODO: assume that d435_octree and mid360_octree have the same resolution

    // agent A's current position and the end position
    Eigen::Vector3d agent_A_current_pos = agents_traj.pwp.eval(current_time);
    Eigen::Vector3d agent_A_end_pos = agents_traj.goal;

    // convert current_pos and goal_pos to octomap::point3d
    octomap::point3d current_pos_octree(agent_A_current_pos[0], agent_A_current_pos[1], agent_A_current_pos[2]);
    octomap::point3d end_pos_octree(agent_A_end_pos[0], agent_A_end_pos[1], agent_A_end_pos[2]);

    // Extract the region of interest from both d435 and mid360 octrees
    extractROI(roi_octree, d435_octree, current_pos_octree, end_pos_octree);
    extractROI(roi_octree, mid360_octree, current_pos_octree, end_pos_octree);

    // Add the roi_octree to the map
    roi_octrees[agents_traj.id] = roi_octree;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Shares dynamic obstacles with other agents.
 * @param std::unordered_map<int, std::vector<dynTraj>> &roi_trajs: Output map of trajectories keyed by agent ID.
 * @param double current_time: Current timestamp.
 */
void DYNUS::shareDynamicObstacles(std::unordered_map<int, std::vector<dynTraj>> &roi_trajs, double current_time)
{

  // Go thru trajs_ and check if we have something useful to share for each agent
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // Filter the local trajs to only include the agents
  std::vector<dynTraj> agents_trajs;
  for (auto traj : local_trajs)
  {
    if (traj.is_agent)
    {
      agents_trajs.push_back(traj);
    }
  }

  // Check for each id
  for (const auto &agents_traj : agents_trajs)
  {

    // agent A's current position and the end position
    Eigen::Vector3d agent_A_current_pos = agents_traj.pwp.eval(current_time);
    Eigen::Vector3d agent_A_end_pos = agents_traj.pwp.eval(agents_traj.pwp.times.back());

    // agent A's trajectory dt
    double agent_A_dt = (agents_traj.pwp.times.back() - current_time) / par_.num_roi_traj_sample;

    double agent_A_x_min = std::min(agent_A_current_pos[0], agent_A_end_pos[0]) - par_.roi_map_share_buffer;
    double agent_A_x_max = std::max(agent_A_current_pos[0], agent_A_end_pos[0]) + par_.roi_map_share_buffer;
    double agent_A_y_min = std::min(agent_A_current_pos[1], agent_A_end_pos[1]) - par_.roi_map_share_buffer;
    double agent_A_y_max = std::max(agent_A_current_pos[1], agent_A_end_pos[1]) + par_.roi_map_share_buffer;
    double agent_A_z_min = std::min(agent_A_current_pos[2], agent_A_end_pos[2]) - par_.roi_map_share_buffer;
    double agent_A_z_max = std::max(agent_A_current_pos[2], agent_A_end_pos[2]) + par_.roi_map_share_buffer;

    for (auto traj_B : local_trajs)
    {

      // Check if the trajectory's id is the agent_A_id
      if (traj_B.id == agents_traj.id)
        continue;

      // Sample points from the trajectory
      vec_Vecf<3> sampled_points;
      for (double t = current_time; t < agents_traj.pwp.times.back(); t += agent_A_dt)
      {
        sampled_points.push_back(traj_B.pwp.eval(t));
      }

      // Check if the trajectory is between the agent A's current position and its goal
      bool is_traj_between = false;
      for (int i = 0; i < sampled_points.size(); i++)
      {
        if (sampled_points[i].x() >= agent_A_x_min && sampled_points[i].x() <= agent_A_x_max &&
            sampled_points[i].y() >= agent_A_y_min && sampled_points[i].y() <= agent_A_y_max &&
            sampled_points[i].z() >= agent_A_z_min && sampled_points[i].z() <= agent_A_z_max)
        {
          is_traj_between = true;
          break;
        }
      }

      // Add the trajectory to the vector if it is between the agent A's current position and its goal
      if (is_traj_between)
      {
        roi_trajs[agents_traj.id].push_back(traj_B);
      }

    } // end of for loop for local_trajs

  } // end of for loop for agent_A_ids
}

// ----------------------------------------------------------------------------

/**
 * @brief Extracts a region of interest (ROI) from an octree.
 * @param std::shared_ptr<octomap::TimedOcTree> &roi_octree: Output ROI octree.
 * @param const std::shared_ptr<octomap::TimedOcTree> &ego_octree: Input octree.
 * @param const octomap::point3d &point_A: One corner of the ROI.
 * @param const octomap::point3d &point_B: Opposite corner of the ROI.
 */
void DYNUS::extractROI(std::shared_ptr<octomap::TimedOcTree> &roi_octree,
                       const std::shared_ptr<octomap::TimedOcTree> &ego_octree,
                       const octomap::point3d &point_A,
                       const octomap::point3d &point_B)
{
  // Determine min and max coordinates for bounding box
  octomap::point3d minPoint(std::min(point_A.x(), point_B.x()), std::min(point_A.y(), point_B.y()), std::min(point_A.z(), point_B.z()));
  octomap::point3d maxPoint(std::max(point_A.x(), point_B.x()), std::max(point_A.y(), point_B.y()), std::max(point_A.z(), point_B.z()));

  // Inflate min and max points by the bounding box inflation factor
  minPoint -= octomap::point3d(par_.roi_map_share_buffer, par_.roi_map_share_buffer, par_.roi_map_share_buffer);
  maxPoint += octomap::point3d(par_.roi_map_share_buffer, par_.roi_map_share_buffer, par_.roi_map_share_buffer);

  // Traverse the original tree and copy nodes within the bounding box
  for (auto it = ego_octree->begin_leafs_bbx(octomap::point3d(minPoint.x(), minPoint.y(), minPoint.z()), octomap::point3d(maxPoint.x(), maxPoint.y(), maxPoint.z())); it != ego_octree->end_leafs_bbx(); ++it)
  {
    // Insert the node into the new tree
    bool occupied = ego_octree->isNodeOccupied(*it);
    roi_octree->updateNode(it.getCoordinate(), it->getOccupancy() > 0.5);
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
    data.yaw -= yaw_init_offset_;
  }

  mtx_state_.lock();
  state_ = data;
  mtx_state_.unlock();

  if (state_initialized_ == false || drone_status_ == DroneStatus::GOAL_REACHED || drone_status_ == DroneStatus::YAWING)
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

    // Push the state to the plan_safe_paths_
    mtx_plan_safe_paths_.lock();
    plan_safe_paths_.clear();
    plan_safe_paths_.push_back(std::vector<state>());
    mtx_plan_safe_paths_.unlock();

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
    return false;

  // Check if the goal is reached
  // if (drone_status_ == DroneStatus::GOAL_REACHED)
  //   return false;

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

    mtx_plan_safe_paths_.lock();
    plan_safe_paths_.pop_front();
    mtx_plan_safe_paths_.unlock();
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
    next_goal.yaw += yaw_init_offset_;
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

  // Initialize parameters
  double diff = 0.0;
  double desired_yaw = 0.0;

  // Get states needec
  state local_G_term, local_state;
  getGterm(local_G_term);
  getState(local_state);

  // If there's no obstacles to look at, then just look at the next goal
  // get local trajs
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  // If there are no obstacles, just look at the next goal
  if (local_trajs.empty())
  {

    // Get look ahead position
    mtx_plan_.lock();
    auto local_plan = plan_;
    mtx_plan_.unlock();

    // The look ahead position index is par_.look_ahead_secs_for_dom * par_.dc - but if this exceeds the size of the plan, then just use the last element
    if (par_.use_hardware)
    {
      // If we are using ground robots, then the geometric controller wants yaw to be the direction of motion (velocity direction)
      if (next_goal.vel.norm() < 1e-1)
      {
        next_goal.yaw = previous_yaw_;
      }
      else
        next_goal.yaw = atan2(next_goal.vel[1], next_goal.vel[0]);

      next_goal.dyaw = (next_goal.yaw - previous_yaw_) / par_.dc;

      dynus_utils::angle_wrap(next_goal.yaw);
    }
    else
    {
      int look_ahead_index = std::min((int)(par_.look_ahead_secs_for_dom * par_.dc), (int)local_plan.size() - 1);
      // desired_yaw = atan2(next_goal.pos[1] - local_state.pos.y(), next_goal.pos[0] - local_state.pos.x());
      desired_yaw = atan2(local_plan[look_ahead_index].pos[1] - local_state.pos.y(), local_plan[look_ahead_index].pos[0] - local_state.pos.x());
      diff = desired_yaw - local_state.yaw;
      next_goal.yaw = desired_yaw;
    }

  }
  else // If there are obstacles, then look at the obstacles
  {
    switch (drone_status_)
    {
    case DroneStatus::YAWING:
      desired_yaw = atan2(local_G_term.pos[1] - next_goal.pos[1], local_G_term.pos[0] - next_goal.pos[0]);
      diff = desired_yaw - local_state.yaw;
      break;

    case DroneStatus::TRAVELING:

      if (next_goal.use_tracking_yaw)
      {
        // next_goal.yaw has already been set in replan()
        diff = next_goal.yaw - local_state.yaw;
        break;
      }
      else
      {
        // Same as GOAL_SEEN
      }

    case DroneStatus::GOAL_SEEN:
      desired_yaw = atan2(next_goal.pos[1] - local_state.pos.y(), next_goal.pos[0] - local_state.pos.x());
      diff = desired_yaw - local_state.yaw;
      next_goal.yaw = desired_yaw;
      break;

    case DroneStatus::GOAL_REACHED:
      desired_yaw = atan2(local_G_term.pos[1] - local_state.pos.y(), local_G_term.pos[0] - local_state.pos.x());
      diff = desired_yaw - local_state.yaw;
      next_goal.yaw = desired_yaw;
    }
  }

  // Wrap the angle
  dynus_utils::angle_wrap(diff);

  // Change the drone status
  // TODO: 0.1 is hardcoded
  if (fabs(diff) < 0.1 && drone_status_ == DroneStatus::YAWING)
  {
    changeDroneStatus(DroneStatus::TRAVELING);
  }

  // Filter the yaw
  filterYaw(diff, next_goal);
}

// ----------------------------------------------------------------------------

/**
 * @brief Filters the yaw rate.
 * @param double diff: Yaw difference.
 * @param state &next_goal: Next goal state to update.
 */
void DYNUS::filterYaw(double diff, state &next_goal)
{

  // Filter yaw
  next_goal.yaw = par_.alpha_filter_yaw * previous_yaw_ + (1 - par_.alpha_filter_yaw) * next_goal.yaw;

  // If the difference between this new yaw and the previous yaw is too large, saturate it
  if (fabs(next_goal.yaw - previous_yaw_) > par_.dc * par_.w_max)
  {
    next_goal.yaw = previous_yaw_ + copysign(1, diff) * par_.dc * par_.w_max;
  }

  // // Saturate the yaw rate
  // dynus_utils::saturate(diff, -par_.dc * par_.w_max, par_.dc * par_.w_max);

  // // Filter the yaw rate
  // double dyaw_not_filtered;
  // dyaw_not_filtered = copysign(1, diff) * par_.w_max;
  // dyaw_filtered_ = par_.alpha_filter_dyaw * dyaw_filtered_ + (1 - par_.alpha_filter_dyaw) * dyaw_not_filtered;
  // next_goal.dyaw = dyaw_filtered_;

  next_goal.dyaw = par_.alpha_filter_dyaw * next_goal.dyaw + (1 - par_.alpha_filter_dyaw) * prev_dyaw_;
  prev_dyaw_ = next_goal.dyaw;

  // Set the previous yaw
  previous_yaw_ = next_goal.yaw;
}

// ----------------------------------------------------------------------------

/**
 * @brief Starts exploration mode.
 */
void DYNUS::startExploration()
{

  mtx_G_.lock();
  mtx_state_.lock();
  G_ = state_;
  mtx_state_.unlock();
  mtx_G_.unlock();

  // Set the drone status to YAWING
  changeDroneStatus(DroneStatus::YAWING);

  // Set the terminal goal
  terminal_goal_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the terminal goal.
 * @param const state &term_goal: Desired terminal goal state.
 */
void DYNUS::setTerminalGoal(const state &term_goal)
{

  // std::cout << "terminal goal is set" << std::endl;

  if (par_.flight_mode == "exploration")
  {
    terminal_goal_initialized_ = true;
    changeDroneStatus(DroneStatus::TRAVELING);
    return;
  }

  // Get the state
  state local_state;
  getState(local_state);

  // Set the terminal goal
  setGterm(term_goal);

  // Project the terminal goal to the sphere
  mtx_G_.lock();
  G_.pos = dynus_utils::projectPointToSphere(local_state.pos, term_goal.pos, par_.horizon);
  mtx_G_.unlock();

  if (drone_status_ == DroneStatus::GOAL_REACHED)
  {
    if (par_.use_initial_yawing)
    {
      changeDroneStatus(DroneStatus::YAWING);
    }
    else
    {
      changeDroneStatus(DroneStatus::TRAVELING);
    }
  }

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

  if (!state_initialized_ || !terminal_goal_initialized_ || !dgp_manager_.isMapInitialized() || !map_size_initialized_ || (par_.use_frontiers && !frontier_initialized_))
    return false;

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if all necessary components are initialized for updating the map.
 * @return bool
 */
bool DYNUS::checkReadyToUpdateMap()
{

  if (!state_initialized_ || !terminal_goal_initialized_ || !dgp_manager_.isMapInitialized())
  {
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Updates the map.
 * @param double octmap_received_time: Timestamp when octomap was received.
 */
void DYNUS::updateMapCallback(double octmap_received_time, const std::shared_ptr<octomap::TimedOcTree> &lidar_octree, const std::shared_ptr<octomap::TimedOcTree> &depth_camera_octree)
{

  // If before planning, we use local_state as the current position
  if (!checkReadyToUpdateMap())
  {
    // use local_state as the current position
    state local_state;
    getState(local_state);
    dgp_manager_.updateMapCallback(wdx_, wdy_, wdz_, local_state.pos, local_state.pos, local_state.pos, octmap_received_time, lidar_octree, depth_camera_octree);
    return;
  }

  // Get Point A and G
  state local_G, local_A;
  getG(local_G);
  getA(local_A);

  // Get the current global_path_
  vec_Vecf<3> global_path;
  getGlobalPath(global_path);

  // compute the minimum and maximum of the global path, local_A, and local_G -> this will determine the map size
  Eigen::Vector3d min_pos = local_A.pos;
  Eigen::Vector3d max_pos = local_A.pos;
  for (int i = 0; i < global_path.size(); i++)
  {
    min_pos = min_pos.cwiseMin(global_path[i]);
    max_pos = max_pos.cwiseMax(global_path[i]);
  }
  min_pos = min_pos.cwiseMin(local_G.pos);
  max_pos = max_pos.cwiseMax(local_G.pos);

  // compute the map size
  computeMapSize(min_pos, max_pos, local_A, local_G);

  // update the map
  dgp_manager_.updateMapCallback(wdx_, wdy_, wdz_, map_center_, min_pos, max_pos, octmap_received_time, lidar_octree, depth_camera_octree);

  // Map size initialized
  if (!map_size_initialized_)
    map_size_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Sets the octomap for a given sensor or region.
 * @param std::shared_ptr<octomap::TimedOcTree> &octree: Input octomap.
 * @param std::string octomap_name: Identifier for the octomap type.
 * @param std::string id: Identifier for ROI sharing.
 */
void DYNUS::setOctomap(std::shared_ptr<octomap::TimedOcTree> &octree, std::string octomap_name, std::string id)
{

  if (octomap_name == "roi")
  {

    mtx_state_.lock();
    state local_state = state_;
    mtx_state_.unlock();

    mtx_G_.lock();
    state local_G = G_;
    mtx_G_.unlock();

    octomap::point3d current_pos_octree(local_state.pos[0], local_state.pos[1], local_state.pos[2]);
    octomap::point3d end_pos_octree(local_G.pos[0], local_G.pos[1], local_G.pos[2]);

    std::shared_ptr<octomap::TimedOcTree> merged_roi_octree;
    merged_roi_octree = std::make_shared<octomap::TimedOcTree>(octree->getResolution(), octree->getDecayDuration()); // TODO: assume that all the octrees have the same resolution

    mtx_roi_octrees_.lock();
    roi_octrees_[id] = std::make_shared<octomap::TimedOcTree>(*octree);

    // Combine the roi_octrees
    for (const auto &[id, roi_octree] : roi_octrees_)
    {
      extractROI(merged_roi_octree, roi_octree, current_pos_octree, end_pos_octree);
    }
    mtx_roi_octrees_.unlock();

    // Update the octree
    octree = std::make_shared<octomap::TimedOcTree>(*merged_roi_octree);
  }
  else
  {
    std::cout << "Unknown octomap_name" << std::endl;
  }

  dgp_manager_.setOctomap(octree, octomap_name);
}

// ----------------------------------------------------------------------------

void DYNUS::setOctree(const std::shared_ptr<octomap::TimedOcTree> &lidar_octree, const std::shared_ptr<octomap::TimedOcTree> &depth_camera_octree)
{
  // shallow copy
  mtx_mid360_octree_.lock();
  mid360_octree_ = std::shared_ptr<octomap::TimedOcTree>(lidar_octree);
  mtx_mid360_octree_.unlock();

  mtx_d435_octree_.lock();
  d435_octree_ = std::shared_ptr<octomap::TimedOcTree>(depth_camera_octree);
  mtx_d435_octree_.unlock();

  if (!octree_initialized_)
    octree_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Finds frontiers for exploration.
 * @param Eigen::Vector3d &best_frontier: Output best frontier position.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @return bool
 */
bool DYNUS::findFrontiers(Eigen::Vector3d &best_frontier, const Eigen::Matrix4d &camera_transform)
{

  if (par_.flight_mode == "terminal_goal" || !state_initialized_ || !terminal_goal_initialized_ || !dgp_manager_.isMapInitialized() || !map_size_initialized_ || !octree_initialized_)
    return false;

  // Initialization
  std::shared_ptr<octomap::TimedOcTree> d435_octree;
  std::shared_ptr<octomap::TimedOcTree> mid360_octree;

  // // Get the octrees
  // mtx_d435_octree_.lock();
  // d435_octree = std::make_shared<octomap::TimedOcTree>(*d435_octree_);
  // mtx_d435_octree_.unlock();

  // if (!par_.use_only_d435_for_frontiers)
  // {
  //   mtx_mid360_octree_.lock();
  //   mid360_octree = std::make_shared<octomap::TimedOcTree>(*mid360_octree_);
  //   mtx_mid360_octree_.unlock();
  // }

  MyTimer timer(true);

  // Extract the frontiers
  bool result = extractFrontierPoints(best_frontier, d435_octree_, mid360_octree_, camera_transform);

  // Print the time
  timer.printMs("findFrontiers");

  if (!frontier_initialized_ && result)
    frontier_initialized_ = true;

  return result;
}

// ----------------------------------------------------------------------------

/**
 * @brief Extracts frontier points.
 * @ref https://github.com/RobustFieldAutonomyLab/turtlebot_exploration_3d/blob/master/include/exploration.h
 * @param Eigen::Vector3d &best_frontier: Output best frontier position.
 * @param const std::shared_ptr<octomap::TimedOcTree> &d435_octree_ptr: Pointer to d435 octree.
 * @param const std::shared_ptr<octomap::TimedOcTree> &mid360_octree_ptr: Pointer to mid360 octree.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @return bool
 *
 */
bool DYNUS::extractFrontierPoints(Eigen::Vector3d &best_frontier, const std::shared_ptr<octomap::TimedOcTree> &d435_octree_ptr, const std::shared_ptr<octomap::TimedOcTree> &mid360_octree_ptr, const Eigen::Matrix4d &camera_transform)
{

  // Initialize the frontier points from both octrees
  vec_Vecf<3> d435_frontier_points;
  vec_Vecf<3> d435_frontier_directions;
  vec_Vecf<3> mid360_frontier_points;
  vec_Vecf<3> mid360_frontier_directions;

  computeAllFrontierPoints(d435_octree_ptr, d435_frontier_points, d435_frontier_directions);
  if (!par_.use_only_d435_for_frontiers)
    computeAllFrontierPoints(mid360_octree_ptr, mid360_frontier_points, mid360_frontier_directions);

  // Mid360's frontiers are on the floor and want to clear them using D435's octree
  if (!par_.use_only_d435_for_frontiers)
    filterFrontiers(d435_octree_ptr, mid360_frontier_points);

  // Sum the frontier points
  vec_Vecf<3> frontier_points;
  frontier_points.insert(frontier_points.end(), d435_frontier_points.begin(), d435_frontier_points.end());
  if (!par_.use_only_d435_for_frontiers)
    frontier_points.insert(frontier_points.end(), mid360_frontier_points.begin(), mid360_frontier_points.end());

  // Sum the frontier directions
  vec_Vecf<3> frontier_directions;
  frontier_directions.insert(frontier_directions.end(), d435_frontier_directions.begin(), d435_frontier_directions.end());
  if (!par_.use_only_d435_for_frontiers)
    frontier_directions.insert(frontier_directions.end(), mid360_frontier_directions.begin(), mid360_frontier_directions.end());

  if (frontier_points.empty())
  {
    std::cout << "No frontiers found" << std::endl;
    return false;
  }

  // TODO: Find best frontiers with increased search radius (we first start with small search radius, but if no frontiers are found, we increase the search radius)
  computeBestFrontiers(frontier_points, camera_transform, frontier_directions);

  mtx_best_frontier_.lock();
  best_frontier = best_frontier_.pos;
  mtx_best_frontier_.unlock();

  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes all frontier points from an octree.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to input octree.
 * @param vec_Vecf<3> &frontier_points: Output frontier point coordinates.
 * @param vec_Vecf<3> &frontier_directions: Output frontier directions.
 */
void DYNUS::computeAllFrontierPoints(const std::shared_ptr<octomap::TimedOcTree> &octree,
                                     vec_Vecf<3> &frontier_points,
                                     vec_Vecf<3> &frontier_directions)
{
  // Acquire the current state once.
  mtx_state_.lock();
  state local_state = state_;
  mtx_state_.unlock();

  // Define the bounding box based on the state and parameters.
  octomap::point3d min_point(local_state.pos[0] - par_.depth_camera_depth_max - par_.frontier_search_buffer,
                             local_state.pos[1] - par_.depth_camera_depth_max - par_.frontier_search_buffer,
                             local_state.pos[2] - par_.depth_camera_depth_max - par_.frontier_search_buffer);
  octomap::point3d max_point(local_state.pos[0] + par_.depth_camera_depth_max + par_.frontier_search_buffer,
                             local_state.pos[1] + par_.depth_camera_depth_max + par_.frontier_search_buffer,
                             local_state.pos[2] + par_.depth_camera_depth_max + par_.frontier_search_buffer);

  // Get leaf box
  auto begin_it = octree->begin_leafs_bbx(min_point, max_point);
  auto end_it = octree->end_leafs_bbx();

  // First pass: collect all free leaf nodes within the bounding box.
  std::vector<OctreeLeafData> leaf_data;
  for (auto it = begin_it; it != end_it; ++it)
  {
    // Only consider free voxels.
    if (octree->isNodeOccupied(*it))
      continue;

    OctreeLeafData data;
    data.x = it.getX();
    data.y = it.getY();
    data.z = it.getZ();
    data.center = it.getCoordinate();
    data.size = it.getSize();
    leaf_data.push_back(data);
  }

  // Prepare global containers for frontier results.
  vec_Vec3f frontier_points_local;
  vec_Vec3f frontier_directions_local;

// Process each leaf node in parallel.
#pragma omp parallel
  {
    vec_Vec3f thread_points;
    vec_Vec3f thread_directions;

#pragma omp for nowait
    for (size_t i = 0; i < leaf_data.size(); i++)
    {
      const auto &data = leaf_data[i];
      double x_cur = data.x;
      double y_cur = data.y;
      double z_cur = data.z;
      const octomap::point3d &voxel_center = data.center;
      double node_size = data.size;
      double box_res = node_size / 2.0 + par_.octomap_res;

      bool isOccupiedNeighbor = false;
      bool isFrontier = false;

      int num_known_free = 0;
      octomap::point3d known_free_voxel(0.0, 0.0, 0.0);
      int num_unknown = 0;
      octomap::point3d unknown_voxel(0.0, 0.0, 0.0);

      // Examine neighbors in the XY plane (z fixed).
      for (double x_buf = x_cur - box_res; x_buf < x_cur + box_res; x_buf += par_.octomap_res)
      {
        for (double y_buf = y_cur - box_res; y_buf < y_cur + box_res; y_buf += par_.octomap_res)
        {
          octomap::point3d neighbor_center = voxel_center +
                                             octomap::point3d(x_buf - x_cur, y_buf - y_cur, 0.0);
          octomap::TimedOcTreeNode *neighbor_node = octree->search(octomap::point3d(x_buf, y_buf, z_cur));

          if (neighbor_node && !octree->isNodeOccupied(neighbor_node))
          {
            known_free_voxel = known_free_voxel + neighbor_center;
            num_known_free++;
          }
          if (!neighbor_node)
          {
            unknown_voxel = unknown_voxel + neighbor_center;
            num_unknown++;
          }
          if (neighbor_node && octree->isNodeOccupied(neighbor_node))
          {
            isOccupiedNeighbor = true;
            break;
          }
        }
        if (isOccupiedNeighbor)
          break;
      }

      if (num_known_free > par_.frontier_min_known_free_thresh &&
          num_unknown > par_.frontier_min_unknown_thresh)
      {
        isFrontier = true;
      }

      if (!isOccupiedNeighbor && isFrontier)
      {
        // Compute the average positions for known free and unknown voxels.
        Eigen::Vector3d known_free_avg(known_free_voxel.x(), known_free_voxel.y(), known_free_voxel.z());
        known_free_avg /= num_known_free;
        Eigen::Vector3d unknown_avg(unknown_voxel.x(), unknown_voxel.y(), unknown_voxel.z());
        unknown_avg /= num_unknown;

        Eigen::Vector3d direction = unknown_avg - known_free_avg;
        // Skip if the computed direction is degenerate.
        if (direction[0] == 0 && direction[1] == 0)
          continue;
        direction.normalize();

        thread_directions.push_back(direction);
        thread_points.push_back(Eigen::Vector3d(voxel_center.x(), voxel_center.y(), voxel_center.z()));

        // Optionally, add additional neighboring points if the node size is larger.
        if (node_size > par_.octomap_res)
        {
          for (double x_buf = x_cur - box_res; x_buf < x_cur + box_res; x_buf += box_res)
          {
            for (double y_buf = y_cur - box_res; y_buf < y_cur + box_res; y_buf += box_res)
            {
              for (double z_buf = z_cur - box_res; z_buf < z_cur + box_res; z_buf += box_res)
              {
                thread_points.push_back(Eigen::Vector3d(x_buf, y_buf, z_buf));
              }
            }
          }
        }
      }
    } // end for

// Merge the thread-local frontier data into the global containers.
#pragma omp critical
    {
      frontier_points_local.insert(frontier_points_local.end(), thread_points.begin(), thread_points.end());
      frontier_directions_local.insert(frontier_directions_local.end(),
                                       thread_directions.begin(), thread_directions.end());
    }
  } // end parallel region

  // Update the output containers.
  frontier_points = frontier_points_local;
  frontier_directions = frontier_directions_local;
}

// ----------------------------------------------------------------------------

/**
 * @brief Filters frontier points.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to octree.
 * @param vec_Vecf<3> &frontiers: Frontier points to filter.
 */
void DYNUS::filterFrontiers(const std::shared_ptr<octomap::TimedOcTree> &octree_ptr, vec_Vecf<3> &frontiers)
{

  // Filter the frontiers
  vec_Vecf<3> filtered_frontiers;
  for (const auto &frontier : frontiers)
  {
    // Check if the frontier is in free space of the octree
    octomap::point3d frontier_point(frontier[0], frontier[1], frontier[2]);
    auto *node = octree_ptr->search(frontier_point);

    if (!node) // Unknown voxel
    {
      filtered_frontiers.push_back(frontier);
    }
  }

  frontiers = filtered_frontiers;
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the mean frontier.
 * @param const vec_Vecf<3> &visible_frontiers: Input visible frontier points.
 * @param const std::shared_ptr<octomap::TimedOcTree> &octree_ptr: Pointer to octree.
 */
void DYNUS::findMeanFrontier(const vec_Vecf<3> &visible_frontiers, const std::shared_ptr<octomap::TimedOcTree> &octree_ptr)
{
  if (visible_frontiers.empty())
  {
    // if visible_frontiers is empty, keep the best_frontier as it is
    return;
  }

  // Step 1: Compute the mean frontier
  Eigen::Vector3d mean_frontier = Eigen::Vector3d::Zero();
  for (const auto &frontier : visible_frontiers)
  {
    mean_frontier += frontier;
  }
  mean_frontier /= visible_frontiers.size();

  // Step 2: Find the closest frontier to the mean frontier
  double min_dist = std::numeric_limits<double>::max();
  Eigen::Vector3d closest_frontier = mean_frontier;

  for (const auto &frontier : visible_frontiers)
  {
    double dist = (frontier - mean_frontier).norm();
    if (dist < min_dist)
    {
      min_dist = dist;
      closest_frontier = frontier;
    }
  }

  mean_frontier = closest_frontier; // Update the mean frontier to the closest

  // Step 3: Find unknown neighbors of the mean frontier
  Eigen::Vector3d mean_unknown_neighbors = Eigen::Vector3d::Zero();
  int unknown_count = 0;

  double voxel_size = octree_ptr->getResolution();
  for (double dx = -voxel_size; dx <= voxel_size; dx += voxel_size)
  {
    for (double dy = -voxel_size; dy <= voxel_size; dy += voxel_size)
    {
      for (double dz = -voxel_size; dz <= voxel_size; dz += voxel_size)
      {
        // Skip the center voxel
        if (dx == 0 && dy == 0 && dz == 0)
          continue;

        // Neighbor voxel position
        octomap::point3d neighbor_pos(mean_frontier[0] + dx,
                                      mean_frontier[1] + dy,
                                      mean_frontier[2] + dz);

        // Check if the neighbor is unknown
        auto *node = octree_ptr->search(neighbor_pos);
        if (!node) // Unknown voxel
        {
          mean_unknown_neighbors += Eigen::Vector3d(neighbor_pos.x(), neighbor_pos.y(), neighbor_pos.z());
          unknown_count++;
        }
      }
    }
  }

  if (!is_best_frontier_initialized_)
  {
    mtx_best_frontier_.lock();
    best_frontier_.pos = mean_frontier;
    mtx_best_frontier_.unlock();
  }

  if (unknown_count > 0)
  {
    mean_unknown_neighbors /= unknown_count;

    // Step 4: Compute the direction from mean_frontier to mean_unknown_neighbors
    Eigen::Vector3d direction_to_unknown = (mean_unknown_neighbors - mean_frontier).normalized();

    // Debug output
    std::cout << "Direction to unknown: " << direction_to_unknown.transpose() << std::endl;

    // Step 5: Update the best frontier
    // TODO: make sure the last yaw is used in yaw optimization
    mtx_best_frontier_.lock();
    best_frontier_.pos = par_.frontier_update_alpha * mean_frontier + (1 - par_.frontier_update_alpha) * best_frontier_.pos;
    best_frontier_.yaw = atan2(mean_unknown_neighbors[1] - mean_frontier[1], mean_unknown_neighbors[0] - mean_frontier[0]);
    mtx_best_frontier_.unlock();
  }
  else
  {
    std::cout << "No unknown neighbors found around mean_frontier." << std::endl;
    // Step 5: Update the best frontier
    mtx_best_frontier_.lock();
    best_frontier_.pos = par_.frontier_update_alpha * mean_frontier + (1 - par_.frontier_update_alpha) * best_frontier_.pos;
    mtx_best_frontier_.unlock();
  }

  if (!is_best_frontier_initialized_)
  {
    is_best_frontier_initialized_ = true;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Computes the best frontier.
 * @param const vec_Vecf<3> &global_frontiers: Input candidate frontier points.
 * @param const Eigen::Matrix4d &camera_transform: Camera transformation matrix.
 * @param vec_Vecf<3> &frontier_directions: Output frontier directions.
 */
void DYNUS::computeBestFrontiers(const vec_Vecf<3> &global_frontiers,
                                 const Eigen::Matrix4d &camera_transform,
                                 vec_Vecf<3> &frontier_directions)
{
  // Get local state (protected by mutex)
  mtx_state_.lock();
  state state_local = state_;
  mtx_state_.unlock();

  // Get previous best frontier (protected by mutex)
  state previous_best_frontier;
  mtx_best_frontier_.lock();
  previous_best_frontier = best_frontier_;
  mtx_best_frontier_.unlock();

  // Global reduction variables
  double global_min_cost = std::numeric_limits<double>::max();
  int global_best_frontier_idx = -1;
  vec_Vecf<3> combined_frontier_points;

// Parallel region: each thread computes its own best cost and collects valid frontiers.
#pragma omp parallel
  {
    double local_min_cost = std::numeric_limits<double>::max();
    int local_best_frontier_idx = -1;
    vec_Vecf<3> local_frontier_points;

#pragma omp for nowait
    for (int idx = 0; idx < static_cast<int>(global_frontiers.size()); idx++)
    {
      Eigen::Vector3d frontier = global_frontiers[idx];

      // Filter out NaNs
      if (std::isnan(frontier.x()) || std::isnan(frontier.y()) || std::isnan(frontier.z()))
        continue;

      // Filter based on z-range
      if (frontier.z() < par_.z_min || frontier.z() > par_.z_max)
        continue;

      // Filter based on local sliding window map
      if (!checkPointWithinMap(frontier))
        continue;

      // Filter points too close to camera
      if (dynus_utils::euclideanDistance(frontier, state_local.pos) < par_.min_dist_from_frontier_to_camera)
        continue;

      // Transform the point to camera frame
      Eigen::Vector4d point_world_homo(frontier.x(), frontier.y(), frontier.z(), 1.0);
      Eigen::Vector4d point_camera_homo = camera_transform * point_world_homo;

      // Depth-based filter
      if (par_.d435_depth_min > point_camera_homo(2))
        continue;

      // Height difference filter
      if (std::abs(frontier.z() - state_local.pos.z()) > par_.max_z_diff_from_frontier_to_camera)
        continue;

      // Save this valid frontier point in the thread-local vector.
      local_frontier_points.push_back(frontier);

      // (1) Compute desired velocity cost
      Eigen::Vector3d desired_velocity = (frontier - state_local.pos) * (v_max_ / par_.depth_camera_depth_max);
      double desired_velocity_norm = desired_velocity.norm();
      double desired_velocity_cost = 1.0 / (desired_velocity_norm + 0.0001);

      // (2) Compute cost based on distance from camera's z-axis
      double dist_from_z_axis = std::sqrt(point_camera_homo(0) * point_camera_homo(0) +
                                          point_camera_homo(1) * point_camera_homo(1));

      // (3) Compute cost based on closeness to the previous best frontier (only XY distance)
      double dist_to_prev_best_frontier = std::sqrt(
          (previous_best_frontier.pos(0) - state_local.pos(0)) * (previous_best_frontier.pos(0) - state_local.pos(0)) +
          (previous_best_frontier.pos(1) - state_local.pos(1)) * (previous_best_frontier.pos(1) - state_local.pos(1)));

      // (4) Encourage a positive z-axis in camera direction
      double positive_z_camera = -point_camera_homo(2);

      // (5) Compute goal proximity (if flight_mode is terminal_goal)
      double goal_proximity = 0.0;
      if (par_.flight_mode == "terminal_goal")
      {
        state local_G;
        getG(local_G);
        goal_proximity = (frontier - local_G.pos).norm();
      }

      // (6) Compute information gain cost by counting neighboring frontiers
      int num_neighbor_frontier = 0;
      for (const auto &sub_frontier : global_frontiers)
      {
        if ((frontier - sub_frontier).norm() < par_.frontier_neighbor_thresh_for_info_gain)
        {
          num_neighbor_frontier++;
        }
      }
      double info_gain_cost = global_frontiers.size() - num_neighbor_frontier;

      // Total cost (weighted sum)
      double total_cost = par_.desired_velocity_cost_weight * desired_velocity_cost +
                          par_.dist_from_z_axis_weight * dist_from_z_axis +
                          par_.dist_to_prev_best_frontier_weight * dist_to_prev_best_frontier +
                          par_.positive_z_camera_weight * positive_z_camera +
                          par_.goal_proximity_weight * goal_proximity +
                          par_.info_gain_cost_weight * info_gain_cost;

      // Update the local best candidate if a lower cost is found.
      if (total_cost < local_min_cost)
      {
        local_min_cost = total_cost;
        local_best_frontier_idx = idx;
      }
    } // end for loop

// Merge thread-local results into global variables
#pragma omp critical
    {
      // Append the thread's frontier points to the combined vector.
      combined_frontier_points.insert(combined_frontier_points.end(),
                                      local_frontier_points.begin(),
                                      local_frontier_points.end());
      // Update global best candidate if this thread found a lower cost.
      if (local_min_cost < global_min_cost)
      {
        global_min_cost = local_min_cost;
        global_best_frontier_idx = local_best_frontier_idx;
      }
    }
  } // end parallel region

  // If no frontier passed the filters, then exit.
  if (combined_frontier_points.empty())
  {
    std::cout << "No frontiers found" << std::endl;
    return;
  }

  // Update the shared frontier_points_ with locking.
  mtx_frontier_points_.lock();
  frontier_points_ = combined_frontier_points;
  mtx_frontier_points_.unlock();

  // Update the best frontier's position using the global best candidate.
  state local_best_frontier;
  local_best_frontier.pos = global_frontiers[global_best_frontier_idx];

  // Compute yaw based on flight mode.
  state local_G_term;
  getGterm(local_G_term);

  if (par_.flight_mode == "terminal_goal")
  {
    // Direct the best frontier toward the terminal goal.
    local_best_frontier.yaw = std::atan2(local_G_term.pos[1] - local_best_frontier.pos[1],
                                         local_G_term.pos[0] - local_best_frontier.pos[0]);
  }
  else if (par_.flight_mode == "exploration")
  {
    // Use the average direction from the provided frontier directions.
    Eigen::Vector3d mean_direction = Eigen::Vector3d::Zero();
    for (const auto &direction : frontier_directions)
    {
      mean_direction += direction;
    }
    mean_direction /= frontier_directions.size();
    local_best_frontier.yaw = std::atan2(mean_direction[1], mean_direction[0]);
  }

  // Update the best frontier in the shared variable with locking.
  mtx_best_frontier_.lock();
  best_frontier_ = local_best_frontier;
  mtx_best_frontier_.unlock();

  if (!is_best_frontier_initialized_)
    is_best_frontier_initialized_ = true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets visible frontiers.
 * @param vec_Vecf<3> &visible_frontiers: Output visible frontier points.
 */
void DYNUS::getVisibleFrontiers(vec_Vecf<3> &visible_frontiers)
{
  visible_frontiers = visible_frontiers_;
}

// ----------------------------------------------------------------------------

/**
 * @brief Gets all frontiers.
 * @param vec_Vecf<3> &frontiers: Output all frontier points.
 */
void DYNUS::getAllFrontiers(vec_Vecf<3> &frontiers)
{
  mtx_frontier_points_.lock();
  frontiers = frontier_points_;
  mtx_frontier_points_.unlock();
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

  // Get the inverse of init_pose_ (geometry_msgs::msg::TransformStamped)
  init_pose_transform_inv_ = init_pose_transform.inverse();
  init_pose_transform_rotation_inv_ = init_pose_quat.toRotationMatrix().inverse();
  yaw_init_offset_ = std::atan2(init_pose_transform_rotation_inv_(1, 0),
                                init_pose_transform_rotation_inv_(0, 0));
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
 * @brief Gets the best frontier.
 * @param state &best_frontier: Output best frontier state.
 */
void DYNUS::getBestFrontier(state &best_frontier)
{
  if (is_best_frontier_initialized_)
  {

    mtx_state_.lock();
    state state_local = state_;
    mtx_state_.unlock();

    mtx_best_frontier_.lock();
    state local_best_frontier = best_frontier_;
    mtx_best_frontier_.unlock();

    // Find the point that is free and closest to the projected point
    // TODO: not sure if this is implemented correctly - vertical climb fails with this
    // dgp_manager_.findClosestFreePoint(best_frontier_.pos, best_frontier.pos);
    // std::cout << "actual best_frontier_: " << best_frontier_.pos.transpose() << std::endl;
    // std::cout << "projected best_frontier: " << best_frontier.pos.transpose() << std::endl;

    // TODO: projection not working well - just did hacky solution of making the map smaller
    best_frontier.pos = dynus_utils::projectPointToSphere(state_local.pos, local_best_frontier.pos, par_.horizon);
    best_frontier.yaw = local_best_frontier.yaw;
  }
  else
  {
    std::cout << "Best frontier not initialized" << std::endl;
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if the drone is moving.
 * @return bool
 */
bool DYNUS::isMoving()
{
  if (drone_status_ == DroneStatus::TRAVELING || drone_status_ == DroneStatus::GOAL_SEEN)
  {
    return true;
  }
  return false;
}

// ----------------------------------------------------------------------------

/**
 * @brief Checks if there are nearby trajectories.
 * @param double current_time: Current timestamp.
 * @return bool
 */
bool DYNUS::checkNearbyTrajs(double current_time)
{

  // Get the state
  mtx_state_.lock();
  state state_local = state_;
  mtx_state_.unlock();

  // Get the trajs
  std::vector<dynTraj> local_trajs;
  getTrajs(local_trajs);

  if (local_trajs.empty())
  {
    return false;
  }

  // Check if there's any nearby trajs
  for (const auto &traj : local_trajs)
  {
    if ((traj.pwp.eval(current_time) - state_local.pos).norm() < par_.min_dist_from_agent_to_traj)
    {
      return true;
    }
  }

  return false;
}

// ----------------------------------------------------------------------------

/**
 * @brief Processes a successful detection.
 * @param const std::string &detected_object: Identifier of the detected object.
 */
void DYNUS::detectSuccessfulDetection(const std::string &detected_object)
{
  std::cout << bold << green << "Detected object: " << detected_object << reset << std::endl;
  std::cout << "Changing the drone status to GOAL_REACHED" << std::endl;
  changeDroneStatus(DroneStatus::GOAL_REACHED);
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