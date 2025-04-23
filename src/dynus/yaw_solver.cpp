/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <limits>
#include <memory>
#include "dynus/yaw_solver.hpp"

using namespace termcolor;

// Implementation of callback management methods
void YawSolver::stopOptimization() 
{
    yaw_callback_.should_terminate_ = true;
    if (verbose_) std::cout << "Optimization termination requested." << std::endl;
}

void YawSolver::resetOptimization() 
{
    yaw_callback_.should_terminate_ = false;
    if (verbose_) std::cout << "Optimization reset to normal state." << std::endl;
}

// Constructor
YawSolver::YawSolver() {}

// Initialize the parameters
void YawSolver::initialize(const parameters& par, bool verbose)
{

    // Set the parameters
    par_ = par;
    yaw_fit_p_ = par_.yaw_fit_degree;
    yaw_fit_m_ = par_.num_yaw_fit_poly + 2 * yaw_fit_p_;
    yaw_fit_n_ = yaw_fit_m_ - yaw_fit_p_ - 1;
    yaw_fit_num_segments_ = par_.num_yaw_fit_poly;
    w_max_ = par_.w_max;
    dc_ = par_.dc;
    verbose_ = verbose;

    // Initialize parameters
    BasisConverter basis_converter;
    A_pos_bs_ = basis_converter.getABSpline(par_.num_yaw_fit_poly);

    // Initialize the Gurobi model
    model_.set("OutputFlag", std::to_string(0));  // verbose (1) or not (0)
    model_.set(GRB_DoubleParam_TimeLimit, 0.3);  // 0.3 seconds
    // model_.set(GRB_DoubleParam_Heuristics, 1.0);  // Adjust the heuristic search effort
    // model_.set(GRB_IntParam_Method, 2);           // Use the Barrier method
    // model_.set(GRB_IntParam_Presolve, 2);         // Aggressive presolve
    // model_.set(GRB_IntParam_MIPFocus, 1);         // Focus on finding feasible solutions quickly

    // Attach the callback
    model_.setCallback(&yaw_callback_);

}

// Function to find the optimal yaw sequence
void YawSolver::setThreads(int threads)
{
  model_.set("Threads", std::to_string(threads));
}

// Calculate Mahalanobis distance-based collision likelihood
double YawSolver::getCollisionLikelihood(PieceWisePol& agent_trajectory_pwp, dynTraj& obstacle_trajectory, double current_time) {
    double total_time = agent_trajectory_pwp.getEndTime() - current_time;
    double total_collision_likelihood = 0.0;
    for (int i = 0; i < par_.num_samples_collision_likelihood; ++i) {
        double t = current_time + i * total_time / par_.num_samples_collision_likelihood;
        Eigen::Vector3d agent_position = agent_trajectory_pwp.eval(t);
        Eigen::Vector3d obstacle_position = obstacle_trajectory.pwp.eval(t);
        Eigen::Vector3d diff = agent_position - obstacle_position;
        // TODO: I don't think we need to blend the covariances like this - we should just sum them up
        Eigen::Matrix3d covariances = ((total_time - t) / total_time) * obstacle_trajectory.ekf_cov_p.asDiagonal() + (t / total_time) * obstacle_trajectory.poly_cov.asDiagonal();
        // Eigen::Matrix3d covariances = obstacle_trajectory.poly_cov.asDiagonal();
        double mahalanobis_dist_squared = diff.transpose() * covariances.inverse() * diff;
        total_collision_likelihood += std::exp(-0.5 * mahalanobis_dist_squared);
    }
    return total_collision_likelihood / par_.num_samples_collision_likelihood;
}

// Compute velocity score based on the derivative of PieceWisePol
double YawSolver::getVelocityScore(dynTraj& obstacle_trajectory, double current_time) {
    double total_time = obstacle_trajectory.pwp.getEndTime() - current_time;
    double total_velocity_score = 0.0;
    for (int i = 0; i < par_.num_samples_velocity_score; ++i) 
    {
        double t = current_time + i * total_time / par_.num_samples_velocity_score;
        Eigen::Vector3d velocity = obstacle_trajectory.pwp.velocity(t);
        total_velocity_score += velocity.norm();
    }
    return total_velocity_score / par_.num_samples_velocity_score;
}

// Compute total collision likelihood across all obstacles
double YawSolver::getTotalCollisionLikelihood(PieceWisePol& agent_trajectory_pwp, std::vector<dynTraj>& obstacle_trajectories, double current_time, int obstacle_id) {
    double total_collision_likelihood = 0.0;
    for (auto& obstacle_traj : obstacle_trajectories) {
        if (obstacle_traj.id == dom_id_ || obstacle_traj.id == obstacle_id || (agent_trajectory_pwp.eval(current_time) - obstacle_traj.pwp.eval(current_time)).norm() > par_.cutoff_distance) {
            continue;
        }
        total_collision_likelihood += getCollisionLikelihood(agent_trajectory_pwp, obstacle_traj, current_time);
    }
    return total_collision_likelihood;
}

// Compute total velocity score across all obstacles
double YawSolver::getTotalVelocityScore(PieceWisePol& agent_trajectory_pwp, std::vector<dynTraj>& obstacle_trajectories, double current_time, int obstacle_id) 
{
    double total_velocity_score = 0.0;
    for (auto& obstacle_traj : obstacle_trajectories) {
        if (obstacle_traj.id == dom_id_ || obstacle_traj.id == obstacle_id || (agent_trajectory_pwp.eval(current_time) - obstacle_traj.pwp.eval(current_time)).norm() > par_.cutoff_distance) {
            continue;
        }
        total_velocity_score += getVelocityScore(obstacle_traj, current_time);
    }
    return total_velocity_score;
}

// Compute total time since observed
double YawSolver::getTotalTimeSinceObserved(const AStarNode& current_node, std::vector<dynTraj>& obstacle_trajectories, int obstacle_id) 
{
    double total_time_since_observed = 0.0;
    for (auto& obstacle_traj : obstacle_trajectories) {
        if (obstacle_traj.id != dom_id_ && (current_node.position - obstacle_traj.pwp.eval(current_node.time)).norm() > par_.cutoff_distance) {
            continue;
        }
        total_time_since_observed += (obstacle_traj.id == obstacle_id) ? 0 : current_node.time_since_observed.at(obstacle_traj.id);
    }
    return total_time_since_observed;
}

// Compute proximity score
double YawSolver::getTotalProximity(PieceWisePol& agent_trajectory_pwp, std::vector<dynTraj>& obstacle_trajectories, double current_time, int obstacle_id) {
    double total_reciprocal_proximity = 0.0;
    for (auto& obstacle_traj : obstacle_trajectories) {
        if (obstacle_traj.id == dom_id_ || obstacle_traj.id == obstacle_id || (agent_trajectory_pwp.eval(current_time) - obstacle_traj.pwp.eval(current_time)).norm() > par_.cutoff_distance) {
            continue;
        }
        total_reciprocal_proximity += 1 / ((agent_trajectory_pwp.eval(current_time) - obstacle_traj.pwp.eval(current_time)).norm() + 1e-6);
    }
    return (total_reciprocal_proximity == 0) ? 0 : total_reciprocal_proximity;
}

// Compute the utility of a node
double YawSolver::computeUtility(PieceWisePol& agent_trajectory_pwp, std::vector<dynTraj>& obstacle_trajectories, double current_time, const AStarNode& current_node, int obstacle_id, double yaw_diff) {
    double collision_likelihood = getTotalCollisionLikelihood(agent_trajectory_pwp, obstacle_trajectories, current_time, obstacle_id);
    double velocity_score = getTotalVelocityScore(agent_trajectory_pwp, obstacle_trajectories, current_time, obstacle_id);
    double time_since_observed = getTotalTimeSinceObserved(current_node, obstacle_trajectories, obstacle_id);
    double proximity = getTotalProximity(agent_trajectory_pwp, obstacle_trajectories, current_time, obstacle_id);

    return (par_.yaw_collision_weight * collision_likelihood) + (par_.yaw_velocity_weight * velocity_score) +
           (par_.yaw_time_weight * time_since_observed) + (par_.yaw_proximity_weight * proximity) +
           (par_.yaw_change_weight * yaw_diff);
}

// Update the time since observed for each obstacle
std::unordered_map<int, double> YawSolver::updateTimeSinceObserved(const AStarNode& current_node, int obstacle_id) {
    auto time_since_observed = current_node.time_since_observed;
    for (auto& [id, time] : time_since_observed) {
        time = (id == obstacle_id) ? 0 : time + 1; // TODO: instead of 1, it could be a time_increment defined in findOptimalYaw
    }
    return time_since_observed;
}

// Main function for graph search
std::vector<double> YawSolver::findOptimalYaw(PieceWisePol& agent_trajectory_pwp, std::vector<dynTraj>& obstacle_trajectories, const Eigen::Vector3d& initial_pos, double initial_yaw, double terminal_yaw, double time_horizon) 
{
    // Initialize the open set for graph search
    std::priority_queue<AStarNode> open_set;
    std::shared_ptr<AStarNode> best_path = nullptr;

    // Initialize time since observed for each obstacle and the direction of motion
    std::unordered_map<int, double> time_since_observed;
    for (auto& obstacle_traj : obstacle_trajectories) {
        time_since_observed[obstacle_traj.id] = 0.0;
    }
    time_since_observed[dom_id_] = 0.0;

    // Add direction of motion (dom) as an obstacle
    dynTraj dom;
    dom.id = dom_id_;
    obstacle_trajectories.push_back(dom);

    // Initialize the first node with initial conditions
    open_set.emplace(agent_trajectory_pwp.times.front(), initial_pos, initial_yaw, 0.0, time_since_observed);

    // Main graph loop
    while (!open_set.empty()) {
        AStarNode current_node = open_set.top();
        open_set.pop();

        // Check if we have reached the desired time horizon
        if (current_node.time - agent_trajectory_pwp.times.front() >= time_horizon) {
            best_path = std::make_shared<AStarNode>(current_node);
            break;
        }

        double time_increment = time_horizon / 10.0; // TODO: could be a different parameter like num_yaw_samples_in_graph_search
        double next_time = current_node.time + time_increment; // TODO: could be a different parameter like num_yaw_samples_in_graph_search
        Eigen::Vector3d next_position = agent_trajectory_pwp.eval(next_time);

        // Explore branches for each obstacle and the direction of motion
        for (const auto& obstacle_traj : obstacle_trajectories) 
        {

            // We don't consider obstacles that are too far away
            if (obstacle_traj.id != dom_id_ && (next_position - obstacle_traj.pwp.eval(next_time)).norm() > par_.cutoff_distance) {
                continue;
            }

            // To avoid looking at ghost obstacles, we only consider obstacles that we have predictions for
            if (obstacle_traj.id != dom_id_ && (next_time > obstacle_traj.pwp.times.back())) {
                continue;
            }

            // Determine the next yaw based on the direction of motion or obstacle position
            double next_yaw;
            if (obstacle_traj.id == dom_id_)
            {
                // Next position for dom should be further ahead so that it will look more like a direction of motion
                Eigen::Vector3d look_ahead_pos = agent_trajectory_pwp.eval(agent_trajectory_pwp.times.back());

                // if look_ahead_pos is very close to the current position, then the yaw is the same as the current yaw
                if ((look_ahead_pos - current_node.position).norm() < 1.0) // TODO: could be a different parameter like yaw_change_threshold
                {
                    next_yaw = current_node.yaw;
                }
                else
                {
                    double target_yaw = std::atan2(look_ahead_pos.y() - current_node.position.y(), look_ahead_pos.x() - current_node.position.x());
                    next_yaw = current_node.yaw + normalizeAngle(target_yaw - current_node.yaw); // Find the closest yaw to avoid discontinuities
                }

            }
            else
            {
                double target_yaw = std::atan2(obstacle_traj.pwp.eval(next_time).y() - next_position.y(), obstacle_traj.pwp.eval(next_time).x() - next_position.x());
                next_yaw = current_node.yaw + normalizeAngle(target_yaw - current_node.yaw); // Find the closest yaw to avoid discontinuities
            }
            
            double yaw_diff = std::abs(next_yaw - current_node.yaw);

            // Calculate utility for this potential next state
            double utility = computeUtility(agent_trajectory_pwp, obstacle_trajectories, current_node.time, current_node, obstacle_traj.id, yaw_diff);

            // Add terminal yaw cost
            if (next_time - agent_trajectory_pwp.times.front() >= time_horizon) {
                utility += par_.final_yaw_weight * (next_yaw - terminal_yaw) * (next_yaw - terminal_yaw);
            }

            // Compute graph cost
            double cost = current_node.cost + utility;

            // Update time since observed for each obstacle
            std::unordered_map<int, double> next_time_since_observed = updateTimeSinceObserved(current_node, obstacle_traj.id);

            // Add new state to the open set
            open_set.emplace(next_time, next_position, next_yaw, cost, next_time_since_observed, std::make_shared<AStarNode>(current_node), obstacle_traj.id);
        }
    }

    // Backtrack from the best path to retrieve the yaw sequence
    std::vector<double> yaw_sequence;
    std::vector<int> obstacles_to_track;
    while (best_path) 
    {
        yaw_sequence.push_back(best_path->yaw);
        obstacles_to_track.push_back(best_path->obstacle_to_track);
        best_path = best_path->parent;
    }

    // Reverse the yaw sequence to be in chronological order
    std::reverse(yaw_sequence.begin(), yaw_sequence.end());

    // Reverse the obstacles_to_track sequence to be in chronological order
    std::reverse(obstacles_to_track.begin(), obstacles_to_track.end());

    // Print out the best_path's obstacle_to_track
    // std::cout << "obstacles_to_track = np.array([";
    // for (int i = 0; i < obstacles_to_track.size(); ++i) 
    // {
    //     std::cout << obstacles_to_track[i];
    //     if (i < obstacles_to_track.size() - 1) std::cout << ", ";  // Add commas between elements
    // }
    // std::cout << "])" << std::endl;

    // Remove the direction of motion from the obstacle trajectories
    obstacle_trajectories.pop_back();

    // // Wrap up the yaw_sequence
    // for (int i = 0; i < yaw_sequence.size(); ++i) 
    // {
    //     yaw_sequence[i] = normalizeAngle(yaw_sequence[i]);
    // }

    return yaw_sequence;
}

// Function to print out control point and knots
void printCandT(const std::vector<GRBLinExpr>& q_exp_, const std::vector<double>& knots)
{
    std::cout << "c = np.array([";
    std::cout << std::fixed << std::setprecision(6); // Format for consistent precision
    for (int i = 0; i < q_exp_.size(); ++i) {
        std::cout << q_exp_[i].getValue();
        if (i != q_exp_.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "])  # Yaw values" << std::endl;

    // Optional: Print the knots (if relevant to output)
    std::cout << "t = np.array([";
    for (int i = 0; i < knots.size(); ++i) {
        std::cout << knots[i];
        if (i != knots.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "])   # Knots" << std::endl;

}

// Function to solve cubic B-spline fitting using Gurobi
void YawSolver::BsplineFitting(std::vector<state>& path_out, const std::vector<double>& sampled_time_steps, const std::vector<double>& optimal_yaw_sequence, std::vector<double>& q_exp_values_out, std::vector<double>& knots_out)
{

    // Housekeeping
    resetOptimization();

    // Remove all previous constraints and variables
    removeConstraintsAndVariables();

    try {
        
        // Step 1: Define variables (control points)
        q_exp_.clear();
        for (int i = 0; i <= yaw_fit_n_; ++i) {
            q_exp_.push_back(model_.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "q_" + std::to_string(i)));
        }

        // Step 2: Define uniform knots (clamped at the ends)
        // std::vector<double> knots(yaw_fit_m_ + 1);
        std::vector<double> knots(yaw_fit_m_ + 1 - 2 * yaw_fit_p_); // for clamped knots
        double t_min = sampled_time_steps.front();
        double t_max = sampled_time_steps.back();
        for (int i = 0; i < knots.size(); ++i) {
            knots[i] = t_min + i * (t_max - t_min) / (knots.size() - 1);
        }
        // Add clamped knots
        knots.insert(knots.begin(), yaw_fit_p_, knots.front());
        knots.insert(knots.end(), yaw_fit_p_, knots.back());

        // Step 3: Define the objective function (minimize squared error)
        GRBQuadExpr objective = 0.0;
        for (int i = 0; i < sampled_time_steps.size(); ++i) {
            
            double t = sampled_time_steps[i];

            // Compute B-spline value
            
            // Identify the knot interval
            int interval = 0;
            while (t > knots[interval + 1]) {
                interval++;
            }

            // It's clamped knots, so need to adjust the interval
            interval = std::max(0, std::min(yaw_fit_n_, interval - yaw_fit_p_));

            // Compute the B-spline value
            GRBMatrix Qbs;

            Qbs.push_back(GRBVector{ q_exp_[interval], q_exp_[interval + 1], q_exp_[interval + 2], q_exp_[interval + 3] });
            Eigen::Matrix<double, 4, 1> u_interval = computeUjVector(t, interval, knots);
            Eigen::Matrix<double, -1, 1> A_i_times_u = A_pos_bs_[interval] * u_interval;
            GRBLinExpr b_spline_value = matrixMultiply(Qbs, eigenVector2std(A_i_times_u));
            // Add squared error term to the objective
            auto diff = optimal_yaw_sequence[i] - b_spline_value;
            objective += diff * diff;
        }

        model_.setObjective(objective, GRB_MINIMIZE);

        // Step 4: Add velocity constraints
        for (int i = 2; i <= (yaw_fit_n_ - 1); i++)
        {

            double ciM2 = yaw_fit_p_ / (knots[i + yaw_fit_p_ + 1 - 2] - knots[i + 1 - 2]);
            double ciM1 = yaw_fit_p_ / (knots[i + yaw_fit_p_ + 1 - 1] - knots[i + 1 - 1]);
            double ci = yaw_fit_p_ / (knots[i + yaw_fit_p_ + 1] - knots[i + 1]);

            GRBLinExpr v_iM2 = ciM2 * (q_exp_[i - 1] - q_exp_[i - 2]);
            GRBLinExpr v_iM1 = ciM1 * (q_exp_[i] - q_exp_[i - 1]);
            GRBLinExpr v_i = ci * (q_exp_[i + 1] - q_exp_[i]);

            // Add velocity constraints
            model_.addConstr(v_iM2 <= w_max_);
            model_.addConstr(v_iM1 <= w_max_);
            model_.addConstr(v_i <= w_max_);
            model_.addConstr(v_iM2 >= -w_max_);
            model_.addConstr(v_iM1 >= -w_max_);
            model_.addConstr(v_i >= -w_max_);

        }

        // Print the number of variables and constraints
        // std::cout << "Number of variables: " << model_.get(GRB_IntAttr_NumVars) << std::endl;
        // std::cout << "Number of constraints: " << model_.get(GRB_IntAttr_NumConstrs) << std::endl;

        // Step 5: Optimize the model
        model_.optimize();

        // Step 6: Output the optimized control points
        if (model_.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {

            // std::cout << bold << green << "Optimization was successful." << reset << std::endl;
            
            // Convert the control points to trajectory and PieceWisePol
            std::vector<double> q_exp_values;
            for (int i = 0; i < q_exp_.size(); ++i) {
                q_exp_values.push_back(q_exp_[i].getValue());
            }

            Eigen::RowVectorXd knots_eigen = std2EigenRowVector(knots);
            convertCPs2Path(q_exp_values, path_out, knots_eigen);

            // Print the optimized control points as yaw values
            if (verbose_) printCandT(q_exp_, knots);

            // Output the control points and knots
            q_exp_values_out = q_exp_values;
            knots_out = knots;

        } else {
            // std::cout << bold << red << "Optimization was not successful." << reset << std::endl;
        }

    } catch (GRBException& e) {
        std::cerr << "[Yaw optimization] Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "[Yaw optimization] Exception during optimization" << std::endl;
    }
}

void YawSolver::removeConstraintsAndVariables()
{
    GRBConstr* c = 0;
    c = model_.getConstrs();
    for (int i = 0; i < model_.get(GRB_IntAttr_NumConstrs); ++i)
    {
        model_.remove(c[i]);
    }

    GRBVar* vars = 0;
    vars = model_.getVars();
    for (int i = 0; i < model_.get(GRB_IntAttr_NumVars); ++i)
    {
        model_.remove(vars[i]);
    }

    model_.reset();  // Note that this function, only by itself, does NOT remove vars or constraints
}

// compute u_j
Eigen::Matrix<double, 4, 1> YawSolver::computeUjVector(double t, int j, const std::vector<double>& knots) 
{

    double u_j = (t - knots[yaw_fit_p_ + j]) / (knots[yaw_fit_p_ + j + 1] - knots[yaw_fit_p_ + j]);

    Eigen::Matrix<double, 4, 1> u_j_vector;
    u_j_vector << u_j * u_j * u_j, u_j * u_j, u_j, 1.0;

    return u_j_vector;

}

template <typename T, typename R>
GRBLinExpr YawSolver::matrixMultiply(const std::vector<std::vector<R>>& A, const std::vector<T>& x)
{
    GRBLinExpr lin_exp = 0.0;

    for (int m = 0; m < x.size(); m++)
    {
        lin_exp = lin_exp + A[0][m] * x[m];
    }

    return lin_exp;
}

template <typename T>
std::vector<T> YawSolver::eigenVector2std(const Eigen::Matrix<T, -1, 1>& x)
{

    std::vector<T> result;
    for (int i = 0; i < x.rows(); i++)
    {
        result.push_back(x(i));
    }
    return result;
}

// convert std::vector to Eigen::RowVectorXd
Eigen::RowVectorXd YawSolver::std2EigenRowVector(const std::vector<double>& x)
{
    Eigen::RowVectorXd result(x.size());
    for (int i = 0; i < x.size(); i++)
    {
        result(i) = x[i];
    }
    return result;
}

// Utility function to normalize angles between -pi and pi
double YawSolver::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Given the control points, this function returns the associated traj and PieceWisePol
// Note that if q.size()!=(N+1), then only some of the knots are used
void YawSolver::convertCPs2Path(std::vector<double>& q, std::vector<state>& path_out, Eigen::RowVectorXd& knots)
{

    int N_effective = q.size() - 1;
    Eigen::RowVectorXd knots_effective = knots.block(0, 0, 1, N_effective + yaw_fit_p_ + 2);
    int num_effective_pol = (N_effective + 1 - yaw_fit_p_); // This is not num_pol when q.size() != N+1

    Eigen::VectorXd control_points(N_effective + 1);
    #pragma omp parallel
    for (int i = 0; i < (N_effective + 1); i++)
    {
        control_points(i) = q[i];
    }

    // Construct the B-Spline
    Eigen::Spline<double, 1, Eigen::Dynamic> spline(knots_effective, control_points);

    // Fill yaw and dyaw in path_out
    #pragma omp parallel
    for (int i = 0; i < path_out.size(); i++)
    {
        double t = i * dc_;
        
        Eigen::MatrixXd derivatives = spline.derivatives(t, 2); // Compute all derivatives up to order 2

        path_out[i].setYaw(derivatives(0));  // Yaw
        path_out[i].setDYaw(derivatives(1)); // Derivative of yaw
        path_out[i].use_tracking_yaw = true;
    
    }

}
