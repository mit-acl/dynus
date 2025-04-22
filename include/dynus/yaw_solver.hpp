/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef YAW_SOLVER_HPP
#define YAW_SOLVER_HPP

// Other includes
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include "gurobi_c++.h"
#include <unordered_map>
#include <queue>
#include <functional>
#include <memory>
#include <vector>
#include "timer.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>

// Dynus includes
#include <dynus/dynus_type.hpp>
#include <dynus/utils.hpp>
#include "dgp/termcolor.hpp"

// custom typedefs
typedef std::vector<GRBLinExpr> GRBVector;
typedef std::vector<std::vector<GRBLinExpr>> GRBMatrix;

// Define prefix and placeholders
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Timer
typedef timer::Timer MyTimer;

// Define the A* search node structure
class AStarNode {
public:
    double time;
    Eigen::Vector3d position;
    double yaw;
    double cost;
    std::unordered_map<int, double> time_since_observed;
    std::shared_ptr<AStarNode> parent;
    int obstacle_to_track; // Debugging purposes

    AStarNode(double t, Eigen::Vector3d pos, double y, double c, std::unordered_map<int, double> tso,
              std::shared_ptr<AStarNode> p = nullptr, int obstacle_to_track = -100)
        : time(t), position(pos), yaw(y), cost(c), time_since_observed(tso), parent(p), obstacle_to_track(obstacle_to_track) {}

    // Custom comparison for priority queue (min-heap)
    bool operator<(const AStarNode& other) const {
        return cost > other.cost;
    }
};

class YawCallback : public GRBCallback 
{
    public:
        bool should_terminate_ = false;

        void callback() override 
        {
            if (should_terminate_) abort(); // Gracefully terminate the optimization
        }
};

// Main YawSolver class
class YawSolver 
{
public:
    // Constructor
    YawSolver();

    // Function to set parameters
    void initialize(const parameters& par, bool verbose = false);

    // Function for B-spline fitting
    void BsplineFitting(std::vector<state>& path_out, const std::vector<double>& sampled_time_steps, const std::vector<double>& optimal_yaw_sequence, std::vector<double>& q_exp_values, std::vector<double>& knots);
    
    // Function to find the optimal yaw sequence
    std::vector<double> findOptimalYaw(PieceWisePol& agent_trajectory_pwp,
                                       std::vector<dynTraj>& obstacle_trajectories,
                                       const Eigen::Vector3d& initial_pos, double initial_yaw, 
                                       double terminal_yaw, double time_horizon);

    // convert control points to trajectory and PieceWisePol
    void convertCPs2Path(std::vector<double> &q, std::vector<state> &path_out, Eigen::RowVectorXd &knots);

    // Function to stop the optimization
    void stopOptimization();

    // Function to reset the optimization
    void resetOptimization();

    // Function to set the number of threads
    void setThreads(int threads);

private:
    // Parameters
    parameters par_;

    // A* search parameters
    int dom_id_ = -1; // dom's ID is hardcoded as -1 (obstacles IDs are 0 and above)

    // Environment parameters for B-spline fitting
    GRBEnv *env_ = new GRBEnv();
    GRBModel model_ = GRBModel(*env_);

    // Callback for optimization
    YawCallback yaw_callback_;

    // Parameters for B-spline fitting
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs_;
    int yaw_fit_p_;
    int yaw_fit_m_;
    int yaw_fit_n_;
    int yaw_fit_num_segments_;
    double w_max_;
    double dc_;

    // Variables for B-spline fitting
    std::vector<GRBLinExpr> q_exp_; // B-spline control points\

    // Flags
    bool verbose_ = false;

    // Helper functions for utility calculations
    double getCollisionLikelihood(PieceWisePol& agent_trajectory_pwp, dynTraj& obstacle_trajectory, double current_time);
    double getVelocityScore(dynTraj& obstacle_trajectory, double current_time);
    double getTotalCollisionLikelihood(PieceWisePol& agent_trajectory_pwp,
                                       std::vector<dynTraj>& obstacle_trajectories,
                                       double current_time, int obstacle_id);
    double getTotalVelocityScore(PieceWisePol& agent_trajectory_pwp,
                                 std::vector<dynTraj>& obstacle_trajectories,
                                 double current_time, int obstacle_id);
    double getTotalTimeSinceObserved(const AStarNode& current_node,
                                     std::vector<dynTraj>& obstacle_trajectories,
                                     int obstacle_id);
    double getTotalProximity(PieceWisePol& agent_trajectory_pwp,
                             std::vector<dynTraj>& obstacle_trajectories,
                             double current_time, int obstacle_id);

    // Utility function for A* search
    double computeUtility(PieceWisePol& agent_trajectory_pwp,
                          std::vector<dynTraj>& obstacle_trajectories,
                          double current_time, const AStarNode& current_node, int obstacle_id,
                          double yaw_diff);

    // Update the time since observed for each obstacle
    std::unordered_map<int, double> updateTimeSinceObserved(const AStarNode& current_node, int obstacle_id);

    // Function to compute u_j vector
    Eigen::Matrix<double, 4, 1> computeUjVector(double t, int j, const std::vector<double>& knots);

    // Matrix multiplication helper function
    template <typename T, typename R>
    GRBLinExpr matrixMultiply(const std::vector<std::vector<R>>& A, const std::vector<T>& x);

    // Remove constraints and variables
    void removeConstraintsAndVariables();

    // convert std::vector to Eigen::RowVectorXd
    Eigen::RowVectorXd std2EigenRowVector(const std::vector<double>& x);

    // Convert Eigen vector to std::vector
    template <typename T>
    std::vector<T> eigenVector2std(const Eigen::Matrix<T, -1, 1>& x);

    // Utility function to normalize angles between -pi and pi
    double normalizeAngle(double angle);
};

#endif // YAW_SOLVER_HPP