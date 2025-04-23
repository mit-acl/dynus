/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include "timer.hpp"
#include <dynus/yaw_solver.hpp>

// Helper function to create polynomial coefficients for a linear motion
Eigen::Matrix<double, 4, 1> createLinearCoefficients(double a, double b) {
    Eigen::Matrix<double, 4, 1> coeff;
    coeff << 0.0, 0.0, a, b;
    return coeff;
}

// Helper function to create example agent trajectory using PieceWisePol
dynTraj createAgentTrajectory() {
    dynTraj agent;
    agent.pwp.times.resize(2);  // 10 intervals, 11 time points (0 to 10 seconds)
    agent.pwp.times[0] = 0.0;
    agent.pwp.times[1] = 10.0;

    // Define linear motion for the agent with constant velocity along x and constant y, z
    agent.pwp.coeff_x.push_back(createLinearCoefficients(1.5, -3.0)); // x = 1.5 * t - 3
    agent.pwp.coeff_y.push_back(createLinearCoefficients(0.0, 1.5));  // y = 1.5
    agent.pwp.coeff_z.push_back(createLinearCoefficients(0.0, 0.0));  // z = 0

    return agent;
}

// Helper function to create an obstacle trajectory with oscillatory motion
dynTraj createObstacleTrajectory(double offset_x = 0.0, double offset_y = 0.0, int id = 0)
{
    dynTraj obstacle;
    obstacle.pwp.times.resize(11);  // 10 intervals, 11 time points (0 to 10 seconds)
    for (int i = 0; i <= 10; ++i) {
        obstacle.pwp.times[i] = i;
    }

    // Create oscillatory coefficients for each interval
    // Note that this part is different from Python example (test_optimize_yaw_multiple_obsts.ipynb)
    for (int i = 0; i < 10; ++i) {
        double t_offset = obstacle.pwp.times[i] + 1;  // Shifting time by 1 as in the Python example
        obstacle.pwp.coeff_x.push_back(createLinearCoefficients(0.6, offset_x + 2 * 0.6 * std::sin(2 * t_offset / 10)));
        obstacle.pwp.coeff_y.push_back(createLinearCoefficients(0.6, offset_y - 2 * 0.6 * std::cos(t_offset / 10)));
        obstacle.pwp.coeff_z.push_back(createLinearCoefficients(-0.6, 0.0));
    }

    obstacle.ekf_cov_p = Eigen::Vector3d(0.5, 0.5, 0.5);
    obstacle.poly_cov = Eigen::Vector3d(2.0, 2.0, 2.0);
    obstacle.id = id;

    return obstacle;
}

// Helper function to print out the results
void printResults(const std::vector<double>& optimal_yaw_sequence, dynTraj& agent_trajectory, std::vector<dynTraj>& obstacle_trajectories) 
{

    // Output optimal yaw sequence as a Python numpy array
    std::cout << "optimal_yaw_sequence = np.array([";
    for (int i = 0; i < optimal_yaw_sequence.size(); ++i) 
    {
        std::cout << optimal_yaw_sequence[i];
        if (i < optimal_yaw_sequence.size() - 1) std::cout << ", ";  // Add commas between elements
    }
    std::cout << "])" << std::endl;

    // Output agent position sequence as a Python numpy array ( you can just copy and past this to visualize_test_yaw_solver_cpp.ipynb)
    std::cout << "agent_trajectory = np.array([";
    for (double t = 0.0; t <= 10.0; t += 1.0) {
        Eigen::Vector3d pos = agent_trajectory.pwp.eval(t);
        std::cout << "[" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]";
        if (t < 10.0) std::cout << ", ";  // Add commas between elements
    }
    std::cout << "])" << std::endl;

    // Output obstacle trajectories as Python numpy arrays
    for (int i = 0; i < obstacle_trajectories.size(); ++i) 
    {
        std::cout << "obstacle_trajectory_" << i << " = np.array([";
        for (double t = 0.0; t <= 10.0; t += 1.0) {
            Eigen::Vector3d pos = obstacle_trajectories[i].pwp.eval(t);
            std::cout << "[" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]";
            if (t < 10.0) std::cout << ", ";  // Add commas between elements
        }
        std::cout << "])" << std::endl;
    }

}

// Helper function to print out the path results
void printPathResults(const std::vector<state>& path_out, const std::vector<double>& path_out_t)
{
    std::cout << "path_out_yaw = np.array([";
    for (int i = 0; i < path_out.size(); ++i) 
    {
        std::cout << path_out[i].yaw;
        if (i < path_out.size() - 1) std::cout << ", ";  // Add commas between elements
    }
    std::cout << "])" << std::endl;

    std::cout << "path_out_dyaw = np.array([";
    for (int i = 0; i < path_out.size(); ++i) 
    {
        std::cout << path_out[i].dyaw;
        if (i < path_out.size() - 1) std::cout << ", ";  // Add commas between elements
    }
    std::cout << "])" << std::endl;

    std::cout << "t = np.array([";
    for (int i = 0; i < path_out_t.size(); ++i) 
    {
        std::cout << path_out_t[i];
        if (i < path_out_t.size() - 1) std::cout << ", ";  // Add commas between elements
    }
    std::cout << "])" << std::endl;
}

// Main function to test YawSolver
int main() 
{

    // Initialize parameters
    parameters params;

    params.dc = 0.01;
    params.yaw_collision_weight = 10.0;
    params.yaw_time_weight = 1.0;
    params.yaw_proximity_weight = 30.0;  
    params.yaw_velocity_weight = 1.0;    
    params.yaw_change_weight = 0.1;      
    params.cutoff_distance = 4.0;
    params.num_yaw_fit_poly = 5;
    params.yaw_fit_degree = 3;
    params.w_max = 2.0;
    params.num_samples_collision_likelihood = 10;
    params.num_samples_velocity_score = 10;

    // Create agent and obstacle trajectories
    dynTraj agent_trajectory = createAgentTrajectory();
    std::vector<dynTraj> obstacle_trajectories;
    obstacle_trajectories.push_back(createObstacleTrajectory(0.0, 0.0, 0));
    obstacle_trajectories.push_back(createObstacleTrajectory(5.0, 3.0, 1));
    // obstacle_trajectories.push_back(createObstacleTrajectory(8.0, 0.0, 2));

    // Initialize YawSolver
    YawSolver yaw_solver;

    // Set parameters
    yaw_solver.initialize(params, true);

    // Define initial position, yaw, and time horizon
    Eigen::Vector3d initial_pos = agent_trajectory.pwp.eval(0.0);
    double initial_yaw = 0.0;
    double terminal_yaw = 0.0;
    double time_horizon = 10.0;

    // Get path_out from agent_trajectory
    std::vector<state> path_out;
    std::vector<double> path_out_t;
    for (double t = 0.0; t <= 10.0; t += params.dc) 
    {
        state s;
        s.pos = agent_trajectory.pwp.eval(t);
        path_out.push_back(s);
        path_out_t.push_back(t);
    }

    // Find optimal yaw sequence
    // MyTimer timer(true);
    std::vector<double> optimal_yaw_sequence = yaw_solver.findOptimalYaw(agent_trajectory.pwp, obstacle_trajectories, initial_pos, initial_yaw, terminal_yaw, time_horizon);
    // std::cout << "Elapsed time for A*: " << timer.getElapsedMs() << " ms" << std::endl;

    // Bspline fitting
    // MyTimer timer2(true);
    std::vector<double> q_exp_values;
    std::vector<double> knots;
    yaw_solver.BsplineFitting(path_out, obstacle_trajectories.front().pwp.times, optimal_yaw_sequence, q_exp_values, knots);
    // std::cout << "Elapsed time for Bspline fitting: " << timer2.getElapsedMs() << " ms" << std::endl;

    // Output the results in a Python-friendly format
    printResults(optimal_yaw_sequence, agent_trajectory, obstacle_trajectories);

    // Output path_out in a Python-friendly format
    printPathResults(path_out, path_out_t);

    return 0;
}
