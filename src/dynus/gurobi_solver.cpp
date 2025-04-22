/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <dynus/gurobi_solver.hpp>
#include <dynus/gurobi_solver_utils.hpp>
#include <chrono>
#include <unistd.h>

void mycallback::callback()
{ // This function is called periodically along the optimization process.
  //  It is called several times more after terminating the program
  if (should_terminate_ == true)
  {
    GRBCallback::abort(); // This function only does effect when inside the function callback() of this class
    // terminated_ = true;
  }
}

void SolverGurobi::stopExecution()
{
  cb_.should_terminate_ = true;
}

void SolverGurobi::resetToNominalState()
{
  cb_.should_terminate_ = false;
}

SolverGurobi::SolverGurobi()
{

  // Model
  m_.set(GRB_StringAttr_ModelName, "planning");

  // Debug
  m_.set(GRB_IntParam_OutputFlag, 1); // 0: no output, 1: output

  // MIPFocus
  m_.set(GRB_IntParam_MIPFocus, 1); // 1: quick, 2: optimal, 3: bound

  // // MIPGap
  // m_.set(GRB_DoubleParam_MIPGap, 0.01);

  // Time limit
  m_.set(GRB_DoubleParam_TimeLimit, 1.0); // seconds

  // ScaleFlag
  // m_.set(GRB_IntParam_ScaleFlag, 2);

  // Nonconvex
  // m_.set(GRB_IntParam_NonConvex, 2);

  // // Solution pool
  // m_.set(GRB_IntParam_SolutionNumber, 10);

  // Presolve
  // m_.set(GRB_IntParam_Presolve, 2);

  // // Heuristics
  // m_.set(GRB_DoubleParam_Heuristics, 1.0);

  // // Cuts
  // m_.set(GRB_IntParam_Cuts, 2);

  // Method
  // m_.set(GRB_IntParam_Method, 2);  // Barrier method

  // Numeric focus
  m_.set(GRB_IntParam_NumericFocus, 1); // 0: automatic 1:speed, 3: stability

  // ConcurrentMIP
  // m_.set(GRB_IntParam_ConcurrentMIP, 8);

  // Set the callback
  m_.setCallback(&cb_); // The callback will be called periodically along the optimization

  // Get basis converter (Q_{MINVO} = M_{BE2MV} * Q_{BEZIER})
  // Get std version of the basis converter
  M_be2mv_ = eigenMatrix2std(basis_converter_.getMinvoPosConverterFromBezier());
  A_pos_mv_rest_inv_ = basis_converter_.A_pos_mv_rest_inv;
  A_vel_mv_rest_inv_ = basis_converter_.A_vel_mv_rest_inv;
  A_accel_mv_rest_inv_ = basis_converter_.A_accel_mv_rest_inv;
}

void SolverGurobi::setUseRefPoints(bool use_ref_points, int num_ref_sample_points)
{
  use_ref_points_ = use_ref_points;
  num_ref_sample_points_ = num_ref_sample_points;
}

void SolverGurobi::setClosedFormSolutionParams(int closed_form_time_allocation_adj_iter_max, double closed_form_initial_factor, double closed_form_factor_increment, double closed_form_factor_initial_decrement)
{
  closed_form_time_allocation_adj_iter_max_ = closed_form_time_allocation_adj_iter_max;
  closed_form_initial_factor_ = closed_form_initial_factor;
  closed_form_factor_ = closed_form_initial_factor_;
  closed_form_factor_increment_ = closed_form_factor_increment;
  closed_form_factor_initial_decrement_ = closed_form_initial_factor_ - closed_form_factor_increment_;
}

void SolverGurobi::setOptimizationType(std::string optimization_type)
{
  optimization_type_ = optimization_type;
}

void SolverGurobi::setN(int N)
{
  N_ = N;
}

void SolverGurobi::setMode(int mode)
{
  mode_ = mode;
}

void SolverGurobi::setT0(double t0)
{
  t0_ = t0;
}

void SolverGurobi::setMapSize(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, double res)
{
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  z_min_ = z_min;
  z_max_ = z_max;
  res_ = res;
}

void SolverGurobi::setVehicleType(std::string vehicle_type)
{
  vehicle_type_ = vehicle_type;
}

std::vector<double> SolverGurobi::computeCubicPolynomial3D(const Eigen::Vector3d &pos0, const Eigen::Vector3d &vel0,
                                                           const Eigen::Vector3d &posf, const Eigen::Vector3d &velf,
                                                           double t0, double tf)
{
  // Time matrix
  Eigen::Matrix4d T;
  T << std::pow(t0, 3), std::pow(t0, 2), t0, 1,
      std::pow(tf, 3), std::pow(tf, 2), tf, 1,
      3 * std::pow(t0, 2), 2 * t0, 1, 0,
      3 * std::pow(tf, 2), 2 * tf, 1, 0;

  // Solve for each component (x, y, z)
  Eigen::Vector3d a, b, c, d;
  for (int i = 0; i < 3; ++i)
  {
    // Boundary conditions vector for the i-th component
    Eigen::Vector4d B(pos0[i], posf[i], vel0[i], velf[i]);

    // Solve for coefficients
    Eigen::Vector4d C = T.colPivHouseholderQr().solve(B);

    // Store coefficients
    a[i] = C(0);
    b[i] = C(1);
    c[i] = C(2);
    d[i] = C(3);
  }

  // Return the coefficients
  return {a.x(), b.x(), c.x(), d.x(), a.y(), b.y(), c.y(), d.y(), a.z(), b.z(), c.z(), d.z()};
}

void SolverGurobi::setPlannerMode(std::string planner_mode)
{
  planner_mode_ = planner_mode;
}

void SolverGurobi::setNumObsDistSamples(int num_obs_dist_samples)
{
  num_obs_dist_samples_ = num_obs_dist_samples;
}

void SolverGurobi::setWeights(double control_cost_weight, double dynamic_weight, double final_pos_weight, double final_vel_weight, double final_accel_weight, double final_yaw_cost_weight, double total_yaw_diff_weight, double ref_point_weight)
{
  control_cost_weight_ = control_cost_weight;
  dynamic_weight_ = dynamic_weight;
  final_pos_weight_ = final_pos_weight;
  final_vel_weight_ = final_vel_weight;
  final_accel_weight_ = final_accel_weight;
  final_yaw_cost_weight_ = final_yaw_cost_weight;
  total_yaw_diff_weight_ = total_yaw_diff_weight;
  ref_point_weight_ = ref_point_weight;
}

void SolverGurobi::setCollisionClearance(double collision_clearance)
{
  collision_clearance_ = collision_clearance;
}

void SolverGurobi::setFlags(bool use_hard_constr_for_final_yaw, bool debug_verbose)
{
  use_hard_constr_for_final_yaw_ = use_hard_constr_for_final_yaw;
  debug_verbose_ = debug_verbose;
}

void SolverGurobi::setConstraintFlags(bool use_hard_constr_for_final_state, bool use_hard_dynamic_constraints)
{
  use_hard_constr_for_final_state_ = use_hard_constr_for_final_state;
  use_hard_dynamic_constraints_ = use_hard_dynamic_constraints;
}

void SolverGurobi::createVars()
{

  if (N_ >= 4)
  {
    // Create free parameter for each coordinate:
    GRBVar d3x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d3x");
    GRBVar d3y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d3y");
    GRBVar d3z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d3z");

    // Save them in a member variable for later use in constraints and objective.
    d3_.clear();
    d3_.push_back(d3x);
    d3_.push_back(d3y);
    d3_.push_back(d3z);
  }

  if (N_ >= 5)
  {
    // Create free parameter for each coordinate:
    GRBVar d4x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d4x");
    GRBVar d4y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d4y");
    GRBVar d4z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d4z");

    // Save them in a member variable for later use in constraints and objective.
    d4_.clear();
    d4_.push_back(d4x);
    d4_.push_back(d4y);
    d4_.push_back(d4z);
  }

  if (N_ >= 6)
  {
    // Create free parameter for each coordinate:
    GRBVar d5x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d5x");
    GRBVar d5y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d5y");
    GRBVar d5z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d5z");

    // Save them in a member variable for later use in constraints and objective.
    d5_.clear();
    d5_.push_back(d5x);
    d5_.push_back(d5y);
    d5_.push_back(d5z);
  }

  if (N_ >= 7)
  {
    // Create free parameter for each coordinate:
    GRBVar d6x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d6x");
    GRBVar d6y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d6y");
    GRBVar d6z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d6z");

    // Save them in a member variable for later use in constraints and objective.
    d6_.clear();
    d6_.push_back(d6x);
    d6_.push_back(d6y);
    d6_.push_back(d6z);
  }

  if (N_ >= 8)
  {
    // Create free parameter for each coordinate:
    GRBVar d7x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d7x");
    GRBVar d7y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d7y");
    GRBVar d7z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d7z");

    // Save them in a member variable for later use in constraints and objective.
    d7_.clear();
    d7_.push_back(d7x);
    d7_.push_back(d7y);
    d7_.push_back(d7z);
  }

  if (N_ >= 9)
  {
    // Create free parameter for each coordinate:
    GRBVar d8x = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d8x");
    GRBVar d8y = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d8y");
    GRBVar d8z = m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "d8z");

    // Save them in a member variable for later use in constraints and objective.
    d8_.clear();
    d8_.push_back(d8x);
    d8_.push_back(d8y);
    d8_.push_back(d8z);
  }
}

void SolverGurobi::setX()
{
  // Depending on N_, we initialize coefficients as GRBLinExpr
  if (N_ == 4)
  {
    computeDependentCoefficientsN4(); // Compute dependent coefficients (a, b, c, d)
    // computeControlPointsN4();         // Compute control points for pos, vel, accel, jerk
  }
  else if (N_ == 5)
  {
    computeDependentCoefficientsN5();
    // computeControlPointsN5();
  }
  else if (N_ == 6)
  {
    computeDependentCoefficientsN6();
    // computeControlPointsN6();
  }
  else
  {
    std::cout << "N should be 6" << std::endl;
  }
}

void SolverGurobi::getInitialAndFinalConditions(double &P0, double &V0, double &A0, double &Pf, double &Vf, double &Af, int axis)
{
  // Get initial and final conditions
  P0 = x0_[axis];
  V0 = x0_[axis + 3];
  A0 = x0_[axis + 6];
  Pf = xf_[axis];
  Vf = xf_[axis + 3];
  Af = xf_[axis + 6];
}

void SolverGurobi::computeDependentCoefficientsN4()
{

  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++)
  {

    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];

    // C++ expressions for composite cubic Bézier control coefficients (one coordinate) in terms of d3:
    GRBLinExpr a0 = d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) - 2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) / (6 * pow(T0, 3) * pow(T3, 2) + 12 * pow(T0, 2) * T1 * pow(T3, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) + 6 * T0 * pow(T1, 2) * pow(T3, 2) + 6 * T0 * T1 * T2 * pow(T3, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 = d3_var * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 3) * pow(T3, 2) + 4 * A0 * pow(T0, 2) * T1 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * A0 * T0 * pow(T1, 2) * pow(T3, 2) + 3 * A0 * T0 * T1 * T2 * pow(T3, 2) + A0 * T0 * pow(T2, 2) * pow(T3, 2) + 2 * Af * pow(T0, 2) * T2 * pow(T3, 2) + Af * pow(T0, 2) * pow(T3, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) + 3 * Af * T0 * T1 * pow(T3, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) + 2 * Af * pow(T2, 3) * pow(T3, 2) + 2 * Af * pow(T2, 2) * pow(T3, 3) + 6 * P0 * T0 * pow(T3, 2) + 12 * P0 * T1 * pow(T3, 2) + 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * pow(T0, 2) * T2 + 6 * Pf * pow(T0, 2) * T3 + 18 * Pf * T0 * T1 * T2 + 18 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 + 18 * Pf * pow(T1, 2) * T2 + 18 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) + 36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 12 * Pf * pow(T2, 2) * T3 - 6 * pow(T0, 2) * T2 * T3 * Vf + 4 * pow(T0, 2) * pow(T3, 2) * V0 - 4 * pow(T0, 2) * pow(T3, 2) * Vf - 18 * T0 * T1 * T2 * T3 * Vf + 12 * T0 * T1 * pow(T3, 2) * V0 - 12 * T0 * T1 * pow(T3, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * Vf + 6 * T0 * T2 * pow(T3, 2) * V0 - 12 * T0 * T2 * pow(T3, 2) * Vf - 18 * pow(T1, 2) * T2 * T3 * Vf + 6 * pow(T1, 2) * pow(T3, 2) * V0 - 12 * pow(T1, 2) * pow(T3, 2) * Vf - 24 * T1 * pow(T2, 2) * T3 * Vf + 6 * T1 * T2 * pow(T3, 2) * V0 - 24 * T1 * T2 * pow(T3, 2) * Vf - 6 * pow(T2, 3) * T3 * Vf + 2 * pow(T2, 2) * pow(T3, 2) * V0 - 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 6 * pow(T0, 2) * T1 * T2 * pow(T3, 2) + 12 * T0 * pow(T1, 3) * pow(T3, 2) + 18 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 4) * pow(T3, 2) + 12 * pow(T1, 3) * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr b1 = d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) - A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    GRBLinExpr c1 = d3_var * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) + A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) - 2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 - 6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 - 4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf + 4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf + 8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 + 2 * T1 * T2 * pow(T3, 2) * V0) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    GRBLinExpr d1 = d3_var * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) - Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) + 6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) + 6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 - 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf + 8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf + 6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * pow(T3, 2) * V0) / (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
    GRBLinExpr a2 = d3_var * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + (-A0 * pow(T0, 2) * pow(T3, 2) - A0 * T0 * T1 * pow(T3, 2) - 2 * Af * T0 * T1 * pow(T3, 2) - 4 * Af * T0 * T2 * pow(T3, 2) - Af * T0 * pow(T3, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) - 8 * Af * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * pow(T3, 3) - 6 * Af * pow(T2, 2) * pow(T3, 2) - 3 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T0 * T1 - 12 * Pf * T0 * T2 - 6 * Pf * T0 * T3 - 6 * Pf * pow(T1, 2) - 24 * Pf * T1 * T2 - 12 * Pf * T1 * T3 - 18 * Pf * pow(T2, 2) - 18 * Pf * T2 * T3 + 6 * T0 * T1 * T3 * Vf + 12 * T0 * T2 * T3 * Vf - 4 * T0 * pow(T3, 2) * V0 + 4 * T0 * pow(T3, 2) * Vf + 6 * pow(T1, 2) * T3 * Vf + 24 * T1 * T2 * T3 * Vf - 2 * T1 * pow(T3, 2) * V0 + 8 * T1 * pow(T3, 2) * Vf + 18 * pow(T2, 2) * T3 * Vf + 12 * T2 * pow(T3, 2) * Vf) / (6 * T0 * T1 * T2 * pow(T3, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T2, 3) * pow(T3, 2));
    GRBLinExpr b2 = d3_var * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) + Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) + 6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 + 12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf + 4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf + 2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf - 12 * T2 * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr c2 = d3_var * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) + 2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) + 2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) - 2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) - 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 + 6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) - 12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf - 4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf - 4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 + 6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr d2 = d3_var * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
    GRBLinExpr a3 = -d3_var / pow(T3, 3) + (1.0 / 2.0) * (Af * pow(T3, 2) + 2 * Pf - 2 * T3 * Vf) / pow(T3, 3);
    GRBLinExpr b3 = 3 * d3_var / pow(T3, 2) + (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf) / pow(T3, 2);
    GRBLinExpr c3 = -3 * d3_var / T3 + (1.0 / 2.0) * (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3;
    GRBLinExpr d3 = d3_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3});
  }
}

void SolverGurobi::computeDependentCoefficientsN5()
{

  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++)
  {

    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];
    GRBVar d4_var = d4_[axis];

    // C++ expressions for composite cubic Bézier control coefficients
    GRBLinExpr a0 = d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + d4_var * (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) - T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) - 2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) / (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 4 * A0 * T0 * T1 * T3 * pow(T4, 2) - 2 * A0 * T0 * T2 * T3 * pow(T4, 2) - A0 * pow(T1, 2) * T3 * pow(T4, 2) - A0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) / (6 * pow(T0, 3) * T3 * pow(T4, 2) + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 = d3_var * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + d4_var * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 3) * T3 * pow(T4, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + 3 * A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + A0 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T0, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T3 * pow(T4, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * pow(T4, 3) - 6 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T1 * T3 * pow(T4, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 3) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T2 * T3 * pow(T4, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 6 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T4, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T4, 3) - 12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * T2 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 3) * pow(T4, 3) - 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * P0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 - 18 * Pf * pow(T0, 2) * T2 * T4 - 6 * Pf * pow(T0, 2) * pow(T3, 2) - 12 * Pf * pow(T0, 2) * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 - 54 * Pf * T0 * T1 * T2 * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) - 36 * Pf * T0 * T1 * T3 * T4 - 24 * Pf * T0 * pow(T2, 2) * T3 - 36 * Pf * T0 * pow(T2, 2) * T4 - 18 * Pf * T0 * T2 * pow(T3, 2) - 36 * Pf * T0 * T2 * T3 * T4 - 36 * Pf * pow(T1, 2) * T2 * T3 - 54 * Pf * pow(T1, 2) * T2 * T4 - 18 * Pf * pow(T1, 2) * pow(T3, 2) - 36 * Pf * pow(T1, 2) * T3 * T4 - 48 * Pf * T1 * pow(T2, 2) * T3 - 72 * Pf * T1 * pow(T2, 2) * T4 - 36 * Pf * T1 * T2 * pow(T3, 2) - 72 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * pow(T2, 3) * T3 - 18 * Pf * pow(T2, 3) * T4 - 12 * Pf * pow(T2, 2) * pow(T3, 2) - 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * pow(T0, 2) * T2 * T3 * T4 * Vf + 12 * pow(T0, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T0, 2) * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 + 8 * pow(T0, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * T1 * T2 * T3 * T4 * Vf + 36 * T0 * T1 * T2 * pow(T4, 2) * Vf + 18 * T0 * T1 * pow(T3, 2) * T4 * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * V0 + 24 * T0 * T1 * T3 * pow(T4, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T0 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T2 * T3 * pow(T4, 2) * V0 + 24 * T0 * T2 * T3 * pow(T4, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * T4 * Vf + 36 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 18 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 6 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T4, 2) * Vf + 36 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T1 * T2 * T3 * pow(T4, 2) * V0 + 48 * T1 * T2 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 3) * T3 * T4 * Vf + 12 * pow(T2, 3) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * pow(T2, 2) * T3 * pow(T4, 2) * V0 + 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 12 * T0 * pow(T1, 3) * T3 * pow(T4, 2) + 18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 4) * T3 * pow(T4, 2) + 12 * pow(T1, 3) * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr b1 = d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) - A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr c1 = d3_var * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) + 12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 + 18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) + 24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 - 12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf + 2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr d1 = d3_var * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 + 18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) + 12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 + 18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf - 12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf + 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 - 16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) / (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * pow(T4, 2));
    GRBLinExpr a2 = d3_var * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + d4_var * (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) - 4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) - T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) - 2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) - 8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) - 2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 2) * T3 * pow(T4, 2) - A0 * T0 * T1 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * pow(T4, 3) + 8 * Af * T0 * T2 * T3 * pow(T4, 2) + 6 * Af * T0 * T2 * pow(T4, 3) + 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T3 * pow(T4, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T1, 2) * pow(T4, 3) + 16 * Af * T1 * T2 * T3 * pow(T4, 2) + 12 * Af * T1 * T2 * pow(T4, 3) + 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T1 * T3 * pow(T4, 3) + 12 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 9 * Af * pow(T2, 2) * pow(T4, 3) + 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T3 + 18 * Pf * T0 * T1 * T4 + 24 * Pf * T0 * T2 * T3 + 36 * Pf * T0 * T2 * T4 + 6 * Pf * T0 * pow(T3, 2) + 12 * Pf * T0 * T3 * T4 + 12 * Pf * pow(T1, 2) * T3 + 18 * Pf * pow(T1, 2) * T4 + 48 * Pf * T1 * T2 * T3 + 72 * Pf * T1 * T2 * T4 + 12 * Pf * T1 * pow(T3, 2) + 24 * Pf * T1 * T3 * T4 + 36 * Pf * pow(T2, 2) * T3 + 54 * Pf * pow(T2, 2) * T4 + 18 * Pf * T2 * pow(T3, 2) + 36 * Pf * T2 * T3 * T4 - 12 * T0 * T1 * T3 * T4 * Vf - 12 * T0 * T1 * pow(T4, 2) * Vf - 24 * T0 * T2 * T3 * T4 * Vf - 24 * T0 * T2 * pow(T4, 2) * Vf - 6 * T0 * pow(T3, 2) * T4 * Vf - 4 * T0 * T3 * pow(T4, 2) * V0 - 8 * T0 * T3 * pow(T4, 2) * Vf - 12 * pow(T1, 2) * T3 * T4 * Vf - 12 * pow(T1, 2) * pow(T4, 2) * Vf - 48 * T1 * T2 * T3 * T4 * Vf - 48 * T1 * T2 * pow(T4, 2) * Vf - 12 * T1 * pow(T3, 2) * T4 * Vf - 2 * T1 * T3 * pow(T4, 2) * V0 - 16 * T1 * T3 * pow(T4, 2) * Vf - 36 * pow(T2, 2) * T3 * T4 * Vf - 36 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T2 * pow(T3, 2) * T4 * Vf - 24 * T2 * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 3) * T3 * pow(T4, 2));
    GRBLinExpr b2 = d3_var * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) - 4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) - 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) - 8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) - 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) - 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 - 6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 - 36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 - 24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) - 36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf + 6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 + 8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf + 24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf + 2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr c2 = d3_var * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) - 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) + 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 - 6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 - 12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 - 6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 + 12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) + 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf + 12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf + 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 + 12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf - 2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf - 12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr d2 = d3_var * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * Af * T1 * pow(T2, 3) * pow(T4, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 + 18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 + 12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 + 12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 + 24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 + 18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 + 24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 + 24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 - 24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * pow(T4, 2));
    GRBLinExpr a3 = (1.0 / 2.0) * (-2 * Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 6 * Pf * T3 - 6 * Pf * T4 + 6 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / (pow(T3, 2) * pow(T4, 2)) - d3_var / pow(T3, 3) + d4_var * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    GRBLinExpr b3 = (1.0 / 2.0) * (4 * Af * T3 * pow(T4, 2) + 3 * Af * pow(T4, 3) + 12 * Pf * T3 + 18 * Pf * T4 - 12 * T3 * T4 * Vf - 12 * pow(T4, 2) * Vf) / (T3 * pow(T4, 2)) + 3 * d3_var / pow(T3, 2) + d4_var * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    GRBLinExpr c3 = (-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 + 3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / pow(T4, 2) - 3 * d3_var / T3 + d4_var * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    GRBLinExpr d3 = d3_var;
    GRBLinExpr a4 = -d4_var / pow(T4, 3) + (1.0 / 2.0) * (Af * pow(T4, 2) + 2 * Pf - 2 * T4 * Vf) / pow(T4, 3);
    GRBLinExpr b4 = 3 * d4_var / pow(T4, 2) + (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf) / pow(T4, 2);
    GRBLinExpr c4 = -3 * d4_var / T4 + (1.0 / 2.0) * (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4;
    GRBLinExpr d4 = d4_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4});
  }
}

void SolverGurobi::computeDependentCoefficientsN6()
{

  // Clear x_
  x_.clear();

  // Get time allocations
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];
  double T5 = dt_[5];

  // Loop thru each axis
  for (int axis = 0; axis < 3; axis++)
  {

    // Get initial and final conditions
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // Get free parameter for the cubic Bézier control coefficients
    GRBVar d3_var = d3_[axis];
    GRBVar d4_var = d4_[axis];
    GRBVar d5_var = d5_[axis];

    // C++ expressions for composite cubic Bézier control coefficients
    GRBLinExpr a0 = d3_var * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + d4_var * (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) - T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) - 2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) / (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_var * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 3) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (6 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 12 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 6 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr b0 = (1.0 / 2.0) * A0;
    GRBLinExpr c0 = V0;
    GRBLinExpr d0 = P0;
    GRBLinExpr a1 = d3_var * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + d4_var * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_var * (-4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 6 * pow(T0, 2) * T2 * T3 * T4 * T5 - 2 * pow(T0, 2) * T2 * T3 * pow(T5, 2) - 3 * pow(T0, 2) * T2 * pow(T4, 3) - 6 * pow(T0, 2) * T2 * pow(T4, 2) * T5 - 3 * pow(T0, 2) * T2 * T4 * pow(T5, 2) - 2 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 3 * pow(T0, 2) * pow(T3, 2) * T4 * T5 - pow(T0, 2) * pow(T3, 2) * pow(T5, 2) - 2 * pow(T0, 2) * T3 * pow(T4, 3) - 4 * pow(T0, 2) * T3 * pow(T4, 2) * T5 - 2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * T0 * pow(T2, 2) * T3 * T4 * T5 - 4 * T0 * pow(T2, 2) * T3 * pow(T5, 2) - 6 * T0 * pow(T2, 2) * pow(T4, 3) - 12 * T0 * pow(T2, 2) * pow(T4, 2) * T5 - 6 * T0 * pow(T2, 2) * T4 * pow(T5, 2) - 6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T2 * pow(T3, 2) * T4 * T5 - 3 * T0 * T2 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T2 * T3 * pow(T4, 3) - 12 * T0 * T2 * T3 * pow(T4, 2) * T5 - 6 * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 24 * T1 * pow(T2, 2) * T3 * T4 * T5 - 8 * T1 * pow(T2, 2) * T3 * pow(T5, 2) - 12 * T1 * pow(T2, 2) * pow(T4, 3) - 24 * T1 * pow(T2, 2) * pow(T4, 2) * T5 - 12 * T1 * pow(T2, 2) * T4 * pow(T5, 2) - 12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * T2 * pow(T3, 2) * T4 * T5 - 6 * T1 * T2 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T2 * T3 * pow(T4, 3) - 24 * T1 * T2 * T3 * pow(T4, 2) * T5 - 12 * T1 * T2 * T3 * T4 * pow(T5, 2) - 4 * pow(T2, 3) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * T3 * T4 * T5 - 2 * pow(T2, 3) * T3 * pow(T5, 2) - 3 * pow(T2, 3) * pow(T4, 3) - 6 * pow(T2, 3) * pow(T4, 2) * T5 - 3 * pow(T2, 3) * T4 * pow(T5, 2) - 4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) - 4 * pow(T2, 2) * T3 * pow(T4, 3) - 8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) + 3 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 4) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T0, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T0, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T0, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T0, 2) * T3 * T4 * pow(T5, 3) + 24 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 18 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 12 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) + 24 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 18 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 12 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 12 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) + 32 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T3 * pow(T5, 3) + 24 * Af * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T4 * pow(T5, 3) + 24 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 18 * Af * T1 * T2 * pow(T3, 2) * pow(T5, 3) + 24 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * T2 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) + 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) + 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) + 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * T0 * T3 * T4 * pow(T5, 2) + 12 * P0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 + 36 * Pf * pow(T0, 2) * T2 * T3 * T5 + 18 * Pf * pow(T0, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T2 * T4 * T5 + 12 * Pf * pow(T0, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T0, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T0, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T0, 2) * T3 * T4 * T5 + 72 * Pf * T0 * T1 * T2 * T3 * T4 + 108 * Pf * T0 * T1 * T2 * T3 * T5 + 54 * Pf * T0 * T1 * T2 * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * T4 * T5 + 36 * Pf * T0 * T1 * pow(T3, 2) * T4 + 54 * Pf * T0 * T1 * pow(T3, 2) * T5 + 36 * Pf * T0 * T1 * T3 * pow(T4, 2) + 72 * Pf * T0 * T1 * T3 * T4 * T5 + 48 * Pf * T0 * pow(T2, 2) * T3 * T4 + 72 * Pf * T0 * pow(T2, 2) * T3 * T5 + 36 * Pf * T0 * pow(T2, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * T4 * T5 + 36 * Pf * T0 * T2 * pow(T3, 2) * T4 + 54 * Pf * T0 * T2 * pow(T3, 2) * T5 + 36 * Pf * T0 * T2 * T3 * pow(T4, 2) + 72 * Pf * T0 * T2 * T3 * T4 * T5 + 72 * Pf * pow(T1, 2) * T2 * T3 * T4 + 108 * Pf * pow(T1, 2) * T2 * T3 * T5 + 54 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * T4 * T5 + 36 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 54 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 36 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 72 * Pf * pow(T1, 2) * T3 * T4 * T5 + 96 * Pf * T1 * pow(T2, 2) * T3 * T4 + 144 * Pf * T1 * pow(T2, 2) * T3 * T5 + 72 * Pf * T1 * pow(T2, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * T4 * T5 + 72 * Pf * T1 * T2 * pow(T3, 2) * T4 + 108 * Pf * T1 * T2 * pow(T3, 2) * T5 + 72 * Pf * T1 * T2 * T3 * pow(T4, 2) + 144 * Pf * T1 * T2 * T3 * T4 * T5 + 24 * Pf * pow(T2, 3) * T3 * T4 + 36 * Pf * pow(T2, 3) * T3 * T5 + 18 * Pf * pow(T2, 3) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * T4 * T5 + 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 + 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) + 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * pow(T0, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T0, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T0, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T0, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T0, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T0, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 - 16 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 72 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 54 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 72 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 72 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 54 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 72 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 36 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 36 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 - 48 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 96 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf - 96 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 96 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf - 72 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 - 96 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 3) * T3 * T4 * T5 * Vf - 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf - 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf - 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 - 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T1, 3) * T3 * T4 * pow(T5, 2) + 18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 4) * T3 * T4 * pow(T5, 2) + 12 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr b1 = d3_var * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_var * (12 * T1 * T2 * T3 * pow(T4, 2) + 18 * T1 * T2 * T3 * T4 * T5 + 6 * T1 * T2 * T3 * pow(T5, 2) + 9 * T1 * T2 * pow(T4, 3) + 18 * T1 * T2 * pow(T4, 2) * T5 + 9 * T1 * T2 * T4 * pow(T5, 2) + 6 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T1 * pow(T3, 2) * T4 * T5 + 3 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T1 * T3 * pow(T4, 3) + 12 * T1 * T3 * pow(T4, 2) * T5 + 6 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T2 * pow(T3, 2) * T4 * T5 + 6 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T2 * T3 * pow(T4, 3) + 24 * T2 * T3 * pow(T4, 2) * T5 + 12 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr c1 = d3_var * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_var * (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 + 6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 + 6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 + 6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) - 24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 - 36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) - 48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 + 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr d1 = d3_var * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_var * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_var * (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 + pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 + 2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) + A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) + 12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 - 18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 - 12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 - 24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf + 8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T1 * T2 * T3 * T4 * pow(T5, 2));
    GRBLinExpr a2 = d3_var * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + d4_var * (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) - 4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) - T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) - 2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) - 8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) - 2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) + d5_var * (4 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T1 * T3 * T4 * T5 + 2 * T0 * T1 * T3 * pow(T5, 2) + 3 * T0 * T1 * pow(T4, 3) + 6 * T0 * T1 * pow(T4, 2) * T5 + 3 * T0 * T1 * T4 * pow(T5, 2) + 8 * T0 * T2 * T3 * pow(T4, 2) + 12 * T0 * T2 * T3 * T4 * T5 + 4 * T0 * T2 * T3 * pow(T5, 2) + 6 * T0 * T2 * pow(T4, 3) + 12 * T0 * T2 * pow(T4, 2) * T5 + 6 * T0 * T2 * T4 * pow(T5, 2) + 2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 + T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) + 4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) + 4 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * T4 * T5 + 2 * pow(T1, 2) * T3 * pow(T5, 2) + 3 * pow(T1, 2) * pow(T4, 3) + 6 * pow(T1, 2) * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T4 * pow(T5, 2) + 16 * T1 * T2 * T3 * pow(T4, 2) + 24 * T1 * T2 * T3 * T4 * T5 + 8 * T1 * T2 * T3 * pow(T5, 2) + 12 * T1 * T2 * pow(T4, 3) + 24 * T1 * T2 * pow(T4, 2) * T5 + 12 * T1 * T2 * T4 * pow(T5, 2) + 4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 + 2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) + 8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 + 3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) + 12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T4 * pow(T5, 3) - 16 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T2 * T3 * pow(T5, 3) - 12 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T1, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 2) * T4 * pow(T5, 3) - 32 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Af * T1 * T2 * T3 * pow(T5, 3) - 24 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 24 * Af * T1 * T2 * T4 * pow(T5, 3) - 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T1 * T3 * T4 * pow(T5, 3) - 24 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 18 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 18 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T3 * T4 - 36 * Pf * T0 * T1 * T3 * T5 - 18 * Pf * T0 * T1 * pow(T4, 2) - 36 * Pf * T0 * T1 * T4 * T5 - 48 * Pf * T0 * T2 * T3 * T4 - 72 * Pf * T0 * T2 * T3 * T5 - 36 * Pf * T0 * T2 * pow(T4, 2) - 72 * Pf * T0 * T2 * T4 * T5 - 12 * Pf * T0 * pow(T3, 2) * T4 - 18 * Pf * T0 * pow(T3, 2) * T5 - 12 * Pf * T0 * T3 * pow(T4, 2) - 24 * Pf * T0 * T3 * T4 * T5 - 24 * Pf * pow(T1, 2) * T3 * T4 - 36 * Pf * pow(T1, 2) * T3 * T5 - 18 * Pf * pow(T1, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 2) * T4 * T5 - 96 * Pf * T1 * T2 * T3 * T4 - 144 * Pf * T1 * T2 * T3 * T5 - 72 * Pf * T1 * T2 * pow(T4, 2) - 144 * Pf * T1 * T2 * T4 * T5 - 24 * Pf * T1 * pow(T3, 2) * T4 - 36 * Pf * T1 * pow(T3, 2) * T5 - 24 * Pf * T1 * T3 * pow(T4, 2) - 48 * Pf * T1 * T3 * T4 * T5 - 72 * Pf * pow(T2, 2) * T3 * T4 - 108 * Pf * pow(T2, 2) * T3 * T5 - 54 * Pf * pow(T2, 2) * pow(T4, 2) - 108 * Pf * pow(T2, 2) * T4 * T5 - 36 * Pf * T2 * pow(T3, 2) * T4 - 54 * Pf * T2 * pow(T3, 2) * T5 - 36 * Pf * T2 * T3 * pow(T4, 2) - 72 * Pf * T2 * T3 * T4 * T5 + 24 * T0 * T1 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T4 * pow(T5, 2) * Vf + 48 * T0 * T2 * T3 * T4 * T5 * Vf + 48 * T0 * T2 * T3 * pow(T5, 2) * Vf + 36 * T0 * T2 * pow(T4, 2) * T5 * Vf + 48 * T0 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T3 * pow(T4, 2) * T5 * Vf - 4 * T0 * T3 * T4 * pow(T5, 2) * V0 + 16 * T0 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T1, 2) * T3 * T4 * T5 * Vf + 24 * pow(T1, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T1, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T1, 2) * T4 * pow(T5, 2) * Vf + 96 * T1 * T2 * T3 * T4 * T5 * Vf + 96 * T1 * T2 * T3 * pow(T5, 2) * Vf + 72 * T1 * T2 * pow(T4, 2) * T5 * Vf + 96 * T1 * T2 * T4 * pow(T5, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * T5 * Vf + 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T1 * T3 * pow(T4, 2) * T5 * Vf - 2 * T1 * T3 * T4 * pow(T5, 2) * V0 + 32 * T1 * T3 * T4 * pow(T5, 2) * Vf + 72 * pow(T2, 2) * T3 * T4 * T5 * Vf + 72 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 54 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 72 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 3) * T3 * T4 * pow(T5, 2));
    GRBLinExpr b2 = d3_var * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_var * (-12 * T0 * T2 * T3 * pow(T4, 2) - 18 * T0 * T2 * T3 * T4 * T5 - 6 * T0 * T2 * T3 * pow(T5, 2) - 9 * T0 * T2 * pow(T4, 3) - 18 * T0 * T2 * pow(T4, 2) * T5 - 9 * T0 * T2 * T4 * pow(T5, 2) - 6 * T0 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * pow(T3, 2) * T4 * T5 - 3 * T0 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T3 * pow(T4, 3) - 12 * T0 * T3 * pow(T4, 2) * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) - 24 * T1 * T2 * T3 * pow(T4, 2) - 36 * T1 * T2 * T3 * T4 * T5 - 12 * T1 * T2 * T3 * pow(T5, 2) - 18 * T1 * T2 * pow(T4, 3) - 36 * T1 * T2 * pow(T4, 2) * T5 - 18 * T1 * T2 * T4 * pow(T5, 2) - 12 * T1 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * pow(T3, 2) * T4 * T5 - 6 * T1 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T3 * pow(T4, 3) - 24 * T1 * T3 * pow(T4, 2) * T5 - 12 * T1 * T3 * T4 * pow(T5, 2) - 24 * pow(T2, 2) * T3 * pow(T4, 2) - 36 * pow(T2, 2) * T3 * T4 * T5 - 12 * pow(T2, 2) * T3 * pow(T5, 2) - 18 * pow(T2, 2) * pow(T4, 3) - 36 * pow(T2, 2) * pow(T4, 2) * T5 - 18 * pow(T2, 2) * T4 * pow(T5, 2) - 18 * T2 * pow(T3, 2) * pow(T4, 2) - 27 * T2 * pow(T3, 2) * T4 * T5 - 9 * T2 * pow(T3, 2) * pow(T5, 2) - 18 * T2 * T3 * pow(T4, 3) - 36 * T2 * T3 * pow(T4, 2) * T5 - 18 * T2 * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) + 16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) + 12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) + 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) + 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) + 6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 + 18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 + 12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 + 12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 + 48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 + 36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 + 24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 + 24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 + 48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 + 36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 + 36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 + 36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 - 24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 - 16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf - 48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf - 48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf - 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr c2 = d3_var * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_var * (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 + 6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) + 18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) + 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 + 18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 + 12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 + 24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 + 18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 - 24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 - 18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr d2 = d3_var * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_var * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_var * (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 + 2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) + 6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) + 6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 + 6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 - 54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) - 72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 - 36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 - 72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    GRBLinExpr a3 = (1.0 / 2.0) * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 2 * Af * pow(T4, 2) * pow(T5, 2) + 2 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 6 * Pf * pow(T4, 2) + 12 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 6 * pow(T4, 2) * T5 * Vf - 8 * T4 * pow(T5, 2) * Vf) / (pow(T3, 2) * T4 * pow(T5, 2)) + d5_var * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 3 * pow(T4, 3) - 6 * pow(T4, 2) * T5 - 3 * T4 * pow(T5, 2)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2)) - d3_var / pow(T3, 3) + d4_var * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    GRBLinExpr b3 = (-4 * Af * T3 * T4 * pow(T5, 2) - 3 * Af * T3 * pow(T5, 3) - 3 * Af * pow(T4, 2) * pow(T5, 2) - 3 * Af * T4 * pow(T5, 3) - 12 * Pf * T3 * T4 - 18 * Pf * T3 * T5 - 9 * Pf * pow(T4, 2) - 18 * Pf * T4 * T5 + 12 * T3 * T4 * T5 * Vf + 12 * T3 * pow(T5, 2) * Vf + 9 * pow(T4, 2) * T5 * Vf + 12 * T4 * pow(T5, 2) * Vf) / (T3 * T4 * pow(T5, 2)) + d5_var * (12 * T3 * pow(T4, 2) + 18 * T3 * T4 * T5 + 6 * T3 * pow(T5, 2) + 9 * pow(T4, 3) + 18 * pow(T4, 2) * T5 + 9 * T4 * pow(T5, 2)) / (T3 * pow(T4, 2) * pow(T5, 2)) + 3 * d3_var / pow(T3, 2) + d4_var * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    GRBLinExpr c3 = (1.0 / 2.0) * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + d5_var * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) - 12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2)) - 3 * d3_var / T3 + d4_var * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    GRBLinExpr d3 = d3_var;
    GRBLinExpr a4 = (1.0 / 2.0) * (-2 * Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 6 * Pf * T4 - 6 * Pf * T5 + 6 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / (pow(T4, 2) * pow(T5, 2)) - d4_var / pow(T4, 3) + d5_var * (3 * pow(T4, 2) + 3 * T4 * T5 + pow(T5, 2)) / (pow(T4, 3) * pow(T5, 2));
    GRBLinExpr b4 = (1.0 / 2.0) * (4 * Af * T4 * pow(T5, 2) + 3 * Af * pow(T5, 3) + 12 * Pf * T4 + 18 * Pf * T5 - 12 * T4 * T5 * Vf - 12 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + 3 * d4_var / pow(T4, 2) + d5_var * (-6 * pow(T4, 2) - 9 * T4 * T5 - 3 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2));
    GRBLinExpr c4 = (-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 + 3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / pow(T5, 2) - 3 * d4_var / T4 + d5_var * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2));
    GRBLinExpr d4 = d4_var;
    GRBLinExpr a5 = -d5_var / pow(T5, 3) + (1.0 / 2.0) * (Af * pow(T5, 2) + 2 * Pf - 2 * T5 * Vf) / pow(T5, 3);
    GRBLinExpr b5 = 3 * d5_var / pow(T5, 2) + (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf) / pow(T5, 2);
    GRBLinExpr c5 = -3 * d5_var / T5 + (1.0 / 2.0) * (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5;
    GRBLinExpr d5 = d5_var;

    // Fill x_ with the coefficients
    x_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4, a5, b5, c5, d5});
  }
}

void SolverGurobi::getDependentCoefficientsN4Double()
{

  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];

  for (int axis = 0; axis < 3; axis++)
  {

    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);

    // C++ expressions for composite cubic Bézier control coefficients (one coordinate) in terms of lambda:
    double a0 = d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) - 2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) / (6 * pow(T0, 3) * pow(T3, 2) + 12 * pow(T0, 2) * T1 * pow(T3, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) + 6 * T0 * pow(T1, 2) * pow(T3, 2) + 6 * T0 * T1 * T2 * pow(T3, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 = d3_val * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 3) * pow(T3, 2) + 4 * A0 * pow(T0, 2) * T1 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * A0 * T0 * pow(T1, 2) * pow(T3, 2) + 3 * A0 * T0 * T1 * T2 * pow(T3, 2) + A0 * T0 * pow(T2, 2) * pow(T3, 2) + 2 * Af * pow(T0, 2) * T2 * pow(T3, 2) + Af * pow(T0, 2) * pow(T3, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) + 3 * Af * T0 * T1 * pow(T3, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) + 2 * Af * pow(T2, 3) * pow(T3, 2) + 2 * Af * pow(T2, 2) * pow(T3, 3) + 6 * P0 * T0 * pow(T3, 2) + 12 * P0 * T1 * pow(T3, 2) + 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * pow(T0, 2) * T2 + 6 * Pf * pow(T0, 2) * T3 + 18 * Pf * T0 * T1 * T2 + 18 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 + 18 * Pf * pow(T1, 2) * T2 + 18 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) + 36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 12 * Pf * pow(T2, 2) * T3 - 6 * pow(T0, 2) * T2 * T3 * Vf + 4 * pow(T0, 2) * pow(T3, 2) * V0 - 4 * pow(T0, 2) * pow(T3, 2) * Vf - 18 * T0 * T1 * T2 * T3 * Vf + 12 * T0 * T1 * pow(T3, 2) * V0 - 12 * T0 * T1 * pow(T3, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * Vf + 6 * T0 * T2 * pow(T3, 2) * V0 - 12 * T0 * T2 * pow(T3, 2) * Vf - 18 * pow(T1, 2) * T2 * T3 * Vf + 6 * pow(T1, 2) * pow(T3, 2) * V0 - 12 * pow(T1, 2) * pow(T3, 2) * Vf - 24 * T1 * pow(T2, 2) * T3 * Vf + 6 * T1 * T2 * pow(T3, 2) * V0 - 24 * T1 * T2 * pow(T3, 2) * Vf - 6 * pow(T2, 3) * T3 * Vf + 2 * pow(T2, 2) * pow(T3, 2) * V0 - 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 6 * pow(T0, 2) * T1 * T2 * pow(T3, 2) + 12 * T0 * pow(T1, 3) * pow(T3, 2) + 18 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 4) * pow(T3, 2) + 12 * pow(T1, 3) * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2));
    double b1 = d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) - A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    double c1 = d3_val * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) + A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) - 2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 - 6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 - 4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf + 4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf + 8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 + 2 * T1 * T2 * pow(T3, 2) * V0) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2));
    double d1 = d3_val * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) - Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) + 6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) + 6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 - 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf + 8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf + 6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * pow(T3, 2) * V0) / (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
    double a2 = d3_val * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + (-A0 * pow(T0, 2) * pow(T3, 2) - A0 * T0 * T1 * pow(T3, 2) - 2 * Af * T0 * T1 * pow(T3, 2) - 4 * Af * T0 * T2 * pow(T3, 2) - Af * T0 * pow(T3, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) - 8 * Af * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * pow(T3, 3) - 6 * Af * pow(T2, 2) * pow(T3, 2) - 3 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T0 * T1 - 12 * Pf * T0 * T2 - 6 * Pf * T0 * T3 - 6 * Pf * pow(T1, 2) - 24 * Pf * T1 * T2 - 12 * Pf * T1 * T3 - 18 * Pf * pow(T2, 2) - 18 * Pf * T2 * T3 + 6 * T0 * T1 * T3 * Vf + 12 * T0 * T2 * T3 * Vf - 4 * T0 * pow(T3, 2) * V0 + 4 * T0 * pow(T3, 2) * Vf + 6 * pow(T1, 2) * T3 * Vf + 24 * T1 * T2 * T3 * Vf - 2 * T1 * pow(T3, 2) * V0 + 8 * T1 * pow(T3, 2) * Vf + 18 * pow(T2, 2) * T3 * Vf + 12 * T2 * pow(T3, 2) * Vf) / (6 * T0 * T1 * T2 * pow(T3, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T2, 3) * pow(T3, 2));
    double b2 = d3_val * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) + Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) + 6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 + 12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf + 4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf + 2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf - 12 * T2 * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    double c2 = d3_val * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) + 2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) + 2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) - 2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) - 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 + 6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) - 12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf - 4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf - 4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 + 6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2));
    double d2 = d3_val * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
    double a3 = -d3_val / pow(T3, 3) + (1.0 / 2.0) * (Af * pow(T3, 2) + 2 * Pf - 2 * T3 * Vf) / pow(T3, 3);
    double b3 = 3 * d3_val / pow(T3, 2) + (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf) / pow(T3, 2);
    double c3 = -3 * d3_val / T3 + (1.0 / 2.0) * (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3;
    double d3 = d3_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3});
  }
}

void SolverGurobi::getDependentCoefficientsN5Double()
{

  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];

  for (int axis = 0; axis < 3; axis++)
  {

    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);
    double d4_val = d4_[axis].get(GRB_DoubleAttr_X);

    // C++ Code for the coefficients:
    double a0 = d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + d4_val * (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) - T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) - 2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) / (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 4 * A0 * T0 * T1 * T3 * pow(T4, 2) - 2 * A0 * T0 * T2 * T3 * pow(T4, 2) - A0 * pow(T1, 2) * T3 * pow(T4, 2) - A0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) / (6 * pow(T0, 3) * T3 * pow(T4, 2) + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 = d3_val * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + d4_val * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 3) * T3 * pow(T4, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + 3 * A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + A0 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T0, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T3 * pow(T4, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * pow(T4, 3) - 6 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T1 * T3 * pow(T4, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 3) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T2 * T3 * pow(T4, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 6 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T4, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T4, 3) - 12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * T2 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 3) * pow(T4, 3) - 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * P0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 - 18 * Pf * pow(T0, 2) * T2 * T4 - 6 * Pf * pow(T0, 2) * pow(T3, 2) - 12 * Pf * pow(T0, 2) * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 - 54 * Pf * T0 * T1 * T2 * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) - 36 * Pf * T0 * T1 * T3 * T4 - 24 * Pf * T0 * pow(T2, 2) * T3 - 36 * Pf * T0 * pow(T2, 2) * T4 - 18 * Pf * T0 * T2 * pow(T3, 2) - 36 * Pf * T0 * T2 * T3 * T4 - 36 * Pf * pow(T1, 2) * T2 * T3 - 54 * Pf * pow(T1, 2) * T2 * T4 - 18 * Pf * pow(T1, 2) * pow(T3, 2) - 36 * Pf * pow(T1, 2) * T3 * T4 - 48 * Pf * T1 * pow(T2, 2) * T3 - 72 * Pf * T1 * pow(T2, 2) * T4 - 36 * Pf * T1 * T2 * pow(T3, 2) - 72 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * pow(T2, 3) * T3 - 18 * Pf * pow(T2, 3) * T4 - 12 * Pf * pow(T2, 2) * pow(T3, 2) - 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * pow(T0, 2) * T2 * T3 * T4 * Vf + 12 * pow(T0, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T0, 2) * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 + 8 * pow(T0, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * T1 * T2 * T3 * T4 * Vf + 36 * T0 * T1 * T2 * pow(T4, 2) * Vf + 18 * T0 * T1 * pow(T3, 2) * T4 * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * V0 + 24 * T0 * T1 * T3 * pow(T4, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T0 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T2 * T3 * pow(T4, 2) * V0 + 24 * T0 * T2 * T3 * pow(T4, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * T4 * Vf + 36 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 18 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 6 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T4, 2) * Vf + 36 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T1 * T2 * T3 * pow(T4, 2) * V0 + 48 * T1 * T2 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 3) * T3 * T4 * Vf + 12 * pow(T2, 3) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * pow(T2, 2) * T3 * pow(T4, 2) * V0 + 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 12 * T0 * pow(T1, 3) * T3 * pow(T4, 2) + 18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 6 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 4) * T3 * pow(T4, 2) + 12 * pow(T1, 3) * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2));
    double b1 = d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) - A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2));
    double c1 = d3_val * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) + 12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 + 18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) + 24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 - 12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf + 2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2));
    double d1 = d3_val * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 + 18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) + 12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 + 18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf - 12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf + 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 - 16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) / (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * pow(T4, 2));
    double a2 = d3_val * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + d4_val * (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) - 4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) - T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) - 2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) - 8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) - 2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 2) * T3 * pow(T4, 2) - A0 * T0 * T1 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * pow(T4, 3) + 8 * Af * T0 * T2 * T3 * pow(T4, 2) + 6 * Af * T0 * T2 * pow(T4, 3) + 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T3 * pow(T4, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T1, 2) * pow(T4, 3) + 16 * Af * T1 * T2 * T3 * pow(T4, 2) + 12 * Af * T1 * T2 * pow(T4, 3) + 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T1 * T3 * pow(T4, 3) + 12 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 9 * Af * pow(T2, 2) * pow(T4, 3) + 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T3 + 18 * Pf * T0 * T1 * T4 + 24 * Pf * T0 * T2 * T3 + 36 * Pf * T0 * T2 * T4 + 6 * Pf * T0 * pow(T3, 2) + 12 * Pf * T0 * T3 * T4 + 12 * Pf * pow(T1, 2) * T3 + 18 * Pf * pow(T1, 2) * T4 + 48 * Pf * T1 * T2 * T3 + 72 * Pf * T1 * T2 * T4 + 12 * Pf * T1 * pow(T3, 2) + 24 * Pf * T1 * T3 * T4 + 36 * Pf * pow(T2, 2) * T3 + 54 * Pf * pow(T2, 2) * T4 + 18 * Pf * T2 * pow(T3, 2) + 36 * Pf * T2 * T3 * T4 - 12 * T0 * T1 * T3 * T4 * Vf - 12 * T0 * T1 * pow(T4, 2) * Vf - 24 * T0 * T2 * T3 * T4 * Vf - 24 * T0 * T2 * pow(T4, 2) * Vf - 6 * T0 * pow(T3, 2) * T4 * Vf - 4 * T0 * T3 * pow(T4, 2) * V0 - 8 * T0 * T3 * pow(T4, 2) * Vf - 12 * pow(T1, 2) * T3 * T4 * Vf - 12 * pow(T1, 2) * pow(T4, 2) * Vf - 48 * T1 * T2 * T3 * T4 * Vf - 48 * T1 * T2 * pow(T4, 2) * Vf - 12 * T1 * pow(T3, 2) * T4 * Vf - 2 * T1 * T3 * pow(T4, 2) * V0 - 16 * T1 * T3 * pow(T4, 2) * Vf - 36 * pow(T2, 2) * T3 * T4 * Vf - 36 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T2 * pow(T3, 2) * T4 * Vf - 24 * T2 * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T2 * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 3) * T3 * pow(T4, 2));
    double b2 = d3_val * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) - 4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) - 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) - 8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) - 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) - 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 - 6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 - 36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 - 24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) - 36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf + 6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 + 8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf + 24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf + 2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2));
    double c2 = d3_val * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) - 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) + 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 - 6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 - 12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 - 6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 + 12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) + 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf + 12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf + 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 + 12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf - 2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf - 12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2));
    double d2 = d3_val * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * Af * T1 * pow(T2, 3) * pow(T4, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 + 18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 + 12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 + 12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 + 24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 + 18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 + 24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 + 24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 - 24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * pow(T4, 2));
    double a3 = (1.0 / 2.0) * (-2 * Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 6 * Pf * T3 - 6 * Pf * T4 + 6 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / (pow(T3, 2) * pow(T4, 2)) - d3_val / pow(T3, 3) + d4_val * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    double b3 = (1.0 / 2.0) * (4 * Af * T3 * pow(T4, 2) + 3 * Af * pow(T4, 3) + 12 * Pf * T3 + 18 * Pf * T4 - 12 * T3 * T4 * Vf - 12 * pow(T4, 2) * Vf) / (T3 * pow(T4, 2)) + 3 * d3_val / pow(T3, 2) + d4_val * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    double c3 = (-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 + 3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / pow(T4, 2) - 3 * d3_val / T3 + d4_val * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    double d3 = d3_val;
    double a4 = -d4_val / pow(T4, 3) + (1.0 / 2.0) * (Af * pow(T4, 2) + 2 * Pf - 2 * T4 * Vf) / pow(T4, 3);
    double b4 = 3 * d4_val / pow(T4, 2) + (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf) / pow(T4, 2);
    double c4 = -3 * d4_val / T4 + (1.0 / 2.0) * (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4;
    double d4 = d4_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4});
  }
}

void SolverGurobi::getDependentCoefficientsN6Double()
{

  // Clear the vector
  x_double_.clear();

  // get the time intervals from the optimization
  double T0 = dt_[0];
  double T1 = dt_[1];
  double T2 = dt_[2];
  double T3 = dt_[3];
  double T4 = dt_[4];
  double T5 = dt_[5];

  for (int axis = 0; axis < 3; axis++)
  {

    // get the initial and final values from the optimization
    double P0, V0, A0, Pf, Vf, Af;
    getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

    // get free parameters
    double d3_val = d3_[axis].get(GRB_DoubleAttr_X);
    double d4_val = d4_[axis].get(GRB_DoubleAttr_X);
    double d5_val = d5_[axis].get(GRB_DoubleAttr_X);

    // C++ Code for the coefficients:
    double a0 = d3_val * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) / (pow(T0, 3) * pow(T3, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) + pow(T0, 2) * T2 * pow(T3, 2) + T0 * pow(T1, 2) * pow(T3, 2) + T0 * T1 * T2 * pow(T3, 2)) + d4_val * (-2 * T1 * T2 * pow(T3, 2) - 3 * T1 * T2 * T3 * T4 - T1 * T2 * pow(T4, 2) - T1 * pow(T3, 3) - 2 * T1 * pow(T3, 2) * T4 - T1 * T3 * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 2) - 3 * pow(T2, 2) * T3 * T4 - pow(T2, 2) * pow(T4, 2) - 2 * T2 * pow(T3, 3) - 4 * T2 * pow(T3, 2) * T4 - 2 * T2 * T3 * pow(T4, 2)) / (pow(T0, 3) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_val * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 3) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (6 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 12 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 6 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double b0 = (1.0 / 2.0) * A0;
    double c0 = V0;
    double d0 = P0;
    double a1 = d3_val * (-pow(T0, 2) * T2 - pow(T0, 2) * T3 - 3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 2 * T0 * pow(T2, 2) - 3 * T0 * T2 * T3 - T0 * pow(T3, 2) - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 - 4 * T1 * pow(T2, 2) - 6 * T1 * T2 * T3 - 2 * T1 * pow(T3, 2) - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T1, 4) * pow(T3, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2)) + d4_val * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 4) * pow(T3, 2) * pow(T4, 2) + 2 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_val * (-4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 6 * pow(T0, 2) * T2 * T3 * T4 * T5 - 2 * pow(T0, 2) * T2 * T3 * pow(T5, 2) - 3 * pow(T0, 2) * T2 * pow(T4, 3) - 6 * pow(T0, 2) * T2 * pow(T4, 2) * T5 - 3 * pow(T0, 2) * T2 * T4 * pow(T5, 2) - 2 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 3 * pow(T0, 2) * pow(T3, 2) * T4 * T5 - pow(T0, 2) * pow(T3, 2) * pow(T5, 2) - 2 * pow(T0, 2) * T3 * pow(T4, 3) - 4 * pow(T0, 2) * T3 * pow(T4, 2) * T5 - 2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * T0 * pow(T2, 2) * T3 * T4 * T5 - 4 * T0 * pow(T2, 2) * T3 * pow(T5, 2) - 6 * T0 * pow(T2, 2) * pow(T4, 3) - 12 * T0 * pow(T2, 2) * pow(T4, 2) * T5 - 6 * T0 * pow(T2, 2) * T4 * pow(T5, 2) - 6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T2 * pow(T3, 2) * T4 * T5 - 3 * T0 * T2 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T2 * T3 * pow(T4, 3) - 12 * T0 * T2 * T3 * pow(T4, 2) * T5 - 6 * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 24 * T1 * pow(T2, 2) * T3 * T4 * T5 - 8 * T1 * pow(T2, 2) * T3 * pow(T5, 2) - 12 * T1 * pow(T2, 2) * pow(T4, 3) - 24 * T1 * pow(T2, 2) * pow(T4, 2) * T5 - 12 * T1 * pow(T2, 2) * T4 * pow(T5, 2) - 12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * T2 * pow(T3, 2) * T4 * T5 - 6 * T1 * T2 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T2 * T3 * pow(T4, 3) - 24 * T1 * T2 * T3 * pow(T4, 2) * T5 - 12 * T1 * T2 * T3 * T4 * pow(T5, 2) - 4 * pow(T2, 3) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * T3 * T4 * T5 - 2 * pow(T2, 3) * T3 * pow(T5, 2) - 3 * pow(T2, 3) * pow(T4, 3) - 6 * pow(T2, 3) * pow(T4, 2) * T5 - 3 * pow(T2, 3) * T4 * pow(T5, 2) - 4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) - 4 * pow(T2, 2) * T3 * pow(T4, 3) - 8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) + 3 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 4) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T0, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T0, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T0, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T0, 2) * T3 * T4 * pow(T5, 3) + 24 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 18 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 12 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) + 24 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 18 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 12 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 12 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) + 32 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T3 * pow(T5, 3) + 24 * Af * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T4 * pow(T5, 3) + 24 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 18 * Af * T1 * T2 * pow(T3, 2) * pow(T5, 3) + 24 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * T2 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) + 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) + 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) + 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * T0 * T3 * T4 * pow(T5, 2) + 12 * P0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 + 36 * Pf * pow(T0, 2) * T2 * T3 * T5 + 18 * Pf * pow(T0, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T2 * T4 * T5 + 12 * Pf * pow(T0, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T0, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T0, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T0, 2) * T3 * T4 * T5 + 72 * Pf * T0 * T1 * T2 * T3 * T4 + 108 * Pf * T0 * T1 * T2 * T3 * T5 + 54 * Pf * T0 * T1 * T2 * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * T4 * T5 + 36 * Pf * T0 * T1 * pow(T3, 2) * T4 + 54 * Pf * T0 * T1 * pow(T3, 2) * T5 + 36 * Pf * T0 * T1 * T3 * pow(T4, 2) + 72 * Pf * T0 * T1 * T3 * T4 * T5 + 48 * Pf * T0 * pow(T2, 2) * T3 * T4 + 72 * Pf * T0 * pow(T2, 2) * T3 * T5 + 36 * Pf * T0 * pow(T2, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * T4 * T5 + 36 * Pf * T0 * T2 * pow(T3, 2) * T4 + 54 * Pf * T0 * T2 * pow(T3, 2) * T5 + 36 * Pf * T0 * T2 * T3 * pow(T4, 2) + 72 * Pf * T0 * T2 * T3 * T4 * T5 + 72 * Pf * pow(T1, 2) * T2 * T3 * T4 + 108 * Pf * pow(T1, 2) * T2 * T3 * T5 + 54 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * T4 * T5 + 36 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 54 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 36 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 72 * Pf * pow(T1, 2) * T3 * T4 * T5 + 96 * Pf * T1 * pow(T2, 2) * T3 * T4 + 144 * Pf * T1 * pow(T2, 2) * T3 * T5 + 72 * Pf * T1 * pow(T2, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * T4 * T5 + 72 * Pf * T1 * T2 * pow(T3, 2) * T4 + 108 * Pf * T1 * T2 * pow(T3, 2) * T5 + 72 * Pf * T1 * T2 * T3 * pow(T4, 2) + 144 * Pf * T1 * T2 * T3 * T4 * T5 + 24 * Pf * pow(T2, 3) * T3 * T4 + 36 * Pf * pow(T2, 3) * T3 * T5 + 18 * Pf * pow(T2, 3) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * T4 * T5 + 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 + 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) + 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * pow(T0, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T0, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T0, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T0, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T0, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T0, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 - 16 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 72 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 54 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 72 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 72 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 54 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 72 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 36 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 36 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 - 48 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 96 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf - 96 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 96 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf - 72 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 - 96 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 3) * T3 * T4 * T5 * Vf - 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf - 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf - 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 - 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T1, 3) * T3 * T4 * pow(T5, 2) + 18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 4) * T3 * T4 * pow(T5, 2) + 12 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double b1 = d3_val * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_val * (12 * T1 * T2 * T3 * pow(T4, 2) + 18 * T1 * T2 * T3 * T4 * T5 + 6 * T1 * T2 * T3 * pow(T5, 2) + 9 * T1 * T2 * pow(T4, 3) + 18 * T1 * T2 * pow(T4, 2) * T5 + 9 * T1 * T2 * T4 * pow(T5, 2) + 6 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T1 * pow(T3, 2) * T4 * T5 + 3 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T1 * T3 * pow(T4, 3) + 12 * T1 * T3 * pow(T4, 2) * T5 + 6 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T2 * pow(T3, 2) * T4 * T5 + 6 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T2 * T3 * pow(T4, 3) + 24 * T2 * T3 * pow(T4, 2) * T5 + 12 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double c1 = d3_val * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_val * (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 + 6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 + 6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 + 6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) - 24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 - 36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) - 48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 + 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double d1 = d3_val * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4_val * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5_val * (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 + pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 + 2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) + A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) + 12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 - 18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 - 12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 - 24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf + 8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T1 * T2 * T3 * T4 * pow(T5, 2));
    double a2 = d3_val * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) / (T0 * T1 * T2 * pow(T3, 2) + T0 * pow(T2, 2) * pow(T3, 2) + pow(T1, 2) * T2 * pow(T3, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) + pow(T2, 3) * pow(T3, 2)) + d4_val * (-2 * T0 * T1 * pow(T3, 2) - 3 * T0 * T1 * T3 * T4 - T0 * T1 * pow(T4, 2) - 4 * T0 * T2 * pow(T3, 2) - 6 * T0 * T2 * T3 * T4 - 2 * T0 * T2 * pow(T4, 2) - T0 * pow(T3, 3) - 2 * T0 * pow(T3, 2) * T4 - T0 * T3 * pow(T4, 2) - 2 * pow(T1, 2) * pow(T3, 2) - 3 * pow(T1, 2) * T3 * T4 - pow(T1, 2) * pow(T4, 2) - 8 * T1 * T2 * pow(T3, 2) - 12 * T1 * T2 * T3 * T4 - 4 * T1 * T2 * pow(T4, 2) - 2 * T1 * pow(T3, 3) - 4 * T1 * pow(T3, 2) * T4 - 2 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 3 * T2 * pow(T3, 3) - 6 * T2 * pow(T3, 2) * T4 - 3 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + pow(T2, 3) * pow(T3, 2) * pow(T4, 2)) + d5_val * (4 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T1 * T3 * T4 * T5 + 2 * T0 * T1 * T3 * pow(T5, 2) + 3 * T0 * T1 * pow(T4, 3) + 6 * T0 * T1 * pow(T4, 2) * T5 + 3 * T0 * T1 * T4 * pow(T5, 2) + 8 * T0 * T2 * T3 * pow(T4, 2) + 12 * T0 * T2 * T3 * T4 * T5 + 4 * T0 * T2 * T3 * pow(T5, 2) + 6 * T0 * T2 * pow(T4, 3) + 12 * T0 * T2 * pow(T4, 2) * T5 + 6 * T0 * T2 * T4 * pow(T5, 2) + 2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 + T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) + 4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) + 4 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * T4 * T5 + 2 * pow(T1, 2) * T3 * pow(T5, 2) + 3 * pow(T1, 2) * pow(T4, 3) + 6 * pow(T1, 2) * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T4 * pow(T5, 2) + 16 * T1 * T2 * T3 * pow(T4, 2) + 24 * T1 * T2 * T3 * T4 * T5 + 8 * T1 * T2 * T3 * pow(T5, 2) + 12 * T1 * T2 * pow(T4, 3) + 24 * T1 * T2 * pow(T4, 2) * T5 + 12 * T1 * T2 * T4 * pow(T5, 2) + 4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 + 2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) + 8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 + 3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) + 12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T4 * pow(T5, 3) - 16 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T2 * T3 * pow(T5, 3) - 12 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T1, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 2) * T4 * pow(T5, 3) - 32 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Af * T1 * T2 * T3 * pow(T5, 3) - 24 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 24 * Af * T1 * T2 * T4 * pow(T5, 3) - 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T1 * T3 * T4 * pow(T5, 3) - 24 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 18 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 18 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T3 * T4 - 36 * Pf * T0 * T1 * T3 * T5 - 18 * Pf * T0 * T1 * pow(T4, 2) - 36 * Pf * T0 * T1 * T4 * T5 - 48 * Pf * T0 * T2 * T3 * T4 - 72 * Pf * T0 * T2 * T3 * T5 - 36 * Pf * T0 * T2 * pow(T4, 2) - 72 * Pf * T0 * T2 * T4 * T5 - 12 * Pf * T0 * pow(T3, 2) * T4 - 18 * Pf * T0 * pow(T3, 2) * T5 - 12 * Pf * T0 * T3 * pow(T4, 2) - 24 * Pf * T0 * T3 * T4 * T5 - 24 * Pf * pow(T1, 2) * T3 * T4 - 36 * Pf * pow(T1, 2) * T3 * T5 - 18 * Pf * pow(T1, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 2) * T4 * T5 - 96 * Pf * T1 * T2 * T3 * T4 - 144 * Pf * T1 * T2 * T3 * T5 - 72 * Pf * T1 * T2 * pow(T4, 2) - 144 * Pf * T1 * T2 * T4 * T5 - 24 * Pf * T1 * pow(T3, 2) * T4 - 36 * Pf * T1 * pow(T3, 2) * T5 - 24 * Pf * T1 * T3 * pow(T4, 2) - 48 * Pf * T1 * T3 * T4 * T5 - 72 * Pf * pow(T2, 2) * T3 * T4 - 108 * Pf * pow(T2, 2) * T3 * T5 - 54 * Pf * pow(T2, 2) * pow(T4, 2) - 108 * Pf * pow(T2, 2) * T4 * T5 - 36 * Pf * T2 * pow(T3, 2) * T4 - 54 * Pf * T2 * pow(T3, 2) * T5 - 36 * Pf * T2 * T3 * pow(T4, 2) - 72 * Pf * T2 * T3 * T4 * T5 + 24 * T0 * T1 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T4 * pow(T5, 2) * Vf + 48 * T0 * T2 * T3 * T4 * T5 * Vf + 48 * T0 * T2 * T3 * pow(T5, 2) * Vf + 36 * T0 * T2 * pow(T4, 2) * T5 * Vf + 48 * T0 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T3 * pow(T4, 2) * T5 * Vf - 4 * T0 * T3 * T4 * pow(T5, 2) * V0 + 16 * T0 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T1, 2) * T3 * T4 * T5 * Vf + 24 * pow(T1, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T1, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T1, 2) * T4 * pow(T5, 2) * Vf + 96 * T1 * T2 * T3 * T4 * T5 * Vf + 96 * T1 * T2 * T3 * pow(T5, 2) * Vf + 72 * T1 * T2 * pow(T4, 2) * T5 * Vf + 96 * T1 * T2 * T4 * pow(T5, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * T5 * Vf + 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T1 * T3 * pow(T4, 2) * T5 * Vf - 2 * T1 * T3 * T4 * pow(T5, 2) * V0 + 32 * T1 * T3 * T4 * pow(T5, 2) * Vf + 72 * pow(T2, 2) * T3 * T4 * T5 * Vf + 72 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 54 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 72 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 3) * T3 * T4 * pow(T5, 2));
    double b2 = d3_val * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_val * (-12 * T0 * T2 * T3 * pow(T4, 2) - 18 * T0 * T2 * T3 * T4 * T5 - 6 * T0 * T2 * T3 * pow(T5, 2) - 9 * T0 * T2 * pow(T4, 3) - 18 * T0 * T2 * pow(T4, 2) * T5 - 9 * T0 * T2 * T4 * pow(T5, 2) - 6 * T0 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * pow(T3, 2) * T4 * T5 - 3 * T0 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T3 * pow(T4, 3) - 12 * T0 * T3 * pow(T4, 2) * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) - 24 * T1 * T2 * T3 * pow(T4, 2) - 36 * T1 * T2 * T3 * T4 * T5 - 12 * T1 * T2 * T3 * pow(T5, 2) - 18 * T1 * T2 * pow(T4, 3) - 36 * T1 * T2 * pow(T4, 2) * T5 - 18 * T1 * T2 * T4 * pow(T5, 2) - 12 * T1 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * pow(T3, 2) * T4 * T5 - 6 * T1 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T3 * pow(T4, 3) - 24 * T1 * T3 * pow(T4, 2) * T5 - 12 * T1 * T3 * T4 * pow(T5, 2) - 24 * pow(T2, 2) * T3 * pow(T4, 2) - 36 * pow(T2, 2) * T3 * T4 * T5 - 12 * pow(T2, 2) * T3 * pow(T5, 2) - 18 * pow(T2, 2) * pow(T4, 3) - 36 * pow(T2, 2) * pow(T4, 2) * T5 - 18 * pow(T2, 2) * T4 * pow(T5, 2) - 18 * T2 * pow(T3, 2) * pow(T4, 2) - 27 * T2 * pow(T3, 2) * T4 * T5 - 9 * T2 * pow(T3, 2) * pow(T5, 2) - 18 * T2 * T3 * pow(T4, 3) - 36 * T2 * T3 * pow(T4, 2) * T5 - 18 * T2 * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) + 16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) + 12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) + 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) + 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) + 6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 + 18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 + 12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 + 12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 + 48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 + 36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 + 24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 + 24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 + 48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 + 36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 + 36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 + 36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 - 24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 - 16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf - 48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf - 48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf - 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double c2 = d3_val * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_val * (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 + 6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) + 18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) + 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 + 18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 + 12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 + 24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 + 18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 - 24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 - 18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double d2 = d3_val * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4_val * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5_val * (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 + 2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) + 6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) + 6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 + 6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 - 54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) - 72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 - 36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 - 72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
    double a3 = (1.0 / 2.0) * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 2 * Af * pow(T4, 2) * pow(T5, 2) + 2 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 6 * Pf * pow(T4, 2) + 12 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 6 * pow(T4, 2) * T5 * Vf - 8 * T4 * pow(T5, 2) * Vf) / (pow(T3, 2) * T4 * pow(T5, 2)) + d5_val * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 3 * pow(T4, 3) - 6 * pow(T4, 2) * T5 - 3 * T4 * pow(T5, 2)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2)) - d3_val / pow(T3, 3) + d4_val * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2)) / (pow(T3, 3) * pow(T4, 2));
    double b3 = (-4 * Af * T3 * T4 * pow(T5, 2) - 3 * Af * T3 * pow(T5, 3) - 3 * Af * pow(T4, 2) * pow(T5, 2) - 3 * Af * T4 * pow(T5, 3) - 12 * Pf * T3 * T4 - 18 * Pf * T3 * T5 - 9 * Pf * pow(T4, 2) - 18 * Pf * T4 * T5 + 12 * T3 * T4 * T5 * Vf + 12 * T3 * pow(T5, 2) * Vf + 9 * pow(T4, 2) * T5 * Vf + 12 * T4 * pow(T5, 2) * Vf) / (T3 * T4 * pow(T5, 2)) + d5_val * (12 * T3 * pow(T4, 2) + 18 * T3 * T4 * T5 + 6 * T3 * pow(T5, 2) + 9 * pow(T4, 3) + 18 * pow(T4, 2) * T5 + 9 * T4 * pow(T5, 2)) / (T3 * pow(T4, 2) * pow(T5, 2)) + 3 * d3_val / pow(T3, 2) + d4_val * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2));
    double c3 = (1.0 / 2.0) * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + d5_val * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) - 12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2)) - 3 * d3_val / T3 + d4_val * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2));
    double d3 = d3_val;
    double a4 = (1.0 / 2.0) * (-2 * Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 6 * Pf * T4 - 6 * Pf * T5 + 6 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / (pow(T4, 2) * pow(T5, 2)) - d4_val / pow(T4, 3) + d5_val * (3 * pow(T4, 2) + 3 * T4 * T5 + pow(T5, 2)) / (pow(T4, 3) * pow(T5, 2));
    double b4 = (1.0 / 2.0) * (4 * Af * T4 * pow(T5, 2) + 3 * Af * pow(T5, 3) + 12 * Pf * T4 + 18 * Pf * T5 - 12 * T4 * T5 * Vf - 12 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + 3 * d4_val / pow(T4, 2) + d5_val * (-6 * pow(T4, 2) - 9 * T4 * T5 - 3 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2));
    double c4 = (-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 + 3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / pow(T5, 2) - 3 * d4_val / T4 + d5_val * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2));
    double d4 = d4_val;
    double a5 = -d5_val / pow(T5, 3) + (1.0 / 2.0) * (Af * pow(T5, 2) + 2 * Pf - 2 * T5 * Vf) / pow(T5, 3);
    double b5 = 3 * d5_val / pow(T5, 2) + (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf) / pow(T5, 2);
    double c5 = -3 * d5_val / T5 + (1.0 / 2.0) * (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5;
    double d5 = d5_val;

    // Fill x_double_ with the coefficients
    x_double_.push_back({a0, b0, c0, d0, a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3, a4, b4, c4, d4, a5, b5, c5, d5});
  }
}

// void SolverGurobi::computeControlPointsN4()
// {

//   p_cp_.clear(); // Clear the position control points
//   v_cp_.clear(); // Clear the velocity control points
//   a_cp_.clear(); // Clear the acceleration control points
//   j_cp_.clear(); // Clear the jerk control points

//   // Compute the control points for each axis

//   for (int axis = 0; axis < 3; axis++)
//   {

//     // get the initial and final values from the optimization
//     double P0, V0, A0, Pf, Vf, Af;
//     getInitialAndFinalConditions(P0, V0, A0, Pf, Vf, Af, axis);

//     // get free parameters
//     double d3_val = d3_[axis].get(GRB_DoubleAttr_X);

//     // POSITION CONTROL POINTS:
//     // Segment 0:
//     double CP0_0 = P0;
//     double CP1_0 = P0 + (1.0 / 3.0) * T0 * V0;
//     double CP2_0 = (1.0 / 6.0) * A0 * pow(T0, 2) + P0 + (2.0 / 3.0) * T0 * V0;
//     double CP3_0 = (1.0 / 6.0) * (pow(T0, 2) * (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) - 2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf + 6 * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2))) + 3 * pow(T3, 2) * (A0 * pow(T0, 2) + 2 * P0 + 2 * T0 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double CP0_1 = d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) - Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) + 6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) + 6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 - 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf + 8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf + 6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * pow(T3, 2) * V0) / (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
//     double CP1_1 = (1.0 / 3.0) * T1 * (d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) + A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) - 2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 - 6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 - 4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf + 4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf + 8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 + 2 * T1 * T2 * pow(T3, 2) * V0) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) - Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) + 6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) + 6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 - 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf + 8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf + 6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * pow(T3, 2) * V0) / (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
//     double CP2_1 = (1.0 / 3.0) * pow(T1, 2) * (d3 * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) - A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2))) + (1.0 / 3.0) * T1 * (2 * d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + 2 * (-A0 * pow(T0, 3) * pow(T3, 2) + A0 * T0 * pow(T1, 2) * pow(T3, 2) + A0 * T0 * T1 * T2 * pow(T3, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 2) - 2 * Af * T0 * T2 * pow(T3, 3) - 6 * P0 * T0 * pow(T3, 2) - 6 * Pf * T0 * T1 * T2 - 6 * Pf * T0 * T1 * T3 - 6 * Pf * T0 * pow(T2, 2) - 12 * Pf * T0 * T2 * T3 - 4 * pow(T0, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * T3 * Vf + 4 * T0 * T1 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 2) * T3 * Vf + 8 * T0 * T2 * pow(T3, 2) * Vf + 2 * pow(T1, 2) * pow(T3, 2) * V0 + 2 * T1 * T2 * pow(T3, 2) * V0) / (2 * pow(T0, 2) * pow(T3, 2) + 4 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + (2 * A0 * pow(T0, 3) * T1 * pow(T3, 2) + A0 * pow(T0, 3) * T2 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) - Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 2 * Af * pow(T0, 2) * T2 * pow(T3, 3) + 12 * P0 * T0 * T1 * pow(T3, 2) + 6 * P0 * T0 * T2 * pow(T3, 2) + 6 * P0 * pow(T1, 2) * pow(T3, 2) + 6 * P0 * T1 * T2 * pow(T3, 2) - 6 * Pf * pow(T0, 2) * T1 * T2 - 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * pow(T0, 2) * pow(T2, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 + 6 * pow(T0, 2) * T1 * T2 * T3 * Vf + 8 * pow(T0, 2) * T1 * pow(T3, 2) * V0 + 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf + 6 * pow(T0, 2) * pow(T2, 2) * T3 * Vf + 4 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 8 * pow(T0, 2) * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 6 * T0 * T1 * T2 * pow(T3, 2) * V0) / (6 * pow(T0, 2) * pow(T3, 2) + 12 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 6 * T1 * T2 * pow(T3, 2));
//     double CP3_1 = (1.0 / 6.0) * (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * pow(T2, 2) * d3 + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 18 * T0 * T1 * T2 * T3 * d3 + 6 * T0 * T1 * pow(T3, 2) * d3 + 6 * T0 * pow(T2, 3) * T3 * Vf + 6 * T0 * pow(T2, 3) * d3 + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * T0 * pow(T2, 2) * T3 * d3 + 6 * T0 * T2 * pow(T3, 2) * d3 + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * pow(T2, 2) * d3 + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 18 * pow(T1, 2) * T2 * T3 * d3 + 6 * pow(T1, 2) * pow(T3, 2) * d3 + 12 * T1 * pow(T2, 3) * T3 * Vf + 12 * T1 * pow(T2, 3) * d3 + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf + 24 * T1 * pow(T2, 2) * T3 * d3 + 12 * T1 * T2 * pow(T3, 2) * d3) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double CP0_2 = d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
//     double CP1_2 = (1.0 / 3.0) * T2 * (d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) + 2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) + 2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) - 2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) - 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 + 6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) - 12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf - 4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf - 4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 + 6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
//     double CP2_2 = (1.0 / 3.0) * pow(T2, 2) * (d3 * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) + Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) + 6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 + 12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf + 4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf + 2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf - 12 * T2 * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2))) + (1.0 / 3.0) * T2 * (2 * d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + 2 * (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) + 2 * Af * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * pow(T3, 3) + 2 * Af * pow(T1, 2) * T2 * pow(T3, 2) + Af * pow(T1, 2) * pow(T3, 3) - 2 * Af * pow(T2, 3) * pow(T3, 2) - 2 * Af * pow(T2, 2) * pow(T3, 3) - 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * T0 * T1 * T2 + 6 * Pf * T0 * T1 * T3 + 6 * Pf * pow(T1, 2) * T2 + 6 * Pf * pow(T1, 2) * T3 - 6 * Pf * pow(T2, 3) - 12 * Pf * pow(T2, 2) * T3 - 6 * T0 * T1 * T2 * T3 * Vf - 4 * T0 * T1 * pow(T3, 2) * Vf - 4 * T0 * T2 * pow(T3, 2) * V0 - 6 * pow(T1, 2) * T2 * T3 * Vf - 4 * pow(T1, 2) * pow(T3, 2) * Vf - 2 * T1 * T2 * pow(T3, 2) * V0 + 6 * pow(T2, 3) * T3 * Vf + 8 * pow(T2, 2) * pow(T3, 2) * Vf) / (2 * T0 * T1 * pow(T3, 2) + 2 * T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 4 * T1 * T2 * pow(T3, 2) + 2 * pow(T2, 2) * pow(T3, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 4 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 3 * Af * T0 * T1 * T2 * pow(T3, 3) - 2 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 2 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 4 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 4 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 4 * Af * T1 * pow(T2, 2) * pow(T3, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) - 12 * Pf * T0 * T1 * pow(T2, 2) - 18 * Pf * T0 * T1 * T2 * T3 - 6 * Pf * T0 * pow(T2, 3) - 12 * Pf * T0 * pow(T2, 2) * T3 - 12 * Pf * pow(T1, 2) * pow(T2, 2) - 18 * Pf * pow(T1, 2) * T2 * T3 - 12 * Pf * T1 * pow(T2, 3) - 24 * Pf * T1 * pow(T2, 2) * T3 + 12 * T0 * T1 * pow(T2, 2) * T3 * Vf + 12 * T0 * T1 * T2 * pow(T3, 2) * Vf + 6 * T0 * pow(T2, 3) * T3 * Vf + 4 * T0 * pow(T2, 2) * pow(T3, 2) * V0 + 8 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 12 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 12 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 12 * T1 * pow(T2, 3) * T3 * Vf + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 16 * T1 * pow(T2, 2) * pow(T3, 2) * Vf) / (6 * T0 * T1 * pow(T3, 2) + 6 * T0 * T2 * pow(T3, 2) + 6 * pow(T1, 2) * pow(T3, 2) + 12 * T1 * T2 * pow(T3, 2) + 6 * pow(T2, 2) * pow(T3, 2));
//     double CP3_2 = d3;
//     // Segment 3:
//     double CP0_3 = d3;
//     double CP1_3 = (1.0 / 3.0) * T3 * (-3 * d3 / T3 + (1.0 / 2.0) * (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3) + d3;
//     double CP2_3 = (1.0 / 3.0) * pow(T3, 2) * (3 * d3 / pow(T3, 2) + (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf) / pow(T3, 2)) + (1.0 / 3.0) * T3 * (-6 * d3 / T3 + (Af * pow(T3, 2) + 6 * Pf - 4 * T3 * Vf) / T3) + d3;
//     double CP3_3 = Pf;

//     // Add control points to the list:
//     p_cp_.push_back({CP0_0, CP1_0, CP2_0, CP3_0, CP0_1, CP1_1, CP2_1, CP3_1, CP0_2, CP1_2, CP2_2, CP3_2, CP0_3, CP1_3, CP2_3, CP3_3});

//     // VELOCITY CONTROL POINTS:
//     // Segment 0:
//     double vCP0_0 = V0;
//     double vCP1_0 = (1.0 / 8.0) * (T0 * (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) - 2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf + 6 * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2))) + 4 * pow(T3, 2) * (A0 * T0 + 2 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP2_0 = (-1.0 / 2.0 * A0 * pow(T0, 3) * pow(T3, 2) + (1.0 / 2.0) * A0 * T0 * pow(T1, 2) * pow(T3, 2) + (1.0 / 2.0) * A0 * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * T2 * pow(T3, 2) - 1.0 / 2.0 * Af * T0 * T1 * pow(T3, 3) - Af * T0 * pow(T2, 2) * pow(T3, 2) - Af * T0 * T2 * pow(T3, 3) - 3 * P0 * T0 * pow(T3, 2) - 3 * Pf * T0 * T1 * T2 - 3 * Pf * T0 * T1 * T3 - 3 * Pf * T0 * pow(T2, 2) - 6 * Pf * T0 * T2 * T3 - 2 * pow(T0, 2) * pow(T3, 2) * V0 + 3 * T0 * T1 * T2 * T3 * Vf + 3 * T0 * T1 * T2 * d3 + 2 * T0 * T1 * pow(T3, 2) * Vf + 3 * T0 * T1 * T3 * d3 + 3 * T0 * pow(T2, 2) * T3 * Vf + 3 * T0 * pow(T2, 2) * d3 + 4 * T0 * T2 * pow(T3, 2) * Vf + 6 * T0 * T2 * T3 * d3 + 3 * T0 * pow(T3, 2) * d3 + pow(T1, 2) * pow(T3, 2) * V0 + T1 * T2 * pow(T3, 2) * V0) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double vCP0_1 = (-1.0 / 2.0 * A0 * pow(T0, 3) * pow(T3, 2) + (1.0 / 2.0) * A0 * T0 * pow(T1, 2) * pow(T3, 2) + (1.0 / 2.0) * A0 * T0 * T1 * T2 * pow(T3, 2) - Af * T0 * T1 * T2 * pow(T3, 2) - 1.0 / 2.0 * Af * T0 * T1 * pow(T3, 3) - Af * T0 * pow(T2, 2) * pow(T3, 2) - Af * T0 * T2 * pow(T3, 3) - 3 * P0 * T0 * pow(T3, 2) - 3 * Pf * T0 * T1 * T2 - 3 * Pf * T0 * T1 * T3 - 3 * Pf * T0 * pow(T2, 2) - 6 * Pf * T0 * T2 * T3 - 2 * pow(T0, 2) * pow(T3, 2) * V0 + 3 * T0 * T1 * T2 * T3 * Vf + 2 * T0 * T1 * pow(T3, 2) * Vf + 3 * T0 * pow(T2, 2) * T3 * Vf + 4 * T0 * T2 * pow(T3, 2) * Vf + 3 * T0 * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) + pow(T1, 2) * pow(T3, 2) * V0 + T1 * T2 * pow(T3, 2) * V0) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP1_1 = (1.0 / 8.0) * (-3 * A0 * pow(T0, 3) * T1 * pow(T3, 2) - 4 * A0 * pow(T0, 3) * T2 * pow(T3, 2) - 4 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) - 6 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - A0 * T0 * pow(T1, 3) * pow(T3, 2) - A0 * T0 * pow(T1, 2) * T2 * pow(T3, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 2 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) + Af * pow(T0, 2) * T1 * pow(T3, 3) - 2 * Af * T0 * pow(T1, 2) * T2 * pow(T3, 2) - Af * T0 * pow(T1, 2) * pow(T3, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 3) - 8 * Af * T0 * pow(T2, 3) * pow(T3, 2) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 3) - 2 * Af * pow(T1, 3) * T2 * pow(T3, 2) - Af * pow(T1, 3) * pow(T3, 3) - 8 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * Af * pow(T1, 2) * T2 * pow(T3, 3) - 6 * Af * T1 * pow(T2, 3) * pow(T3, 2) - 6 * Af * T1 * pow(T2, 2) * pow(T3, 3) - 18 * P0 * T0 * T1 * pow(T3, 2) - 24 * P0 * T0 * T2 * pow(T3, 2) - 12 * P0 * pow(T1, 2) * pow(T3, 2) - 18 * P0 * T1 * T2 * pow(T3, 2) + 6 * Pf * pow(T0, 2) * T1 * T2 + 6 * Pf * pow(T0, 2) * T1 * T3 - 6 * Pf * T0 * pow(T1, 2) * T2 - 6 * Pf * T0 * pow(T1, 2) * T3 - 36 * Pf * T0 * T1 * pow(T2, 2) - 54 * Pf * T0 * T1 * T2 * T3 - 24 * Pf * T0 * pow(T2, 3) - 48 * Pf * T0 * pow(T2, 2) * T3 - 6 * Pf * pow(T1, 3) * T2 - 6 * Pf * pow(T1, 3) * T3 - 24 * Pf * pow(T1, 2) * pow(T2, 2) - 36 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * T1 * pow(T2, 3) - 36 * Pf * T1 * pow(T2, 2) * T3 - 6 * pow(T0, 2) * T1 * T2 * T3 * Vf - 6 * pow(T0, 2) * T1 * T2 * d3 - 12 * pow(T0, 2) * T1 * pow(T3, 2) * V0 - 4 * pow(T0, 2) * T1 * pow(T3, 2) * Vf - 6 * pow(T0, 2) * T1 * T3 * d3 - 16 * pow(T0, 2) * T2 * pow(T3, 2) * V0 + 6 * T0 * pow(T1, 2) * T2 * T3 * Vf + 6 * T0 * pow(T1, 2) * T2 * d3 - 12 * T0 * pow(T1, 2) * pow(T3, 2) * V0 + 4 * T0 * pow(T1, 2) * pow(T3, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * d3 + 36 * T0 * T1 * pow(T2, 2) * T3 * Vf + 36 * T0 * T1 * pow(T2, 2) * d3 - 18 * T0 * T1 * T2 * pow(T3, 2) * V0 + 36 * T0 * T1 * T2 * pow(T3, 2) * Vf + 54 * T0 * T1 * T2 * T3 * d3 + 18 * T0 * T1 * pow(T3, 2) * d3 + 24 * T0 * pow(T2, 3) * T3 * Vf + 24 * T0 * pow(T2, 3) * d3 + 32 * T0 * pow(T2, 2) * pow(T3, 2) * Vf + 48 * T0 * pow(T2, 2) * T3 * d3 + 24 * T0 * T2 * pow(T3, 2) * d3 + 6 * pow(T1, 3) * T2 * T3 * Vf + 6 * pow(T1, 3) * T2 * d3 - 2 * pow(T1, 3) * pow(T3, 2) * V0 + 4 * pow(T1, 3) * pow(T3, 2) * Vf + 6 * pow(T1, 3) * T3 * d3 + 24 * pow(T1, 2) * pow(T2, 2) * T3 * Vf + 24 * pow(T1, 2) * pow(T2, 2) * d3 - 2 * pow(T1, 2) * T2 * pow(T3, 2) * V0 + 24 * pow(T1, 2) * T2 * pow(T3, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * d3 + 12 * pow(T1, 2) * pow(T3, 2) * d3 + 18 * T1 * pow(T2, 3) * T3 * Vf + 18 * T1 * pow(T2, 3) * d3 + 2 * T1 * pow(T2, 2) * pow(T3, 2) * V0 + 24 * T1 * pow(T2, 2) * pow(T3, 2) * Vf + 36 * T1 * pow(T2, 2) * T3 * d3 + 18 * T1 * T2 * pow(T3, 2) * d3) / (pow(T3, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     double vCP2_1 = (-1.0 / 2.0 * A0 * pow(T0, 2) * T2 * pow(T3, 2) - 1.0 / 2.0 * A0 * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * T2 * pow(T3, 2) + (1.0 / 2.0) * Af * T0 * T1 * pow(T3, 3) + Af * pow(T1, 2) * T2 * pow(T3, 2) + (1.0 / 2.0) * Af * pow(T1, 2) * pow(T3, 3) - Af * pow(T2, 3) * pow(T3, 2) - Af * pow(T2, 2) * pow(T3, 3) - 3 * P0 * T2 * pow(T3, 2) + 3 * Pf * T0 * T1 * T2 + 3 * Pf * T0 * T1 * T3 + 3 * Pf * pow(T1, 2) * T2 + 3 * Pf * pow(T1, 2) * T3 - 3 * Pf * pow(T2, 3) - 6 * Pf * pow(T2, 2) * T3 - 3 * T0 * T1 * T2 * T3 * Vf - 3 * T0 * T1 * T2 * d3 - 2 * T0 * T1 * pow(T3, 2) * Vf - 3 * T0 * T1 * T3 * d3 - 2 * T0 * T2 * pow(T3, 2) * V0 - 3 * pow(T1, 2) * T2 * T3 * Vf - 3 * pow(T1, 2) * T2 * d3 - 2 * pow(T1, 2) * pow(T3, 2) * Vf - 3 * pow(T1, 2) * T3 * d3 - T1 * T2 * pow(T3, 2) * V0 + 3 * pow(T2, 3) * T3 * Vf + 3 * pow(T2, 3) * d3 + 4 * pow(T2, 2) * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * d3 + 3 * T2 * pow(T3, 2) * d3) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double vCP0_2 = (-1.0 / 2.0 * A0 * pow(T0, 2) * T2 * pow(T3, 2) - 1.0 / 2.0 * A0 * T0 * T1 * T2 * pow(T3, 2) + Af * T0 * T1 * T2 * pow(T3, 2) + (1.0 / 2.0) * Af * T0 * T1 * pow(T3, 3) + Af * pow(T1, 2) * T2 * pow(T3, 2) + (1.0 / 2.0) * Af * pow(T1, 2) * pow(T3, 3) - Af * pow(T2, 3) * pow(T3, 2) - Af * pow(T2, 2) * pow(T3, 3) - 3 * P0 * T2 * pow(T3, 2) + 3 * Pf * T0 * T1 * T2 + 3 * Pf * T0 * T1 * T3 + 3 * Pf * pow(T1, 2) * T2 + 3 * Pf * pow(T1, 2) * T3 - 3 * Pf * pow(T2, 3) - 6 * Pf * pow(T2, 2) * T3 - 3 * T0 * T1 * T2 * T3 * Vf - 2 * T0 * T1 * pow(T3, 2) * Vf - 2 * T0 * T2 * pow(T3, 2) * V0 - 3 * pow(T1, 2) * T2 * T3 * Vf - 2 * pow(T1, 2) * pow(T3, 2) * Vf - T1 * T2 * pow(T3, 2) * V0 + 3 * pow(T2, 3) * T3 * Vf + 4 * pow(T2, 2) * pow(T3, 2) * Vf - 3 * d3 * (T0 * T1 * T2 + T0 * T1 * T3 + pow(T1, 2) * T2 + pow(T1, 2) * T3 - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2))) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP1_2 = (1.0 / 8.0) * (-A0 * pow(T0, 2) * T2 * pow(T3, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) + 4 * Af * T0 * T1 * pow(T3, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 4 * Af * pow(T1, 2) * pow(T3, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) + 2 * Af * pow(T2, 3) * pow(T3, 2) + Af * pow(T2, 2) * pow(T3, 3) - 6 * P0 * T2 * pow(T3, 2) + 18 * Pf * T0 * T1 * T2 + 24 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 + 18 * Pf * pow(T1, 2) * T2 + 24 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) + 36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 6 * Pf * pow(T2, 2) * T3 - 18 * T0 * T1 * T2 * T3 * Vf - 18 * T0 * T1 * T2 * d3 - 16 * T0 * T1 * pow(T3, 2) * Vf - 24 * T0 * T1 * T3 * d3 - 12 * T0 * pow(T2, 2) * T3 * Vf - 12 * T0 * pow(T2, 2) * d3 - 4 * T0 * T2 * pow(T3, 2) * V0 - 12 * T0 * T2 * pow(T3, 2) * Vf - 18 * T0 * T2 * T3 * d3 - 18 * pow(T1, 2) * T2 * T3 * Vf - 18 * pow(T1, 2) * T2 * d3 - 16 * pow(T1, 2) * pow(T3, 2) * Vf - 24 * pow(T1, 2) * T3 * d3 - 24 * T1 * pow(T2, 2) * T3 * Vf - 24 * T1 * pow(T2, 2) * d3 - 2 * T1 * T2 * pow(T3, 2) * V0 - 24 * T1 * T2 * pow(T3, 2) * Vf - 36 * T1 * T2 * T3 * d3 - 6 * pow(T2, 3) * T3 * Vf - 6 * pow(T2, 3) * d3 - 4 * pow(T2, 2) * pow(T3, 2) * Vf - 6 * pow(T2, 2) * T3 * d3 + 6 * T2 * pow(T3, 2) * d3) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP2_2 = (1.0 / 2.0) * Af * T3 + 3 * Pf / T3 - 2 * Vf - 3 * d3 / T3;
//     // Segment 3:
//     double vCP0_3 = (1.0 / 2.0) * Af * T3 + 3 * Pf / T3 - 2 * Vf - 3 * d3 / T3;
//     double vCP1_3 = (1.0 / 8.0) * (-Af * pow(T3, 2) + 6 * Pf + 2 * T3 * Vf - 6 * d3) / T3;
//     double vCP2_3 = Vf;

//     // Add control points to the list:
//     v_cp_.push_back({vCP0_0, vCP1_0, vCP2_0, vCP0_1, vCP1_1, vCP2_1, vCP0_2, vCP1_2, vCP2_2, vCP0_3, vCP1_3, vCP2_3});

//     // ACCELERATION CONTROL POINTS:
//     // Segment 0:
//     double aCP0_0 = A0;
//     double aCP1_0 = (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) - A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf + 6 * T1 * T2 * d3 - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * T1 * T3 * d3 + 6 * pow(T2, 2) * T3 * Vf + 6 * pow(T2, 2) * d3 - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf + 12 * T2 * T3 * d3 + 6 * pow(T3, 2) * d3) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double aCP0_1 = (-2 * A0 * pow(T0, 2) * pow(T3, 2) - 2 * A0 * T0 * T1 * pow(T3, 2) - A0 * T0 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf + 6 * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2))) / (pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double aCP1_1 = (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) + Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) + 6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 + 12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf - 6 * T0 * T2 * d3 + 4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 6 * T0 * T3 * d3 - 12 * T1 * T2 * T3 * Vf - 12 * T1 * T2 * d3 + 2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * T1 * T3 * d3 - 12 * pow(T2, 2) * T3 * Vf - 12 * pow(T2, 2) * d3 - 12 * T2 * pow(T3, 2) * Vf - 18 * T2 * T3 * d3 - 6 * pow(T3, 2) * d3) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double aCP0_2 = (A0 * pow(T0, 2) * pow(T3, 2) + A0 * T0 * T1 * pow(T3, 2) + 2 * Af * T0 * T2 * pow(T3, 2) + Af * T0 * pow(T3, 3) + 4 * Af * T1 * T2 * pow(T3, 2) + 2 * Af * T1 * pow(T3, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) + 3 * Af * T2 * pow(T3, 3) + 6 * P0 * pow(T3, 2) + 6 * Pf * T0 * T2 + 6 * Pf * T0 * T3 + 12 * Pf * T1 * T2 + 12 * Pf * T1 * T3 + 12 * Pf * pow(T2, 2) + 18 * Pf * T2 * T3 - 6 * T0 * T2 * T3 * Vf + 4 * T0 * pow(T3, 2) * V0 - 4 * T0 * pow(T3, 2) * Vf - 12 * T1 * T2 * T3 * Vf + 2 * T1 * pow(T3, 2) * V0 - 8 * T1 * pow(T3, 2) * Vf - 12 * pow(T2, 2) * T3 * Vf - 12 * T2 * pow(T3, 2) * Vf - 6 * d3 * (T0 * T2 + T0 * T3 + 2 * T1 * T2 + 2 * T1 * T3 + 2 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2))) / (pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double aCP1_2 = 2 * (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf + 3 * d3) / pow(T3, 2);
//     // Segment 3:
//     double aCP0_3 = 2 * (-Af * pow(T3, 2) - 3 * Pf + 3 * T3 * Vf + 3 * d3) / pow(T3, 2);
//     double aCP1_3 = Af;

//     // Add control points to the list:
//     a_cp_.push_back({aCP0_0, aCP1_0, aCP0_1, aCP1_1, aCP0_2, aCP1_2, aCP0_3, aCP1_3});

//     // JERK CONTROL POINTS:
//     // Segment 0:
//     double jCP0_0 = (-3 * A0 * pow(T0, 2) * pow(T3, 2) - 4 * A0 * T0 * T1 * pow(T3, 2) - 2 * A0 * T0 * T2 * pow(T3, 2) - A0 * pow(T1, 2) * pow(T3, 2) - A0 * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * T2 * pow(T3, 2) - Af * T1 * pow(T3, 3) - 2 * Af * pow(T2, 2) * pow(T3, 2) - 2 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T1 * T2 - 6 * Pf * T1 * T3 - 6 * Pf * pow(T2, 2) - 12 * Pf * T2 * T3 - 6 * T0 * pow(T3, 2) * V0 + 6 * T1 * T2 * T3 * Vf - 4 * T1 * pow(T3, 2) * V0 + 4 * T1 * pow(T3, 2) * Vf + 6 * pow(T2, 2) * T3 * Vf - 2 * T2 * pow(T3, 2) * V0 + 8 * T2 * pow(T3, 2) * Vf + 6 * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2))) / (T0 * pow(T3, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double jCP0_1 = (A0 * pow(T0, 3) * pow(T3, 2) + 4 * A0 * pow(T0, 2) * T1 * pow(T3, 2) + 2 * A0 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * A0 * T0 * pow(T1, 2) * pow(T3, 2) + 3 * A0 * T0 * T1 * T2 * pow(T3, 2) + A0 * T0 * pow(T2, 2) * pow(T3, 2) + 2 * Af * pow(T0, 2) * T2 * pow(T3, 2) + Af * pow(T0, 2) * pow(T3, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) + 3 * Af * T0 * T1 * pow(T3, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) + 3 * Af * T0 * T2 * pow(T3, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Af * T1 * T2 * pow(T3, 3) + 2 * Af * pow(T2, 3) * pow(T3, 2) + 2 * Af * pow(T2, 2) * pow(T3, 3) + 6 * P0 * T0 * pow(T3, 2) + 12 * P0 * T1 * pow(T3, 2) + 6 * P0 * T2 * pow(T3, 2) + 6 * Pf * pow(T0, 2) * T2 + 6 * Pf * pow(T0, 2) * T3 + 18 * Pf * T0 * T1 * T2 + 18 * Pf * T0 * T1 * T3 + 12 * Pf * T0 * pow(T2, 2) + 18 * Pf * T0 * T2 * T3 + 18 * Pf * pow(T1, 2) * T2 + 18 * Pf * pow(T1, 2) * T3 + 24 * Pf * T1 * pow(T2, 2) + 36 * Pf * T1 * T2 * T3 + 6 * Pf * pow(T2, 3) + 12 * Pf * pow(T2, 2) * T3 - 6 * pow(T0, 2) * T2 * T3 * Vf + 4 * pow(T0, 2) * pow(T3, 2) * V0 - 4 * pow(T0, 2) * pow(T3, 2) * Vf - 18 * T0 * T1 * T2 * T3 * Vf + 12 * T0 * T1 * pow(T3, 2) * V0 - 12 * T0 * T1 * pow(T3, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * Vf + 6 * T0 * T2 * pow(T3, 2) * V0 - 12 * T0 * T2 * pow(T3, 2) * Vf - 18 * pow(T1, 2) * T2 * T3 * Vf + 6 * pow(T1, 2) * pow(T3, 2) * V0 - 12 * pow(T1, 2) * pow(T3, 2) * Vf - 24 * T1 * pow(T2, 2) * T3 * Vf + 6 * T1 * T2 * pow(T3, 2) * V0 - 24 * T1 * T2 * pow(T3, 2) * Vf - 6 * pow(T2, 3) * T3 * Vf + 2 * pow(T2, 2) * pow(T3, 2) * V0 - 8 * pow(T2, 2) * pow(T3, 2) * Vf - 6 * d3 * (pow(T0, 2) * T2 + pow(T0, 2) * T3 + 3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 2 * T0 * pow(T2, 2) + 3 * T0 * T2 * T3 + T0 * pow(T3, 2) + 3 * pow(T1, 2) * T2 + 3 * pow(T1, 2) * T3 + 4 * T1 * pow(T2, 2) + 6 * T1 * T2 * T3 + 2 * T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2))) / (T1 * pow(T3, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     // Segment 2:
//     double jCP0_2 = (-A0 * pow(T0, 2) * pow(T3, 2) - A0 * T0 * T1 * pow(T3, 2) - 2 * Af * T0 * T1 * pow(T3, 2) - 4 * Af * T0 * T2 * pow(T3, 2) - Af * T0 * pow(T3, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) - 8 * Af * T1 * T2 * pow(T3, 2) - 2 * Af * T1 * pow(T3, 3) - 6 * Af * pow(T2, 2) * pow(T3, 2) - 3 * Af * T2 * pow(T3, 3) - 6 * P0 * pow(T3, 2) - 6 * Pf * T0 * T1 - 12 * Pf * T0 * T2 - 6 * Pf * T0 * T3 - 6 * Pf * pow(T1, 2) - 24 * Pf * T1 * T2 - 12 * Pf * T1 * T3 - 18 * Pf * pow(T2, 2) - 18 * Pf * T2 * T3 + 6 * T0 * T1 * T3 * Vf + 12 * T0 * T2 * T3 * Vf - 4 * T0 * pow(T3, 2) * V0 + 4 * T0 * pow(T3, 2) * Vf + 6 * pow(T1, 2) * T3 * Vf + 24 * T1 * T2 * T3 * Vf - 2 * T1 * pow(T3, 2) * V0 + 8 * T1 * pow(T3, 2) * Vf + 18 * pow(T2, 2) * T3 * Vf + 12 * T2 * pow(T3, 2) * Vf + 6 * d3 * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2))) / (T2 * pow(T3, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 3:
//     double jCP0_3 = 3 * (Af * pow(T3, 2) + 2 * Pf - 2 * T3 * Vf - 2 * d3) / pow(T3, 3);

//     // Add control points to the list:
//     j_cp_.push_back({jCP0_0, jCP0_1, jCP0_2, jCP0_3});
//   }
// }

// void SolverGurobi::computeControlPointsN5()
// {

//   p_cp_.clear(); // Clear the position control points
//   v_cp_.clear(); // Clear the velocity control points
//   a_cp_.clear(); // Clear the acceleration control points
//   j_cp_.clear(); // Clear the jerk control points

//   // Compute the control points for each axis

//   for (int axis = 0; axis < 3; axis++)
//   {
//     // POSITION CONTROL POINTS:
//     // Segment 0:
//     double CP0_0 = P0;
//     double CP1_0 = P0 + (1.0 / 3.0) * T0 * V0;
//     double CP2_0 = (1.0 / 6.0) * A0 * pow(T0, 2) + P0 + (2.0 / 3.0) * T0 * V0;
//     double CP3_0 = (1.0 / 6.0) * (pow(T0, 2) * (-T3 * (3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * A0 * T0 * T1 * T3 * pow(T4, 2) + 2 * A0 * T0 * T2 * T3 * pow(T4, 2) + A0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T1 * T2 * pow(T4, 3) - 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 2) * pow(T4, 3) - 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T1 * T2 * T3 - 18 * Pf * T1 * T2 * T4 - 6 * Pf * T1 * pow(T3, 2) - 12 * Pf * T1 * T3 * T4 - 12 * Pf * pow(T2, 2) * T3 - 18 * Pf * pow(T2, 2) * T4 - 12 * Pf * T2 * pow(T3, 2) - 24 * Pf * T2 * T3 * T4 + 6 * T0 * T3 * pow(T4, 2) * V0 + 12 * T1 * T2 * T3 * T4 * Vf + 12 * T1 * T2 * pow(T4, 2) * Vf + 6 * T1 * pow(T3, 2) * T4 * Vf + 4 * T1 * T3 * pow(T4, 2) * V0 + 8 * T1 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 2) * T3 * T4 * Vf + 12 * pow(T2, 2) * pow(T4, 2) * Vf + 12 * T2 * pow(T3, 2) * T4 * Vf + 2 * T2 * T3 * pow(T4, 2) * V0 + 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + 3 * pow(T3, 2) * pow(T4, 2) * (A0 * pow(T0, 2) + 2 * P0 + 2 * T0 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double CP0_1 = d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 + 18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) + 12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 + 18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf - 12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf + 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 - 16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) / (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * pow(T4, 2));
//     double CP1_1 = (1.0 / 3.0) * T1 * (d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) + 12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 + 18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) + 24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 - 12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf + 2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 + 18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) + 12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 + 18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf - 12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf + 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 - 16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) / (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * pow(T4, 2));
//     double CP2_1 = (1.0 / 3.0) * pow(T1, 2) * (d3 * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) - A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2))) + (1.0 / 3.0) * T1 * (2 * d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + 2 * d4 * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + 2 * (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) + 12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 + 18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) + 24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 - 12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf + 2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0) / (2 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 2 * T1 * T2 * T3 * pow(T4, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * pow(T4, 2) + A0 * pow(T0, 3) * T2 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 4 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 12 * P0 * T0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T0 * T2 * T3 * pow(T4, 2) + 6 * P0 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * P0 * T1 * T2 * T3 * pow(T4, 2) + 12 * Pf * pow(T0, 2) * T1 * T2 * T3 + 18 * Pf * pow(T0, 2) * T1 * T2 * T4 + 6 * Pf * pow(T0, 2) * T1 * pow(T3, 2) + 12 * Pf * pow(T0, 2) * T1 * T3 * T4 + 12 * Pf * pow(T0, 2) * pow(T2, 2) * T3 + 18 * Pf * pow(T0, 2) * pow(T2, 2) * T4 + 12 * Pf * pow(T0, 2) * T2 * pow(T3, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 - 12 * pow(T0, 2) * T1 * T2 * T3 * T4 * Vf - 12 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * Vf - 6 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * Vf + 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * V0 - 8 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 12 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * V0 - 16 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * V0) / (6 * pow(T0, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * pow(T4, 2));
//     double CP3_1 = ((1.0 / 6.0) * A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + (1.0 / 6.0) * A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + (4.0 / 3.0) * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + Af * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) + Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) + (2.0 / 3.0) * Af * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + (1.0 / 2.0) * Af * T0 * pow(T2, 3) * T3 * pow(T4, 3) + (2.0 / 3.0) * Af * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + (2.0 / 3.0) * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) + (4.0 / 3.0) * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) + Af * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) + Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) + (4.0 / 3.0) * Af * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + Af * T1 * pow(T2, 3) * T3 * pow(T4, 3) + (4.0 / 3.0) * Af * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + (4.0 / 3.0) * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) + P0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 6 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 + 3 * Pf * T0 * T1 * T2 * pow(T3, 3) + 6 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 + 2 * Pf * T0 * pow(T2, 3) * pow(T3, 2) + 3 * Pf * T0 * pow(T2, 3) * T3 * T4 + 2 * Pf * T0 * pow(T2, 2) * pow(T3, 3) + 4 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 + 4 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) + 6 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 + 3 * Pf * pow(T1, 2) * T2 * pow(T3, 3) + 6 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 + 4 * Pf * T1 * pow(T2, 3) * pow(T3, 2) + 6 * Pf * T1 * pow(T2, 3) * T3 * T4 + 4 * Pf * T1 * pow(T2, 2) * pow(T3, 3) + 8 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * d4 - 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 * d4 + 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * d3 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * d4 - 3 * T0 * T1 * T2 * pow(T3, 3) * T4 * Vf - 3 * T0 * T1 * T2 * pow(T3, 3) * d4 - 4 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * Vf - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 * d4 + 3 * T0 * T1 * T2 * T3 * pow(T4, 2) * d3 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) * d4 + T0 * T1 * pow(T3, 2) * pow(T4, 2) * d3 - 2 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * Vf - 2 * T0 * pow(T2, 3) * pow(T3, 2) * d4 - 2 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * Vf - 3 * T0 * pow(T2, 3) * T3 * T4 * d4 + T0 * pow(T2, 3) * pow(T4, 2) * d3 - T0 * pow(T2, 3) * pow(T4, 2) * d4 - 2 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * Vf - 2 * T0 * pow(T2, 2) * pow(T3, 3) * d4 + (2.0 / 3.0) * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * V0 - 8.0 / 3.0 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * d4 + 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * d3 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + T0 * T2 * pow(T3, 2) * pow(T4, 2) * d3 - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * d4 - 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * d4 + 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * d3 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * d4 - 3 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * Vf - 3 * pow(T1, 2) * T2 * pow(T3, 3) * d4 - 4 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * Vf - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * d4 + 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * d3 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * d4 + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * d3 - 4 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * Vf - 4 * T1 * pow(T2, 3) * pow(T3, 2) * d4 - 4 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * Vf - 6 * T1 * pow(T2, 3) * T3 * T4 * d4 + 2 * T1 * pow(T2, 3) * pow(T4, 2) * d3 - 2 * T1 * pow(T2, 3) * pow(T4, 2) * d4 - 4 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * Vf - 4 * T1 * pow(T2, 2) * pow(T3, 3) * d4 + (1.0 / 3.0) * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * V0 - 16.0 / 3.0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * d4 + 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * d3 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double CP0_2 = d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * Af * T1 * pow(T2, 3) * pow(T4, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 + 18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 + 12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 + 12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 + 24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 + 18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 + 24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 + 24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 - 24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * pow(T4, 2));
//     double CP1_2 = (1.0 / 3.0) * T2 * (d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) - 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) + 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 - 6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 - 12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 - 6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 + 12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) + 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf + 12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf + 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 + 12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf - 2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf - 12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * Af * T1 * pow(T2, 3) * pow(T4, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 + 18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 + 12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 + 12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 + 24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 + 18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 + 24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 + 24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 - 24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * pow(T4, 2));
//     double CP2_2 = (1.0 / 3.0) * pow(T2, 2) * (d3 * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) - 4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) - 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) - 8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) - 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) - 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 - 6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 - 36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 - 24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) - 36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf + 6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 + 8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf + 24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf + 2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2))) + (1.0 / 3.0) * T2 * (2 * d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + 2 * d4 * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + 2 * (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) - 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) + 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 - 6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 - 12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 - 6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 + 12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) + 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf + 12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf + 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 + 12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf - 2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf - 12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (2 * T0 * T1 * T3 * pow(T4, 2) + 2 * T0 * T2 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 2) + 4 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * T3 * pow(T4, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 8 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 3) * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 8 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * Af * T1 * pow(T2, 3) * pow(T4, 3) + 8 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 8 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * pow(T2, 2) * T3 + 36 * Pf * T0 * T1 * pow(T2, 2) * T4 + 18 * Pf * T0 * T1 * T2 * pow(T3, 2) + 36 * Pf * T0 * T1 * T2 * T3 * T4 + 12 * Pf * T0 * pow(T2, 3) * T3 + 18 * Pf * T0 * pow(T2, 3) * T4 + 12 * Pf * T0 * pow(T2, 2) * pow(T3, 2) + 24 * Pf * T0 * pow(T2, 2) * T3 * T4 + 24 * Pf * pow(T1, 2) * pow(T2, 2) * T3 + 36 * Pf * pow(T1, 2) * pow(T2, 2) * T4 + 18 * Pf * pow(T1, 2) * T2 * pow(T3, 2) + 36 * Pf * pow(T1, 2) * T2 * T3 * T4 + 24 * Pf * T1 * pow(T2, 3) * T3 + 36 * Pf * T1 * pow(T2, 3) * T4 + 24 * Pf * T1 * pow(T2, 2) * pow(T3, 2) + 48 * Pf * T1 * pow(T2, 2) * T3 * T4 - 24 * T0 * T1 * pow(T2, 2) * T3 * T4 * Vf - 24 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 3) * T3 * T4 * Vf - 12 * T0 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 16 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 24 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * Vf - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 3) * T3 * T4 * Vf - 24 * T1 * pow(T2, 3) * pow(T4, 2) * Vf - 24 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * V0 - 32 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) / (6 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * pow(T4, 2));
//     double CP3_2 = d3;
//     // Segment 3:
//     double CP0_3 = d3;
//     double CP1_3 = (1.0 / 3.0) * T3 * ((-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 + 3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / pow(T4, 2) - 3 * d3 / T3 + d4 * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2))) + d3;
//     double CP2_3 = (1.0 / 3.0) * pow(T3, 2) * ((1.0 / 2.0) * (4 * Af * T3 * pow(T4, 2) + 3 * Af * pow(T4, 3) + 12 * Pf * T3 + 18 * Pf * T4 - 12 * T3 * T4 * Vf - 12 * pow(T4, 2) * Vf) / (T3 * pow(T4, 2)) + 3 * d3 / pow(T3, 2) + d4 * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2))) + (1.0 / 3.0) * T3 * (2 * (-Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 3 * Pf * T3 - 6 * Pf * T4 + 3 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) / pow(T4, 2) - 6 * d3 / T3 + 2 * d4 * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2))) + d3;
//     double CP3_3 = d4;
//     // Segment 4:
//     double CP0_4 = d4;
//     double CP1_4 = (1.0 / 3.0) * T4 * (-3 * d4 / T4 + (1.0 / 2.0) * (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4) + d4;
//     double CP2_4 = (1.0 / 3.0) * pow(T4, 2) * (3 * d4 / pow(T4, 2) + (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf) / pow(T4, 2)) + (1.0 / 3.0) * T4 * (-6 * d4 / T4 + (Af * pow(T4, 2) + 6 * Pf - 4 * T4 * Vf) / T4) + d4;
//     double CP3_4 = Pf;

//     // Add control points to the list
//     p_cp_.push_back({CP0_0, CP1_0, CP2_0, CP3_0, CP0_1, CP1_1, CP2_1, CP3_1, CP0_2, CP1_2, CP2_2, CP3_2, CP0_3, CP1_3, CP2_3, CP3_3, CP0_4, CP1_4, CP2_4, CP3_4});

//     // VELOCITY CONTROL POINTS:
//     // Segment 0:
//     double vCP0_0 = V0;
//     double vCP1_0 = (1.0 / 8.0) * (T0 * (-T3 * (3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * A0 * T0 * T1 * T3 * pow(T4, 2) + 2 * A0 * T0 * T2 * T3 * pow(T4, 2) + A0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T1 * T2 * pow(T4, 3) - 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 2) * pow(T4, 3) - 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T1 * T2 * T3 - 18 * Pf * T1 * T2 * T4 - 6 * Pf * T1 * pow(T3, 2) - 12 * Pf * T1 * T3 * T4 - 12 * Pf * pow(T2, 2) * T3 - 18 * Pf * pow(T2, 2) * T4 - 12 * Pf * T2 * pow(T3, 2) - 24 * Pf * T2 * T3 * T4 + 6 * T0 * T3 * pow(T4, 2) * V0 + 12 * T1 * T2 * T3 * T4 * Vf + 12 * T1 * T2 * pow(T4, 2) * Vf + 6 * T1 * pow(T3, 2) * T4 * Vf + 4 * T1 * T3 * pow(T4, 2) * V0 + 8 * T1 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 2) * T3 * T4 * Vf + 12 * pow(T2, 2) * pow(T4, 2) * Vf + 12 * T2 * pow(T3, 2) * T4 * Vf + 2 * T2 * T3 * pow(T4, 2) * V0 + 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + 4 * pow(T3, 2) * pow(T4, 2) * (A0 * T0 + 2 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP2_0 = ((1.0 / 2.0) * T0 * (-T3 * (3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * A0 * T0 * T1 * T3 * pow(T4, 2) + 2 * A0 * T0 * T2 * T3 * pow(T4, 2) + A0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T1 * T2 * pow(T4, 3) - 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 2) * pow(T4, 3) - 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T1 * T2 * T3 - 18 * Pf * T1 * T2 * T4 - 6 * Pf * T1 * pow(T3, 2) - 12 * Pf * T1 * T3 * T4 - 12 * Pf * pow(T2, 2) * T3 - 18 * Pf * pow(T2, 2) * T4 - 12 * Pf * T2 * pow(T3, 2) - 24 * Pf * T2 * T3 * T4 + 6 * T0 * T3 * pow(T4, 2) * V0 + 12 * T1 * T2 * T3 * T4 * Vf + 12 * T1 * T2 * pow(T4, 2) * Vf + 6 * T1 * pow(T3, 2) * T4 * Vf + 4 * T1 * T3 * pow(T4, 2) * V0 + 8 * T1 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 2) * T3 * T4 * Vf + 12 * pow(T2, 2) * pow(T4, 2) * Vf + 12 * T2 * pow(T3, 2) * T4 * Vf + 2 * T2 * T3 * pow(T4, 2) * V0 + 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + pow(T3, 2) * pow(T4, 2) * (A0 * T0 + V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double vCP0_1 = (1.0 / 2.0) * (6 * T0 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * T0 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2)) + T3 * (-A0 * pow(T0, 3) * T3 * pow(T4, 2) + A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * T2 * pow(T4, 3) + 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T1 * T3 * pow(T4, 3) + 4 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * T0 * pow(T2, 2) * pow(T4, 3) + 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T0 * T2 * T3 * pow(T4, 3) - 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T2 * T3 + 18 * Pf * T0 * T1 * T2 * T4 + 6 * Pf * T0 * T1 * pow(T3, 2) + 12 * Pf * T0 * T1 * T3 * T4 + 12 * Pf * T0 * pow(T2, 2) * T3 + 18 * Pf * T0 * pow(T2, 2) * T4 + 12 * Pf * T0 * T2 * pow(T3, 2) + 24 * Pf * T0 * T2 * T3 * T4 - 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 - 12 * T0 * T1 * T2 * T3 * T4 * Vf - 12 * T0 * T1 * T2 * pow(T4, 2) * Vf - 6 * T0 * T1 * pow(T3, 2) * T4 * Vf - 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 12 * T0 * pow(T2, 2) * T3 * T4 * Vf - 12 * T0 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T0 * T2 * pow(T3, 2) * T4 * Vf - 16 * T0 * T2 * T3 * pow(T4, 2) * Vf + 2 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 2 * T1 * T2 * T3 * pow(T4, 2) * V0)) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP1_1 = (1.0 / 8.0) * (-3 * A0 * pow(T0, 3) * T1 * pow(T3, 2) * pow(T4, 2) - 4 * A0 * pow(T0, 3) * T2 * pow(T3, 2) * pow(T4, 2) - 4 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) - A0 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) - A0 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 3 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 3) - 2 * Af * pow(T0, 2) * T1 * pow(T3, 3) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 3) + 4 * Af * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 3 * Af * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 2 * Af * T0 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) + 2 * Af * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) + 24 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 18 * Af * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) + 18 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) + 16 * Af * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + 12 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 3) + 16 * Af * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + 16 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) + 4 * Af * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) + 3 * Af * pow(T1, 3) * T2 * T3 * pow(T4, 3) + 2 * Af * pow(T1, 3) * pow(T3, 3) * pow(T4, 2) + 2 * Af * pow(T1, 3) * pow(T3, 2) * pow(T4, 3) + 16 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) + 12 * Af * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) + 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) + 12 * Af * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + 9 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 3) + 12 * Af * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 18 * P0 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 24 * P0 * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * P0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 18 * P0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 18 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 6 * Pf * pow(T0, 2) * T1 * pow(T3, 3) - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 + 12 * Pf * T0 * pow(T1, 2) * T2 * pow(T3, 2) + 18 * Pf * T0 * pow(T1, 2) * T2 * T3 * T4 + 6 * Pf * T0 * pow(T1, 2) * pow(T3, 3) + 12 * Pf * T0 * pow(T1, 2) * pow(T3, 2) * T4 + 72 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) + 108 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 + 54 * Pf * T0 * T1 * T2 * pow(T3, 3) + 108 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 + 48 * Pf * T0 * pow(T2, 3) * pow(T3, 2) + 72 * Pf * T0 * pow(T2, 3) * T3 * T4 + 48 * Pf * T0 * pow(T2, 2) * pow(T3, 3) + 96 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 + 12 * Pf * pow(T1, 3) * T2 * pow(T3, 2) + 18 * Pf * pow(T1, 3) * T2 * T3 * T4 + 6 * Pf * pow(T1, 3) * pow(T3, 3) + 12 * Pf * pow(T1, 3) * pow(T3, 2) * T4 + 48 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) + 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * pow(T3, 3) + 72 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T2, 3) * pow(T3, 2) + 54 * Pf * T1 * pow(T2, 3) * T3 * T4 + 36 * Pf * T1 * pow(T2, 2) * pow(T3, 3) + 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 + 12 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * T4 * Vf + 12 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * d4 + 12 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * T3 * T4 * d4 - 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * d3 + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * d4 + 6 * pow(T0, 2) * T1 * pow(T3, 3) * T4 * Vf + 6 * pow(T0, 2) * T1 * pow(T3, 3) * d4 - 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * V0 + 8 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * d4 - 6 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * d3 + 6 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * d4 - 16 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) * V0 - 12 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf - 12 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * d4 - 12 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf - 18 * T0 * pow(T1, 2) * T2 * T3 * T4 * d4 + 6 * T0 * pow(T1, 2) * T2 * pow(T4, 2) * d3 - 6 * T0 * pow(T1, 2) * T2 * pow(T4, 2) * d4 - 6 * T0 * pow(T1, 2) * pow(T3, 3) * T4 * Vf - 6 * T0 * pow(T1, 2) * pow(T3, 3) * d4 - 12 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * V0 - 8 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 12 * T0 * pow(T1, 2) * pow(T3, 2) * T4 * d4 + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * d3 - 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * d4 - 72 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 72 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * d4 - 72 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 108 * T0 * T1 * pow(T2, 2) * T3 * T4 * d4 + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * d3 - 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * d4 - 54 * T0 * T1 * T2 * pow(T3, 3) * T4 * Vf - 54 * T0 * T1 * T2 * pow(T3, 3) * d4 - 18 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * V0 - 72 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * Vf - 108 * T0 * T1 * T2 * pow(T3, 2) * T4 * d4 + 54 * T0 * T1 * T2 * T3 * pow(T4, 2) * d3 - 54 * T0 * T1 * T2 * T3 * pow(T4, 2) * d4 + 18 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * d3 - 48 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * Vf - 48 * T0 * pow(T2, 3) * pow(T3, 2) * d4 - 48 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * Vf - 72 * T0 * pow(T2, 3) * T3 * T4 * d4 + 24 * T0 * pow(T2, 3) * pow(T4, 2) * d3 - 24 * T0 * pow(T2, 3) * pow(T4, 2) * d4 - 48 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * Vf - 48 * T0 * pow(T2, 2) * pow(T3, 3) * d4 - 64 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 96 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * d4 + 48 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * d3 - 48 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + 24 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * d3 - 12 * pow(T1, 3) * T2 * pow(T3, 2) * T4 * Vf - 12 * pow(T1, 3) * T2 * pow(T3, 2) * d4 - 12 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * Vf - 18 * pow(T1, 3) * T2 * T3 * T4 * d4 + 6 * pow(T1, 3) * T2 * pow(T4, 2) * d3 - 6 * pow(T1, 3) * T2 * pow(T4, 2) * d4 - 6 * pow(T1, 3) * pow(T3, 3) * T4 * Vf - 6 * pow(T1, 3) * pow(T3, 3) * d4 - 2 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * V0 - 8 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * Vf - 12 * pow(T1, 3) * pow(T3, 2) * T4 * d4 + 6 * pow(T1, 3) * T3 * pow(T4, 2) * d3 - 6 * pow(T1, 3) * T3 * pow(T4, 2) * d4 - 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * d4 - 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * Vf - 72 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * d4 + 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * d3 - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * d4 - 36 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * Vf - 36 * pow(T1, 2) * T2 * pow(T3, 3) * d4 - 2 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * V0 - 48 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * Vf - 72 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * d4 + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * d3 - 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * d4 + 12 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * d3 - 36 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * Vf - 36 * T1 * pow(T2, 3) * pow(T3, 2) * d4 - 36 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * Vf - 54 * T1 * pow(T2, 3) * T3 * T4 * d4 + 18 * T1 * pow(T2, 3) * pow(T4, 2) * d3 - 18 * T1 * pow(T2, 3) * pow(T4, 2) * d4 - 36 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * Vf - 36 * T1 * pow(T2, 2) * pow(T3, 3) * d4 + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * V0 - 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * d4 + 36 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * d3 - 36 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + 18 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     double vCP2_1 = (-1.0 / 2.0 * A0 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) - 1.0 / 2.0 * A0 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 3.0 / 2.0 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) - Af * T0 * T1 * pow(T3, 3) * pow(T4, 2) - Af * T0 * T1 * pow(T3, 2) * pow(T4, 3) - 2 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) - 3.0 / 2.0 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) - Af * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) - Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) + 2 * Af * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + (3.0 / 2.0) * Af * pow(T2, 3) * T3 * pow(T4, 3) + 2 * Af * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + 2 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 3 * P0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Pf * T0 * T1 * T2 * pow(T3, 2) - 9 * Pf * T0 * T1 * T2 * T3 * T4 - 3 * Pf * T0 * T1 * pow(T3, 3) - 6 * Pf * T0 * T1 * pow(T3, 2) * T4 - 6 * Pf * pow(T1, 2) * T2 * pow(T3, 2) - 9 * Pf * pow(T1, 2) * T2 * T3 * T4 - 3 * Pf * pow(T1, 2) * pow(T3, 3) - 6 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 6 * Pf * pow(T2, 3) * pow(T3, 2) + 9 * Pf * pow(T2, 3) * T3 * T4 + 6 * Pf * pow(T2, 2) * pow(T3, 3) + 12 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 6 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T1 * T2 * pow(T3, 2) * d4 + 6 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf + 9 * T0 * T1 * T2 * T3 * T4 * d4 - 3 * T0 * T1 * T2 * pow(T4, 2) * d3 + 3 * T0 * T1 * T2 * pow(T4, 2) * d4 + 3 * T0 * T1 * pow(T3, 3) * T4 * Vf + 3 * T0 * T1 * pow(T3, 3) * d4 + 4 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * d4 - 3 * T0 * T1 * T3 * pow(T4, 2) * d3 + 3 * T0 * T1 * T3 * pow(T4, 2) * d4 - 2 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * V0 + 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf + 6 * pow(T1, 2) * T2 * pow(T3, 2) * d4 + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf + 9 * pow(T1, 2) * T2 * T3 * T4 * d4 - 3 * pow(T1, 2) * T2 * pow(T4, 2) * d3 + 3 * pow(T1, 2) * T2 * pow(T4, 2) * d4 + 3 * pow(T1, 2) * pow(T3, 3) * T4 * Vf + 3 * pow(T1, 2) * pow(T3, 3) * d4 + 4 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * d4 - 3 * pow(T1, 2) * T3 * pow(T4, 2) * d3 + 3 * pow(T1, 2) * T3 * pow(T4, 2) * d4 - T1 * T2 * pow(T3, 2) * pow(T4, 2) * V0 - 6 * pow(T2, 3) * pow(T3, 2) * T4 * Vf - 6 * pow(T2, 3) * pow(T3, 2) * d4 - 6 * pow(T2, 3) * T3 * pow(T4, 2) * Vf - 9 * pow(T2, 3) * T3 * T4 * d4 + 3 * pow(T2, 3) * pow(T4, 2) * d3 - 3 * pow(T2, 3) * pow(T4, 2) * d4 - 6 * pow(T2, 2) * pow(T3, 3) * T4 * Vf - 6 * pow(T2, 2) * pow(T3, 3) * d4 - 8 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * d4 + 6 * pow(T2, 2) * T3 * pow(T4, 2) * d3 - 6 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + 3 * T2 * pow(T3, 2) * pow(T4, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double vCP0_2 = (1.0 / 2.0) * (T3 * (-A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) - A0 * T0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T1 * T2 * pow(T4, 3) - 2 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T1, 2) * T3 * pow(T4, 3) + 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 3) * pow(T4, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * T0 * T1 * T2 * T3 - 18 * Pf * T0 * T1 * T2 * T4 - 6 * Pf * T0 * T1 * pow(T3, 2) - 12 * Pf * T0 * T1 * T3 * T4 - 12 * Pf * pow(T1, 2) * T2 * T3 - 18 * Pf * pow(T1, 2) * T2 * T4 - 6 * Pf * pow(T1, 2) * pow(T3, 2) - 12 * Pf * pow(T1, 2) * T3 * T4 + 12 * Pf * pow(T2, 3) * T3 + 18 * Pf * pow(T2, 3) * T4 + 12 * Pf * pow(T2, 2) * pow(T3, 2) + 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * T0 * T1 * T2 * T3 * T4 * Vf + 12 * T0 * T1 * T2 * pow(T4, 2) * Vf + 6 * T0 * T1 * pow(T3, 2) * T4 * Vf + 8 * T0 * T1 * T3 * pow(T4, 2) * Vf - 4 * T0 * T2 * T3 * pow(T4, 2) * V0 + 12 * pow(T1, 2) * T2 * T3 * T4 * Vf + 12 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 8 * pow(T1, 2) * T3 * pow(T4, 2) * Vf - 2 * T1 * T2 * T3 * pow(T4, 2) * V0 - 12 * pow(T2, 3) * T3 * T4 * Vf - 12 * pow(T2, 3) * pow(T4, 2) * Vf - 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf - 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) - 6 * pow(T4, 2) * d3 * (T0 * T1 * T2 + T0 * T1 * T3 + pow(T1, 2) * T2 + pow(T1, 2) * T3 - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) + 6 * d4 * (2 * T0 * T1 * T2 * pow(T3, 2) + 3 * T0 * T1 * T2 * T3 * T4 + T0 * T1 * T2 * pow(T4, 2) + T0 * T1 * pow(T3, 3) + 2 * T0 * T1 * pow(T3, 2) * T4 + T0 * T1 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T2 * pow(T3, 2) + 3 * pow(T1, 2) * T2 * T3 * T4 + pow(T1, 2) * T2 * pow(T4, 2) + pow(T1, 2) * pow(T3, 3) + 2 * pow(T1, 2) * pow(T3, 2) * T4 + pow(T1, 2) * T3 * pow(T4, 2) - 2 * pow(T2, 3) * pow(T3, 2) - 3 * pow(T2, 3) * T3 * T4 - pow(T2, 3) * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 3) - 4 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * pow(T2, 2) * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP1_2 = (1.0 / 8.0) * (-A0 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) - 8 * Af * T0 * T1 * pow(T3, 3) * pow(T4, 2) - 8 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) - 6 * Af * T0 * T2 * pow(T3, 3) * pow(T4, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) - 8 * Af * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) - 8 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) - 12 * Af * T1 * T2 * pow(T3, 3) * pow(T4, 2) - 12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 3) - 4 * Af * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 3 * Af * pow(T2, 3) * T3 * pow(T4, 3) - 2 * Af * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 2 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 6 * P0 * T2 * pow(T3, 2) * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) - 54 * Pf * T0 * T1 * T2 * T3 * T4 - 24 * Pf * T0 * T1 * pow(T3, 3) - 48 * Pf * T0 * T1 * pow(T3, 2) * T4 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) - 36 * Pf * T0 * pow(T2, 2) * T3 * T4 - 18 * Pf * T0 * T2 * pow(T3, 3) - 36 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) - 54 * Pf * pow(T1, 2) * T2 * T3 * T4 - 24 * Pf * pow(T1, 2) * pow(T3, 3) - 48 * Pf * pow(T1, 2) * pow(T3, 2) * T4 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) - 72 * Pf * T1 * pow(T2, 2) * T3 * T4 - 36 * Pf * T1 * T2 * pow(T3, 3) - 72 * Pf * T1 * T2 * pow(T3, 2) * T4 - 12 * Pf * pow(T2, 3) * pow(T3, 2) - 18 * Pf * pow(T2, 3) * T3 * T4 - 6 * Pf * pow(T2, 2) * pow(T3, 3) - 12 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * d4 + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * Vf + 54 * T0 * T1 * T2 * T3 * T4 * d4 - 18 * T0 * T1 * T2 * pow(T4, 2) * d3 + 18 * T0 * T1 * T2 * pow(T4, 2) * d4 + 24 * T0 * T1 * pow(T3, 3) * T4 * Vf + 24 * T0 * T1 * pow(T3, 3) * d4 + 32 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * Vf + 48 * T0 * T1 * pow(T3, 2) * T4 * d4 - 24 * T0 * T1 * T3 * pow(T4, 2) * d3 + 24 * T0 * T1 * T3 * pow(T4, 2) * d4 + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * d4 + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * pow(T2, 2) * T3 * T4 * d4 - 12 * T0 * pow(T2, 2) * pow(T4, 2) * d3 + 12 * T0 * pow(T2, 2) * pow(T4, 2) * d4 + 18 * T0 * T2 * pow(T3, 3) * T4 * Vf + 18 * T0 * T2 * pow(T3, 3) * d4 - 4 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * V0 + 24 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * Vf + 36 * T0 * T2 * pow(T3, 2) * T4 * d4 - 18 * T0 * T2 * T3 * pow(T4, 2) * d3 + 18 * T0 * T2 * T3 * pow(T4, 2) * d4 + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * d4 + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * Vf + 54 * pow(T1, 2) * T2 * T3 * T4 * d4 - 18 * pow(T1, 2) * T2 * pow(T4, 2) * d3 + 18 * pow(T1, 2) * T2 * pow(T4, 2) * d4 + 24 * pow(T1, 2) * pow(T3, 3) * T4 * Vf + 24 * pow(T1, 2) * pow(T3, 3) * d4 + 32 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * Vf + 48 * pow(T1, 2) * pow(T3, 2) * T4 * d4 - 24 * pow(T1, 2) * T3 * pow(T4, 2) * d3 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * d4 + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * d4 + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * Vf + 72 * T1 * pow(T2, 2) * T3 * T4 * d4 - 24 * T1 * pow(T2, 2) * pow(T4, 2) * d3 + 24 * T1 * pow(T2, 2) * pow(T4, 2) * d4 + 36 * T1 * T2 * pow(T3, 3) * T4 * Vf + 36 * T1 * T2 * pow(T3, 3) * d4 - 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * V0 + 48 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * Vf + 72 * T1 * T2 * pow(T3, 2) * T4 * d4 - 36 * T1 * T2 * T3 * pow(T4, 2) * d3 + 36 * T1 * T2 * T3 * pow(T4, 2) * d4 + 12 * pow(T2, 3) * pow(T3, 2) * T4 * Vf + 12 * pow(T2, 3) * pow(T3, 2) * d4 + 12 * pow(T2, 3) * T3 * pow(T4, 2) * Vf + 18 * pow(T2, 3) * T3 * T4 * d4 - 6 * pow(T2, 3) * pow(T4, 2) * d3 + 6 * pow(T2, 3) * pow(T4, 2) * d4 + 6 * pow(T2, 2) * pow(T3, 3) * T4 * Vf + 6 * pow(T2, 2) * pow(T3, 3) * d4 + 8 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * d4 - 6 * pow(T2, 2) * T3 * pow(T4, 2) * d3 + 6 * pow(T2, 2) * T3 * pow(T4, 2) * d4 + 6 * T2 * pow(T3, 2) * pow(T4, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP2_2 = -Af * T3 - Af * T4 - 3 * Pf * T3 / pow(T4, 2) - 6 * Pf / T4 + 3 * T3 * Vf / T4 + 3 * T3 * d4 / pow(T4, 2) + 4 * Vf + 6 * d4 / T4 - 3 * d3 / T3 + 3 * d4 / T3;
//     // Segment 3:
//     double vCP0_3 = -Af * T3 - Af * T4 - 3 * Pf * T3 / pow(T4, 2) - 6 * Pf / T4 + 3 * T3 * Vf / T4 + 3 * T3 * d4 / pow(T4, 2) + 4 * Vf + 6 * d4 / T4 - 3 * d3 / T3 + 3 * d4 / T3;
//     double vCP1_3 = (1.0 / 4.0) * Af * T3 + (1.0 / 8.0) * Af * T4 + (3.0 / 4.0) * Pf * T3 / pow(T4, 2) + (3.0 / 4.0) * Pf / T4 - 3.0 / 4.0 * T3 * Vf / T4 - 3.0 / 4.0 * T3 * d4 / pow(T4, 2) - 1.0 / 2.0 * Vf - 3.0 / 4.0 * d4 / T4 - 3.0 / 4.0 * d3 / T3 + (3.0 / 4.0) * d4 / T3;
//     double vCP2_3 = (1.0 / 2.0) * Af * T4 + 3 * Pf / T4 - 2 * Vf - 3 * d4 / T4;
//     // Segment 4:
//     double vCP0_4 = (1.0 / 2.0) * Af * T4 + 3 * Pf / T4 - 2 * Vf - 3 * d4 / T4;
//     double vCP1_4 = (1.0 / 8.0) * (-Af * pow(T4, 2) + 6 * Pf + 2 * T4 * Vf - 6 * d4) / T4;
//     double vCP2_4 = Vf;

//     // Add control points to the list
//     v_cp_.push_back({vCP0_0, vCP1_0, vCP2_0, vCP0_1, vCP1_1, vCP2_1, vCP0_2, vCP1_2, vCP2_2, vCP0_3, vCP1_3, vCP2_3, vCP0_4, vCP1_4, vCP2_4});

//     // ACCELERATION CONTROL POINTS:
//     // Segment 0:
//     double aCP0_0 = A0;
//     double aCP1_0 = (A0 * pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2) - T3 * (3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) + 4 * A0 * T0 * T1 * T3 * pow(T4, 2) + 2 * A0 * T0 * T2 * T3 * pow(T4, 2) + A0 * pow(T1, 2) * T3 * pow(T4, 2) + A0 * T1 * T2 * T3 * pow(T4, 2) - 4 * Af * T1 * T2 * T3 * pow(T4, 2) - 3 * Af * T1 * T2 * pow(T4, 3) - 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T1 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 2) * pow(T4, 3) - 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T1 * T2 * T3 - 18 * Pf * T1 * T2 * T4 - 6 * Pf * T1 * pow(T3, 2) - 12 * Pf * T1 * T3 * T4 - 12 * Pf * pow(T2, 2) * T3 - 18 * Pf * pow(T2, 2) * T4 - 12 * Pf * T2 * pow(T3, 2) - 24 * Pf * T2 * T3 * T4 + 6 * T0 * T3 * pow(T4, 2) * V0 + 12 * T1 * T2 * T3 * T4 * Vf + 12 * T1 * T2 * pow(T4, 2) * Vf + 6 * T1 * pow(T3, 2) * T4 * Vf + 4 * T1 * T3 * pow(T4, 2) * V0 + 8 * T1 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 2) * T3 * T4 * Vf + 12 * pow(T2, 2) * pow(T4, 2) * Vf + 12 * T2 * pow(T3, 2) * T4 * Vf + 2 * T2 * T3 * pow(T4, 2) * V0 + 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double aCP0_1 = (T3 * (-2 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 2 * A0 * T0 * T1 * T3 * pow(T4, 2) - A0 * T0 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double aCP1_1 = (A0 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + A0 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 3 * Af * T0 * T2 * T3 * pow(T4, 3) - 2 * Af * T0 * pow(T3, 3) * pow(T4, 2) - 2 * Af * T0 * pow(T3, 2) * pow(T4, 3) - 8 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T1 * T2 * T3 * pow(T4, 3) - 4 * Af * T1 * pow(T3, 3) * pow(T4, 2) - 4 * Af * T1 * pow(T3, 2) * pow(T4, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T4, 3) - 6 * Af * T2 * pow(T3, 3) * pow(T4, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T4, 3) + 6 * P0 * pow(T3, 2) * pow(T4, 2) - 12 * Pf * T0 * T2 * pow(T3, 2) - 18 * Pf * T0 * T2 * T3 * T4 - 6 * Pf * T0 * pow(T3, 3) - 12 * Pf * T0 * pow(T3, 2) * T4 - 24 * Pf * T1 * T2 * pow(T3, 2) - 36 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * T1 * pow(T3, 3) - 24 * Pf * T1 * pow(T3, 2) * T4 - 24 * Pf * pow(T2, 2) * pow(T3, 2) - 36 * Pf * pow(T2, 2) * T3 * T4 - 18 * Pf * T2 * pow(T3, 3) - 36 * Pf * T2 * pow(T3, 2) * T4 + 12 * T0 * T2 * pow(T3, 2) * T4 * Vf + 12 * T0 * T2 * pow(T3, 2) * d4 + 12 * T0 * T2 * T3 * pow(T4, 2) * Vf + 18 * T0 * T2 * T3 * T4 * d4 - 6 * T0 * T2 * pow(T4, 2) * d3 + 6 * T0 * T2 * pow(T4, 2) * d4 + 6 * T0 * pow(T3, 3) * T4 * Vf + 6 * T0 * pow(T3, 3) * d4 + 4 * T0 * pow(T3, 2) * pow(T4, 2) * V0 + 8 * T0 * pow(T3, 2) * pow(T4, 2) * Vf + 12 * T0 * pow(T3, 2) * T4 * d4 - 6 * T0 * T3 * pow(T4, 2) * d3 + 6 * T0 * T3 * pow(T4, 2) * d4 + 24 * T1 * T2 * pow(T3, 2) * T4 * Vf + 24 * T1 * T2 * pow(T3, 2) * d4 + 24 * T1 * T2 * T3 * pow(T4, 2) * Vf + 36 * T1 * T2 * T3 * T4 * d4 - 12 * T1 * T2 * pow(T4, 2) * d3 + 12 * T1 * T2 * pow(T4, 2) * d4 + 12 * T1 * pow(T3, 3) * T4 * Vf + 12 * T1 * pow(T3, 3) * d4 + 2 * T1 * pow(T3, 2) * pow(T4, 2) * V0 + 16 * T1 * pow(T3, 2) * pow(T4, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * d4 - 12 * T1 * T3 * pow(T4, 2) * d3 + 12 * T1 * T3 * pow(T4, 2) * d4 + 24 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * d4 + 24 * pow(T2, 2) * T3 * pow(T4, 2) * Vf + 36 * pow(T2, 2) * T3 * T4 * d4 - 12 * pow(T2, 2) * pow(T4, 2) * d3 + 12 * pow(T2, 2) * pow(T4, 2) * d4 + 18 * T2 * pow(T3, 3) * T4 * Vf + 18 * T2 * pow(T3, 3) * d4 + 24 * T2 * pow(T3, 2) * pow(T4, 2) * Vf + 36 * T2 * pow(T3, 2) * T4 * d4 - 18 * T2 * T3 * pow(T4, 2) * d3 + 18 * T2 * T3 * pow(T4, 2) * d4 - 6 * pow(T3, 2) * pow(T4, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double aCP0_2 = (T3 * (A0 * pow(T0, 2) * T3 * pow(T4, 2) + A0 * T0 * T1 * T3 * pow(T4, 2) - 4 * Af * T0 * T2 * T3 * pow(T4, 2) - 3 * Af * T0 * T2 * pow(T4, 3) - 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) - 2 * Af * T0 * T3 * pow(T4, 3) - 8 * Af * T1 * T2 * T3 * pow(T4, 2) - 6 * Af * T1 * T2 * pow(T4, 3) - 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) - 4 * Af * T1 * T3 * pow(T4, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * pow(T2, 2) * pow(T4, 3) - 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T2 * T3 * pow(T4, 3) + 6 * P0 * T3 * pow(T4, 2) - 12 * Pf * T0 * T2 * T3 - 18 * Pf * T0 * T2 * T4 - 6 * Pf * T0 * pow(T3, 2) - 12 * Pf * T0 * T3 * T4 - 24 * Pf * T1 * T2 * T3 - 36 * Pf * T1 * T2 * T4 - 12 * Pf * T1 * pow(T3, 2) - 24 * Pf * T1 * T3 * T4 - 24 * Pf * pow(T2, 2) * T3 - 36 * Pf * pow(T2, 2) * T4 - 18 * Pf * T2 * pow(T3, 2) - 36 * Pf * T2 * T3 * T4 + 12 * T0 * T2 * T3 * T4 * Vf + 12 * T0 * T2 * pow(T4, 2) * Vf + 6 * T0 * pow(T3, 2) * T4 * Vf + 4 * T0 * T3 * pow(T4, 2) * V0 + 8 * T0 * T3 * pow(T4, 2) * Vf + 24 * T1 * T2 * T3 * T4 * Vf + 24 * T1 * T2 * pow(T4, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * Vf + 2 * T1 * T3 * pow(T4, 2) * V0 + 16 * T1 * T3 * pow(T4, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * Vf + 24 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T2 * pow(T3, 2) * T4 * Vf + 24 * T2 * T3 * pow(T4, 2) * Vf) - 6 * pow(T4, 2) * d3 * (T0 * T2 + T0 * T3 + 2 * T1 * T2 + 2 * T1 * T3 + 2 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) + 6 * d4 * (2 * T0 * T2 * pow(T3, 2) + 3 * T0 * T2 * T3 * T4 + T0 * T2 * pow(T4, 2) + T0 * pow(T3, 3) + 2 * T0 * pow(T3, 2) * T4 + T0 * T3 * pow(T4, 2) + 4 * T1 * T2 * pow(T3, 2) + 6 * T1 * T2 * T3 * T4 + 2 * T1 * T2 * pow(T4, 2) + 2 * T1 * pow(T3, 3) + 4 * T1 * pow(T3, 2) * T4 + 2 * T1 * T3 * pow(T4, 2) + 4 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T2, 2) * T3 * T4 + 2 * pow(T2, 2) * pow(T4, 2) + 3 * T2 * pow(T3, 3) + 6 * T2 * pow(T3, 2) * T4 + 3 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double aCP1_2 = 4 * Af + 3 * Af * T4 / T3 + 12 * Pf / pow(T4, 2) + 18 * Pf / (T3 * T4) - 12 * Vf / T4 - 12 * d4 / pow(T4, 2) - 12 * Vf / T3 - 18 * d4 / (T3 * T4) + 6 * d3 / pow(T3, 2) - 6 * d4 / pow(T3, 2);
//     // Segment 3:
//     double aCP0_3 = 4 * Af + 3 * Af * T4 / T3 + 12 * Pf / pow(T4, 2) + 18 * Pf / (T3 * T4) - 12 * Vf / T4 - 12 * d4 / pow(T4, 2) - 12 * Vf / T3 - 18 * d4 / (T3 * T4) + 6 * d3 / pow(T3, 2) - 6 * d4 / pow(T3, 2);
//     double aCP1_3 = 2 * (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf + 3 * d4) / pow(T4, 2);
//     // Segment 4:
//     double aCP0_4 = 2 * (-Af * pow(T4, 2) - 3 * Pf + 3 * T4 * Vf + 3 * d4) / pow(T4, 2);
//     double aCP1_4 = Af;

//     // Add control points to the list
//     a_cp_.push_back({aCP0_0, aCP1_0, aCP0_1, aCP1_1, aCP0_2, aCP1_2, aCP0_3, aCP1_3, aCP0_4, aCP1_4});

//     // JERK CONTROL POINTS:
//     // Segment 0:
//     double jCP0_0 = (T3 * (-3 * A0 * pow(T0, 2) * T3 * pow(T4, 2) - 4 * A0 * T0 * T1 * T3 * pow(T4, 2) - 2 * A0 * T0 * T2 * T3 * pow(T4, 2) - A0 * pow(T1, 2) * T3 * pow(T4, 2) - A0 * T1 * T2 * T3 * pow(T4, 2) + 4 * Af * T1 * T2 * T3 * pow(T4, 2) + 3 * Af * T1 * T2 * pow(T4, 3) + 2 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T1 * T3 * pow(T4, 3) + 4 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T2, 2) * pow(T4, 3) + 4 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T1 * T2 * T3 + 18 * Pf * T1 * T2 * T4 + 6 * Pf * T1 * pow(T3, 2) + 12 * Pf * T1 * T3 * T4 + 12 * Pf * pow(T2, 2) * T3 + 18 * Pf * pow(T2, 2) * T4 + 12 * Pf * T2 * pow(T3, 2) + 24 * Pf * T2 * T3 * T4 - 6 * T0 * T3 * pow(T4, 2) * V0 - 12 * T1 * T2 * T3 * T4 * Vf - 12 * T1 * T2 * pow(T4, 2) * Vf - 6 * T1 * pow(T3, 2) * T4 * Vf - 4 * T1 * T3 * pow(T4, 2) * V0 - 8 * T1 * T3 * pow(T4, 2) * Vf - 12 * pow(T2, 2) * T3 * T4 * Vf - 12 * pow(T2, 2) * pow(T4, 2) * Vf - 12 * T2 * pow(T3, 2) * T4 * Vf - 2 * T2 * T3 * pow(T4, 2) * V0 - 16 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (T0 * pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double jCP0_1 = (T3 * (A0 * pow(T0, 3) * T3 * pow(T4, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * pow(T4, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * pow(T4, 2) + 3 * A0 * T0 * T1 * T2 * T3 * pow(T4, 2) + A0 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 3 * Af * pow(T0, 2) * T2 * pow(T4, 3) - 2 * Af * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) - 2 * Af * pow(T0, 2) * T3 * pow(T4, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) - 9 * Af * T0 * T1 * T2 * pow(T4, 3) - 6 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T1 * T3 * pow(T4, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 3) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) - 6 * Af * T0 * T2 * T3 * pow(T4, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T4, 3) - 6 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T4, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T4, 3) - 12 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) - 12 * Af * T1 * T2 * T3 * pow(T4, 3) - 4 * Af * pow(T2, 3) * T3 * pow(T4, 2) - 3 * Af * pow(T2, 3) * pow(T4, 3) - 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 4 * Af * pow(T2, 2) * T3 * pow(T4, 3) + 6 * P0 * T0 * T3 * pow(T4, 2) + 12 * P0 * T1 * T3 * pow(T4, 2) + 6 * P0 * T2 * T3 * pow(T4, 2) - 12 * Pf * pow(T0, 2) * T2 * T3 - 18 * Pf * pow(T0, 2) * T2 * T4 - 6 * Pf * pow(T0, 2) * pow(T3, 2) - 12 * Pf * pow(T0, 2) * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 - 54 * Pf * T0 * T1 * T2 * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) - 36 * Pf * T0 * T1 * T3 * T4 - 24 * Pf * T0 * pow(T2, 2) * T3 - 36 * Pf * T0 * pow(T2, 2) * T4 - 18 * Pf * T0 * T2 * pow(T3, 2) - 36 * Pf * T0 * T2 * T3 * T4 - 36 * Pf * pow(T1, 2) * T2 * T3 - 54 * Pf * pow(T1, 2) * T2 * T4 - 18 * Pf * pow(T1, 2) * pow(T3, 2) - 36 * Pf * pow(T1, 2) * T3 * T4 - 48 * Pf * T1 * pow(T2, 2) * T3 - 72 * Pf * T1 * pow(T2, 2) * T4 - 36 * Pf * T1 * T2 * pow(T3, 2) - 72 * Pf * T1 * T2 * T3 * T4 - 12 * Pf * pow(T2, 3) * T3 - 18 * Pf * pow(T2, 3) * T4 - 12 * Pf * pow(T2, 2) * pow(T3, 2) - 24 * Pf * pow(T2, 2) * T3 * T4 + 12 * pow(T0, 2) * T2 * T3 * T4 * Vf + 12 * pow(T0, 2) * T2 * pow(T4, 2) * Vf + 6 * pow(T0, 2) * pow(T3, 2) * T4 * Vf + 4 * pow(T0, 2) * T3 * pow(T4, 2) * V0 + 8 * pow(T0, 2) * T3 * pow(T4, 2) * Vf + 36 * T0 * T1 * T2 * T3 * T4 * Vf + 36 * T0 * T1 * T2 * pow(T4, 2) * Vf + 18 * T0 * T1 * pow(T3, 2) * T4 * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * V0 + 24 * T0 * T1 * T3 * pow(T4, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * Vf + 24 * T0 * pow(T2, 2) * pow(T4, 2) * Vf + 18 * T0 * T2 * pow(T3, 2) * T4 * Vf + 6 * T0 * T2 * T3 * pow(T4, 2) * V0 + 24 * T0 * T2 * T3 * pow(T4, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * T4 * Vf + 36 * pow(T1, 2) * T2 * pow(T4, 2) * Vf + 18 * pow(T1, 2) * pow(T3, 2) * T4 * Vf + 6 * pow(T1, 2) * T3 * pow(T4, 2) * V0 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * T4 * Vf + 48 * T1 * pow(T2, 2) * pow(T4, 2) * Vf + 36 * T1 * T2 * pow(T3, 2) * T4 * Vf + 6 * T1 * T2 * T3 * pow(T4, 2) * V0 + 48 * T1 * T2 * T3 * pow(T4, 2) * Vf + 12 * pow(T2, 3) * T3 * T4 * Vf + 12 * pow(T2, 3) * pow(T4, 2) * Vf + 12 * pow(T2, 2) * pow(T3, 2) * T4 * Vf + 2 * pow(T2, 2) * T3 * pow(T4, 2) * V0 + 16 * pow(T2, 2) * T3 * pow(T4, 2) * Vf) - 6 * pow(T4, 2) * d3 * (pow(T0, 2) * T2 + pow(T0, 2) * T3 + 3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 2 * T0 * pow(T2, 2) + 3 * T0 * T2 * T3 + T0 * pow(T3, 2) + 3 * pow(T1, 2) * T2 + 3 * pow(T1, 2) * T3 + 4 * T1 * pow(T2, 2) + 6 * T1 * T2 * T3 + 2 * T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)) + 6 * d4 * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2))) / (T1 * pow(T3, 2) * pow(T4, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     // Segment 2:
//     double jCP0_2 = (T3 * (-A0 * pow(T0, 2) * T3 * pow(T4, 2) - A0 * T0 * T1 * T3 * pow(T4, 2) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) + 3 * Af * T0 * T1 * pow(T4, 3) + 8 * Af * T0 * T2 * T3 * pow(T4, 2) + 6 * Af * T0 * T2 * pow(T4, 3) + 2 * Af * T0 * pow(T3, 2) * pow(T4, 2) + 2 * Af * T0 * T3 * pow(T4, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) + 3 * Af * pow(T1, 2) * pow(T4, 3) + 16 * Af * T1 * T2 * T3 * pow(T4, 2) + 12 * Af * T1 * T2 * pow(T4, 3) + 4 * Af * T1 * pow(T3, 2) * pow(T4, 2) + 4 * Af * T1 * T3 * pow(T4, 3) + 12 * Af * pow(T2, 2) * T3 * pow(T4, 2) + 9 * Af * pow(T2, 2) * pow(T4, 3) + 6 * Af * T2 * pow(T3, 2) * pow(T4, 2) + 6 * Af * T2 * T3 * pow(T4, 3) - 6 * P0 * T3 * pow(T4, 2) + 12 * Pf * T0 * T1 * T3 + 18 * Pf * T0 * T1 * T4 + 24 * Pf * T0 * T2 * T3 + 36 * Pf * T0 * T2 * T4 + 6 * Pf * T0 * pow(T3, 2) + 12 * Pf * T0 * T3 * T4 + 12 * Pf * pow(T1, 2) * T3 + 18 * Pf * pow(T1, 2) * T4 + 48 * Pf * T1 * T2 * T3 + 72 * Pf * T1 * T2 * T4 + 12 * Pf * T1 * pow(T3, 2) + 24 * Pf * T1 * T3 * T4 + 36 * Pf * pow(T2, 2) * T3 + 54 * Pf * pow(T2, 2) * T4 + 18 * Pf * T2 * pow(T3, 2) + 36 * Pf * T2 * T3 * T4 - 12 * T0 * T1 * T3 * T4 * Vf - 12 * T0 * T1 * pow(T4, 2) * Vf - 24 * T0 * T2 * T3 * T4 * Vf - 24 * T0 * T2 * pow(T4, 2) * Vf - 6 * T0 * pow(T3, 2) * T4 * Vf - 4 * T0 * T3 * pow(T4, 2) * V0 - 8 * T0 * T3 * pow(T4, 2) * Vf - 12 * pow(T1, 2) * T3 * T4 * Vf - 12 * pow(T1, 2) * pow(T4, 2) * Vf - 48 * T1 * T2 * T3 * T4 * Vf - 48 * T1 * T2 * pow(T4, 2) * Vf - 12 * T1 * pow(T3, 2) * T4 * Vf - 2 * T1 * T3 * pow(T4, 2) * V0 - 16 * T1 * T3 * pow(T4, 2) * Vf - 36 * pow(T2, 2) * T3 * T4 * Vf - 36 * pow(T2, 2) * pow(T4, 2) * Vf - 18 * T2 * pow(T3, 2) * T4 * Vf - 24 * T2 * T3 * pow(T4, 2) * Vf) + 6 * pow(T4, 2) * d3 * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) - 6 * d4 * (2 * T0 * T1 * pow(T3, 2) + 3 * T0 * T1 * T3 * T4 + T0 * T1 * pow(T4, 2) + 4 * T0 * T2 * pow(T3, 2) + 6 * T0 * T2 * T3 * T4 + 2 * T0 * T2 * pow(T4, 2) + T0 * pow(T3, 3) + 2 * T0 * pow(T3, 2) * T4 + T0 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 3 * pow(T1, 2) * T3 * T4 + pow(T1, 2) * pow(T4, 2) + 8 * T1 * T2 * pow(T3, 2) + 12 * T1 * T2 * T3 * T4 + 4 * T1 * T2 * pow(T4, 2) + 2 * T1 * pow(T3, 3) + 4 * T1 * pow(T3, 2) * T4 + 2 * T1 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * pow(T3, 2) + 9 * pow(T2, 2) * T3 * T4 + 3 * pow(T2, 2) * pow(T4, 2) + 3 * T2 * pow(T3, 3) + 6 * T2 * pow(T3, 2) * T4 + 3 * T2 * T3 * pow(T4, 2))) / (T2 * pow(T3, 2) * pow(T4, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 3:
//     double jCP0_3 = 3 * (T3 * (-2 * Af * T3 * pow(T4, 2) - Af * pow(T4, 3) - 6 * Pf * T3 - 6 * Pf * T4 + 6 * T3 * T4 * Vf + 4 * pow(T4, 2) * Vf) - 2 * pow(T4, 2) * d3 + 2 * d4 * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2))) / (pow(T3, 3) * pow(T4, 2));
//     // Segment 4:
//     double jCP0_4 = 3 * (Af * pow(T4, 2) + 2 * Pf - 2 * T4 * Vf - 2 * d4) / pow(T4, 3);

//     // Add control points to the list
//     j_cp_.push_back({jCP0_0, jCP0_1, jCP0_2, jCP0_3, jCP0_4});
//   }
// }

// void SolverGurobi::computeControlPointsN6()
// {

//   p_cp_.clear(); // Clear the position control points
//   v_cp_.clear(); // Clear the velocity control points
//   a_cp_.clear(); // Clear the acceleration control points
//   j_cp_.clear(); // Clear the jerk control points

//   // Compute the control points for each axis

//   for (int axis = 0; axis < 3; axis++)
//   {

//     // POSITION CONTROL POINTS:
//     // Segment 0:
//     double CP0_0 = P0;
//     double CP1_0 = P0 + (1.0 / 3.0) * T0 * V0;
//     double CP2_0 = (1.0 / 6.0) * A0 * pow(T0, 2) + P0 + (2.0 / 3.0) * T0 * V0;
//     double CP3_0 = (1.0 / 6.0) * (pow(T0, 2) * (T3 * T4 * (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + 3 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (A0 * pow(T0, 2) + 2 * P0 + 2 * T0 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double CP0_1 = d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5 * (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 + pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 + 2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) + A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) + 12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 - 18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 - 12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 - 24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf + 8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T1 * T2 * T3 * T4 * pow(T5, 2));
//     double CP1_1 = (1.0 / 3.0) * T1 * (d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5 * (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 + 6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 + 6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 + 6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) - 24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 - 36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) - 48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 + 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5 * (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 + pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 + 2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) + A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) + 12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 - 18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 - 12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 - 24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf + 8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T1 * T2 * T3 * T4 * pow(T5, 2));
//     double CP2_1 = (1.0 / 3.0) * pow(T1, 2) * (d3 * (3 * T1 * T2 + 3 * T1 * T3 + 3 * pow(T2, 2) + 6 * T2 * T3 + 3 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-6 * T1 * T2 * pow(T3, 2) - 9 * T1 * T2 * T3 * T4 - 3 * T1 * T2 * pow(T4, 2) - 3 * T1 * pow(T3, 3) - 6 * T1 * pow(T3, 2) * T4 - 3 * T1 * T3 * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) - 9 * pow(T2, 2) * T3 * T4 - 3 * pow(T2, 2) * pow(T4, 2) - 6 * T2 * pow(T3, 3) - 12 * T2 * pow(T3, 2) * T4 - 6 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5 * (12 * T1 * T2 * T3 * pow(T4, 2) + 18 * T1 * T2 * T3 * T4 * T5 + 6 * T1 * T2 * T3 * pow(T5, 2) + 9 * T1 * T2 * pow(T4, 3) + 18 * T1 * T2 * pow(T4, 2) * T5 + 9 * T1 * T2 * T4 * pow(T5, 2) + 6 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T1 * pow(T3, 2) * T4 * T5 + 3 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T1 * T3 * pow(T4, 3) + 12 * T1 * T3 * pow(T4, 2) * T5 + 6 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T2 * pow(T3, 2) * T4 * T5 + 6 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T2 * T3 * pow(T4, 3) + 24 * T2 * T3 * pow(T4, 2) * T5 + 12 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2))) + (1.0 / 3.0) * T1 * (2 * d3 * (3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 3 * T0 * pow(T2, 2) + 6 * T0 * T2 * T3 + 3 * T0 * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + 2 * d4 * (-6 * T0 * T1 * T2 * pow(T3, 2) - 9 * T0 * T1 * T2 * T3 * T4 - 3 * T0 * T1 * T2 * pow(T4, 2) - 3 * T0 * T1 * pow(T3, 3) - 6 * T0 * T1 * pow(T3, 2) * T4 - 3 * T0 * T1 * T3 * pow(T4, 2) - 6 * T0 * pow(T2, 2) * pow(T3, 2) - 9 * T0 * pow(T2, 2) * T3 * T4 - 3 * T0 * pow(T2, 2) * pow(T4, 2) - 6 * T0 * T2 * pow(T3, 3) - 12 * T0 * T2 * pow(T3, 2) * T4 - 6 * T0 * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + 2 * d5 * (12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 + 6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * T0 * pow(T2, 2) * T3 * T4 * T5 + 6 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * T0 * pow(T2, 2) * pow(T4, 3) + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * T0 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T0 * T2 * pow(T3, 2) * T4 * T5 + 6 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T0 * T2 * T3 * pow(T4, 3) + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * T0 * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + 2 * (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) - 24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 - 36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) - 48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 + 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 4 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * T1 * T2 * T3 * T4 * pow(T5, 2))) + d3 * (pow(T0, 2) * T1 * T2 + pow(T0, 2) * T1 * T3 + pow(T0, 2) * pow(T2, 2) + 2 * pow(T0, 2) * T2 * T3 + pow(T0, 2) * pow(T3, 2)) / (pow(T0, 2) * pow(T3, 2) + 2 * T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + T1 * T2 * pow(T3, 2)) + d4 * (-2 * pow(T0, 2) * T1 * T2 * pow(T3, 2) - 3 * pow(T0, 2) * T1 * T2 * T3 * T4 - pow(T0, 2) * T1 * T2 * pow(T4, 2) - pow(T0, 2) * T1 * pow(T3, 3) - 2 * pow(T0, 2) * T1 * pow(T3, 2) * T4 - pow(T0, 2) * T1 * T3 * pow(T4, 2) - 2 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) - 3 * pow(T0, 2) * pow(T2, 2) * T3 * T4 - pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 2 * pow(T0, 2) * T2 * pow(T3, 3) - 4 * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 2 * pow(T0, 2) * T2 * T3 * pow(T4, 2)) / (pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + T1 * T2 * pow(T3, 2) * pow(T4, 2)) + d5 * (4 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T1 * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 + pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T1 * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T0, 2) * pow(T2, 2) * pow(T4, 3) + 6 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 4 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 6 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 + 2 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 4 * pow(T0, 2) * T2 * T3 * pow(T4, 3) + 8 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2)) / (pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2)) + (2 * A0 * pow(T0, 3) * T1 * T3 * T4 * pow(T5, 2) + A0 * pow(T0, 3) * T2 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T0, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 3) + 12 * P0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * P0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * P0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Pf * pow(T0, 2) * T1 * T2 * T3 * T4 - 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * T5 - 18 * Pf * pow(T0, 2) * T1 * T2 * pow(T4, 2) - 36 * Pf * pow(T0, 2) * T1 * T2 * T4 * T5 - 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T4 - 18 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * T5 - 12 * Pf * pow(T0, 2) * T1 * T3 * pow(T4, 2) - 24 * Pf * pow(T0, 2) * T1 * T3 * T4 * T5 - 24 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T0, 2) * pow(T2, 2) * T4 * T5 - 24 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T4 - 36 * Pf * pow(T0, 2) * T2 * pow(T3, 2) * T5 - 24 * Pf * pow(T0, 2) * T2 * T3 * pow(T4, 2) - 48 * Pf * pow(T0, 2) * T2 * T3 * T4 * T5 + 24 * pow(T0, 2) * T1 * T2 * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * T5 * Vf + 8 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T0, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T0, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T0, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 6 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0) / (6 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 6 * T1 * T2 * T3 * T4 * pow(T5, 2));
//     double CP3_1 = (1.0 / 6.0) * (A0 * pow(T0, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 3) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 6 * P0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) - 54 * Pf * T0 * T1 * T2 * pow(T3, 3) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) - 72 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 - 24 * Pf * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * pow(T3, 2) * T4 * T5 - 18 * Pf * T0 * pow(T2, 3) * T3 * pow(T4, 3) - 36 * Pf * T0 * pow(T2, 3) * T3 * pow(T4, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 48 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 3) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) - 72 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 - 48 * Pf * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * pow(T3, 2) * T4 * T5 - 36 * Pf * T1 * pow(T2, 3) * T3 * pow(T4, 3) - 72 * Pf * T1 * pow(T2, 3) * T3 * pow(T4, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 96 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 + 48 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 + 48 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 72 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 - 24 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 + 24 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 + 36 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf + 36 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * d5 + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 72 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 - 36 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 + 36 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 - 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 + 36 * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * d5 + 36 * T0 * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 54 * T0 * T1 * T2 * pow(T3, 3) * T4 * T5 * d5 - 18 * T0 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d4 + 18 * T0 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d5 + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * d5 + 48 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 72 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 18 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 18 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 24 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 + 24 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 - 12 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 + 12 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 + 18 * T0 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf + 18 * T0 * pow(T2, 3) * T3 * pow(T4, 3) * d5 + 24 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 36 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 - 18 * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 + 18 * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 + 6 * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 - 6 * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 + 24 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 24 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 36 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 - 12 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 12 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 48 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 12 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 + 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 72 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 - 24 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 + 24 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 + 36 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf + 36 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * d5 + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 72 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 - 36 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 + 36 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 - 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 + 36 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * d5 + 36 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 54 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * T5 * d5 - 18 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T5, 2) * d4 + 18 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T5, 2) * d5 + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * d5 + 48 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 72 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 18 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 18 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 48 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 + 48 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 72 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 - 24 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 + 24 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 + 36 * T1 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf + 36 * T1 * pow(T2, 3) * T3 * pow(T4, 3) * d5 + 48 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 72 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 - 36 * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 + 36 * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 + 12 * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 - 12 * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 + 48 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 48 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 72 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 - 24 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 24 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 96 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 24 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 24 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double CP0_2 = d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5 * (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 + 2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) + 6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) + 6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 + 6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 - 54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) - 72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 - 36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 - 72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
//     double CP1_2 = (1.0 / 3.0) * T2 * (d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5 * (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 + 6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) + 18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) + 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 + 18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 + 12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 + 24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 + 18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 - 24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 - 18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5 * (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 + 2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) + 6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) + 6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 + 6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 - 54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) - 72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 - 36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 - 72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
//     double CP2_2 = (1.0 / 3.0) * pow(T2, 2) * (d3 * (-3 * T0 * T2 - 3 * T0 * T3 - 6 * T1 * T2 - 6 * T1 * T3 - 6 * pow(T2, 2) - 9 * T2 * T3 - 3 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (6 * T0 * T2 * pow(T3, 2) + 9 * T0 * T2 * T3 * T4 + 3 * T0 * T2 * pow(T4, 2) + 3 * T0 * pow(T3, 3) + 6 * T0 * pow(T3, 2) * T4 + 3 * T0 * T3 * pow(T4, 2) + 12 * T1 * T2 * pow(T3, 2) + 18 * T1 * T2 * T3 * T4 + 6 * T1 * T2 * pow(T4, 2) + 6 * T1 * pow(T3, 3) + 12 * T1 * pow(T3, 2) * T4 + 6 * T1 * T3 * pow(T4, 2) + 12 * pow(T2, 2) * pow(T3, 2) + 18 * pow(T2, 2) * T3 * T4 + 6 * pow(T2, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 3) + 18 * T2 * pow(T3, 2) * T4 + 9 * T2 * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5 * (-12 * T0 * T2 * T3 * pow(T4, 2) - 18 * T0 * T2 * T3 * T4 * T5 - 6 * T0 * T2 * T3 * pow(T5, 2) - 9 * T0 * T2 * pow(T4, 3) - 18 * T0 * T2 * pow(T4, 2) * T5 - 9 * T0 * T2 * T4 * pow(T5, 2) - 6 * T0 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * pow(T3, 2) * T4 * T5 - 3 * T0 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T3 * pow(T4, 3) - 12 * T0 * T3 * pow(T4, 2) * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) - 24 * T1 * T2 * T3 * pow(T4, 2) - 36 * T1 * T2 * T3 * T4 * T5 - 12 * T1 * T2 * T3 * pow(T5, 2) - 18 * T1 * T2 * pow(T4, 3) - 36 * T1 * T2 * pow(T4, 2) * T5 - 18 * T1 * T2 * T4 * pow(T5, 2) - 12 * T1 * pow(T3, 2) * pow(T4, 2) - 18 * T1 * pow(T3, 2) * T4 * T5 - 6 * T1 * pow(T3, 2) * pow(T5, 2) - 12 * T1 * T3 * pow(T4, 3) - 24 * T1 * T3 * pow(T4, 2) * T5 - 12 * T1 * T3 * T4 * pow(T5, 2) - 24 * pow(T2, 2) * T3 * pow(T4, 2) - 36 * pow(T2, 2) * T3 * T4 * T5 - 12 * pow(T2, 2) * T3 * pow(T5, 2) - 18 * pow(T2, 2) * pow(T4, 3) - 36 * pow(T2, 2) * pow(T4, 2) * T5 - 18 * pow(T2, 2) * T4 * pow(T5, 2) - 18 * T2 * pow(T3, 2) * pow(T4, 2) - 27 * T2 * pow(T3, 2) * T4 * T5 - 9 * T2 * pow(T3, 2) * pow(T5, 2) - 18 * T2 * T3 * pow(T4, 3) - 36 * T2 * T3 * pow(T4, 2) * T5 - 18 * T2 * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) + 16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) + 12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) + 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) + 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) + 6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 + 18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 + 12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 + 12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 + 48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 + 36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 + 24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 + 24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 + 48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 + 36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 + 36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 + 36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 - 24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 - 16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf - 48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf - 48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf - 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2))) + (1.0 / 3.0) * T2 * (2 * d3 * (-3 * T0 * T1 * T2 - 3 * T0 * T1 * T3 - 3 * pow(T1, 2) * T2 - 3 * pow(T1, 2) * T3 + 3 * pow(T2, 3) + 6 * pow(T2, 2) * T3 + 3 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + 2 * d4 * (6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * pow(T3, 2) - 9 * pow(T2, 3) * T3 * T4 - 3 * pow(T2, 3) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 3) - 12 * pow(T2, 2) * pow(T3, 2) * T4 - 6 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + 2 * d5 * (-12 * T0 * T1 * T2 * T3 * pow(T4, 2) - 18 * T0 * T1 * T2 * T3 * T4 * T5 - 6 * T0 * T1 * T2 * T3 * pow(T5, 2) - 9 * T0 * T1 * T2 * pow(T4, 3) - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 - 9 * T0 * T1 * T2 * T4 * pow(T5, 2) - 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) - 9 * T0 * T1 * pow(T3, 2) * T4 * T5 - 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) - 6 * T0 * T1 * T3 * pow(T4, 3) - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 - 6 * T0 * T1 * T3 * T4 * pow(T5, 2) - 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 18 * pow(T1, 2) * T2 * T3 * T4 * T5 - 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) - 9 * pow(T1, 2) * T2 * pow(T4, 3) - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 - 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) - 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) - 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 - 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) - 6 * pow(T1, 2) * T3 * pow(T4, 3) - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 - 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 3) * T3 * pow(T4, 2) + 18 * pow(T2, 3) * T3 * T4 * T5 + 6 * pow(T2, 3) * T3 * pow(T5, 2) + 9 * pow(T2, 3) * pow(T4, 3) + 18 * pow(T2, 3) * pow(T4, 2) * T5 + 9 * pow(T2, 3) * T4 * pow(T5, 2) + 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 18 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 6 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 3) + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 12 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + 2 * (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 + 18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 + 12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 + 24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 + 18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 - 24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 - 18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 2 * T0 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 4 * T1 * T2 * T3 * T4 * pow(T5, 2) + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2))) + d3 * (2 * T0 * T1 * pow(T2, 2) + 3 * T0 * T1 * T2 * T3 + T0 * T1 * pow(T3, 2) + T0 * pow(T2, 3) + 2 * T0 * pow(T2, 2) * T3 + T0 * T2 * pow(T3, 2) + 2 * pow(T1, 2) * pow(T2, 2) + 3 * pow(T1, 2) * T2 * T3 + pow(T1, 2) * pow(T3, 2) + 2 * T1 * pow(T2, 3) + 4 * T1 * pow(T2, 2) * T3 + 2 * T1 * T2 * pow(T3, 2)) / (T0 * T1 * pow(T3, 2) + T0 * T2 * pow(T3, 2) + pow(T1, 2) * pow(T3, 2) + 2 * T1 * T2 * pow(T3, 2) + pow(T2, 2) * pow(T3, 2)) + d4 * (-4 * T0 * T1 * pow(T2, 2) * pow(T3, 2) - 6 * T0 * T1 * pow(T2, 2) * T3 * T4 - 2 * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 3 * T0 * T1 * T2 * pow(T3, 3) - 6 * T0 * T1 * T2 * pow(T3, 2) * T4 - 3 * T0 * T1 * T2 * T3 * pow(T4, 2) - 2 * T0 * pow(T2, 3) * pow(T3, 2) - 3 * T0 * pow(T2, 3) * T3 * T4 - T0 * pow(T2, 3) * pow(T4, 2) - 2 * T0 * pow(T2, 2) * pow(T3, 3) - 4 * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 4 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) - 6 * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 2 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 3 * pow(T1, 2) * T2 * pow(T3, 3) - 6 * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 3 * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 4 * T1 * pow(T2, 3) * pow(T3, 2) - 6 * T1 * pow(T2, 3) * T3 * T4 - 2 * T1 * pow(T2, 3) * pow(T4, 2) - 4 * T1 * pow(T2, 2) * pow(T3, 3) - 8 * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 4 * T1 * pow(T2, 2) * T3 * pow(T4, 2)) / (T0 * T1 * pow(T3, 2) * pow(T4, 2) + T0 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + pow(T2, 2) * pow(T3, 2) * pow(T4, 2)) + d5 * (8 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * T1 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T2 * T3 * pow(T4, 3) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * T0 * pow(T2, 3) * T3 * T4 * T5 + 2 * T0 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * T0 * pow(T2, 3) * pow(T4, 3) + 6 * T0 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * T0 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T2, 2) * pow(T4, 3) + 12 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 3) * T3 * pow(T4, 2) + 12 * T1 * pow(T2, 3) * T3 * T4 * T5 + 4 * T1 * pow(T2, 3) * T3 * pow(T5, 2) + 6 * T1 * pow(T2, 3) * pow(T4, 3) + 12 * T1 * pow(T2, 3) * pow(T4, 2) * T5 + 6 * T1 * pow(T2, 3) * T4 * pow(T5, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 12 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 4 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 8 * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 8 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) / (T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 2 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2)) + (A0 * pow(T0, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 16 * Af * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 16 * Af * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 3) - 12 * Af * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T3 * pow(T5, 3) - 12 * Af * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 12 * Af * T1 * pow(T2, 3) * T4 * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 12 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 16 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 16 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 48 * Pf * T0 * T1 * pow(T2, 2) * T3 * T4 - 72 * Pf * T0 * T1 * pow(T2, 2) * T3 * T5 - 36 * Pf * T0 * T1 * pow(T2, 2) * pow(T4, 2) - 72 * Pf * T0 * T1 * pow(T2, 2) * T4 * T5 - 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 - 54 * Pf * T0 * T1 * T2 * pow(T3, 2) * T5 - 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) - 72 * Pf * T0 * T1 * T2 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 3) * T3 * T4 - 36 * Pf * T0 * pow(T2, 3) * T3 * T5 - 18 * Pf * T0 * pow(T2, 3) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 3) * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * T0 * pow(T2, 2) * T3 * T4 * T5 - 48 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T4 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * T5 - 36 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T4 * T5 - 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 - 54 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T5 - 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) - 72 * Pf * pow(T1, 2) * T2 * T3 * T4 * T5 - 48 * Pf * T1 * pow(T2, 3) * T3 * T4 - 72 * Pf * T1 * pow(T2, 3) * T3 * T5 - 36 * Pf * T1 * pow(T2, 3) * pow(T4, 2) - 72 * Pf * T1 * pow(T2, 3) * T4 * T5 - 48 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T5 - 48 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) - 96 * Pf * T1 * pow(T2, 2) * T3 * T4 * T5 + 48 * T0 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 32 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 3) * T3 * T4 * T5 * Vf + 48 * T1 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 36 * T1 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 48 * T1 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 48 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 + 64 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) / (6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * pow(T2, 2) * T3 * T4 * pow(T5, 2));
//     double CP3_2 = d3;
//     // Segment 3:
//     double CP0_3 = d3;
//     double CP1_3 = (1.0 / 3.0) * T3 * ((1.0 / 2.0) * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + d5 * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) - 12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2)) - 3 * d3 / T3 + d4 * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2))) + d3;
//     double CP2_3 = (1.0 / 3.0) * pow(T3, 2) * ((-4 * Af * T3 * T4 * pow(T5, 2) - 3 * Af * T3 * pow(T5, 3) - 3 * Af * pow(T4, 2) * pow(T5, 2) - 3 * Af * T4 * pow(T5, 3) - 12 * Pf * T3 * T4 - 18 * Pf * T3 * T5 - 9 * Pf * pow(T4, 2) - 18 * Pf * T4 * T5 + 12 * T3 * T4 * T5 * Vf + 12 * T3 * pow(T5, 2) * Vf + 9 * pow(T4, 2) * T5 * Vf + 12 * T4 * pow(T5, 2) * Vf) / (T3 * T4 * pow(T5, 2)) + d5 * (12 * T3 * pow(T4, 2) + 18 * T3 * T4 * T5 + 6 * T3 * pow(T5, 2) + 9 * pow(T4, 3) + 18 * pow(T4, 2) * T5 + 9 * T4 * pow(T5, 2)) / (T3 * pow(T4, 2) * pow(T5, 2)) + 3 * d3 / pow(T3, 2) + d4 * (-6 * pow(T3, 2) - 9 * T3 * T4 - 3 * pow(T4, 2)) / (pow(T3, 2) * pow(T4, 2))) + (1.0 / 3.0) * T3 * ((4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 4 * Af * pow(T4, 2) * pow(T5, 2) + 4 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 12 * Pf * pow(T4, 2) + 24 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 12 * pow(T4, 2) * T5 * Vf - 16 * T4 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + 2 * d5 * (-6 * T3 * pow(T4, 2) - 9 * T3 * T4 * T5 - 3 * T3 * pow(T5, 2) - 6 * pow(T4, 3) - 12 * pow(T4, 2) * T5 - 6 * T4 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2)) - 6 * d3 / T3 + 2 * d4 * (3 * pow(T3, 2) + 6 * T3 * T4 + 3 * pow(T4, 2)) / (T3 * pow(T4, 2))) + d3;
//     double CP3_3 = d4;
//     // Segment 4:
//     double CP0_4 = d4;
//     double CP1_4 = (1.0 / 3.0) * T4 * ((-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 + 3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / pow(T5, 2) - 3 * d4 / T4 + d5 * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2))) + d4;
//     double CP2_4 = (1.0 / 3.0) * pow(T4, 2) * ((1.0 / 2.0) * (4 * Af * T4 * pow(T5, 2) + 3 * Af * pow(T5, 3) + 12 * Pf * T4 + 18 * Pf * T5 - 12 * T4 * T5 * Vf - 12 * pow(T5, 2) * Vf) / (T4 * pow(T5, 2)) + 3 * d4 / pow(T4, 2) + d5 * (-6 * pow(T4, 2) - 9 * T4 * T5 - 3 * pow(T5, 2)) / (pow(T4, 2) * pow(T5, 2))) + (1.0 / 3.0) * T4 * (2 * (-Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 3 * Pf * T4 - 6 * Pf * T5 + 3 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) / pow(T5, 2) - 6 * d4 / T4 + 2 * d5 * (3 * pow(T4, 2) + 6 * T4 * T5 + 3 * pow(T5, 2)) / (T4 * pow(T5, 2))) + d4;
//     double CP3_4 = d5;
//     // Segment 5:
//     double CP0_5 = d5;
//     double CP1_5 = (1.0 / 3.0) * T5 * (-3 * d5 / T5 + (1.0 / 2.0) * (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5) + d5;
//     double CP2_5 = (1.0 / 3.0) * pow(T5, 2) * (3 * d5 / pow(T5, 2) + (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf) / pow(T5, 2)) + (1.0 / 3.0) * T5 * (-6 * d5 / T5 + (Af * pow(T5, 2) + 6 * Pf - 4 * T5 * Vf) / T5) + d5;
//     double CP3_5 = Pf;

//     // Add control points to the list
//     p_cp_.push_back({CP0_0, CP1_0, CP2_0, CP3_0, CP0_1, CP1_1, CP2_1, CP3_1, CP0_2, CP1_2, CP2_2, CP3_2, CP0_3, CP1_3, CP2_3, CP3_3, CP0_4, CP1_4, CP2_4, CP3_4, CP0_5, CP1_5, CP2_5, CP3_5});

//     // VELOCITY CONTROL POINTS:
//     // Segment 0:
//     double vCP0_0 = V0;
//     double vCP1_0 = (1.0 / 8.0) * (T0 * (T3 * T4 * (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + 4 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (A0 * T0 + 2 * V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP2_0 = ((1.0 / 2.0) * T0 * (T3 * T4 * (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) + pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (A0 * T0 + V0) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double vCP0_1 = (1.0 / 2.0) * (6 * T0 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * T0 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * T0 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2)) + T3 * T4 * (-A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T2 * T3 * T4 - 36 * Pf * T0 * T1 * T2 * T3 * T5 - 18 * Pf * T0 * T1 * T2 * pow(T4, 2) - 36 * Pf * T0 * T1 * T2 * T4 * T5 - 12 * Pf * T0 * T1 * pow(T3, 2) * T4 - 18 * Pf * T0 * T1 * pow(T3, 2) * T5 - 12 * Pf * T0 * T1 * T3 * pow(T4, 2) - 24 * Pf * T0 * T1 * T3 * T4 * T5 - 24 * Pf * T0 * pow(T2, 2) * T3 * T4 - 36 * Pf * T0 * pow(T2, 2) * T3 * T5 - 18 * Pf * T0 * pow(T2, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T2, 2) * T4 * T5 - 24 * Pf * T0 * T2 * pow(T3, 2) * T4 - 36 * Pf * T0 * T2 * pow(T3, 2) * T5 - 24 * Pf * T0 * T2 * T3 * pow(T4, 2) - 48 * Pf * T0 * T2 * T3 * T4 * T5 - 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 + 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 32 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 + 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0)) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double vCP1_1 = (1.0 / 8.0) * (-3 * A0 * pow(T0, 3) * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 4 * A0 * pow(T0, 3) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 4 * A0 * pow(T0, 2) * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 6 * A0 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - A0 * T0 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - A0 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + A0 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 8 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T0, 2) * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 6 * Af * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 4 * Af * pow(T0, 2) * T1 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 3 * Af * pow(T0, 2) * T1 * pow(T3, 3) * T4 * pow(T5, 3) + 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 4 * Af * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 8 * Af * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 3) - 6 * Af * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * pow(T5, 2) - 6 * Af * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 3) - 4 * Af * T0 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 3 * Af * T0 * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 4 * Af * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 4 * Af * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 48 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 36 * Af * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) - 36 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) - 36 * Af * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) - 36 * Af * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 27 * Af * T0 * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 3) - 36 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 36 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 32 * Af * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 24 * Af * T0 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) - 24 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) - 24 * Af * T0 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) - 32 * Af * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 24 * Af * T0 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 32 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 32 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 8 * Af * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 3) * T2 * pow(T3, 2) * T4 * pow(T5, 3) - 6 * Af * pow(T1, 3) * T2 * T3 * pow(T4, 3) * pow(T5, 2) - 6 * Af * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 3) - 4 * Af * pow(T1, 3) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 3 * Af * pow(T1, 3) * pow(T3, 3) * T4 * pow(T5, 3) - 4 * Af * pow(T1, 3) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 4 * Af * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 32 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 24 * Af * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) - 24 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) - 24 * Af * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) - 24 * Af * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T1, 2) * T2 * pow(T3, 3) * T4 * pow(T5, 3) - 24 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 24 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 24 * Af * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * T1 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) - 18 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) - 18 * Af * T1 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) - 24 * Af * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 18 * Af * T1 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 24 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 24 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 18 * P0 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 24 * P0 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 12 * P0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 18 * P0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Pf * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T1 * T2 * pow(T3, 2) * T4 * T5 + 18 * Pf * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 3) + 36 * Pf * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * T5 + 12 * Pf * pow(T0, 2) * T1 * pow(T3, 3) * pow(T4, 2) + 18 * Pf * pow(T0, 2) * T1 * pow(T3, 3) * T4 * T5 + 12 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 3) + 24 * Pf * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * T5 - 24 * Pf * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) - 36 * Pf * T0 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 - 18 * Pf * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 3) - 36 * Pf * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 - 12 * Pf * T0 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) - 18 * Pf * T0 * pow(T1, 2) * pow(T3, 3) * T4 * T5 - 12 * Pf * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) - 24 * Pf * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 144 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 216 * Pf * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 108 * Pf * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) - 216 * Pf * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 108 * Pf * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) - 162 * Pf * T0 * T1 * T2 * pow(T3, 3) * T4 * T5 - 108 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) - 216 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 - 96 * Pf * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 144 * Pf * T0 * pow(T2, 3) * pow(T3, 2) * T4 * T5 - 72 * Pf * T0 * pow(T2, 3) * T3 * pow(T4, 3) - 144 * Pf * T0 * pow(T2, 3) * T3 * pow(T4, 2) * T5 - 96 * Pf * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 144 * Pf * T0 * pow(T2, 2) * pow(T3, 3) * T4 * T5 - 96 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 192 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 24 * Pf * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 3) * T2 * pow(T3, 2) * T4 * T5 - 18 * Pf * pow(T1, 3) * T2 * T3 * pow(T4, 3) - 36 * Pf * pow(T1, 3) * T2 * T3 * pow(T4, 2) * T5 - 12 * Pf * pow(T1, 3) * pow(T3, 3) * pow(T4, 2) - 18 * Pf * pow(T1, 3) * pow(T3, 3) * T4 * T5 - 12 * Pf * pow(T1, 3) * pow(T3, 2) * pow(T4, 3) - 24 * Pf * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * T5 - 96 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 144 * Pf * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 72 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) - 144 * Pf * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 72 * Pf * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) - 108 * Pf * pow(T1, 2) * T2 * pow(T3, 3) * T4 * T5 - 72 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) - 144 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 - 72 * Pf * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 108 * Pf * T1 * pow(T2, 3) * pow(T3, 2) * T4 * T5 - 54 * Pf * T1 * pow(T2, 3) * T3 * pow(T4, 3) - 108 * Pf * T1 * pow(T2, 3) * T3 * pow(T4, 2) * T5 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 108 * Pf * T1 * pow(T2, 2) * pow(T3, 3) * T4 * T5 - 72 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 144 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 24 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 24 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 36 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * T4 * T5 * d5 + 12 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 12 * pow(T0, 2) * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 18 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 3) * T5 * Vf - 18 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 3) * d5 - 24 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 36 * pow(T0, 2) * T1 * T2 * T3 * pow(T4, 2) * T5 * d5 + 18 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) * d4 - 18 * pow(T0, 2) * T1 * T2 * T3 * T4 * pow(T5, 2) * d5 - 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T0, 2) * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 12 * pow(T0, 2) * T1 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 12 * pow(T0, 2) * T1 * pow(T3, 3) * pow(T4, 2) * d5 - 12 * pow(T0, 2) * T1 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 18 * pow(T0, 2) * T1 * pow(T3, 3) * T4 * T5 * d5 + 6 * pow(T0, 2) * T1 * pow(T3, 3) * pow(T5, 2) * d4 - 6 * pow(T0, 2) * T1 * pow(T3, 3) * pow(T5, 2) * d5 - 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 3) * d5 - 12 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 16 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 24 * pow(T0, 2) * T1 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 12 * pow(T0, 2) * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 6 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T0, 2) * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 16 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 24 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 24 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * d5 + 24 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 36 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * d5 - 12 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d4 + 12 * T0 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d5 + 18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * T5 * Vf + 18 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * d5 + 24 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 36 * T0 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * d5 - 18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d4 + 18 * T0 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d5 + 6 * T0 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d3 - 6 * T0 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d4 + 12 * T0 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 12 * T0 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 12 * T0 * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 18 * T0 * pow(T1, 2) * pow(T3, 3) * T4 * T5 * d5 - 6 * T0 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 6 * T0 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 12 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 12 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * d5 - 12 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 16 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 24 * T0 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 12 * T0 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 12 * T0 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 6 * T0 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 144 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 144 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 + 144 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 216 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 - 72 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 + 72 * T0 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 + 108 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf + 108 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * d5 + 144 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 216 * T0 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 - 108 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 + 108 * T0 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 + 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 - 36 * T0 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 + 108 * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 108 * T0 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * d5 + 108 * T0 * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 162 * T0 * T1 * T2 * pow(T3, 3) * T4 * T5 * d5 - 54 * T0 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d4 + 54 * T0 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d5 + 108 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 108 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * d5 - 18 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 144 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 216 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 108 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 108 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 54 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 54 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 18 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 96 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 96 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 + 96 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 144 * T0 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 - 48 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 + 48 * T0 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 + 72 * T0 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf + 72 * T0 * pow(T2, 3) * T3 * pow(T4, 3) * d5 + 96 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 144 * T0 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 - 72 * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 + 72 * T0 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 + 24 * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 - 24 * T0 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 + 96 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 96 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 96 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 144 * T0 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 - 48 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 48 * T0 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 96 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 96 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 + 128 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 192 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 96 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 96 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 48 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 48 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 24 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 24 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T4, 2) * d5 + 24 * pow(T1, 3) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T1, 3) * T2 * pow(T3, 2) * T4 * T5 * d5 - 12 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T5, 2) * d4 + 12 * pow(T1, 3) * T2 * pow(T3, 2) * pow(T5, 2) * d5 + 18 * pow(T1, 3) * T2 * T3 * pow(T4, 3) * T5 * Vf + 18 * pow(T1, 3) * T2 * T3 * pow(T4, 3) * d5 + 24 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 36 * pow(T1, 3) * T2 * T3 * pow(T4, 2) * T5 * d5 - 18 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) * d4 + 18 * pow(T1, 3) * T2 * T3 * T4 * pow(T5, 2) * d5 + 6 * pow(T1, 3) * T2 * pow(T4, 2) * pow(T5, 2) * d3 - 6 * pow(T1, 3) * T2 * pow(T4, 2) * pow(T5, 2) * d4 + 12 * pow(T1, 3) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 12 * pow(T1, 3) * pow(T3, 3) * pow(T4, 2) * d5 + 12 * pow(T1, 3) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 18 * pow(T1, 3) * pow(T3, 3) * T4 * T5 * d5 - 6 * pow(T1, 3) * pow(T3, 3) * pow(T5, 2) * d4 + 6 * pow(T1, 3) * pow(T3, 3) * pow(T5, 2) * d5 + 12 * pow(T1, 3) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 12 * pow(T1, 3) * pow(T3, 2) * pow(T4, 3) * d5 - 2 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 16 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 24 * pow(T1, 3) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 12 * pow(T1, 3) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 12 * pow(T1, 3) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 6 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 6 * pow(T1, 3) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 96 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 96 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 + 96 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 144 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 - 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 + 48 * pow(T1, 2) * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 + 72 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf + 72 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 3) * d5 + 96 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 144 * pow(T1, 2) * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 - 72 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 + 72 * pow(T1, 2) * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 + 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 - 24 * pow(T1, 2) * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 + 72 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 72 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T4, 2) * d5 + 72 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 108 * pow(T1, 2) * T2 * pow(T3, 3) * T4 * T5 * d5 - 36 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T5, 2) * d4 + 36 * pow(T1, 2) * T2 * pow(T3, 3) * pow(T5, 2) * d5 + 72 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 72 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 3) * d5 - 2 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 96 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 144 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 72 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 72 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 12 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 72 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 72 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 + 72 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 108 * T1 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 - 36 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 + 36 * T1 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 + 54 * T1 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf + 54 * T1 * pow(T2, 3) * T3 * pow(T4, 3) * d5 + 72 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 108 * T1 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 - 54 * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 + 54 * T1 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 + 18 * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 - 18 * T1 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 + 72 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 72 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 72 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 108 * T1 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 - 36 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 36 * T1 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 72 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 72 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 + 2 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 96 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 144 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 72 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 72 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 36 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 36 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 18 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     double vCP2_1 = (1.0 / 2.0) * (-A0 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 3) * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 4 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 4 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 8 * Af * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) - 6 * Af * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) - 8 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 6 * P0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 18 * Pf * T0 * T1 * T2 * T3 * pow(T4, 3) + 36 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 12 * Pf * T0 * T1 * pow(T3, 3) * pow(T4, 2) + 18 * Pf * T0 * T1 * pow(T3, 3) * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * pow(T4, 3) + 24 * Pf * T0 * T1 * pow(T3, 2) * pow(T4, 2) * T5 + 24 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 18 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 36 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) + 18 * Pf * pow(T1, 2) * pow(T3, 3) * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) + 24 * Pf * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 24 * Pf * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * pow(T3, 2) * T4 * T5 - 18 * Pf * pow(T2, 3) * T3 * pow(T4, 3) - 36 * Pf * pow(T2, 3) * T3 * pow(T4, 2) * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * pow(T3, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) - 48 * Pf * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 24 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 24 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * d5 + 12 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 12 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 18 * T0 * T1 * T2 * T3 * pow(T4, 3) * T5 * Vf - 18 * T0 * T1 * T2 * T3 * pow(T4, 3) * d5 - 24 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * d5 + 18 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * d4 - 18 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * d5 - 6 * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 12 * T0 * T1 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 12 * T0 * T1 * pow(T3, 3) * pow(T4, 2) * d5 - 12 * T0 * T1 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 18 * T0 * T1 * pow(T3, 3) * T4 * T5 * d5 + 6 * T0 * T1 * pow(T3, 3) * pow(T5, 2) * d4 - 6 * T0 * T1 * pow(T3, 3) * pow(T5, 2) * d5 - 12 * T0 * T1 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T4, 3) * d5 - 16 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 24 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 12 * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 12 * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 6 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 4 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 24 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * d5 + 12 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 12 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 18 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * T5 * Vf - 18 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * d5 - 24 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 36 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * d5 + 18 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d4 - 18 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d5 - 6 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 12 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * d5 - 12 * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * pow(T3, 3) * T4 * T5 * d5 + 6 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d4 - 6 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d5 - 12 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * d5 - 16 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 24 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 12 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 12 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 6 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 + 24 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 + 24 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf + 36 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 - 12 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 + 12 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 + 18 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf + 18 * pow(T2, 3) * T3 * pow(T4, 3) * d5 + 24 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf + 36 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 - 18 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 + 18 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 + 6 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 - 6 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 + 24 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 + 24 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf + 36 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 - 12 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 + 12 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 + 24 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 + 32 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf + 48 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 - 24 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 + 24 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 + 12 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 - 12 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 6 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double vCP0_2 = (1.0 / 2.0) * (T3 * T4 * (-A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) - 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T1 * T2 * T3 * T4 + 36 * Pf * T0 * T1 * T2 * T3 * T5 + 18 * Pf * T0 * T1 * T2 * pow(T4, 2) + 36 * Pf * T0 * T1 * T2 * T4 * T5 + 12 * Pf * T0 * T1 * pow(T3, 2) * T4 + 18 * Pf * T0 * T1 * pow(T3, 2) * T5 + 12 * Pf * T0 * T1 * T3 * pow(T4, 2) + 24 * Pf * T0 * T1 * T3 * T4 * T5 + 24 * Pf * pow(T1, 2) * T2 * T3 * T4 + 36 * Pf * pow(T1, 2) * T2 * T3 * T5 + 18 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T1, 2) * T2 * T4 * T5 + 12 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T1, 2) * T3 * T4 * T5 - 24 * Pf * pow(T2, 3) * T3 * T4 - 36 * Pf * pow(T2, 3) * T3 * T5 - 18 * Pf * pow(T2, 3) * pow(T4, 2) - 36 * Pf * pow(T2, 3) * T4 * T5 - 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 - 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 - 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) - 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf - 16 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 4 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 24 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf - 16 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 2 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 + 24 * pow(T2, 3) * T3 * T4 * T5 * Vf + 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) - 6 * T3 * d5 * (4 * T0 * T1 * T2 * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * T3 * T4 * T5 + 2 * T0 * T1 * T2 * T3 * pow(T5, 2) + 3 * T0 * T1 * T2 * pow(T4, 3) + 6 * T0 * T1 * T2 * pow(T4, 2) * T5 + 3 * T0 * T1 * T2 * T4 * pow(T5, 2) + 2 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 2) * T4 * T5 + T0 * T1 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T1 * T3 * pow(T4, 3) + 4 * T0 * T1 * T3 * pow(T4, 2) * T5 + 2 * T0 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * T3 * T4 * T5 + 2 * pow(T1, 2) * T2 * T3 * pow(T5, 2) + 3 * pow(T1, 2) * T2 * pow(T4, 3) + 6 * pow(T1, 2) * T2 * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T2 * T4 * pow(T5, 2) + 2 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 2) * T4 * T5 + pow(T1, 2) * pow(T3, 2) * pow(T5, 2) + 2 * pow(T1, 2) * T3 * pow(T4, 3) + 4 * pow(T1, 2) * T3 * pow(T4, 2) * T5 + 2 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 4 * pow(T2, 3) * T3 * pow(T4, 2) - 6 * pow(T2, 3) * T3 * T4 * T5 - 2 * pow(T2, 3) * T3 * pow(T5, 2) - 3 * pow(T2, 3) * pow(T4, 3) - 6 * pow(T2, 3) * pow(T4, 2) * T5 - 3 * pow(T2, 3) * T4 * pow(T5, 2) - 4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) - 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 - 2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) - 4 * pow(T2, 2) * T3 * pow(T4, 3) - 8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 - 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) - 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T0 * T1 * T2 + T0 * T1 * T3 + pow(T1, 2) * T2 + pow(T1, 2) * T3 - pow(T2, 3) - 2 * pow(T2, 2) * T3 - T2 * pow(T3, 2)) + 6 * pow(T5, 2) * d4 * (2 * T0 * T1 * T2 * pow(T3, 2) + 3 * T0 * T1 * T2 * T3 * T4 + T0 * T1 * T2 * pow(T4, 2) + T0 * T1 * pow(T3, 3) + 2 * T0 * T1 * pow(T3, 2) * T4 + T0 * T1 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * T2 * pow(T3, 2) + 3 * pow(T1, 2) * T2 * T3 * T4 + pow(T1, 2) * T2 * pow(T4, 2) + pow(T1, 2) * pow(T3, 3) + 2 * pow(T1, 2) * pow(T3, 2) * T4 + pow(T1, 2) * T3 * pow(T4, 2) - 2 * pow(T2, 3) * pow(T3, 2) - 3 * pow(T2, 3) * T3 * T4 - pow(T2, 3) * pow(T4, 2) - 2 * pow(T2, 2) * pow(T3, 3) - 4 * pow(T2, 2) * pow(T3, 2) * T4 - 2 * pow(T2, 2) * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP1_2 = (1.0 / 8.0) * (-A0 * pow(T0, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) - A0 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Af * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 18 * Af * T0 * T1 * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 16 * Af * T0 * T1 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T1 * pow(T3, 3) * T4 * pow(T5, 3) + 16 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 16 * Af * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 16 * Af * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) + 12 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) + 12 * Af * T0 * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 9 * Af * T0 * T2 * pow(T3, 3) * T4 * pow(T5, 3) + 12 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 12 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 24 * Af * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 18 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 16 * Af * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 3) + 16 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 16 * Af * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 32 * Af * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) + 24 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) + 24 * Af * T1 * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 18 * Af * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 3) + 24 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 24 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 8 * Af * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 3) + 6 * Af * pow(T2, 3) * T3 * pow(T4, 3) * pow(T5, 2) + 6 * Af * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 3) + 4 * Af * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 3 * Af * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 3) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 4 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) - 6 * P0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 72 * Pf * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 + 54 * Pf * T0 * T1 * T2 * T3 * pow(T4, 3) + 108 * Pf * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 + 48 * Pf * T0 * T1 * pow(T3, 3) * pow(T4, 2) + 72 * Pf * T0 * T1 * pow(T3, 3) * T4 * T5 + 48 * Pf * T0 * T1 * pow(T3, 2) * pow(T4, 3) + 96 * Pf * T0 * T1 * pow(T3, 2) * pow(T4, 2) * T5 + 48 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 36 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 3) + 72 * Pf * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 36 * Pf * T0 * T2 * pow(T3, 3) * pow(T4, 2) + 54 * Pf * T0 * T2 * pow(T3, 3) * T4 * T5 + 36 * Pf * T0 * T2 * pow(T3, 2) * pow(T4, 3) + 72 * Pf * T0 * T2 * pow(T3, 2) * pow(T4, 2) * T5 + 72 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 + 54 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 3) + 108 * Pf * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 + 48 * Pf * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) + 72 * Pf * pow(T1, 2) * pow(T3, 3) * T4 * T5 + 48 * Pf * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) + 96 * Pf * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 + 96 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 72 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 3) + 144 * Pf * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 72 * Pf * T1 * T2 * pow(T3, 3) * pow(T4, 2) + 108 * Pf * T1 * T2 * pow(T3, 3) * T4 * T5 + 72 * Pf * T1 * T2 * pow(T3, 2) * pow(T4, 3) + 144 * Pf * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 + 24 * Pf * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * pow(T3, 2) * T4 * T5 + 18 * Pf * pow(T2, 3) * T3 * pow(T4, 3) + 36 * Pf * pow(T2, 3) * T3 * pow(T4, 2) * T5 + 12 * Pf * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) + 18 * Pf * pow(T2, 2) * pow(T3, 3) * T4 * T5 + 12 * Pf * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) + 24 * Pf * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 - 72 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 72 * T0 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 72 * T0 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 108 * T0 * T1 * T2 * pow(T3, 2) * T4 * T5 * d5 + 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 36 * T0 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 54 * T0 * T1 * T2 * T3 * pow(T4, 3) * T5 * Vf - 54 * T0 * T1 * T2 * T3 * pow(T4, 3) * d5 - 72 * T0 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 108 * T0 * T1 * T2 * T3 * pow(T4, 2) * T5 * d5 + 54 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * d4 - 54 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) * d5 - 18 * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 18 * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 48 * T0 * T1 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 48 * T0 * T1 * pow(T3, 3) * pow(T4, 2) * d5 - 48 * T0 * T1 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * pow(T3, 3) * T4 * T5 * d5 + 24 * T0 * T1 * pow(T3, 3) * pow(T5, 2) * d4 - 24 * T0 * T1 * pow(T3, 3) * pow(T5, 2) * d5 - 48 * T0 * T1 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 48 * T0 * T1 * pow(T3, 2) * pow(T4, 3) * d5 - 64 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 96 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 48 * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 48 * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 24 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 24 * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 48 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 - 48 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 72 * T0 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 + 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 - 24 * T0 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 - 36 * T0 * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf - 36 * T0 * pow(T2, 2) * T3 * pow(T4, 3) * d5 - 48 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 72 * T0 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 + 36 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 - 36 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 - 12 * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 12 * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 - 36 * T0 * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 36 * T0 * T2 * pow(T3, 3) * pow(T4, 2) * d5 - 36 * T0 * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 54 * T0 * T2 * pow(T3, 3) * T4 * T5 * d5 + 18 * T0 * T2 * pow(T3, 3) * pow(T5, 2) * d4 - 18 * T0 * T2 * pow(T3, 3) * pow(T5, 2) * d5 - 36 * T0 * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T4, 3) * d5 - 4 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 48 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 72 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 36 * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 36 * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 18 * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 18 * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 72 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 72 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 72 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 108 * pow(T1, 2) * T2 * pow(T3, 2) * T4 * T5 * d5 + 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 36 * pow(T1, 2) * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 54 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * T5 * Vf - 54 * pow(T1, 2) * T2 * T3 * pow(T4, 3) * d5 - 72 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 108 * pow(T1, 2) * T2 * T3 * pow(T4, 2) * T5 * d5 + 54 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d4 - 54 * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) * d5 - 18 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 18 * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 48 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 48 * pow(T1, 2) * pow(T3, 3) * pow(T4, 2) * d5 - 48 * pow(T1, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * pow(T3, 3) * T4 * T5 * d5 + 24 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d4 - 24 * pow(T1, 2) * pow(T3, 3) * pow(T5, 2) * d5 - 48 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 48 * pow(T1, 2) * pow(T3, 2) * pow(T4, 3) * d5 - 64 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 96 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 48 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 48 * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 24 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 24 * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 96 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 96 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 - 96 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 144 * T1 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 + 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 - 48 * T1 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 - 72 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf - 72 * T1 * pow(T2, 2) * T3 * pow(T4, 3) * d5 - 96 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 144 * T1 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 + 72 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 - 72 * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 - 24 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 24 * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 - 72 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 72 * T1 * T2 * pow(T3, 3) * pow(T4, 2) * d5 - 72 * T1 * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 108 * T1 * T2 * pow(T3, 3) * T4 * T5 * d5 + 36 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d4 - 36 * T1 * T2 * pow(T3, 3) * pow(T5, 2) * d5 - 72 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 72 * T1 * T2 * pow(T3, 2) * pow(T4, 3) * d5 - 2 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 96 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 144 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 72 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 72 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 36 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 36 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 24 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * pow(T3, 2) * pow(T4, 2) * d5 - 24 * pow(T2, 3) * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 36 * pow(T2, 3) * pow(T3, 2) * T4 * T5 * d5 + 12 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d4 - 12 * pow(T2, 3) * pow(T3, 2) * pow(T5, 2) * d5 - 18 * pow(T2, 3) * T3 * pow(T4, 3) * T5 * Vf - 18 * pow(T2, 3) * T3 * pow(T4, 3) * d5 - 24 * pow(T2, 3) * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 36 * pow(T2, 3) * T3 * pow(T4, 2) * T5 * d5 + 18 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d4 - 18 * pow(T2, 3) * T3 * T4 * pow(T5, 2) * d5 - 6 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) * d4 - 12 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 12 * pow(T2, 2) * pow(T3, 3) * pow(T4, 2) * d5 - 12 * pow(T2, 2) * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 18 * pow(T2, 2) * pow(T3, 3) * T4 * T5 * d5 + 6 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d4 - 6 * pow(T2, 2) * pow(T3, 3) * pow(T5, 2) * d5 - 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 12 * pow(T2, 2) * pow(T3, 2) * pow(T4, 3) * d5 - 16 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 24 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 12 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 12 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 6 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * d4 + 6 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double vCP2_2 = 2 * Af * T3 + (3.0 / 2.0) * Af * T3 * T5 / T4 + 2 * Af * T4 + 2 * Af * T5 + 6 * Pf * T3 / pow(T5, 2) + 9 * Pf * T3 / (T4 * T5) + 6 * Pf * T4 / pow(T5, 2) + 12 * Pf / T5 - 6 * T3 * Vf / T5 - 6 * T3 * d5 / pow(T5, 2) - 6 * T3 * Vf / T4 - 9 * T3 * d5 / (T4 * T5) + 3 * T3 * d4 / pow(T4, 2) - 3 * T3 * d5 / pow(T4, 2) - 6 * T4 * Vf / T5 - 6 * T4 * d5 / pow(T5, 2) - 8 * Vf - 12 * d5 / T5 + 6 * d4 / T4 - 6 * d5 / T4 - 3 * d3 / T3 + 3 * d4 / T3;
//     // Segment 3:
//     double vCP0_3 = 2 * Af * T3 + (3.0 / 2.0) * Af * T3 * T5 / T4 + 2 * Af * T4 + 2 * Af * T5 + 6 * Pf * T3 / pow(T5, 2) + 9 * Pf * T3 / (T4 * T5) + 6 * Pf * T4 / pow(T5, 2) + 12 * Pf / T5 - 6 * T3 * Vf / T5 - 6 * T3 * d5 / pow(T5, 2) - 6 * T3 * Vf / T4 - 9 * T3 * d5 / (T4 * T5) + 3 * T3 * d4 / pow(T4, 2) - 3 * T3 * d5 / pow(T4, 2) - 6 * T4 * Vf / T5 - 6 * T4 * d5 / pow(T5, 2) - 8 * Vf - 12 * d5 / T5 + 6 * d4 / T4 - 6 * d5 / T4 - 3 * d3 / T3 + 3 * d4 / T3;
//     double vCP1_3 = -1.0 / 2.0 * Af * T3 - 3.0 / 8.0 * Af * T3 * T5 / T4 - 1.0 / 4.0 * Af * T4 - 1.0 / 4.0 * Af * T5 - 3.0 / 2.0 * Pf * T3 / pow(T5, 2) - 9.0 / 4.0 * Pf * T3 / (T4 * T5) - 3.0 / 4.0 * Pf * T4 / pow(T5, 2) - 3.0 / 2.0 * Pf / T5 + (3.0 / 2.0) * T3 * Vf / T5 + (3.0 / 2.0) * T3 * d5 / pow(T5, 2) + (3.0 / 2.0) * T3 * Vf / T4 + (9.0 / 4.0) * T3 * d5 / (T4 * T5) - 3.0 / 4.0 * T3 * d4 / pow(T4, 2) + (3.0 / 4.0) * T3 * d5 / pow(T4, 2) + (3.0 / 4.0) * T4 * Vf / T5 + (3.0 / 4.0) * T4 * d5 / pow(T5, 2) + Vf + (3.0 / 2.0) * d5 / T5 - 3.0 / 4.0 * d4 / T4 + (3.0 / 4.0) * d5 / T4 - 3.0 / 4.0 * d3 / T3 + (3.0 / 4.0) * d4 / T3;
//     double vCP2_3 = -Af * T4 - Af * T5 - 3 * Pf * T4 / pow(T5, 2) - 6 * Pf / T5 + 3 * T4 * Vf / T5 + 3 * T4 * d5 / pow(T5, 2) + 4 * Vf + 6 * d5 / T5 - 3 * d4 / T4 + 3 * d5 / T4;
//     // Segment 4:
//     double vCP0_4 = -Af * T4 - Af * T5 - 3 * Pf * T4 / pow(T5, 2) - 6 * Pf / T5 + 3 * T4 * Vf / T5 + 3 * T4 * d5 / pow(T5, 2) + 4 * Vf + 6 * d5 / T5 - 3 * d4 / T4 + 3 * d5 / T4;
//     double vCP1_4 = (1.0 / 4.0) * Af * T4 + (1.0 / 8.0) * Af * T5 + (3.0 / 4.0) * Pf * T4 / pow(T5, 2) + (3.0 / 4.0) * Pf / T5 - 3.0 / 4.0 * T4 * Vf / T5 - 3.0 / 4.0 * T4 * d5 / pow(T5, 2) - 1.0 / 2.0 * Vf - 3.0 / 4.0 * d5 / T5 - 3.0 / 4.0 * d4 / T4 + (3.0 / 4.0) * d5 / T4;
//     double vCP2_4 = (1.0 / 2.0) * Af * T5 + 3 * Pf / T5 - 2 * Vf - 3 * d5 / T5;
//     // Segment 5:
//     double vCP0_5 = (1.0 / 2.0) * Af * T5 + 3 * Pf / T5 - 2 * Vf - 3 * d5 / T5;
//     double vCP1_5 = (1.0 / 8.0) * (-Af * pow(T5, 2) + 6 * Pf + 2 * T5 * Vf - 6 * d5) / T5;
//     double vCP2_5 = Vf;

//     // Add control points to the list
//     v_cp_.push_back({vCP0_0, vCP1_0, vCP2_0, vCP0_1, vCP1_1, vCP2_1, vCP0_2, vCP1_2, vCP2_2, vCP0_3, vCP1_3, vCP2_3, vCP0_4, vCP1_4, vCP2_4, vCP0_5, vCP1_5, vCP2_5});

//     // ACCELERATION CONTROL POINTS:
//     // Segment 0:
//     double aCP0_0 = A0;
//     double aCP1_0 = (A0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2) + T3 * T4 * (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double aCP0_1 = (T3 * T4 * (-2 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     double aCP1_1 = (A0 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + A0 * T0 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 8 * Af * T0 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 6 * Af * T0 * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 4 * Af * T0 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 3 * Af * T0 * pow(T3, 3) * T4 * pow(T5, 3) + 4 * Af * T0 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 4 * Af * T0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 16 * Af * T1 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 3) + 12 * Af * T1 * T2 * T3 * pow(T4, 3) * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 3) + 8 * Af * T1 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 6 * Af * T1 * pow(T3, 3) * T4 * pow(T5, 3) + 8 * Af * T1 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 8 * Af * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 16 * Af * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 3) + 12 * Af * pow(T2, 2) * T3 * pow(T4, 3) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 3) + 12 * Af * T2 * pow(T3, 3) * pow(T4, 2) * pow(T5, 2) + 9 * Af * T2 * pow(T3, 3) * T4 * pow(T5, 3) + 12 * Af * T2 * pow(T3, 2) * pow(T4, 3) * pow(T5, 2) + 12 * Af * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 3) + 6 * P0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Pf * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 36 * Pf * T0 * T2 * pow(T3, 2) * T4 * T5 + 18 * Pf * T0 * T2 * T3 * pow(T4, 3) + 36 * Pf * T0 * T2 * T3 * pow(T4, 2) * T5 + 12 * Pf * T0 * pow(T3, 3) * pow(T4, 2) + 18 * Pf * T0 * pow(T3, 3) * T4 * T5 + 12 * Pf * T0 * pow(T3, 2) * pow(T4, 3) + 24 * Pf * T0 * pow(T3, 2) * pow(T4, 2) * T5 + 48 * Pf * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 72 * Pf * T1 * T2 * pow(T3, 2) * T4 * T5 + 36 * Pf * T1 * T2 * T3 * pow(T4, 3) + 72 * Pf * T1 * T2 * T3 * pow(T4, 2) * T5 + 24 * Pf * T1 * pow(T3, 3) * pow(T4, 2) + 36 * Pf * T1 * pow(T3, 3) * T4 * T5 + 24 * Pf * T1 * pow(T3, 2) * pow(T4, 3) + 48 * Pf * T1 * pow(T3, 2) * pow(T4, 2) * T5 + 48 * Pf * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 36 * Pf * pow(T2, 2) * T3 * pow(T4, 3) + 72 * Pf * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 36 * Pf * T2 * pow(T3, 3) * pow(T4, 2) + 54 * Pf * T2 * pow(T3, 3) * T4 * T5 + 36 * Pf * T2 * pow(T3, 2) * pow(T4, 3) + 72 * Pf * T2 * pow(T3, 2) * pow(T4, 2) * T5 - 24 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 24 * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 36 * T0 * T2 * pow(T3, 2) * T4 * T5 * d5 + 12 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 12 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 18 * T0 * T2 * T3 * pow(T4, 3) * T5 * Vf - 18 * T0 * T2 * T3 * pow(T4, 3) * d5 - 24 * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 36 * T0 * T2 * T3 * pow(T4, 2) * T5 * d5 + 18 * T0 * T2 * T3 * T4 * pow(T5, 2) * d4 - 18 * T0 * T2 * T3 * T4 * pow(T5, 2) * d5 - 6 * T0 * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * T0 * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 12 * T0 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 12 * T0 * pow(T3, 3) * pow(T4, 2) * d5 - 12 * T0 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 18 * T0 * pow(T3, 3) * T4 * T5 * d5 + 6 * T0 * pow(T3, 3) * pow(T5, 2) * d4 - 6 * T0 * pow(T3, 3) * pow(T5, 2) * d5 - 12 * T0 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T4, 3) * d5 + 4 * T0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 16 * T0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 24 * T0 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 12 * T0 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 12 * T0 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 6 * T0 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 6 * T0 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 48 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 48 * T1 * T2 * pow(T3, 2) * pow(T4, 2) * d5 - 48 * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * d5 + 24 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d4 - 24 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * d5 - 36 * T1 * T2 * T3 * pow(T4, 3) * T5 * Vf - 36 * T1 * T2 * T3 * pow(T4, 3) * d5 - 48 * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * d5 + 36 * T1 * T2 * T3 * T4 * pow(T5, 2) * d4 - 36 * T1 * T2 * T3 * T4 * pow(T5, 2) * d5 - 12 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d3 + 12 * T1 * T2 * pow(T4, 2) * pow(T5, 2) * d4 - 24 * T1 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 24 * T1 * pow(T3, 3) * pow(T4, 2) * d5 - 24 * T1 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 36 * T1 * pow(T3, 3) * T4 * T5 * d5 + 12 * T1 * pow(T3, 3) * pow(T5, 2) * d4 - 12 * T1 * pow(T3, 3) * pow(T5, 2) * d5 - 24 * T1 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 24 * T1 * pow(T3, 2) * pow(T4, 3) * d5 + 2 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * V0 - 32 * T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 48 * T1 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 24 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 24 * T1 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 12 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 12 * T1 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 48 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) * d5 - 48 * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) * Vf - 72 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * d5 + 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d4 - 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * d5 - 36 * pow(T2, 2) * T3 * pow(T4, 3) * T5 * Vf - 36 * pow(T2, 2) * T3 * pow(T4, 3) * d5 - 48 * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) * Vf - 72 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * d5 + 36 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d4 - 36 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * d5 - 12 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d3 + 12 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) * d4 - 36 * T2 * pow(T3, 3) * pow(T4, 2) * T5 * Vf - 36 * T2 * pow(T3, 3) * pow(T4, 2) * d5 - 36 * T2 * pow(T3, 3) * T4 * pow(T5, 2) * Vf - 54 * T2 * pow(T3, 3) * T4 * T5 * d5 + 18 * T2 * pow(T3, 3) * pow(T5, 2) * d4 - 18 * T2 * pow(T3, 3) * pow(T5, 2) * d5 - 36 * T2 * pow(T3, 2) * pow(T4, 3) * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T4, 3) * d5 - 48 * T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * Vf - 72 * T2 * pow(T3, 2) * pow(T4, 2) * T5 * d5 + 36 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d4 - 36 * T2 * pow(T3, 2) * T4 * pow(T5, 2) * d5 - 18 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d3 + 18 * T2 * T3 * pow(T4, 2) * pow(T5, 2) * d4 - 6 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * d3) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 2:
//     double aCP0_2 = (T3 * T4 * (A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + A0 * T0 * T1 * T3 * T4 * pow(T5, 2) + 8 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * T0 * T2 * T3 * pow(T5, 3) + 6 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * T0 * T2 * T4 * pow(T5, 3) + 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) + 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * T0 * T3 * T4 * pow(T5, 3) + 16 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) + 12 * Af * T1 * T2 * T3 * pow(T5, 3) + 12 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T1 * T2 * T4 * pow(T5, 3) + 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) + 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T2 * T3 * T4 * pow(T5, 3) + 6 * P0 * T3 * T4 * pow(T5, 2) + 24 * Pf * T0 * T2 * T3 * T4 + 36 * Pf * T0 * T2 * T3 * T5 + 18 * Pf * T0 * T2 * pow(T4, 2) + 36 * Pf * T0 * T2 * T4 * T5 + 12 * Pf * T0 * pow(T3, 2) * T4 + 18 * Pf * T0 * pow(T3, 2) * T5 + 12 * Pf * T0 * T3 * pow(T4, 2) + 24 * Pf * T0 * T3 * T4 * T5 + 48 * Pf * T1 * T2 * T3 * T4 + 72 * Pf * T1 * T2 * T3 * T5 + 36 * Pf * T1 * T2 * pow(T4, 2) + 72 * Pf * T1 * T2 * T4 * T5 + 24 * Pf * T1 * pow(T3, 2) * T4 + 36 * Pf * T1 * pow(T3, 2) * T5 + 24 * Pf * T1 * T3 * pow(T4, 2) + 48 * Pf * T1 * T3 * T4 * T5 + 48 * Pf * pow(T2, 2) * T3 * T4 + 72 * Pf * pow(T2, 2) * T3 * T5 + 36 * Pf * pow(T2, 2) * pow(T4, 2) + 72 * Pf * pow(T2, 2) * T4 * T5 + 36 * Pf * T2 * pow(T3, 2) * T4 + 54 * Pf * T2 * pow(T3, 2) * T5 + 36 * Pf * T2 * T3 * pow(T4, 2) + 72 * Pf * T2 * T3 * T4 * T5 - 24 * T0 * T2 * T3 * T4 * T5 * Vf - 24 * T0 * T2 * T3 * pow(T5, 2) * Vf - 18 * T0 * T2 * pow(T4, 2) * T5 * Vf - 24 * T0 * T2 * T4 * pow(T5, 2) * Vf - 12 * T0 * pow(T3, 2) * T4 * T5 * Vf - 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf - 12 * T0 * T3 * pow(T4, 2) * T5 * Vf + 4 * T0 * T3 * T4 * pow(T5, 2) * V0 - 16 * T0 * T3 * T4 * pow(T5, 2) * Vf - 48 * T1 * T2 * T3 * T4 * T5 * Vf - 48 * T1 * T2 * T3 * pow(T5, 2) * Vf - 36 * T1 * T2 * pow(T4, 2) * T5 * Vf - 48 * T1 * T2 * T4 * pow(T5, 2) * Vf - 24 * T1 * pow(T3, 2) * T4 * T5 * Vf - 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 24 * T1 * T3 * pow(T4, 2) * T5 * Vf + 2 * T1 * T3 * T4 * pow(T5, 2) * V0 - 32 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T2 * T3 * pow(T4, 2) * T5 * Vf - 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) - 6 * T3 * d5 * (4 * T0 * T2 * T3 * pow(T4, 2) + 6 * T0 * T2 * T3 * T4 * T5 + 2 * T0 * T2 * T3 * pow(T5, 2) + 3 * T0 * T2 * pow(T4, 3) + 6 * T0 * T2 * pow(T4, 2) * T5 + 3 * T0 * T2 * T4 * pow(T5, 2) + 2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 + T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) + 4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) + 8 * T1 * T2 * T3 * pow(T4, 2) + 12 * T1 * T2 * T3 * T4 * T5 + 4 * T1 * T2 * T3 * pow(T5, 2) + 6 * T1 * T2 * pow(T4, 3) + 12 * T1 * T2 * pow(T4, 2) * T5 + 6 * T1 * T2 * T4 * pow(T5, 2) + 4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 + 2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) + 8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) + 8 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * pow(T2, 2) * T3 * T4 * T5 + 4 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * pow(T2, 2) * pow(T4, 3) + 12 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 + 3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) + 12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) - 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T0 * T2 + T0 * T3 + 2 * T1 * T2 + 2 * T1 * T3 + 2 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) + 6 * pow(T5, 2) * d4 * (2 * T0 * T2 * pow(T3, 2) + 3 * T0 * T2 * T3 * T4 + T0 * T2 * pow(T4, 2) + T0 * pow(T3, 3) + 2 * T0 * pow(T3, 2) * T4 + T0 * T3 * pow(T4, 2) + 4 * T1 * T2 * pow(T3, 2) + 6 * T1 * T2 * T3 * T4 + 2 * T1 * T2 * pow(T4, 2) + 2 * T1 * pow(T3, 3) + 4 * T1 * pow(T3, 2) * T4 + 2 * T1 * T3 * pow(T4, 2) + 4 * pow(T2, 2) * pow(T3, 2) + 6 * pow(T2, 2) * T3 * T4 + 2 * pow(T2, 2) * pow(T4, 2) + 3 * T2 * pow(T3, 3) + 6 * T2 * pow(T3, 2) * T4 + 3 * T2 * T3 * pow(T4, 2))) / (pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     double aCP1_2 = -8 * Af - 6 * Af * T5 / T4 - 6 * Af * T4 / T3 - 6 * Af * T5 / T3 - 24 * Pf / pow(T5, 2) - 36 * Pf / (T4 * T5) - 18 * Pf * T4 / (T3 * pow(T5, 2)) - 36 * Pf / (T3 * T5) + 24 * Vf / T5 + 24 * d5 / pow(T5, 2) + 24 * Vf / T4 + 36 * d5 / (T4 * T5) - 12 * d4 / pow(T4, 2) + 12 * d5 / pow(T4, 2) + 18 * T4 * Vf / (T3 * T5) + 18 * T4 * d5 / (T3 * pow(T5, 2)) + 24 * Vf / T3 + 36 * d5 / (T3 * T5) - 18 * d4 / (T3 * T4) + 18 * d5 / (T3 * T4) + 6 * d3 / pow(T3, 2) - 6 * d4 / pow(T3, 2);
//     // Segment 3:
//     double aCP0_3 = -8 * Af - 6 * Af * T5 / T4 - 6 * Af * T4 / T3 - 6 * Af * T5 / T3 - 24 * Pf / pow(T5, 2) - 36 * Pf / (T4 * T5) - 18 * Pf * T4 / (T3 * pow(T5, 2)) - 36 * Pf / (T3 * T5) + 24 * Vf / T5 + 24 * d5 / pow(T5, 2) + 24 * Vf / T4 + 36 * d5 / (T4 * T5) - 12 * d4 / pow(T4, 2) + 12 * d5 / pow(T4, 2) + 18 * T4 * Vf / (T3 * T5) + 18 * T4 * d5 / (T3 * pow(T5, 2)) + 24 * Vf / T3 + 36 * d5 / (T3 * T5) - 18 * d4 / (T3 * T4) + 18 * d5 / (T3 * T4) + 6 * d3 / pow(T3, 2) - 6 * d4 / pow(T3, 2);
//     double aCP1_3 = 4 * Af + 3 * Af * T5 / T4 + 12 * Pf / pow(T5, 2) + 18 * Pf / (T4 * T5) - 12 * Vf / T5 - 12 * d5 / pow(T5, 2) - 12 * Vf / T4 - 18 * d5 / (T4 * T5) + 6 * d4 / pow(T4, 2) - 6 * d5 / pow(T4, 2);
//     // Segment 4:
//     double aCP0_4 = 4 * Af + 3 * Af * T5 / T4 + 12 * Pf / pow(T5, 2) + 18 * Pf / (T4 * T5) - 12 * Vf / T5 - 12 * d5 / pow(T5, 2) - 12 * Vf / T4 - 18 * d5 / (T4 * T5) + 6 * d4 / pow(T4, 2) - 6 * d5 / pow(T4, 2);
//     double aCP1_4 = 2 * (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf + 3 * d5) / pow(T5, 2);
//     // Segment 5:
//     double aCP0_5 = 2 * (-Af * pow(T5, 2) - 3 * Pf + 3 * T5 * Vf + 3 * d5) / pow(T5, 2);
//     double aCP1_5 = Af;

//     // Add control points to the list
//     a_cp_.push_back({aCP0_0, aCP1_0, aCP0_1, aCP1_1, aCP0_2, aCP1_2, aCP0_3, aCP1_3, aCP0_4, aCP1_4, aCP0_5, aCP1_5});

//     // JERK CONTROL POINTS:
//     // Segment 0:
//     double jCP0_0 = (T3 * T4 * (-3 * A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - 4 * A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 2 * A0 * T0 * T2 * T3 * T4 * pow(T5, 2) - A0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) - A0 * T1 * T2 * T3 * T4 * pow(T5, 2) - 8 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 6 * Af * T1 * T2 * T3 * pow(T5, 3) - 6 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T1 * T2 * T4 * pow(T5, 3) - 4 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T1 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 8 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T1 * T2 * T3 * T4 - 36 * Pf * T1 * T2 * T3 * T5 - 18 * Pf * T1 * T2 * pow(T4, 2) - 36 * Pf * T1 * T2 * T4 * T5 - 12 * Pf * T1 * pow(T3, 2) * T4 - 18 * Pf * T1 * pow(T3, 2) * T5 - 12 * Pf * T1 * T3 * pow(T4, 2) - 24 * Pf * T1 * T3 * T4 * T5 - 24 * Pf * pow(T2, 2) * T3 * T4 - 36 * Pf * pow(T2, 2) * T3 * T5 - 18 * Pf * pow(T2, 2) * pow(T4, 2) - 36 * Pf * pow(T2, 2) * T4 * T5 - 24 * Pf * T2 * pow(T3, 2) * T4 - 36 * Pf * T2 * pow(T3, 2) * T5 - 24 * Pf * T2 * T3 * pow(T4, 2) - 48 * Pf * T2 * T3 * T4 * T5 - 6 * T0 * T3 * T4 * pow(T5, 2) * V0 + 24 * T1 * T2 * T3 * T4 * T5 * Vf + 24 * T1 * T2 * T3 * pow(T5, 2) * Vf + 18 * T1 * T2 * pow(T4, 2) * T5 * Vf + 24 * T1 * T2 * T4 * pow(T5, 2) * Vf + 12 * T1 * pow(T3, 2) * T4 * T5 * Vf + 12 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T1 * T3 * pow(T4, 2) * T5 * Vf - 4 * T1 * T3 * T4 * pow(T5, 2) * V0 + 16 * T1 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T2, 2) * T3 * T4 * T5 * Vf + 24 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 24 * T2 * pow(T3, 2) * T4 * T5 * Vf + 24 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T2 * T3 * pow(T4, 2) * T5 * Vf - 2 * T2 * T3 * T4 * pow(T5, 2) * V0 + 32 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T1 * T2 * T3 * pow(T4, 2) + 6 * T1 * T2 * T3 * T4 * T5 + 2 * T1 * T2 * T3 * pow(T5, 2) + 3 * T1 * T2 * pow(T4, 3) + 6 * T1 * T2 * pow(T4, 2) * T5 + 3 * T1 * T2 * T4 * pow(T5, 2) + 2 * T1 * pow(T3, 2) * pow(T4, 2) + 3 * T1 * pow(T3, 2) * T4 * T5 + T1 * pow(T3, 2) * pow(T5, 2) + 2 * T1 * T3 * pow(T4, 3) + 4 * T1 * T3 * pow(T4, 2) * T5 + 2 * T1 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 2) + 6 * pow(T2, 2) * T3 * T4 * T5 + 2 * pow(T2, 2) * T3 * pow(T5, 2) + 3 * pow(T2, 2) * pow(T4, 3) + 6 * pow(T2, 2) * pow(T4, 2) * T5 + 3 * pow(T2, 2) * T4 * pow(T5, 2) + 4 * T2 * pow(T3, 2) * pow(T4, 2) + 6 * T2 * pow(T3, 2) * T4 * T5 + 2 * T2 * pow(T3, 2) * pow(T5, 2) + 4 * T2 * T3 * pow(T4, 3) + 8 * T2 * T3 * pow(T4, 2) * T5 + 4 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T1 * T2 * pow(T3, 2) + 3 * T1 * T2 * T3 * T4 + T1 * T2 * pow(T4, 2) + T1 * pow(T3, 3) + 2 * T1 * pow(T3, 2) * T4 + T1 * T3 * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 2) + 3 * pow(T2, 2) * T3 * T4 + pow(T2, 2) * pow(T4, 2) + 2 * T2 * pow(T3, 3) + 4 * T2 * pow(T3, 2) * T4 + 2 * T2 * T3 * pow(T4, 2))) / (T0 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) + 2 * T0 * T1 + T0 * T2 + pow(T1, 2) + T1 * T2));
//     // Segment 1:
//     double jCP0_1 = (T3 * T4 * (A0 * pow(T0, 3) * T3 * T4 * pow(T5, 2) + 4 * A0 * pow(T0, 2) * T1 * T3 * T4 * pow(T5, 2) + 2 * A0 * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 3 * A0 * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + A0 * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 8 * Af * pow(T0, 2) * T2 * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T3 * pow(T5, 3) + 6 * Af * pow(T0, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T0, 2) * T2 * T4 * pow(T5, 3) + 4 * Af * pow(T0, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 3 * Af * pow(T0, 2) * pow(T3, 2) * pow(T5, 3) + 4 * Af * pow(T0, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 4 * Af * pow(T0, 2) * T3 * T4 * pow(T5, 3) + 24 * Af * T0 * T1 * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T3 * pow(T5, 3) + 18 * Af * T0 * T1 * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * T0 * T1 * T2 * T4 * pow(T5, 3) + 12 * Af * T0 * T1 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T1 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T1 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T1 * T3 * T4 * pow(T5, 3) + 16 * Af * T0 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T3 * pow(T5, 3) + 12 * Af * T0 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * pow(T2, 2) * T4 * pow(T5, 3) + 12 * Af * T0 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * T0 * T2 * pow(T3, 2) * pow(T5, 3) + 12 * Af * T0 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * T0 * T2 * T3 * T4 * pow(T5, 3) + 24 * Af * pow(T1, 2) * T2 * T3 * T4 * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T3 * pow(T5, 3) + 18 * Af * pow(T1, 2) * T2 * pow(T4, 2) * pow(T5, 2) + 18 * Af * pow(T1, 2) * T2 * T4 * pow(T5, 3) + 12 * Af * pow(T1, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 9 * Af * pow(T1, 2) * pow(T3, 2) * pow(T5, 3) + 12 * Af * pow(T1, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 12 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 3) + 32 * Af * T1 * pow(T2, 2) * T3 * T4 * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T3 * pow(T5, 3) + 24 * Af * T1 * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * pow(T2, 2) * T4 * pow(T5, 3) + 24 * Af * T1 * T2 * pow(T3, 2) * T4 * pow(T5, 2) + 18 * Af * T1 * T2 * pow(T3, 2) * pow(T5, 3) + 24 * Af * T1 * T2 * T3 * pow(T4, 2) * pow(T5, 2) + 24 * Af * T1 * T2 * T3 * T4 * pow(T5, 3) + 8 * Af * pow(T2, 3) * T3 * T4 * pow(T5, 2) + 6 * Af * pow(T2, 3) * T3 * pow(T5, 3) + 6 * Af * pow(T2, 3) * pow(T4, 2) * pow(T5, 2) + 6 * Af * pow(T2, 3) * T4 * pow(T5, 3) + 8 * Af * pow(T2, 2) * pow(T3, 2) * T4 * pow(T5, 2) + 6 * Af * pow(T2, 2) * pow(T3, 2) * pow(T5, 3) + 8 * Af * pow(T2, 2) * T3 * pow(T4, 2) * pow(T5, 2) + 8 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 3) + 6 * P0 * T0 * T3 * T4 * pow(T5, 2) + 12 * P0 * T1 * T3 * T4 * pow(T5, 2) + 6 * P0 * T2 * T3 * T4 * pow(T5, 2) + 24 * Pf * pow(T0, 2) * T2 * T3 * T4 + 36 * Pf * pow(T0, 2) * T2 * T3 * T5 + 18 * Pf * pow(T0, 2) * T2 * pow(T4, 2) + 36 * Pf * pow(T0, 2) * T2 * T4 * T5 + 12 * Pf * pow(T0, 2) * pow(T3, 2) * T4 + 18 * Pf * pow(T0, 2) * pow(T3, 2) * T5 + 12 * Pf * pow(T0, 2) * T3 * pow(T4, 2) + 24 * Pf * pow(T0, 2) * T3 * T4 * T5 + 72 * Pf * T0 * T1 * T2 * T3 * T4 + 108 * Pf * T0 * T1 * T2 * T3 * T5 + 54 * Pf * T0 * T1 * T2 * pow(T4, 2) + 108 * Pf * T0 * T1 * T2 * T4 * T5 + 36 * Pf * T0 * T1 * pow(T3, 2) * T4 + 54 * Pf * T0 * T1 * pow(T3, 2) * T5 + 36 * Pf * T0 * T1 * T3 * pow(T4, 2) + 72 * Pf * T0 * T1 * T3 * T4 * T5 + 48 * Pf * T0 * pow(T2, 2) * T3 * T4 + 72 * Pf * T0 * pow(T2, 2) * T3 * T5 + 36 * Pf * T0 * pow(T2, 2) * pow(T4, 2) + 72 * Pf * T0 * pow(T2, 2) * T4 * T5 + 36 * Pf * T0 * T2 * pow(T3, 2) * T4 + 54 * Pf * T0 * T2 * pow(T3, 2) * T5 + 36 * Pf * T0 * T2 * T3 * pow(T4, 2) + 72 * Pf * T0 * T2 * T3 * T4 * T5 + 72 * Pf * pow(T1, 2) * T2 * T3 * T4 + 108 * Pf * pow(T1, 2) * T2 * T3 * T5 + 54 * Pf * pow(T1, 2) * T2 * pow(T4, 2) + 108 * Pf * pow(T1, 2) * T2 * T4 * T5 + 36 * Pf * pow(T1, 2) * pow(T3, 2) * T4 + 54 * Pf * pow(T1, 2) * pow(T3, 2) * T5 + 36 * Pf * pow(T1, 2) * T3 * pow(T4, 2) + 72 * Pf * pow(T1, 2) * T3 * T4 * T5 + 96 * Pf * T1 * pow(T2, 2) * T3 * T4 + 144 * Pf * T1 * pow(T2, 2) * T3 * T5 + 72 * Pf * T1 * pow(T2, 2) * pow(T4, 2) + 144 * Pf * T1 * pow(T2, 2) * T4 * T5 + 72 * Pf * T1 * T2 * pow(T3, 2) * T4 + 108 * Pf * T1 * T2 * pow(T3, 2) * T5 + 72 * Pf * T1 * T2 * T3 * pow(T4, 2) + 144 * Pf * T1 * T2 * T3 * T4 * T5 + 24 * Pf * pow(T2, 3) * T3 * T4 + 36 * Pf * pow(T2, 3) * T3 * T5 + 18 * Pf * pow(T2, 3) * pow(T4, 2) + 36 * Pf * pow(T2, 3) * T4 * T5 + 24 * Pf * pow(T2, 2) * pow(T3, 2) * T4 + 36 * Pf * pow(T2, 2) * pow(T3, 2) * T5 + 24 * Pf * pow(T2, 2) * T3 * pow(T4, 2) + 48 * Pf * pow(T2, 2) * T3 * T4 * T5 - 24 * pow(T0, 2) * T2 * T3 * T4 * T5 * Vf - 24 * pow(T0, 2) * T2 * T3 * pow(T5, 2) * Vf - 18 * pow(T0, 2) * T2 * pow(T4, 2) * T5 * Vf - 24 * pow(T0, 2) * T2 * T4 * pow(T5, 2) * Vf - 12 * pow(T0, 2) * pow(T3, 2) * T4 * T5 * Vf - 12 * pow(T0, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 12 * pow(T0, 2) * T3 * pow(T4, 2) * T5 * Vf + 4 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * V0 - 16 * pow(T0, 2) * T3 * T4 * pow(T5, 2) * Vf - 72 * T0 * T1 * T2 * T3 * T4 * T5 * Vf - 72 * T0 * T1 * T2 * T3 * pow(T5, 2) * Vf - 54 * T0 * T1 * T2 * pow(T4, 2) * T5 * Vf - 72 * T0 * T1 * T2 * T4 * pow(T5, 2) * Vf - 36 * T0 * T1 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T1 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T1 * T3 * pow(T4, 2) * T5 * Vf + 12 * T0 * T1 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T1 * T3 * T4 * pow(T5, 2) * Vf - 48 * T0 * pow(T2, 2) * T3 * T4 * T5 * Vf - 48 * T0 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 36 * T0 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 48 * T0 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 36 * T0 * T2 * pow(T3, 2) * T4 * T5 * Vf - 36 * T0 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 36 * T0 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) * V0 - 48 * T0 * T2 * T3 * T4 * pow(T5, 2) * Vf - 72 * pow(T1, 2) * T2 * T3 * T4 * T5 * Vf - 72 * pow(T1, 2) * T2 * T3 * pow(T5, 2) * Vf - 54 * pow(T1, 2) * T2 * pow(T4, 2) * T5 * Vf - 72 * pow(T1, 2) * T2 * T4 * pow(T5, 2) * Vf - 36 * pow(T1, 2) * pow(T3, 2) * T4 * T5 * Vf - 36 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 36 * pow(T1, 2) * T3 * pow(T4, 2) * T5 * Vf + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * V0 - 48 * pow(T1, 2) * T3 * T4 * pow(T5, 2) * Vf - 96 * T1 * pow(T2, 2) * T3 * T4 * T5 * Vf - 96 * T1 * pow(T2, 2) * T3 * pow(T5, 2) * Vf - 72 * T1 * pow(T2, 2) * pow(T4, 2) * T5 * Vf - 96 * T1 * pow(T2, 2) * T4 * pow(T5, 2) * Vf - 72 * T1 * T2 * pow(T3, 2) * T4 * T5 * Vf - 72 * T1 * T2 * pow(T3, 2) * pow(T5, 2) * Vf - 72 * T1 * T2 * T3 * pow(T4, 2) * T5 * Vf + 6 * T1 * T2 * T3 * T4 * pow(T5, 2) * V0 - 96 * T1 * T2 * T3 * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 3) * T3 * T4 * T5 * Vf - 24 * pow(T2, 3) * T3 * pow(T5, 2) * Vf - 18 * pow(T2, 3) * pow(T4, 2) * T5 * Vf - 24 * pow(T2, 3) * T4 * pow(T5, 2) * Vf - 24 * pow(T2, 2) * pow(T3, 2) * T4 * T5 * Vf - 24 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) * Vf - 24 * pow(T2, 2) * T3 * pow(T4, 2) * T5 * Vf + 2 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * V0 - 32 * pow(T2, 2) * T3 * T4 * pow(T5, 2) * Vf) - 6 * T3 * d5 * (4 * pow(T0, 2) * T2 * T3 * pow(T4, 2) + 6 * pow(T0, 2) * T2 * T3 * T4 * T5 + 2 * pow(T0, 2) * T2 * T3 * pow(T5, 2) + 3 * pow(T0, 2) * T2 * pow(T4, 3) + 6 * pow(T0, 2) * T2 * pow(T4, 2) * T5 + 3 * pow(T0, 2) * T2 * T4 * pow(T5, 2) + 2 * pow(T0, 2) * pow(T3, 2) * pow(T4, 2) + 3 * pow(T0, 2) * pow(T3, 2) * T4 * T5 + pow(T0, 2) * pow(T3, 2) * pow(T5, 2) + 2 * pow(T0, 2) * T3 * pow(T4, 3) + 4 * pow(T0, 2) * T3 * pow(T4, 2) * T5 + 2 * pow(T0, 2) * T3 * T4 * pow(T5, 2) + 12 * T0 * T1 * T2 * T3 * pow(T4, 2) + 18 * T0 * T1 * T2 * T3 * T4 * T5 + 6 * T0 * T1 * T2 * T3 * pow(T5, 2) + 9 * T0 * T1 * T2 * pow(T4, 3) + 18 * T0 * T1 * T2 * pow(T4, 2) * T5 + 9 * T0 * T1 * T2 * T4 * pow(T5, 2) + 6 * T0 * T1 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T1 * pow(T3, 2) * T4 * T5 + 3 * T0 * T1 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T1 * T3 * pow(T4, 3) + 12 * T0 * T1 * T3 * pow(T4, 2) * T5 + 6 * T0 * T1 * T3 * T4 * pow(T5, 2) + 8 * T0 * pow(T2, 2) * T3 * pow(T4, 2) + 12 * T0 * pow(T2, 2) * T3 * T4 * T5 + 4 * T0 * pow(T2, 2) * T3 * pow(T5, 2) + 6 * T0 * pow(T2, 2) * pow(T4, 3) + 12 * T0 * pow(T2, 2) * pow(T4, 2) * T5 + 6 * T0 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T0 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T0 * T2 * pow(T3, 2) * T4 * T5 + 3 * T0 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T0 * T2 * T3 * pow(T4, 3) + 12 * T0 * T2 * T3 * pow(T4, 2) * T5 + 6 * T0 * T2 * T3 * T4 * pow(T5, 2) + 12 * pow(T1, 2) * T2 * T3 * pow(T4, 2) + 18 * pow(T1, 2) * T2 * T3 * T4 * T5 + 6 * pow(T1, 2) * T2 * T3 * pow(T5, 2) + 9 * pow(T1, 2) * T2 * pow(T4, 3) + 18 * pow(T1, 2) * T2 * pow(T4, 2) * T5 + 9 * pow(T1, 2) * T2 * T4 * pow(T5, 2) + 6 * pow(T1, 2) * pow(T3, 2) * pow(T4, 2) + 9 * pow(T1, 2) * pow(T3, 2) * T4 * T5 + 3 * pow(T1, 2) * pow(T3, 2) * pow(T5, 2) + 6 * pow(T1, 2) * T3 * pow(T4, 3) + 12 * pow(T1, 2) * T3 * pow(T4, 2) * T5 + 6 * pow(T1, 2) * T3 * T4 * pow(T5, 2) + 16 * T1 * pow(T2, 2) * T3 * pow(T4, 2) + 24 * T1 * pow(T2, 2) * T3 * T4 * T5 + 8 * T1 * pow(T2, 2) * T3 * pow(T5, 2) + 12 * T1 * pow(T2, 2) * pow(T4, 3) + 24 * T1 * pow(T2, 2) * pow(T4, 2) * T5 + 12 * T1 * pow(T2, 2) * T4 * pow(T5, 2) + 12 * T1 * T2 * pow(T3, 2) * pow(T4, 2) + 18 * T1 * T2 * pow(T3, 2) * T4 * T5 + 6 * T1 * T2 * pow(T3, 2) * pow(T5, 2) + 12 * T1 * T2 * T3 * pow(T4, 3) + 24 * T1 * T2 * T3 * pow(T4, 2) * T5 + 12 * T1 * T2 * T3 * T4 * pow(T5, 2) + 4 * pow(T2, 3) * T3 * pow(T4, 2) + 6 * pow(T2, 3) * T3 * T4 * T5 + 2 * pow(T2, 3) * T3 * pow(T5, 2) + 3 * pow(T2, 3) * pow(T4, 3) + 6 * pow(T2, 3) * pow(T4, 2) * T5 + 3 * pow(T2, 3) * T4 * pow(T5, 2) + 4 * pow(T2, 2) * pow(T3, 2) * pow(T4, 2) + 6 * pow(T2, 2) * pow(T3, 2) * T4 * T5 + 2 * pow(T2, 2) * pow(T3, 2) * pow(T5, 2) + 4 * pow(T2, 2) * T3 * pow(T4, 3) + 8 * pow(T2, 2) * T3 * pow(T4, 2) * T5 + 4 * pow(T2, 2) * T3 * T4 * pow(T5, 2)) - 6 * pow(T4, 2) * pow(T5, 2) * d3 * (pow(T0, 2) * T2 + pow(T0, 2) * T3 + 3 * T0 * T1 * T2 + 3 * T0 * T1 * T3 + 2 * T0 * pow(T2, 2) + 3 * T0 * T2 * T3 + T0 * pow(T3, 2) + 3 * pow(T1, 2) * T2 + 3 * pow(T1, 2) * T3 + 4 * T1 * pow(T2, 2) + 6 * T1 * T2 * T3 + 2 * T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)) + 6 * pow(T5, 2) * d4 * (2 * pow(T0, 2) * T2 * pow(T3, 2) + 3 * pow(T0, 2) * T2 * T3 * T4 + pow(T0, 2) * T2 * pow(T4, 2) + pow(T0, 2) * pow(T3, 3) + 2 * pow(T0, 2) * pow(T3, 2) * T4 + pow(T0, 2) * T3 * pow(T4, 2) + 6 * T0 * T1 * T2 * pow(T3, 2) + 9 * T0 * T1 * T2 * T3 * T4 + 3 * T0 * T1 * T2 * pow(T4, 2) + 3 * T0 * T1 * pow(T3, 3) + 6 * T0 * T1 * pow(T3, 2) * T4 + 3 * T0 * T1 * T3 * pow(T4, 2) + 4 * T0 * pow(T2, 2) * pow(T3, 2) + 6 * T0 * pow(T2, 2) * T3 * T4 + 2 * T0 * pow(T2, 2) * pow(T4, 2) + 3 * T0 * T2 * pow(T3, 3) + 6 * T0 * T2 * pow(T3, 2) * T4 + 3 * T0 * T2 * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T2 * pow(T3, 2) + 9 * pow(T1, 2) * T2 * T3 * T4 + 3 * pow(T1, 2) * T2 * pow(T4, 2) + 3 * pow(T1, 2) * pow(T3, 3) + 6 * pow(T1, 2) * pow(T3, 2) * T4 + 3 * pow(T1, 2) * T3 * pow(T4, 2) + 8 * T1 * pow(T2, 2) * pow(T3, 2) + 12 * T1 * pow(T2, 2) * T3 * T4 + 4 * T1 * pow(T2, 2) * pow(T4, 2) + 6 * T1 * T2 * pow(T3, 3) + 12 * T1 * T2 * pow(T3, 2) * T4 + 6 * T1 * T2 * T3 * pow(T4, 2) + 2 * pow(T2, 3) * pow(T3, 2) + 3 * pow(T2, 3) * T3 * T4 + pow(T2, 3) * pow(T4, 2) + 2 * pow(T2, 2) * pow(T3, 3) + 4 * pow(T2, 2) * pow(T3, 2) * T4 + 2 * pow(T2, 2) * T3 * pow(T4, 2))) / (T1 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (pow(T0, 2) * T1 + pow(T0, 2) * T2 + 2 * T0 * pow(T1, 2) + 3 * T0 * T1 * T2 + T0 * pow(T2, 2) + pow(T1, 3) + 2 * pow(T1, 2) * T2 + T1 * pow(T2, 2)));
//     // Segment 2:
//     double jCP0_2 = (T3 * T4 * (-A0 * pow(T0, 2) * T3 * T4 * pow(T5, 2) - A0 * T0 * T1 * T3 * T4 * pow(T5, 2) - 8 * Af * T0 * T1 * T3 * T4 * pow(T5, 2) - 6 * Af * T0 * T1 * T3 * pow(T5, 3) - 6 * Af * T0 * T1 * pow(T4, 2) * pow(T5, 2) - 6 * Af * T0 * T1 * T4 * pow(T5, 3) - 16 * Af * T0 * T2 * T3 * T4 * pow(T5, 2) - 12 * Af * T0 * T2 * T3 * pow(T5, 3) - 12 * Af * T0 * T2 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T0 * T2 * T4 * pow(T5, 3) - 4 * Af * T0 * pow(T3, 2) * T4 * pow(T5, 2) - 3 * Af * T0 * pow(T3, 2) * pow(T5, 3) - 4 * Af * T0 * T3 * pow(T4, 2) * pow(T5, 2) - 4 * Af * T0 * T3 * T4 * pow(T5, 3) - 8 * Af * pow(T1, 2) * T3 * T4 * pow(T5, 2) - 6 * Af * pow(T1, 2) * T3 * pow(T5, 3) - 6 * Af * pow(T1, 2) * pow(T4, 2) * pow(T5, 2) - 6 * Af * pow(T1, 2) * T4 * pow(T5, 3) - 32 * Af * T1 * T2 * T3 * T4 * pow(T5, 2) - 24 * Af * T1 * T2 * T3 * pow(T5, 3) - 24 * Af * T1 * T2 * pow(T4, 2) * pow(T5, 2) - 24 * Af * T1 * T2 * T4 * pow(T5, 3) - 8 * Af * T1 * pow(T3, 2) * T4 * pow(T5, 2) - 6 * Af * T1 * pow(T3, 2) * pow(T5, 3) - 8 * Af * T1 * T3 * pow(T4, 2) * pow(T5, 2) - 8 * Af * T1 * T3 * T4 * pow(T5, 3) - 24 * Af * pow(T2, 2) * T3 * T4 * pow(T5, 2) - 18 * Af * pow(T2, 2) * T3 * pow(T5, 3) - 18 * Af * pow(T2, 2) * pow(T4, 2) * pow(T5, 2) - 18 * Af * pow(T2, 2) * T4 * pow(T5, 3) - 12 * Af * T2 * pow(T3, 2) * T4 * pow(T5, 2) - 9 * Af * T2 * pow(T3, 2) * pow(T5, 3) - 12 * Af * T2 * T3 * pow(T4, 2) * pow(T5, 2) - 12 * Af * T2 * T3 * T4 * pow(T5, 3) - 6 * P0 * T3 * T4 * pow(T5, 2) - 24 * Pf * T0 * T1 * T3 * T4 - 36 * Pf * T0 * T1 * T3 * T5 - 18 * Pf * T0 * T1 * pow(T4, 2) - 36 * Pf * T0 * T1 * T4 * T5 - 48 * Pf * T0 * T2 * T3 * T4 - 72 * Pf * T0 * T2 * T3 * T5 - 36 * Pf * T0 * T2 * pow(T4, 2) - 72 * Pf * T0 * T2 * T4 * T5 - 12 * Pf * T0 * pow(T3, 2) * T4 - 18 * Pf * T0 * pow(T3, 2) * T5 - 12 * Pf * T0 * T3 * pow(T4, 2) - 24 * Pf * T0 * T3 * T4 * T5 - 24 * Pf * pow(T1, 2) * T3 * T4 - 36 * Pf * pow(T1, 2) * T3 * T5 - 18 * Pf * pow(T1, 2) * pow(T4, 2) - 36 * Pf * pow(T1, 2) * T4 * T5 - 96 * Pf * T1 * T2 * T3 * T4 - 144 * Pf * T1 * T2 * T3 * T5 - 72 * Pf * T1 * T2 * pow(T4, 2) - 144 * Pf * T1 * T2 * T4 * T5 - 24 * Pf * T1 * pow(T3, 2) * T4 - 36 * Pf * T1 * pow(T3, 2) * T5 - 24 * Pf * T1 * T3 * pow(T4, 2) - 48 * Pf * T1 * T3 * T4 * T5 - 72 * Pf * pow(T2, 2) * T3 * T4 - 108 * Pf * pow(T2, 2) * T3 * T5 - 54 * Pf * pow(T2, 2) * pow(T4, 2) - 108 * Pf * pow(T2, 2) * T4 * T5 - 36 * Pf * T2 * pow(T3, 2) * T4 - 54 * Pf * T2 * pow(T3, 2) * T5 - 36 * Pf * T2 * T3 * pow(T4, 2) - 72 * Pf * T2 * T3 * T4 * T5 + 24 * T0 * T1 * T3 * T4 * T5 * Vf + 24 * T0 * T1 * T3 * pow(T5, 2) * Vf + 18 * T0 * T1 * pow(T4, 2) * T5 * Vf + 24 * T0 * T1 * T4 * pow(T5, 2) * Vf + 48 * T0 * T2 * T3 * T4 * T5 * Vf + 48 * T0 * T2 * T3 * pow(T5, 2) * Vf + 36 * T0 * T2 * pow(T4, 2) * T5 * Vf + 48 * T0 * T2 * T4 * pow(T5, 2) * Vf + 12 * T0 * pow(T3, 2) * T4 * T5 * Vf + 12 * T0 * pow(T3, 2) * pow(T5, 2) * Vf + 12 * T0 * T3 * pow(T4, 2) * T5 * Vf - 4 * T0 * T3 * T4 * pow(T5, 2) * V0 + 16 * T0 * T3 * T4 * pow(T5, 2) * Vf + 24 * pow(T1, 2) * T3 * T4 * T5 * Vf + 24 * pow(T1, 2) * T3 * pow(T5, 2) * Vf + 18 * pow(T1, 2) * pow(T4, 2) * T5 * Vf + 24 * pow(T1, 2) * T4 * pow(T5, 2) * Vf + 96 * T1 * T2 * T3 * T4 * T5 * Vf + 96 * T1 * T2 * T3 * pow(T5, 2) * Vf + 72 * T1 * T2 * pow(T4, 2) * T5 * Vf + 96 * T1 * T2 * T4 * pow(T5, 2) * Vf + 24 * T1 * pow(T3, 2) * T4 * T5 * Vf + 24 * T1 * pow(T3, 2) * pow(T5, 2) * Vf + 24 * T1 * T3 * pow(T4, 2) * T5 * Vf - 2 * T1 * T3 * T4 * pow(T5, 2) * V0 + 32 * T1 * T3 * T4 * pow(T5, 2) * Vf + 72 * pow(T2, 2) * T3 * T4 * T5 * Vf + 72 * pow(T2, 2) * T3 * pow(T5, 2) * Vf + 54 * pow(T2, 2) * pow(T4, 2) * T5 * Vf + 72 * pow(T2, 2) * T4 * pow(T5, 2) * Vf + 36 * T2 * pow(T3, 2) * T4 * T5 * Vf + 36 * T2 * pow(T3, 2) * pow(T5, 2) * Vf + 36 * T2 * T3 * pow(T4, 2) * T5 * Vf + 48 * T2 * T3 * T4 * pow(T5, 2) * Vf) + 6 * T3 * d5 * (4 * T0 * T1 * T3 * pow(T4, 2) + 6 * T0 * T1 * T3 * T4 * T5 + 2 * T0 * T1 * T3 * pow(T5, 2) + 3 * T0 * T1 * pow(T4, 3) + 6 * T0 * T1 * pow(T4, 2) * T5 + 3 * T0 * T1 * T4 * pow(T5, 2) + 8 * T0 * T2 * T3 * pow(T4, 2) + 12 * T0 * T2 * T3 * T4 * T5 + 4 * T0 * T2 * T3 * pow(T5, 2) + 6 * T0 * T2 * pow(T4, 3) + 12 * T0 * T2 * pow(T4, 2) * T5 + 6 * T0 * T2 * T4 * pow(T5, 2) + 2 * T0 * pow(T3, 2) * pow(T4, 2) + 3 * T0 * pow(T3, 2) * T4 * T5 + T0 * pow(T3, 2) * pow(T5, 2) + 2 * T0 * T3 * pow(T4, 3) + 4 * T0 * T3 * pow(T4, 2) * T5 + 2 * T0 * T3 * T4 * pow(T5, 2) + 4 * pow(T1, 2) * T3 * pow(T4, 2) + 6 * pow(T1, 2) * T3 * T4 * T5 + 2 * pow(T1, 2) * T3 * pow(T5, 2) + 3 * pow(T1, 2) * pow(T4, 3) + 6 * pow(T1, 2) * pow(T4, 2) * T5 + 3 * pow(T1, 2) * T4 * pow(T5, 2) + 16 * T1 * T2 * T3 * pow(T4, 2) + 24 * T1 * T2 * T3 * T4 * T5 + 8 * T1 * T2 * T3 * pow(T5, 2) + 12 * T1 * T2 * pow(T4, 3) + 24 * T1 * T2 * pow(T4, 2) * T5 + 12 * T1 * T2 * T4 * pow(T5, 2) + 4 * T1 * pow(T3, 2) * pow(T4, 2) + 6 * T1 * pow(T3, 2) * T4 * T5 + 2 * T1 * pow(T3, 2) * pow(T5, 2) + 4 * T1 * T3 * pow(T4, 3) + 8 * T1 * T3 * pow(T4, 2) * T5 + 4 * T1 * T3 * T4 * pow(T5, 2) + 12 * pow(T2, 2) * T3 * pow(T4, 2) + 18 * pow(T2, 2) * T3 * T4 * T5 + 6 * pow(T2, 2) * T3 * pow(T5, 2) + 9 * pow(T2, 2) * pow(T4, 3) + 18 * pow(T2, 2) * pow(T4, 2) * T5 + 9 * pow(T2, 2) * T4 * pow(T5, 2) + 6 * T2 * pow(T3, 2) * pow(T4, 2) + 9 * T2 * pow(T3, 2) * T4 * T5 + 3 * T2 * pow(T3, 2) * pow(T5, 2) + 6 * T2 * T3 * pow(T4, 3) + 12 * T2 * T3 * pow(T4, 2) * T5 + 6 * T2 * T3 * T4 * pow(T5, 2)) + 6 * pow(T4, 2) * pow(T5, 2) * d3 * (T0 * T1 + 2 * T0 * T2 + T0 * T3 + pow(T1, 2) + 4 * T1 * T2 + 2 * T1 * T3 + 3 * pow(T2, 2) + 3 * T2 * T3 + pow(T3, 2)) - 6 * pow(T5, 2) * d4 * (2 * T0 * T1 * pow(T3, 2) + 3 * T0 * T1 * T3 * T4 + T0 * T1 * pow(T4, 2) + 4 * T0 * T2 * pow(T3, 2) + 6 * T0 * T2 * T3 * T4 + 2 * T0 * T2 * pow(T4, 2) + T0 * pow(T3, 3) + 2 * T0 * pow(T3, 2) * T4 + T0 * T3 * pow(T4, 2) + 2 * pow(T1, 2) * pow(T3, 2) + 3 * pow(T1, 2) * T3 * T4 + pow(T1, 2) * pow(T4, 2) + 8 * T1 * T2 * pow(T3, 2) + 12 * T1 * T2 * T3 * T4 + 4 * T1 * T2 * pow(T4, 2) + 2 * T1 * pow(T3, 3) + 4 * T1 * pow(T3, 2) * T4 + 2 * T1 * T3 * pow(T4, 2) + 6 * pow(T2, 2) * pow(T3, 2) + 9 * pow(T2, 2) * T3 * T4 + 3 * pow(T2, 2) * pow(T4, 2) + 3 * T2 * pow(T3, 3) + 6 * T2 * pow(T3, 2) * T4 + 3 * T2 * T3 * pow(T4, 2))) / (T2 * pow(T3, 2) * pow(T4, 2) * pow(T5, 2) * (T0 * T1 + T0 * T2 + pow(T1, 2) + 2 * T1 * T2 + pow(T2, 2)));
//     // Segment 3:
//     double jCP0_3 = 3 * (T3 * T4 * (4 * Af * T3 * T4 * pow(T5, 2) + 3 * Af * T3 * pow(T5, 3) + 2 * Af * pow(T4, 2) * pow(T5, 2) + 2 * Af * T4 * pow(T5, 3) + 12 * Pf * T3 * T4 + 18 * Pf * T3 * T5 + 6 * Pf * pow(T4, 2) + 12 * Pf * T4 * T5 - 12 * T3 * T4 * T5 * Vf - 12 * T3 * pow(T5, 2) * Vf - 6 * pow(T4, 2) * T5 * Vf - 8 * T4 * pow(T5, 2) * Vf) - 6 * T3 * d5 * (2 * T3 * pow(T4, 2) + 3 * T3 * T4 * T5 + T3 * pow(T5, 2) + pow(T4, 3) + 2 * pow(T4, 2) * T5 + T4 * pow(T5, 2)) - 2 * pow(T4, 2) * pow(T5, 2) * d3 + 2 * pow(T5, 2) * d4 * (3 * pow(T3, 2) + 3 * T3 * T4 + pow(T4, 2))) / (pow(T3, 3) * pow(T4, 2) * pow(T5, 2));
//     // Segment 4:
//     double jCP0_4 = 3 * (T4 * (-2 * Af * T4 * pow(T5, 2) - Af * pow(T5, 3) - 6 * Pf * T4 - 6 * Pf * T5 + 6 * T4 * T5 * Vf + 4 * pow(T5, 2) * Vf) - 2 * pow(T5, 2) * d4 + 2 * d5 * (3 * pow(T4, 2) + 3 * T4 * T5 + pow(T5, 2))) / (pow(T4, 3) * pow(T5, 2));
//     // Segment 5:
//     double jCP0_5 = 3 * (Af * pow(T5, 2) + 2 * Pf - 2 * T5 * Vf - 2 * d5) / pow(T5, 3);

//     // Add control points to the list
//     j_cp_.push_back({jCP0_0, jCP0_1, jCP0_2, jCP0_3, jCP0_4, jCP0_5});
//   }
// }

void SolverGurobi::removeVars()
{
  GRBVar *vars = 0;
  vars = m_.getVars();
  for (int i = 0; i < m_.get(GRB_IntAttr_NumVars); ++i)
  {
    m_.remove(vars[i]);
  }
  x_.clear();
  x_double_.clear();
}

// Given the time in the whole trajectory, find the interval in which the time is located and the dt within the interval
void SolverGurobi::findIntervalIdxAndDt(double time_in_whole_traj, int &interval_idx, double &dt_interval)
{

  // Find the interval and the time within the interval
  double dt_sum = 0.0;
  for (int interval = 0; interval < N_; interval++)
  {

    dt_sum += dt_[interval];

    if (time_in_whole_traj < dt_sum)
    {
      interval_idx = interval;
      break;
    }
  }

  if (interval_idx == 0)
  {
    dt_interval = time_in_whole_traj;
  }
  else
  {
    dt_interval = time_in_whole_traj - (dt_sum - dt_[interval_idx]);
  }
}

void SolverGurobi::setObjective(const vec_Vecf<3> &global_path)
{
  GRBQuadExpr control_cost = 0.0;
  GRBQuadExpr dynamic_cost = 0.0;
  GRBQuadExpr final_pos_cost = 0.0;
  GRBQuadExpr final_vel_cost = 0.0;
  GRBQuadExpr final_accel_cost = 0.0;
  GRBQuadExpr final_yaw_cost = 0.0;
  GRBQuadExpr total_yaw_diff_cost = 0.0;
  GRBQuadExpr ref_point_distance_cost = 0.0;

  // Final state cost
  if (!use_hard_constr_for_final_state_)
  {
    std::vector<GRBLinExpr> x_final_pos = {getPos(N_ - 1, dt_.back(), 0), getPos(N_ - 1, dt_.back(), 1), getPos(N_ - 1, dt_.back(), 2)};
    std::vector<GRBLinExpr> x_final_vel = {getVel(N_ - 1, dt_.back(), 0), getVel(N_ - 1, dt_.back(), 1), getVel(N_ - 1, dt_.back(), 2)};
    std::vector<GRBLinExpr> x_final_accel = {getAccel(N_ - 1, dt_.back(), 0), getAccel(N_ - 1, dt_.back(), 1), getAccel(N_ - 1, dt_.back(), 2)};

    std::vector<double> xf_pos(std::begin(xf_), std::begin(xf_) + 3);
    std::vector<double> xf_vel(std::begin(xf_) + 3, std::begin(xf_) + 6);
    std::vector<double> xf_accel(std::begin(xf_) + 6, std::begin(xf_) + 9);

    final_pos_cost = GetNorm2(x_final_pos - xf_pos);
    final_vel_cost = GetNorm2(x_final_vel - xf_vel);
    final_accel_cost = GetNorm2(x_final_accel - xf_accel);
  }

  // Dynamic constraints
  if (!use_hard_dynamic_constraints_)
  {

    for (int t = 0; t < N_; t++)
    {
      std::vector<GRBLinExpr> x_vel = {getVel(t, 0, 0), getVel(t, 0, 1), getVel(t, 0, 2)};
      std::vector<GRBLinExpr> x_accel = {getAccel(t, 0, 0), getAccel(t, 0, 1), getAccel(t, 0, 2)};

      // Dynamic constraints
      dynamic_cost = dynamic_cost + GetNorm2(x_vel) + GetNorm2(x_accel); // Assuming GetNorm2 calculates the L2 norm squared of the vector x_vel
    }
  }

  // Control input cost
  for (int t = 0; t < N_; t++)
  {
    std::vector<GRBLinExpr> ut = {getJerk(t, 0, 0), getJerk(t, 0, 1), getJerk(t, 0, 2)};
    control_cost = control_cost + GetNorm2(ut); // Assuming GetNorm2 calculates the L2 norm squared of the vector ut
  }

  // Reference point cost: use the mean point of each polytope in polytopes_
  if (use_ref_points_) {
    // Compute reference points as the centroid of each polytope.
    vec_Vecf<3> ref_points;
    // Assume polytopes_ is a member variable (e.g. std::vector<LinearConstraint3D> or Polyhedron3D)
    for (const auto &poly : polytopes_) 
    {
      Vec3f ref_point;
      bool tmp = poly.getMeanPoint(ref_point); // TODO: maybe if we couldn't find mean point, we skip? does it ever fail?
      ref_points.push_back(ref_point);
    }
    
    // Now, for each reference point, compute the cost term.
    // Here we assign a time to each reference point by evenly spacing them along the trajectory.
    // total_traj_time_ is the total time of the trajectory.
    int num_ref_points = ref_points.size();
    for (int i = 1; i < num_ref_points - 1; i++) // skip the first and last polytope (since they have the first and last points)  
    {
      // Compute time t corresponding to this reference point.
      double t = i * total_traj_time_ / static_cast<double>(num_ref_points - 1); // -1 because the nubmer of interval between the points is num_ref_points - 1 
      
      // Determine the interval index and the time delta within that interval.
      int interval_idx = 0;
      double dt_interval = 0;
      findIntervalIdxAndDt(t, interval_idx, dt_interval);
      
      // Form the difference between the decision variable (position at (interval_idx, dt_interval))
      // and the computed reference point.
      std::vector<GRBLinExpr> ref_point_expr = {
        getPos(interval_idx, dt_interval, 0) - ref_points[i](0),
        getPos(interval_idx, dt_interval, 1) - ref_points[i](1),
        getPos(interval_idx, dt_interval, 2) - ref_points[i](2)
      };
      
      // Accumulate the squared L2 norm of the difference.
      ref_point_distance_cost += GetNorm2(ref_point_expr);
    }
  }

  // Set the objective
  m_.setObjective(control_cost_weight_ * control_cost + dynamic_weight_ * dynamic_cost + final_pos_weight_ * final_pos_cost + final_vel_weight_ * final_vel_cost + final_accel_weight_ * final_accel_cost + final_yaw_cost_weight_ * final_yaw_cost + total_yaw_diff_weight_ * total_yaw_diff_cost + ref_point_weight_ * ref_point_distance_cost, GRB_MINIMIZE);
}

void SolverGurobi::setInitialGuess(vec_Vecf<3> global_path, std::vector<double> travel_times)
{

  // We will only need num_N reference points
  if (global_path.size() > N_ + 1) // + 1 because path has one more points than the number of polytopes
  {
    global_path.erase(global_path.begin() + N_ + 1, global_path.end());
    travel_times.erase(travel_times.begin() + N_ + 1, travel_times.end());
  }

  // Find the reference points along the global path
  double factor = 0.25;
  Vec3f ref_point = global_path[global_path.size() - 2] + factor * (global_path[global_path.size() - 1] - global_path[global_path.size() - 2]);

  // For each axis
  for (int i = 0; i < 3; i++)
  {
    // Find the a, b, c, d coefficients for the cubic polynomial
    double a, b, c, d;
    findInitialGuessABCDFromRefPoints(a, b, c, d, ref_point[i], ref_point[i], ref_point[i], ref_point[i], travel_times[N_ - 1]);

    // Only d3 is needed
    d3_[i].set(GRB_DoubleAttr_Start, d);
  }
}

void SolverGurobi::findInitialGuessABCDFromRefPoints(double &a, double &b, double &c, double &d, double q0, double q1, double q2, double q3, double dt)
{

  // Closed-form solution for the cubic polynomial coefficients:
  double d_norm = q0; // d = q0
  double a_norm = -9.0 / 2.0 * q0 + (27.0 / 2.0) * q1 - 27.0 / 2.0 * q2 + (9.0 / 2.0) * q3;
  double b_norm = 9 * q0 - 45.0 / 2.0 * q1 + 18 * q2 - 9.0 / 2.0 * q3;
  double c_norm = -11.0 / 2.0 * q0 + 9 * q1 - 9.0 / 2.0 * q2 + q3;

  // Normalize the coefficients
  a = a_norm / std::pow(dt, 3);
  b = b_norm / std::pow(dt, 2);
  c = c_norm / dt;
  d = d_norm;
}

dynTraj SolverGurobi::adjustTrajTime(const dynTraj &traj)
{

  dynTraj adjusted_traj = traj;

  for (int i = 0; i < adjusted_traj.pwp.times.size(); i++)
  {
    adjusted_traj.pwp.times[i] = adjusted_traj.pwp.times[i] - adjusted_traj.pwp.times[0];
  }

  return adjusted_traj;
}

void SolverGurobi::findClosestIndexFromTime(const double t, int &index, const std::vector<double> &time)
{
  // Find the closest index from the time vector
  double min_diff = std::numeric_limits<double>::max();
  for (int i = 0; i < time.size(); i++)
  {
    double diff = std::abs(time[i] - t);
    if (diff < min_diff)
    {
      min_diff = diff;
      index = i;
    }
  }
}

void SolverGurobi::checkDynamicViolation(bool &is_dyn_constraints_satisfied)
{

  // Check if the time allocation satisfies the dynamic constraints
  is_dyn_constraints_satisfied = true;
  for (int segment = 0; segment < N_; segment++)
  {
    /// velocity
    Eigen::Matrix<double, 3, 3> Vn_prime = getMinvoVelControlPointsDouble(segment);

    // for control points (colunmns of Vn_prime)
    for (int i = 0; i < 3; i++)
    {
      if (Vn_prime.col(i)[0] < -v_max_ || Vn_prime.col(i)[0] > v_max_ ||
          Vn_prime.col(i)[1] < -v_max_ || Vn_prime.col(i)[1] > v_max_ ||
          Vn_prime.col(i)[2] < -v_max_ || Vn_prime.col(i)[2] > v_max_)
      {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// acceleration
    Eigen::Matrix<double, 3, 2> Vn_double_prime = getMinvoAccelControlPointsDouble(segment);

    // for control points (colunmns of Vn_double_prime)
    for (int i = 0; i < 2; i++)
    {
      if (Vn_double_prime.col(i)[0] < -a_max_ || Vn_double_prime.col(i)[0] > a_max_ ||
          Vn_double_prime.col(i)[1] < -a_max_ || Vn_double_prime.col(i)[1] > a_max_ ||
          Vn_double_prime.col(i)[2] < -a_max_ || Vn_double_prime.col(i)[2] > a_max_)
      {
        is_dyn_constraints_satisfied = false;
        break;
      }
    }

    /// jerk
    Eigen::Matrix<double, 3, 1> Vn_triple_prime = getMinvoJerkControlPointsDouble(segment);

    // for control points (colunmns of Vn_triple_prime)
    if (Vn_triple_prime.col(0)[0] < -j_max_ || Vn_triple_prime.col(0)[0] > j_max_ ||
        Vn_triple_prime.col(0)[1] < -j_max_ || Vn_triple_prime.col(0)[1] > j_max_ ||
        Vn_triple_prime.col(0)[2] < -j_max_ || Vn_triple_prime.col(0)[2] > j_max_)
    {
      is_dyn_constraints_satisfied = false;
      break;
    }

  } // end for segment
}

void SolverGurobi::checkCollisionViolation(bool &is_collision_free_corridor_satisfied)
{

  is_collision_free_corridor_satisfied = true;
  for (int t = 0; t < N_; t++)
  {                                 // Loop through each segment
    bool segment_satisfied = false; // Will be true if at least one polytope contains this segment

    // Retrieve the closed-form MINVO position control points for segment t.
    // Here we assume the function returns a 3x4 matrix (each column is a control point)
    Eigen::Matrix<double, 3, 4> cps = getMinvoPosControlPointsDouble(t);

    // Check every polytope until one is found that contains the segment
    // for (const auto &polytope : polytopes_)
    for (int polytope_idx = 0; polytope_idx < polytopes_.size(); polytope_idx++)
    {

      // Assume this polytope works until proven otherwise
      bool polytope_satisfied = true; 

      // Check every control point in the segment.
      // (You could also check the convex hull of cps, but here we simply test all cps.)
      for (int j = 0; j < cps.cols(); j++)
      {
        if (!polytopes_[polytope_idx].inside(cps.col(j)))
        {
          polytope_satisfied = false;
          break; // This polytope fails for this constraint
        }
      }

      // If this polytope satisfied all constraints for all control points, mark segment as satisfied.
      if (polytope_satisfied)
      {
        segment_satisfied = true;
        break; // No need to check other polytopes for this segment
      }
    } // end polytope loop

    // If the segment was not contained in any polytope, flag the overall corridor as unsatisfied.
    if (!segment_satisfied)
    {
      is_collision_free_corridor_satisfied = false;
      break; // Early exit: one segment failed, so overall constraint is not satisfied.
    }
  }
}

/**
 * @brief Find the closest index from the time vector
 */
bool SolverGurobi::findClosedFormSolution()
{

  // Find the closed-form solution for each axis
  std::vector<double> p0, v0, a0, pf, vf, af;
  for (int axis = 0; axis < 3; axis++)
  {
    p0.push_back(x0_[axis]);
    v0.push_back(x0_[axis + 3]);
    a0.push_back(x0_[axis + 6]);

    pf.push_back(xf_[axis]);
    vf.push_back(xf_[axis + 3]);
    af.push_back(xf_[axis + 6]);
  }

  // Initialize the variables
  double d0x, c0x, b0x, a0x, d1x, c1x, b1x, a1x, d2x, c2x, b2x, a2x;
  double d0y, c0y, b0y, a0y, d1y, c1y, b1y, a1y, d2y, c2y, b2y, a2y;
  double d0z, c0z, b0z, a0z, d1z, c1z, b1z, a1z, d2z, c2z, b2z, a2z;

  // We adjust the time allocation to satisfy the dynamic constraints

  // we first go down a bit to test limits of time allocation
  // closed_form_factor_ -= closed_form_factor_initial_decrement_;
  closed_form_factor_ = closed_form_initial_factor_;

  bool is_dyn_constraints_satisfied = false;
  bool is_collision_free_corridor_satisfied = false;

  for (int time_allocation_adj_iter = 0; time_allocation_adj_iter < closed_form_time_allocation_adj_iter_max_; time_allocation_adj_iter++)
  {

    findDTForClosedForm(closed_form_factor_);

    // Find the time allocation
    double T1 = dt_[0];
    double T2 = dt_[1];
    double T3 = dt_[2];

    // Find the closed-form solution for each axis
    findClosedFormSolutionForEachAxis(p0[0], v0[0], a0[0], pf[0], vf[0], af[0], T1, T2, T3, d0x, c0x, b0x, a0x, d1x, c1x, b1x, a1x, d2x, c2x, b2x, a2x);
    findClosedFormSolutionForEachAxis(p0[1], v0[1], a0[1], pf[1], vf[1], af[1], T1, T2, T3, d0y, c0y, b0y, a0y, d1y, c1y, b1y, a1y, d2y, c2y, b2y, a2y);
    findClosedFormSolutionForEachAxis(p0[2], v0[2], a0[2], pf[2], vf[2], af[2], T1, T2, T3, d0z, c0z, b0z, a0z, d1z, c1z, b1z, a1z, d2z, c2z, b2z, a2z);

    // Set the solution
    x_double_.clear();
    x_double_.push_back({a0x, b0x, c0x, d0x, a1x, b1x, c1x, d1x, a2x, b2x, c2x, d2x});
    x_double_.push_back({a0y, b0y, c0y, d0y, a1y, b1y, c1y, d1y, a2y, b2y, c2y, d2y});
    x_double_.push_back({a0z, b0z, c0z, d0z, a1z, b1z, c1z, d1z, a2z, b2z, c2z, d2z});

    // Check if x_double_ has any NaN values
    bool has_nan = false;
    for (int i = 0; i < x_double_.size(); i++) // for each axis
    {
      for (int j = 0; j < x_double_[i].size(); j++) // for each coefficient
      {
        if (std::isnan(x_double_[i][j]))
        {
          std::cout << "x_double_ has NaN values" << std::endl;
          has_nan = true;
          break;
        }
      }
      if (has_nan)
        break;
    }

    if (has_nan)
      break;

    // Check if the dynamic constraints and the collision-free corridor constraints are satisfied
    checkDynamicViolation(is_dyn_constraints_satisfied);
    checkCollisionViolation(is_collision_free_corridor_satisfied);

    // If both the dynamic constraints and the collision-free corridor constraints are satisfied, break the loop
    if (is_dyn_constraints_satisfied && is_collision_free_corridor_satisfied)
    {
      break;
    }

    // If the constraints are not satisfied, adjust the time allocation for the next iteration
    closed_form_factor_ += closed_form_factor_increment_;
  }

  // If after the maximum number of iterations, the dynamic constraints are still not satisfied, return false
  if (!is_dyn_constraints_satisfied)
  {
    if (debug_verbose_)
      std::cout << "Dynamic constraints are not satisfied" << std::endl;
    return false;
  }

  // If after the maximum number of iterations, the collision-free corridor constraints are still not satisfied, return false
  if (!is_collision_free_corridor_satisfied)
  {
    if (debug_verbose_)
      std::cout << "Collision-free corridor constraints are not satisfied" << std::endl;
    return false;
  }

  // Success
  initializeGoalSetpoints();
  return true;
}

// ----------------------------------------------------------------------------

/**
 * @brief Find a closed-form solution for each axis.
 */
void SolverGurobi::findClosedFormSolutionForEachAxis(double p0, double v0, double a0, double pf, double vf, double af, double T1, double T2, double T3, double &d1, double &c1, double &b1, double &a1, double &d2, double &c2, double &b2, double &a2, double &d3, double &c3, double &b3, double &a3)
{
  // Segment 1:
  d1 = p0;
  c1 = v0;
  b1 = (1.0 / 2.0) * a0;
  a1 = (1.0 / 6.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (T1 * (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3));
  // Segment 2:
  d2 = ((1.0 / 3.0) * pow(T1, 3) * T2 * a0 + (1.0 / 6.0) * pow(T1, 3) * T3 * a0 + (1.0 / 3.0) * pow(T1, 2) * pow(T2, 2) * a0 + (1.0 / 3.0) * pow(T1, 2) * T2 * T3 * a0 + (1.0 / 6.0) * pow(T1, 2) * T2 * T3 * af + (4.0 / 3.0) * pow(T1, 2) * T2 * v0 - 1.0 / 3.0 * pow(T1, 2) * T2 * vf + (1.0 / 6.0) * pow(T1, 2) * pow(T3, 2) * af + (2.0 / 3.0) * pow(T1, 2) * T3 * v0 - 2.0 / 3.0 * pow(T1, 2) * T3 * vf + pow(T1, 2) * pf + T1 * pow(T2, 2) * v0 + T1 * T2 * T3 * v0 + 2 * T1 * T2 * p0 + T1 * T3 * p0 + pow(T2, 2) * p0 + T2 * T3 * p0) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  c2 = (-1.0 / 2.0 * pow(T1, 3) * a0 - 2 * pow(T1, 2) * v0 + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 + (1.0 / 2.0) * T1 * T2 * T3 * af - T1 * T2 * vf + (1.0 / 2.0) * T1 * pow(T3, 2) * af - 2 * T1 * T3 * vf - 3 * T1 * p0 + 3 * T1 * pf + pow(T2, 2) * v0 + T2 * T3 * v0) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  b2 = (1.0 / 2.0) * a0 + (1.0 / 2.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  a2 = ((1.0 / 6.0) * pow(T1, 3) * a0 + (2.0 / 3.0) * pow(T1, 2) * T2 * a0 + (1.0 / 3.0) * pow(T1, 2) * T3 * a0 - 1.0 / 6.0 * pow(T1, 2) * T3 * af + (2.0 / 3.0) * pow(T1, 2) * v0 + (1.0 / 3.0) * pow(T1, 2) * vf + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + 2 * T1 * T2 * v0 + T1 * T2 * vf + (1.0 / 6.0) * T1 * pow(T3, 2) * a0 - 1.0 / 3.0 * T1 * pow(T3, 2) * af + T1 * T3 * v0 + T1 * T3 * vf + T1 * p0 - T1 * pf - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * v0 + pow(T2, 2) * vf - 2.0 / 3.0 * T2 * pow(T3, 2) * af + T2 * T3 * v0 + 2 * T2 * T3 * vf + 2 * T2 * p0 - 2 * T2 * pf - 1.0 / 6.0 * pow(T3, 3) * af + (1.0 / 3.0) * pow(T3, 2) * v0 + (2.0 / 3.0) * pow(T3, 2) * vf + T3 * p0 - T3 * pf) / (T2 * (pow(T1, 2) * T2 + pow(T1, 2) * T3 + 2 * T1 * pow(T2, 2) + 3 * T1 * T2 * T3 + T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)));
  // Segment 3:
  d3 = ((1.0 / 6.0) * pow(T1, 2) * pow(T3, 2) * a0 + (1.0 / 6.0) * T1 * T2 * pow(T3, 2) * a0 + (1.0 / 3.0) * T1 * T2 * pow(T3, 2) * af - T1 * T2 * T3 * vf + T1 * T2 * pf + (1.0 / 6.0) * T1 * pow(T3, 3) * af + (2.0 / 3.0) * T1 * pow(T3, 2) * v0 - 2.0 / 3.0 * T1 * pow(T3, 2) * vf + T1 * T3 * pf + (1.0 / 3.0) * pow(T2, 2) * pow(T3, 2) * af - pow(T2, 2) * T3 * vf + pow(T2, 2) * pf + (1.0 / 3.0) * T2 * pow(T3, 3) * af + (1.0 / 3.0) * T2 * pow(T3, 2) * v0 - 4.0 / 3.0 * T2 * pow(T3, 2) * vf + 2 * T2 * T3 * pf + pow(T3, 2) * p0) / (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2));
  c3 = (-1.0 / 2.0 * pow(T1, 2) * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + T1 * T2 * vf - 2 * T1 * T3 * v0 - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * vf - T2 * T3 * v0 + (1.0 / 2.0) * pow(T3, 3) * af - 2 * pow(T3, 2) * vf - 3 * T3 * p0 + 3 * T3 * pf) / (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2));
  b3 = (1.0 / 2.0) * a0 + 3 * ((1.0 / 6.0) * pow(T1, 3) * a0 + (2.0 / 3.0) * pow(T1, 2) * T2 * a0 + (1.0 / 3.0) * pow(T1, 2) * T3 * a0 - 1.0 / 6.0 * pow(T1, 2) * T3 * af + (2.0 / 3.0) * pow(T1, 2) * v0 + (1.0 / 3.0) * pow(T1, 2) * vf + (1.0 / 2.0) * T1 * pow(T2, 2) * a0 + (1.0 / 2.0) * T1 * T2 * T3 * a0 - 1.0 / 2.0 * T1 * T2 * T3 * af + 2 * T1 * T2 * v0 + T1 * T2 * vf + (1.0 / 6.0) * T1 * pow(T3, 2) * a0 - 1.0 / 3.0 * T1 * pow(T3, 2) * af + T1 * T3 * v0 + T1 * T3 * vf + T1 * p0 - T1 * pf - 1.0 / 2.0 * pow(T2, 2) * T3 * af + pow(T2, 2) * v0 + pow(T2, 2) * vf - 2.0 / 3.0 * T2 * pow(T3, 2) * af + T2 * T3 * v0 + 2 * T2 * T3 * vf + 2 * T2 * p0 - 2 * T2 * pf - 1.0 / 6.0 * pow(T3, 3) * af + (1.0 / 3.0) * pow(T3, 2) * v0 + (2.0 / 3.0) * pow(T3, 2) * vf + T3 * p0 - T3 * pf) / (pow(T1, 2) * T2 + pow(T1, 2) * T3 + 2 * T1 * pow(T2, 2) + 3 * T1 * T2 * T3 + T1 * pow(T3, 2) + pow(T2, 3) + 2 * pow(T2, 2) * T3 + T2 * pow(T3, 2)) + (1.0 / 2.0) * (-3 * pow(T1, 2) * a0 - 4 * T1 * T2 * a0 - 2 * T1 * T3 * a0 - 6 * T1 * v0 - pow(T2, 2) * a0 - T2 * T3 * a0 + T2 * T3 * af - 4 * T2 * v0 - 2 * T2 * vf + pow(T3, 2) * af - 2 * T3 * v0 - 4 * T3 * vf - 6 * p0 + 6 * pf) / (pow(T1, 2) + 2 * T1 * T2 + T1 * T3 + pow(T2, 2) + T2 * T3);
  a3 = (1.0 / 6.0) * (-pow(T1, 2) * a0 - T1 * T2 * a0 + T1 * T2 * af + 2 * T1 * T3 * af - 4 * T1 * v0 - 2 * T1 * vf + pow(T2, 2) * af + 4 * T2 * T3 * af - 2 * T2 * v0 - 4 * T2 * vf + 3 * pow(T3, 2) * af - 6 * T3 * vf - 6 * p0 + 6 * pf) / (T3 * (T1 * T2 + T1 * T3 + pow(T2, 2) + 2 * T2 * T3 + pow(T3, 2)));
}

void SolverGurobi::fillGoalSetPoints()
{
  const int num_goal_setpoints = goal_setpoints_.size();

  // Precompute timestamps
  std::vector<double> timestamps(num_goal_setpoints);
  for (int i = 0; i < num_goal_setpoints; i++)
    timestamps[i] = (i + 1) * DC;

  #pragma omp parallel for
  for (int i = 0; i < num_goal_setpoints; i++)
  {
    // Get the timestamp
    double t = timestamps[i];

    // Find interval index and dt_interval (assumed thread-safe)
    int interval_idx = 0; // initialize to 0
    double dt_interval = 0; // initialize to 0
    findIntervalIdxAndDt(t, interval_idx, dt_interval);

    // Compute positions
    double posx = getPosDouble(interval_idx, dt_interval, 0);
    double posy = getPosDouble(interval_idx, dt_interval, 1);
    double posz = getPosDouble(interval_idx, dt_interval, 2);

    // Compute velocities
    double velx = getVelDouble(interval_idx, dt_interval, 0);
    double vely = getVelDouble(interval_idx, dt_interval, 1);
    double velz = getVelDouble(interval_idx, dt_interval, 2);

    // Compute accelerations
    double accelx = getAccelDouble(interval_idx, dt_interval, 0);
    double accely = getAccelDouble(interval_idx, dt_interval, 1);
    double accelz = getAccelDouble(interval_idx, dt_interval, 2);

    // Compute jerks
    double jerkx = getJerkDouble(interval_idx, dt_interval, 0);
    double jerky = getJerkDouble(interval_idx, dt_interval, 1);
    double jerkz = getJerkDouble(interval_idx, dt_interval, 2);

    // Set the state (assuming thread-safe)
    state state_i;
    state_i.setTimeStamp(t0_ + t);
    state_i.setPos(posx, posy, posz);
    state_i.setVel(velx, vely, velz);
    state_i.setAccel(accelx, accely, accelz);
    state_i.setJerk(jerkx, jerky, jerkz);

    // Set the state
    goal_setpoints_[i] = state_i;
  }

  // Ensure the final input is explicitly zeroed (serially after parallel loop)
  goal_setpoints_[num_goal_setpoints - 1].vel   = Eigen::Vector3d::Zero().transpose();
  goal_setpoints_[num_goal_setpoints - 1].accel = Eigen::Vector3d::Zero().transpose();
  goal_setpoints_[num_goal_setpoints - 1].jerk  = Eigen::Vector3d::Zero().transpose();
}

void SolverGurobi::getGoalSetpoints(std::vector<state> &goal_setpoints)
{
  goal_setpoints = goal_setpoints_;
}

void SolverGurobi::setPolytopes(std::vector<LinearConstraint3D> polytopes, bool use_closed_form)
{

  // Set polytopes
  polytopes_ = polytopes;

  // Set polytopes size
  current_polytopes_size_ = polytopes.size();

  // If this is for closed_form, we don't care about the relations between N and P
  if (use_closed_form)
    return;

  // If N_ > current_polytopes_size_, set use_miqp_ to true
  if (N_ > current_polytopes_size_)
  {
    use_miqp_ = true;
  }
  else if (N_ == current_polytopes_size_)
  {
    use_miqp_ = false;
  }
  else
  {
    std::cerr << "Error: N_ cannot be smaller than the number of polytopes. N=" << N_ << ", polytopes size=" << current_polytopes_size_ << std::endl;
  }
}

void SolverGurobi::setMapSizeConstraints()
{

  // Remove previous map constraints
  if (!map_cons_.empty())
  {
    for (int i = 0; i < map_cons_.size(); i++)
    {
      m_.remove(map_cons_[i]);
    }
    map_cons_.clear();
  }

  for (int t = 0; t < N_; t++)
  {
    // Set the constraints for the map size (especially for z_min_ and z_max_)
    // auto minvo_cps = getMinvoPosControlPoints(t);
    // std::vector<GRBLinExpr> cp0 = minvo_cps[0];
    // std::vector<GRBLinExpr> cp1 = minvo_cps[1];
    // std::vector<GRBLinExpr> cp2 = minvo_cps[2];
    // std::vector<GRBLinExpr> cp3 = minvo_cps[3];
    std::vector<GRBLinExpr> cp0 = getCP0(t);
    std::vector<GRBLinExpr> cp1 = getCP1(t);
    std::vector<GRBLinExpr> cp2 = getCP2(t);
    std::vector<GRBLinExpr> cp3 = getCP3(t);

    // Set the constraints for the map size (especially for z_min_ and z_max_)

    // cp0 <= z_max_
    map_cons_.push_back(m_.addConstr(cp0[2] <= z_max_ - 2 * res_, "Map_size_cp0_z_max_t_" + std::to_string(t)));
    // cp1 <= z_max_
    map_cons_.push_back(m_.addConstr(cp1[2] <= z_max_ - 2 * res_, "Map_size_cp1_z_max_t_" + std::to_string(t)));
    // cp2 <= z_max_
    map_cons_.push_back(m_.addConstr(cp2[2] <= z_max_ - 2 * res_, "Map_size_cp2_z_max_t_" + std::to_string(t)));
    // cp3 <= z_max_
    map_cons_.push_back(m_.addConstr(cp3[2] <= z_max_ - 2 * res_, "Map_size_cp3_z_max_t_" + std::to_string(t)));

    // cp0 >= z_min_
    map_cons_.push_back(m_.addConstr(cp0[2] >= z_min_ + 2 * res_, "Map_size_cp0_z_min_t_" + std::to_string(t)));
    // cp1 >= z_min_
    map_cons_.push_back(m_.addConstr(cp1[2] >= z_min_ + 2 * res_, "Map_size_cp1_z_min_t_" + std::to_string(t)));
    // cp2 >= z_min_
    map_cons_.push_back(m_.addConstr(cp2[2] >= z_min_ + 2 * res_, "Map_size_cp2_z_min_t_" + std::to_string(t)));
    // cp3 >= z_min_
    map_cons_.push_back(m_.addConstr(cp3[2] >= z_min_ + 2 * res_, "Map_size_cp3_z_min_t_" + std::to_string(t)));
  }
}

void SolverGurobi::setPolytopesConstraints()
{

  // Remove previous polytopes constraints
  if (!polytopes_cons_.empty())
  {
    for (int i = 0; i < polytopes_cons_.size(); i++)
    {
      m_.remove(polytopes_cons_[i]);
    }
    polytopes_cons_.clear();
  }

  // Remove previous at_least_1_pol_cons_ constraints
  if (!at_least_1_pol_cons_.empty())
  {
    for (int i = 0; i < at_least_1_pol_cons_.size(); i++)
    {
      m_.remove(at_least_1_pol_cons_[i]);
    }
    at_least_1_pol_cons_.clear();
  }

  // Remove previous binary variables
  if (!b_.empty())
  {
    for (int i = 0; i < b_.size(); i++)
    {
      for (int j = 0; j < b_[i].size(); j++)
      {
        m_.remove(b_[i][j]);
      }
    }
    b_.clear();
  }

  // Remove previous miqp_polytopes_cons_ constraints
  if (!miqp_polytopes_cons_.empty())
  {
    for (int i = 0; i < miqp_polytopes_cons_.size(); i++)
    {
      m_.remove(miqp_polytopes_cons_[i]);
    }
    miqp_polytopes_cons_.clear();
  }

  // Set polytope constraints (either MIQP or DYNUS approach)
  setPolyConsts();
}

void SolverGurobi::setPolyConsts()
{

  if (!polytopes_.empty()) // If there are polytope constraints
  {

    // MIQP approach
    if (use_miqp_)
    {

      // Declare binary variables
      for (int t = 0; t < N_; t++)
      {
        std::vector<GRBVar> row;
        for (int i = 0; i < polytopes_.size(); i++) // For all the polytopes
        {
          GRBVar variable =
              m_.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_BINARY, "s" + std::to_string(i) + "_" + std::to_string(t));
          row.push_back(variable);
        }
        b_.push_back(row);
      }

      // Polytope constraints (if binary_varible==1 --> In that polytope) and at_least_1_pol_cons_ (at least one polytope)
      // constraints
      for (int t = 0; t < N_; t++)
      {
        // At least one polytope for other segments
        createSafeCorridorConstraintsForPolytopeAtleastOne(t);
      }
    }
    else // QP approach
    {
      // constraints
      for (int t = 0; t < N_; t++)
      {
        createSafeCorridorConstraintsForPolytope(t);
      } // End for t

    } // End else DYNUS approach
  }
}

void SolverGurobi::createSafeCorridorConstraintsForPolytope(int t)
{
  // Get MINVO control points
  // auto minvo_cps = getMinvoPosControlPoints(t);
  // std::vector<GRBLinExpr> cp0 = minvo_cps[0];
  // std::vector<GRBLinExpr> cp1 = minvo_cps[1];
  // std::vector<GRBLinExpr> cp2 = minvo_cps[2];
  // std::vector<GRBLinExpr> cp3 = minvo_cps[3];
  std::vector<GRBLinExpr> cp0 = getCP0(t);
  std::vector<GRBLinExpr> cp1 = getCP1(t);
  std::vector<GRBLinExpr> cp2 = getCP2(t);
  std::vector<GRBLinExpr> cp3 = getCP3(t);

  // Constraint Ax<=b
  Eigen::MatrixXd A;
  Eigen::VectorXd bb;

  // If it's the last segment, use the last polytope
  if (t == N_ - 1)
  {
    A = polytopes_[polytopes_.size() - 1].A();
    bb = polytopes_[polytopes_.size() - 1].b();
  }
  else // Otherwise, use the corresponding polytope
  {
    A = polytopes_[t].A();
    bb = polytopes_[t].b();
  }

  // Compute A times control points (x)
  std::vector<std::vector<double>> Astd = eigenMatrix2std(A);
  std::vector<GRBLinExpr> Acp0 = MatrixMultiply(Astd, cp0); // A times control point 0
  std::vector<GRBLinExpr> Acp1 = MatrixMultiply(Astd, cp1); // A times control point 1
  std::vector<GRBLinExpr> Acp2 = MatrixMultiply(Astd, cp2); // A times control point 2
  std::vector<GRBLinExpr> Acp3 = MatrixMultiply(Astd, cp3); // A times control point 3

  // Loop through the number of faces
  for (int i = 0; i < bb.rows(); i++)
  {
    polytopes_cons_.push_back(m_.addConstr(Acp0[i] <= bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp0"));
    polytopes_cons_.push_back(m_.addConstr(Acp1[i] <= bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp1"));
    polytopes_cons_.push_back(m_.addConstr(Acp2[i] <= bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp2"));
    polytopes_cons_.push_back(m_.addConstr(Acp3[i] <= bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp3"));
  } // End for i
}

void SolverGurobi::createSafeCorridorConstraintsForPolytopeAtleastOne(int t)
{

  GRBLinExpr sum = 0;
  for (int col = 0; col < b_[t].size(); col++)
  {
    sum = sum + b_[t][col];
  }
  at_least_1_pol_cons_.push_back(m_.addConstr(sum >= 1, "At_least_1_pol_t_" + std::to_string(t))); // at least in one polytope

  // Get MINVO control points
  // auto minvo_cps = getMinvoPosControlPoints(t);
  // std::vector<GRBLinExpr> cp0 = minvo_cps[0];
  // std::vector<GRBLinExpr> cp1 = minvo_cps[1];
  // std::vector<GRBLinExpr> cp2 = minvo_cps[2];
  // std::vector<GRBLinExpr> cp3 = minvo_cps[3];
  std::vector<GRBLinExpr> cp0 = getCP0(t);
  std::vector<GRBLinExpr> cp1 = getCP1(t);
  std::vector<GRBLinExpr> cp2 = getCP2(t);
  std::vector<GRBLinExpr> cp3 = getCP3(t);

  // Loop over the number of polytopes
  for (int n_poly = 0; n_poly < polytopes_.size(); n_poly++)
  {
    // Constraint A1x<=b1
    Eigen::MatrixXd A1 = polytopes_[n_poly].A();
    auto bb = polytopes_[n_poly].b();

    std::vector<std::vector<double>> A1std = eigenMatrix2std(A1);
    std::vector<GRBLinExpr> Acp0 = MatrixMultiply(A1std, cp0); // A times control point 0
    std::vector<GRBLinExpr> Acp1 = MatrixMultiply(A1std, cp1); // A times control point 1
    std::vector<GRBLinExpr> Acp2 = MatrixMultiply(A1std, cp2); // A times control point 2
    std::vector<GRBLinExpr> Acp3 = MatrixMultiply(A1std, cp3); // A times control point 3

    for (int i = 0; i < bb.rows(); i++)
    {
      // If b_[t,n_poly] == 1, all the control points are in that polytope
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(b_[t][n_poly], 1, Acp0[i], GRB_LESS_EQUAL, bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp0"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(b_[t][n_poly], 1, Acp1[i], GRB_LESS_EQUAL, bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp1"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(b_[t][n_poly], 1, Acp2[i], GRB_LESS_EQUAL, bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp2"));
      miqp_polytopes_cons_.push_back(m_.addGenConstrIndicator(b_[t][n_poly], 1, Acp3[i], GRB_LESS_EQUAL, bb[i], "safe_corridor_interval_" + std::to_string(t) + "_face" + std::to_string(i) + "_cp3"));
    }
  }
}

void SolverGurobi::setDC(double dc)
{
  DC = dc;
}

void SolverGurobi::setX0(const state &data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  x0_[0] = data.pos.x();
  x0_[1] = data.pos.y();
  x0_[2] = data.pos.z();
  x0_[3] = data.vel.x();
  x0_[4] = data.vel.y();
  x0_[5] = data.vel.z();
  x0_[6] = data.accel.x();
  x0_[7] = data.accel.y();
  x0_[8] = data.accel.z();
}

void SolverGurobi::setXf(const state &data)
{
  Eigen::Vector3d pos = data.pos;
  Eigen::Vector3d vel = data.vel;
  Eigen::Vector3d accel = data.accel;

  xf_[0] = data.pos.x();
  xf_[1] = data.pos.y();
  xf_[2] = data.pos.z();
  xf_[3] = data.vel.x();
  xf_[4] = data.vel.y();
  xf_[5] = data.vel.z();
  xf_[6] = data.accel.x();
  xf_[7] = data.accel.y();
  xf_[8] = data.accel.z();
}

void SolverGurobi::setDirf(double yawf)
{
  dirf_[0] = cos(yawf);
  dirf_[1] = sin(yawf);
}

void SolverGurobi::setConstraintsXf()
{
  // Remove previous final constraints
  if (!final_cons_.empty())
  {
    for (int i = 0; i < final_cons_.size(); i++)
    {
      m_.remove(final_cons_[i]);
    }

    final_cons_.clear();
  }

  // Constraint xT==x_final
  for (int i = 0; i < 3; i++)
  {

    if (use_hard_constr_for_final_state_)
    {
      // Final position should be equal to the final position
      final_cons_.push_back(m_.addConstr(getPos(N_ - 1, dt_.back(), i) - xf_[i] == 0, "FinalPosAxis_" + std::to_string(i))); // Final position
      // Final velocity and acceleration should be 0
      final_cons_.push_back(m_.addConstr(getVel(N_ - 1, dt_.back(), i) - xf_[i + 3] == 0, "FinalVelAxis_" + std::to_string(i))); // Final velocity
      final_cons_.push_back(m_.addConstr(getAccel(N_ - 1, dt_.back(), i) - xf_[i + 6] == 0, "FinalAccel_" + std::to_string(i))); // Final acceleration
    }
  }
}

void SolverGurobi::setConstraintsX0()
{
  // Remove previous initial constraints
  if (!init_cons_.empty())
  {
    for (int i = 0; i < init_cons_.size(); i++)
    {
      m_.remove(init_cons_[i]);
    }
    init_cons_.clear();
  }

  // Constraint x0==x_initial
  for (int i = 0; i < 3; i++)
  {
    init_cons_.push_back(m_.addConstr(getPos(0, 0, i) == x0_[i],
                                      "InitialPosAxis_" + std::to_string(i))); // Initial position
                                                                               // std::cout << "Velocity" << std::endl;
    init_cons_.push_back(m_.addConstr(getVel(0, 0, i) == x0_[i + 3],
                                      "InitialVelAxis_" + std::to_string(i))); // Initial velocity
    // std::cout << "Accel" << std::endl;
    init_cons_.push_back(m_.addConstr(getAccel(0, 0, i) == x0_[i + 6],
                                      "InitialAccelAxis_" + std::to_string(i))); // Initial acceleration}
  }
}

void SolverGurobi::initializeGoalSetpoints()
{
  int size = (int)(total_traj_time_ / DC);
  size = (size < 2) ? 2 : size; // force size to be at least 2
  goal_setpoints_.clear();
  goal_setpoints_.resize(size);
}

void SolverGurobi::setDynamicConstraints()
{

  // Remove previous dynamic constraints
  if (!dyn_cons_.empty())
  {
    for (int i = 0; i < dyn_cons_.size(); i++)
    {
      m_.remove(dyn_cons_[i]);
    }
    dyn_cons_.clear();
  }

  // Loop over all segments.
  for (int segment = 0; segment < N_; segment++)
  {
    // Loop over all dimensions (axes).
    for (int axis = 0; axis < 3; axis++)
    {

      // Get the d3 variable
      GRBVar d3_var = d3_[axis];

      // --- Velocity constraints ---
      std::vector<GRBLinExpr> vel_cps = getVelCP(segment, axis);
      // For each velocity control point:
      for (int i = 0; i < vel_cps.size(); i++)
      {
        if (controlPointDepends(VELOCITY, segment, i))
        {
          dyn_cons_.push_back(m_.addConstr(vel_cps[i] <= v_max_, "max_vel_t" + std::to_string(segment) +
                                                                     "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(vel_cps[i] >= -v_max_, "min_vel_t" + std::to_string(segment) +
                                                                      "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
        }
      }

      // --- Acceleration constraints ---
      std::vector<GRBLinExpr> accel_cps = getAccelCP(segment, axis);
      for (int i = 0; i < accel_cps.size(); i++)
      {
        if (controlPointDepends(ACCELERATION, segment, i))
        {
          dyn_cons_.push_back(m_.addConstr(accel_cps[i] <= a_max_, "max_accel_t" + std::to_string(segment) +
                                                                       "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(accel_cps[i] >= -a_max_, "min_accel_t" + std::to_string(segment) +
                                                                        "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
        }
      }

      // --- Jerk constraints ---
      std::vector<GRBLinExpr> jerk_cps = getJerkCP(segment, axis);
      for (int i = 0; i < jerk_cps.size(); i++)
      {
        if (controlPointDepends(JERK, segment, i))
        {
          dyn_cons_.push_back(m_.addConstr(jerk_cps[i] <= j_max_, "max_jerk_t" + std::to_string(segment) +
                                                                      "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
          dyn_cons_.push_back(m_.addConstr(jerk_cps[i] >= -j_max_, "min_jerk_t" + std::to_string(segment) +
                                                                       "_axis_" + std::to_string(axis) + "_cp" + std::to_string(i)));
        }
      }

    } // end for axis
  } // end for segment
}

bool SolverGurobi::controlPointDepends(ConstraintType type, int seg, int cp)
{
  if (N_ == 4)
  {
    return controlPointDependsOnD3(type, seg, cp);
  }
  else if (N_ == 5)
  {
    return controlPointDependsOnD3OrD4(type, seg, cp);
  }
  else if (N_ == 6)
  {
    return controlPointDependsOnD3OrD4OrD5(type, seg, cp);
  }
  else
  {
    std::cerr << "Error: N_ should be 4, 5, or 6." << std::endl;
    return false;
  }
}

// Helper function that returns true if 'expr' contains d3_var with a nonzero coefficient.
// Returns true if the control point of the given type, in segment 'seg' (0-indexed)
// and with control point index 'cp' (also 0-indexed) depends on d3.
bool SolverGurobi::controlPointDependsOnD3(ConstraintType type, int seg, int cp)
{
  switch (type)
  {
  case POSITION:
    // For position control points:
    // Segment 0: CP0, CP1, CP2 are independent, CP3 depends.
    if (seg == 0)
    {
      if (cp == 0 || cp == 1 || cp == 2)
        return false;
      else if (cp == 3)
        return true;
    }
    // Segment 1 and 2: all control points depend.
    if (seg == 1 || seg == 2)
      return true;
    // Segment 3: CP0, CP1, CP2 depend, CP3 independent.
    if (seg == 3)
    {
      if (cp == 3)
        return false;
      else
        return true;
    }
    break;

  case VELOCITY:
    // For velocity control points (three per segment):
    // Segment 0: vCP0 is independent; vCP1 and vCP2 depend.
    if (seg == 0)
    {
      if (cp == 0)
        return false;
      else
        return true;
    }
    // Segment 1 and 2: all depend.
    if (seg == 1 || seg == 2)
      return true;
    // Segment 3: vCP2 is independent; vCP0 and vCP1 depend.
    if (seg == 3)
    {
      if (cp == 2)
        return false;
      else
        return true;
    }
    break;

  case ACCELERATION:
    // For acceleration control points (two per segment):
    // Segment 0: aCP0 is independent; aCP1 depends.
    if (seg == 0)
    {
      if (cp == 0)
        return false;
      else
        return true;
    }
    // Segment 1 and 2: both depend.
    if (seg == 1 || seg == 2)
      return true;
    // Segment 3: aCP1 is independent; aCP0 depends.
    if (seg == 3)
    {
      if (cp == 1)
        return false;
      else
        return true;
    }
    break;

  case JERK:
    // For jerk, there's only one control point per segment, and according to your table all jerk CPs depend on d3.
    return true;
  }
  return false; // Default: if type is not recognized.
}

bool SolverGurobi::controlPointDependsOnD3OrD4(ConstraintType type, int seg, int cp)
{
  switch (type)
  {
  case POSITION:
    if (seg == 0)
    {
      // Segment 0: CP0, CP1, CP2 independent; CP3 depends.
      if (cp == 0 || cp == 1 || cp == 2)
        return false;
      else if (cp == 3)
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: all depend on d3 and d4.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: CP0, CP1, CP2 depend on d3 and d4; CP3 depends on d3.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: CP0 depends on d3; CP1 and CP2 depend on d3 and d4; CP3 depends on d4.
      return true;
    }
    else if (seg == 4)
    {
      // Segment 4: CP0, CP1, CP2 depend on d4; CP3 independent.
      if (cp == 3)
        return false;
      else
        return true;
    }
    break;

  case VELOCITY:
    if (seg == 0)
    {
      // Segment 0: vCP0 independent; vCP1 and vCP2 depend.
      if (cp == 0)
        return false;
      else
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: all depend.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: all depend.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: all depend.
      return true;
    }
    else if (seg == 4)
    {
      // Segment 4: vCP0 and vCP1 depend on d4; vCP2 independent.
      if (cp == 2)
        return false;
      else
        return true;
    }
    break;

  case ACCELERATION:
    if (seg == 0)
    {
      // Segment 0: aCP0 independent; aCP1 depends.
      if (cp == 0)
        return false;
      else
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: both depend.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: both depend.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: aCP0 depends on d3 and d4; aCP1 depends on d4.
      if (cp == 0)
        return true;
      else // cp==1
        return false;
    }
    else if (seg == 4)
    {
      // Segment 4: aCP0 depends on d4; aCP1 independent.
      if (cp == 0)
        return true;
      else // cp==1
        return false;
    }
    break;

  case JERK:
    // For jerk, there is one control point per segment.
    // Segments 0-3: depend on d3 and d4; Segment 4: depends on d4.
    if (seg >= 0 && seg <= 3)
      return true;
    if (seg == 4)
      return true;
    break;
  }
  return false; // Default if type not recognized.
}

bool SolverGurobi::controlPointDependsOnD3OrD4OrD5(ConstraintType type, int seg, int cp)
{
  // seg: segment index (0...5) for N=6.
  // cp: control point index.
  switch (type)
  {
  case POSITION:
    // There are 4 position control points per segment.
    if (seg == 0)
    {
      // Segment 0:
      //   CP0, CP1, CP2 are independent; CP3 depends on d3, d4, d5.
      if (cp >= 0 && cp <= 2)
        return false;
      else if (cp == 3)
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: all CPs depend on d3, d4, d5.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: CP0, CP1, CP2 depend on d3, d4, d5; CP3 depends on d3.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: CP0 depends on d3; CP1 and CP2 depend on d3, d4, d5; CP3 depends on d4.
      return true;
    }
    else if (seg == 4)
    {
      // Segment 4: CP0 depends on d4; CP1 and CP2 depend on d4, d5; CP3 depends on d5.
      return true;
    }
    else if (seg == 5)
    {
      // Segment 5: CP0, CP1, CP2 depend on d5; CP3 is independent.
      if (cp == 3)
        return false;
      else
        return true;
    }
    break;

  case VELOCITY:
    // There are 3 velocity control points per segment.
    if (seg == 0)
    {
      // Segment 0: vCP0 is independent; vCP1 and vCP2 depend on d3, d4, d5.
      if (cp == 0)
        return false;
      else
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: all velocity CPs depend on d3, d4, d5.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: all depend on d3, d4, d5.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: all depend on d3, d4, d5 (even if vCP2 only depends on d4,d5, it still is free).
      return true;
    }
    else if (seg == 4)
    {
      // Segment 4: vCP0 and vCP1 depend on d4, d5; vCP2 is independent.
      if (cp == 2)
        return false;
      else
        return true;
    }
    else if (seg == 5)
    {
      // Segment 5: vCP0 and vCP1 depend on d5; vCP2 is independent.
      if (cp == 2)
        return false;
      else
        return true;
    }
    break;

  case ACCELERATION:
    // There are 2 acceleration control points per segment.
    if (seg == 0)
    {
      // Segment 0: aCP0 is independent; aCP1 depends on d3, d4, d5.
      if (cp == 0)
        return false;
      else
        return true;
    }
    else if (seg == 1)
    {
      // Segment 1: both depend on d3, d4, d5.
      return true;
    }
    else if (seg == 2)
    {
      // Segment 2: both depend on d3, d4, d5.
      return true;
    }
    else if (seg == 3)
    {
      // Segment 3: aCP0 depends on d3, d4, d5; aCP1 depends on d4.
      return true;
    }
    else if (seg == 4)
    {
      // Segment 4: aCP0 depends on d4, d5; aCP1 depends on d5.
      return true;
    }
    else if (seg == 5)
    {
      // Segment 5: aCP0 depends on d5; aCP1 is independent.
      if (cp == 0)
        return true;
      else
        return false;
    }
    break;

  case JERK:
    // There is one jerk control point per segment.
    // Segment 0–3: depend on d3, d4, d5.
    if (seg >= 0 && seg <= 3)
      return true;
    // Segment 4: depends on d4, d5.
    if (seg == 4)
      return true;
    // Segment 5: depends on d5.
    if (seg == 5)
      return true;
    break;
  }
  return false; // Default if type not recognized.
}

void SolverGurobi::setBounds(double max_values[3])
{
  v_max_ = max_values[0];
  a_max_ = max_values[1];
  j_max_ = max_values[2];
}

void SolverGurobi::setTimeAllocationParameters(double factor_initial, double factor_final, double factor_gamma_up, double factor_gamma_down, double factor_constant_step_size, double factor_minimum, double factor_delta_step_size, bool use_constant_step_size, int count_to_switch_to_constant_step_size)
{
  // Dynamically change these parameters
  factor_initial_ = factor_initial;
  factor_final_ = factor_final;
  factor_gamma_up_ = factor_gamma_up;
  factor_gamma_down_ = factor_gamma_down;
  factor_constant_step_size_ = factor_constant_step_size;
  factor_minimum_ = factor_minimum;
  factor_delta_step_size_ = factor_delta_step_size;
  use_constant_step_size_ = use_constant_step_size;
  count_to_switch_to_constant_step_size_ = count_to_switch_to_constant_step_size;

  // Save the original values
  original_factor_initial_ = factor_initial;
  original_factor_final_ = factor_final;
  original_factor_gamma_up_ = factor_gamma_up;
  original_factor_gamma_down_ = factor_gamma_down;
}

bool SolverGurobi::generateNewTrajectoryWithIteration(bool &gurobi_error_detected, const vec_Vecf<3> &global_path, double& gurobi_computation_time)
{

  if (N_ == 3) // If N_ = 3, then we will use the closed-form solution with given time
    return findClosedFormSolution();

  // If N_ >= 4, then we will solve optimization
  bool solved = false;
  double factor = factor_initial_;
  int iter = 0;
  double step_size;

  // Loop over the factor
  while (!solved && factor <= factor_final_)
  {

    // Determine the step size
    if (use_constant_step_size_)
      step_size = factor_constant_step_size_; // Get the constant step size
    else
      step_size = iter * factor_delta_step_size_; // Compute (incrementally increasing) step size

    // Increase factor
    factor = factor + step_size;

    // Catch the gurobi error
    try
    {

      // find the new dt using the factor
      findDT(factor);

      // Debug print
      if (debug_verbose_)
      {
        std::cout << "Try with factor= " << factor << std::endl;
        for (int j = 0; j < dt_.size(); j++)
        {
          std::cout << "dt[" << j << "]=" << dt_[j] << std::endl;
        }
      }

      // Create constraints
      setX();                       // creates variables
      if (optimization_type_ == "hard" || optimization_type_ == "soft_with_check")
        setPolytopesConstraints();  // creates polytopes constraints
      if (use_hard_dynamic_constraints_)
        setDynamicConstraints();    // creates dynamic constraints
      setObjective(global_path);    // creates objective function
      if (vehicle_type_ == "uav")
        setMapSizeConstraints();
      // Find initial guess
      // setInitialGuess(global_path, travel_times);

      // Call the optimizer
      solved = callOptimizer();

      if (solved)
      {

        // Get gurobi solve time
        gurobi_computation_time = m_.get(GRB_DoubleAttr_Runtime) * 1000;

        // Housekeeping
        initializeGoalSetpoints();      // initialize goal setpoints for output

        if (debug_verbose_)
          std::cout << "Factor= " << factor << " worked" << std::endl;

        // Dynamically adjust factor_initial_ and factor_final_ based on the last successful factor
        dynamicallyAdjustFactorInitialAndFinal(factor);

        // Get the solution (update x_double_)
        if (N_ == 4)
        {
          getDependentCoefficientsN4Double();
        }
        else if (N_ == 5)
        {
          getDependentCoefficientsN5Double();
        }
        else if (N_ == 6)
        {
          getDependentCoefficientsN6Double();
        }

        // If the optimization type is "soft_with_check", then check if the constraints are satisfied
        if (optimization_type_ == "soft_with_check")
        {

          // Check if the constraints are satisfied
          bool is_collision_free_corridor_satisfied = false;
          checkCollisionViolation(is_collision_free_corridor_satisfied);
          if (is_collision_free_corridor_satisfied)
          {
            solved = true;
            if (debug_verbose_)
              std::cout << "[soft_with_check] Constraints are satisfied" << std::endl;
          }
          else
          {
            solved = false;
            if (debug_verbose_)
              std::cout << "[soft_with_check] Constraints are not satisfied" << std::endl;
          }
        }

      }
    }
    catch (GRBException e)
    {
      // std::cout << "Error code = " << e.getErrorCode() << std::endl;
      // std::cout << e.getMessage() << std::endl;
      // std::cout << "Factor= " << factor << std::endl;
      gurobi_error_detected = true;

      // Write the model to a file
      // m_.write("model.lp");

    } // End of try-catch

    iter = iter + 1;

  } // End of while

  // if (!solved)
  // {
  //   // If the replanning failed, reset all the parameters to their original values and if you keep failing, then widen the search space
  //   // changeTimeAllocationParamsInCaseOfFailure();
  // }

  return solved;
}

bool SolverGurobi::generateNewTrajectoryWithFactor(bool &gurobi_error_detected,
                                                   const vec_Vecf<3> &global_path,
                                                   double &gurobi_computation_time,
                                                   double factor)
{
  // For N_ = 3, use the closed-form solution.
  if (N_ == 3)
    return findClosedFormSolution();

  bool solved = false;

  try
  {
    // Check if cancellation was requested before starting.
    if (cb_.should_terminate_)
    {
      // std::cout << "Cancellation requested before optimization started." << std::endl;
      return false;
    }

    // Find the new dt using the factor.
    findDT(factor);
    setX(); // creates variables
    if (optimization_type_ == "hard" || optimization_type_ == "soft_with_check")
      setPolytopesConstraints(); // create polytopes constraints
    if (use_hard_dynamic_constraints_)
      setDynamicConstraints(); // create dynamic constraints
    setObjective(global_path); // create objective function
    if (vehicle_type_ == "uav")
      setMapSizeConstraints(); // create map size constraints for z axis
    
    // Check again before calling the optimizer.
    if (cb_.should_terminate_)
    {
      // std::cout << "Cancellation requested before calling optimizer." << std::endl;
      return false;
    }

    // Call the optimizer.
    solved = callOptimizer();

    if (solved)
    {
      // Get Gurobi solve time.
      gurobi_computation_time = m_.get(GRB_DoubleAttr_Runtime) * 1000;

      // Housekeeping: initialize goal setpoints for output.
      initializeGoalSetpoints();

      // Retrieve the solution (update x_double_).
      if (N_ == 4)
        getDependentCoefficientsN4Double();
      else if (N_ == 5)
        getDependentCoefficientsN5Double();
      else if (N_ == 6)
        getDependentCoefficientsN6Double();

      // If the optimization type is "soft_with_check", verify constraints.
      if (optimization_type_ == "soft_with_check")
      {
        bool is_collision_free_corridor_satisfied = false;
        checkCollisionViolation(is_collision_free_corridor_satisfied);
        if (is_collision_free_corridor_satisfied)
        {
          solved = true;
          if (debug_verbose_)
            std::cout << "[soft_with_check] Constraints are satisfied" << std::endl;
        }
        else
        {
          solved = false;
          if (debug_verbose_)
            std::cout << "[soft_with_check] Constraints are not satisfied" << std::endl;
        }
      }
    }
  }
  catch (GRBException e)
  {
    // std::cout << "Error code = " << e.getErrorCode() << std::endl;
    // std::cout << e.getMessage() << std::endl;
    gurobi_error_detected = true;

    // Optionally write the model to a file for debugging.
    // m_.write("model.lp");
  }

  return solved;
}

void SolverGurobi::dynamicallyAdjustFactorInitialAndFinal(double factor)
{

  // If the replanning was successful, reset all the parameters to their original values
  factor_gamma_up_ = original_factor_gamma_up_;
  factor_gamma_down_ = original_factor_gamma_down_;

  // Dynamically adjust factor_initial_ and factor_final_ based on the last successful factor
  factor_initial_ = std::max(factor - factor_gamma_down_, factor_minimum_);
  factor_final_ = factor + factor_gamma_up_;
  use_constant_step_size_ = false;

  // Make sure that factor_final_ is always greater than factor_initial_
  if (factor_final_ <= factor_initial_)
    factor_final_ = factor_initial_ + factor_gamma_up_;
}

void SolverGurobi::setThreads(int threads)
{
  m_.set("Threads", std::to_string(threads));
}

void SolverGurobi::setVerbose(int verbose)
{
  m_.set("OutputFlag", std::to_string(verbose)); // 1 if you want verbose, 0 if not
}

void SolverGurobi::setWMax(double w_max)
{
  w_max_ = w_max;
}

void SolverGurobi::setClosedFormInitialDt(double closed_form_initial_dt)
{
  closed_form_initial_dt_ = closed_form_initial_dt;
}

void SolverGurobi::setInitialDt(double initial_dt)
{
  initial_dt_ = initial_dt;
}

void SolverGurobi::findDTForClosedForm(double factor)
{

  // Clear dt_
  dt_.clear();

  // Compute the dt for the closed form solution
  for (int i = 0; i < N_; i++)
  {
    dt_.push_back(factor * closed_form_initial_dt_);
  }

  // Compute total_traj_time_
  total_traj_time_ = std::accumulate(dt_.begin(), dt_.end(), 0.0);
}

void SolverGurobi::findDT(double factor)
{

  // Clear the previous dt
  dt_.clear();

  // FASTER's approach
  for (int i = 0; i < N_; i++)
    dt_.push_back(factor * std::max(initial_dt_, 2 * DC));

  // Compute total_traj_time_, which is used to fill goal_setpoints_
  total_traj_time_ = std::accumulate(dt_.begin(), dt_.end(), 0.0);
}

void SolverGurobi::setContinuityConstraints()
{

  // Remove the previous continuity constraints
  if (!continuity_cons_.empty())
  {
    for (int i = 0; i < continuity_cons_.size(); i++)
    {
      m_.remove(continuity_cons_[i]);
    }
    continuity_cons_.clear();
  }

  // Continuity constraints
  for (int t = 0; t < N_ - 1; t++) // From 0 to N_ - 1
  {
    for (int i = 0; i < 3; i++)
    {
      continuity_cons_.push_back(m_.addConstr(getPos(t, dt_[t], i) == getPos(t + 1, 0, i),
                                              "ContPos_t" + std::to_string(t) + "_axis" + std::to_string(i))); // Continuity in
                                                                                                               // position
      continuity_cons_.push_back(m_.addConstr(getVel(t, dt_[t], i) == getVel(t + 1, 0, i),
                                              "ContVel_t" + std::to_string(t) + "_axis" + std::to_string(i))); // Continuity in
                                                                                                               // velocity
      continuity_cons_.push_back(m_.addConstr(getAccel(t, dt_[t], i) == getAccel(t + 1, 0, i),
                                              "ContAccel_t" + std::to_string(t) + "_axis" + std::to_string(i))); // Continuity in acceleration
    }
  }
}

// For ground robots (not used now)
bool SolverGurobi::isWmaxSatisfied()
{

  for (int n = 0; n < N_; n++)
  {
    double xd = getVel(n, 0, 0).getValue();
    double yd = getVel(n, 0, 1).getValue();
    double xd2 = getAccel(n, 0, 0).getValue();
    double yd2 = getAccel(n, 0, 1).getValue();

    double numerator = xd * yd2 - yd * xd2;
    double denominator = xd * xd + yd * yd;
    double w_desired = (denominator > 0.001) ? fabs(numerator / denominator) : 0.5 * w_max_;

    if (w_desired > w_max_)
    {
      std::cout << "w_desired > than w_max: " << w_desired << " > " << w_max_ << "  , solving again" << std::endl;
      return false;
    }
  }
  return true;
}

bool SolverGurobi::callOptimizer()
{

  // House keeping
  bool solved = false; // set the flag to true
  file_t_++;          // increase the temporal counter

  // Optimize
  m_.optimize();
  int optimstatus = m_.get(GRB_IntAttr_Status);

  // Check if the optimization was successful
  if (optimstatus == GRB_OPTIMAL)
  {
    solved = true;
    // m_.write("/media/kkondo/T7/dynus/debug/num_" + std::to_string(file_t_) + ".lp");
  }
  else
  {
    solved = false;

    if (optimstatus == GRB_INF_OR_UNBD || optimstatus == GRB_INFEASIBLE || optimstatus == GRB_UNBOUNDED)
    {
      if (debug_verbose_)
        std::cout << "GUROBI Status: Infeasible or Unbounded" << std::endl;
      // m_.computeIIS(); // Compute the Irreducible Inconsistent Subsystem and write it on a file
      // m_.write("/media/kkondo/T7/dynus/debug/num_" + std::to_string(file_t_) + ".ilp");
    }

    if (optimstatus == GRB_NUMERIC)
    {
      // printf("GUROBI Status: Numerical issues\n");
      // m_.computeIIS();  // Compute the Irreducible Inconsistent Subsystem and write it on a file
      // m_.write("/media/kkondo/T7/dynus/debug/numeric_num_" + std::to_string(file_t_) + ".ilp");
    }

    if (optimstatus == GRB_INTERRUPTED)
    {
      // printf("GUROBI Status: Interrupted\n");
    }
  }

  // Write statistics
  // m_.write("/media/kkondo/T7/dynus/debug/stats_num_" + std::to_string(file_t_) + ".lp");

  return solved;
}

void SolverGurobi::getPieceWisePol(PieceWisePol &pwp)
{
  // Reset the piecewise polynomial
  pwp.clear();

  // Get the times of the piecewise polynomial
  double t = 0.0;
  for (int i = 0; i < N_ + 1; i++)
  {

    if (i == 0)
    {
      pwp.times.push_back(t0_);
      continue;
    }

    t = t + dt_[i - 1];

    // Add the time to the piecewise polynomial
    pwp.times.push_back(t0_ + t);
  }

  // Get the coefficients of the piecewise polynomial
  for (int i = 0; i < N_; i++)
  {

    // Initialize the coefficients
    Eigen::Matrix<double, 4, 1> coeff_x_i;
    Eigen::Matrix<double, 4, 1> coeff_y_i;
    Eigen::Matrix<double, 4, 1> coeff_z_i;

    // a, b, c, d
    for (int j = 0; j < 4; j++)
    {
      coeff_x_i(j) = x_double_[0][i * 4 + j];
      coeff_y_i(j) = x_double_[1][i * 4 + j];
      coeff_z_i(j) = x_double_[2][i * 4 + j];
    }

    // Add the coefficients to the piecewise polynomial
    pwp.coeff_x.push_back(coeff_x_i);
    pwp.coeff_y.push_back(coeff_y_i);
    pwp.coeff_z.push_back(coeff_z_i);
  }
}

// void SolverGurobi::findTimeParameterizedCoefficients(double a, double b, double c, double d, double L, double T0, Eigen::Matrix<double, 4, 1> &coeff_seg)
// {
//   // Coefficients for p(t) = A*t^3 + B*t^2 + C*t + D
//   double A = a / pow(L, 3);
//   double B = b / pow(L, 2) - 3 * T0 * a / pow(L, 3);
//   double C = c / L - 2 * T0 * b / pow(L, 2) + 3 * pow(T0, 2) * a / pow(L, 3);
//   double D = d - T0 * c / L + pow(T0, 2) * b / pow(L, 2) - pow(T0, 3) * a / pow(L, 3);

//   // Set the coefficients
//   coeff_seg(0) = A;
//   coeff_seg(1) = B;
//   coeff_seg(2) = C;
//   coeff_seg(3) = D;
// }

void SolverGurobi::getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++)
  {
    cps.push_back(getMinvoPosControlPointsDouble(t));
  }
}

void SolverGurobi::getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps)
{
  // Clear the control points
  cps.clear();

  for (int t = 0; t < N_; t++)
  {
    cps.push_back(getPosControlPointsDouble(t));
  }
}

void SolverGurobi::getInitialDT() 
{
  // Identify the axis with the largest displacement
  int bestAxis = 0;
  float max_disp = fabs(xf_[0] - x0_[0]);
  for (int i = 1; i < 3; i++)
  {
    float disp = fabs(xf_[i] - x0_[i]);
    if (disp > max_disp)
    {
      max_disp = disp;
      bestAxis = i;
    }
  }

  // Compute time based on maximum velocity for the best axis
  float t_v = max_disp / v_max_;

  // Get initial velocity and acceleration for the selected axis
  float v0 = x0_[bestAxis + 3]; // indices 3,4,5 store velocities for axes 0,1,2
  float a0 = x0_[bestAxis + 6]; // indices 6,7,8 store accelerations for axes 0,1,2

  // Determine the sign for jerk and acceleration calculations
  float sign = copysign(1.0, xf_[bestAxis] - x0_[bestAxis]);

  // Compute acceleration value for the best axis
  float accel = sign * a_max_;

  // Solve for t_a using the quadratic polynomial:
  // (x0 - xf) + v0*t + (0.5*accel)*t^2 = 0
  Eigen::Vector3d coeff_a;
  coeff_a << x0_[bestAxis] - xf_[bestAxis], v0, 0.5 * accel;
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_a(coeff_a);
  std::vector<double> realRoots_a;
  psolve_a.realRoots(realRoots_a);
  float t_a = MinPositiveElement(realRoots_a);

  // Compute jerk value for the best axis
  float jerk = sign * j_max_;

  // Solve for t_j using the cubic polynomial:
  // (x0 - xf) + v0*t + (a0/2)*t^2 + (jerk/6)*t^3 = 0
  Eigen::Vector4d coeff_j;
  coeff_j << x0_[bestAxis] - xf_[bestAxis], v0, a0 / 2.0, jerk / 6.0;
  Eigen::PolynomialSolver<double, Eigen::Dynamic> psolve_j(coeff_j);
  std::vector<double> realRoots_j;
  psolve_j.realRoots(realRoots_j);
  float t_j = MinPositiveElement(realRoots_j);


  // Choose the maximum time among t_v, t_a, and t_j, then scale by N_
  double initial_dt = std::max({t_v, t_a, t_j}) / N_;
  if (initial_dt > 10000) // Failsafe when no valid solution is found
  {
    printf("there is no solution\n");
    initial_dt = 0;
  }
  
  initial_dt_ = initial_dt;
}


inline GRBLinExpr SolverGurobi::getPos(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return x_[axis][base_index] * tau * tau * tau + x_[axis][base_index + 1] * tau * tau + x_[axis][base_index + 2] * tau + x_[axis][base_index + 3];
}

inline double SolverGurobi::getPosDouble(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return x_double_[axis][base_index] * tau * tau * tau + x_double_[axis][base_index + 1] * tau * tau + x_double_[axis][base_index + 2] * tau + x_double_[axis][base_index + 3];
}

inline GRBLinExpr SolverGurobi::getVel(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return 3 * x_[axis][base_index] * tau * tau +
         2 * GRBLinExpr(x_[axis][base_index + 1]) * tau +
         GRBLinExpr(x_[axis][base_index + 2]);
}

inline double SolverGurobi::getVelDouble(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  // For the numeric (double) version we assume the solved (numeric) values are stored in x_double_
  return 3 * x_double_[axis][base_index] * tau * tau +
         2 * x_double_[axis][base_index + 1] * tau +
         x_double_[axis][base_index + 2];
}

inline GRBLinExpr SolverGurobi::getAccel(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return 6 * x_[axis][base_index] * tau +
         2 * x_[axis][base_index + 1];
}

inline double SolverGurobi::getAccelDouble(int interval, double tau, int axis) const
{
  double base_index = 4 * interval;
  return 6 * x_double_[axis][base_index] * tau +
         2 * x_double_[axis][base_index + 1];
}
inline GRBLinExpr SolverGurobi::getJerk(int interval, double tau, int axis) const
{
  return 6 * x_[axis][4 * interval];
}

inline double SolverGurobi::getJerkDouble(int interval, double tau, int axis) const
{
  return 6 * x_double_[axis][4 * interval];
}

// Coefficient getters: At^3 + Bt^2 + Ct + D  , t \in [0, dt]
inline GRBLinExpr SolverGurobi::getA(int interval, int axis) const
{
  return x_[axis][4 * interval];
}

inline GRBLinExpr SolverGurobi::getB(int interval, int axis) const
{
  return x_[axis][4 * interval + 1];
}

inline GRBLinExpr SolverGurobi::getC(int interval, int axis) const
{
  return x_[axis][4 * interval + 2];
}

inline GRBLinExpr SolverGurobi::getD(int interval, int axis) const
{
  return x_[axis][4 * interval + 3];
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
inline GRBLinExpr SolverGurobi::getAn(int interval, int axis) const
{
  // a is always from x_ (the decision variable)
  return x_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getBn(int interval, int axis) const
{
  return x_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getCn(int interval, int axis) const
{
  return x_[axis][4 * interval + 2] * dt_[interval];
}

inline GRBLinExpr SolverGurobi::getDn(int interval, int axis) const
{
  return x_[axis][4 * interval + 3];
}

// Coefficients Normalized: At^3 + Bt^2 + Ct + D  , t \in [0, 1]
inline double SolverGurobi::getAnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval] * dt_[interval] * dt_[interval] * dt_[interval];
}

inline double SolverGurobi::getBnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 1] * dt_[interval] * dt_[interval];
}

inline double SolverGurobi::getCnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 2] * dt_[interval];
}

inline double SolverGurobi::getDnDouble(int interval, int axis) const
{
  return x_double_[axis][4 * interval + 3];
}

// Control Points (of the splines) getters
inline std::vector<GRBLinExpr> SolverGurobi::getCP0(int interval) const
{
  std::vector<GRBLinExpr> cp = {getPos(interval, 0, 0), getPos(interval, 0, 1), getPos(interval, 0, 2)};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP1(int interval) const
{
  GRBLinExpr cpx = (getCn(interval, 0) + 3 * getDn(interval, 0)) / 3;
  GRBLinExpr cpy = (getCn(interval, 1) + 3 * getDn(interval, 1)) / 3;
  GRBLinExpr cpz = (getCn(interval, 2) + 3 * getDn(interval, 2)) / 3;
  std::vector<GRBLinExpr> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP2(int interval) const
{
  GRBLinExpr cpx = (getBn(interval, 0) + 2 * getCn(interval, 0) + 3 * getDn(interval, 0)) / 3;
  GRBLinExpr cpy = (getBn(interval, 1) + 2 * getCn(interval, 1) + 3 * getDn(interval, 1)) / 3;
  GRBLinExpr cpz = (getBn(interval, 2) + 2 * getCn(interval, 2) + 3 * getDn(interval, 2)) / 3;
  std::vector<GRBLinExpr> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<GRBLinExpr> SolverGurobi::getCP3(int interval) const
{
  std::vector<GRBLinExpr> cp = {getPos(interval, dt_[interval], 0), getPos(interval, dt_[interval], 1), getPos(interval, dt_[interval], 2)};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP0Double(int interval) const
{
  std::vector<double> cp = {getPosDouble(interval, 0, 0), getPosDouble(interval, 0, 1), getPosDouble(interval, 0, 2)};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP1Double(int interval) const
{
  double cpx = (getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP2Double(int interval) const
{
  double cpx = (getBnDouble(interval, 0) + 2 * getCnDouble(interval, 0) + 3 * getDnDouble(interval, 0)) / 3;
  double cpy = (getBnDouble(interval, 1) + 2 * getCnDouble(interval, 1) + 3 * getDnDouble(interval, 1)) / 3;
  double cpz = (getBnDouble(interval, 2) + 2 * getCnDouble(interval, 2) + 3 * getDnDouble(interval, 2)) / 3;
  std::vector<double> cp = {cpx, cpy, cpz};
  return cp;
}

inline std::vector<double> SolverGurobi::getCP3Double(int interval) const
{
  std::vector<double> cp = {getPosDouble(interval, dt_[interval], 0), getPosDouble(interval, dt_[interval], 1), getPosDouble(interval, dt_[interval], 2)};
  return cp;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoPosControlPoints(int interval) const
{
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<GRBLinExpr, 3, 4> Pn;
  Pn << getAn(interval, 0), getBn(interval, 0), getCn(interval, 0), getDn(interval, 0),
      getAn(interval, 1), getBn(interval, 1), getCn(interval, 1), getDn(interval, 1),
      getAn(interval, 2), getBn(interval, 2), getCn(interval, 2), getDn(interval, 2);

  // Convert from coefficients of polynomials to MINVO control points using the precomputed conversion matrix.
  Eigen::Matrix<GRBLinExpr, 3, 4> Vn;
  // GRBLinExpr doesn't allow this operation: Vn = Pn * A_pos_mv_rest_inv_;
  for (int i = 0; i < Vn.rows(); i++)
  {
    for (int j = 0; j < Vn.cols(); j++)
    {
      Vn(i, j) = 0;
      for (int k = 0; k < Pn.cols(); k++)
      {
        Vn(i, j) += Pn(i, k) * A_pos_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_vec;
  for (int j = 0; j < Vn.cols(); j++)
  {
    std::vector<GRBLinExpr> Vn_col;
    for (int i = 0; i < Vn.rows(); i++)
    {
      Vn_col.push_back(Vn(i, j));
    }
    Vn_vec.push_back(Vn_col);
  }

  return Vn_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoVelControlPoints(int interval) const
{
  // Retrieve the duration for segment t.
  double deltaT = dt_[interval];

  // Compute the derivative with respect to the normalized variable u:
  // dp/du = 3*Aₙ*u² + 2*Bₙ*u + Cₙ.
  // Then, using the chain rule, dp/dt = (dp/du) (du/dt).
  // du / dt = 1 / deltaT.
  Eigen::Matrix<GRBLinExpr, 3, 3> Pn_prime;
  Pn_prime << (3 * getAn(interval, 0)) / deltaT, (2 * getBn(interval, 0)) / deltaT, getCn(interval, 0) / deltaT,
      (3 * getAn(interval, 1)) / deltaT, (2 * getBn(interval, 1)) / deltaT, getCn(interval, 1) / deltaT,
      (3 * getAn(interval, 2)) / deltaT, (2 * getBn(interval, 2)) / deltaT, getCn(interval, 2) / deltaT;

  // Convert the normalized derivative to MINVO velocity control points.
  Eigen::Matrix<GRBLinExpr, 3, 3> Vn_prime;
  // Vn_prime = Pn_prime * A_vel_mv_rest_inv_;
  for (int i = 0; i < Vn_prime.rows(); i++)
  {
    for (int j = 0; j < Vn_prime.cols(); j++)
    {
      Vn_prime(i, j) = 0;
      for (int k = 0; k < Pn_prime.cols(); k++)
      {
        Vn_prime(i, j) += Pn_prime(i, k) * A_vel_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_prime_vec;
  for (int j = 0; j < Vn_prime.cols(); j++)
  {
    std::vector<GRBLinExpr> Vn_prime_col;
    for (int i = 0; i < Vn_prime.rows(); i++)
    {
      Vn_prime_col.push_back(Vn_prime(i, j));
    }
    Vn_prime_vec.push_back(Vn_prime_col);
  }

  return Vn_prime_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoAccelControlPoints(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT2 = deltaT * deltaT;

  // Compute the second derivative with respect to u:
  // d²p/du² = 6*Aₙ*u + 2*Bₙ.
  // Then, using the chain rule, d²p/dt² = (d²p/du²) (du/dt)².
  Eigen::Matrix<GRBLinExpr, 3, 2> Pn_double_prime;
  Pn_double_prime << (6 * getAn(interval, 0)) / deltaT2, (2 * getBn(interval, 0)) / deltaT2,
      (6 * getAn(interval, 1)) / deltaT2, (2 * getBn(interval, 1)) / deltaT2,
      (6 * getAn(interval, 2)) / deltaT2, (2 * getBn(interval, 2)) / deltaT2;

  // Convert the normalized second derivative to MINVO acceleration control points.
  Eigen::Matrix<GRBLinExpr, 3, 2> Vn_double_prime;
  // Vn_double_prime = Pn_double_prime * A_accel_mv_rest_inv_;
  for (int i = 0; i < Vn_double_prime.rows(); i++)
  {
    for (int j = 0; j < Vn_double_prime.cols(); j++)
    {
      Vn_double_prime(i, j) = 0;
      for (int k = 0; k < Pn_double_prime.cols(); k++)
      {
        Vn_double_prime(i, j) += Pn_double_prime(i, k) * A_accel_mv_rest_inv_(k, j);
      }
    }
  }

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_double_prime_vec;
  for (int j = 0; j < Vn_double_prime.cols(); j++)
  {
    std::vector<GRBLinExpr> Vn_double_prime_col;
    for (int i = 0; i < Vn_double_prime.rows(); i++)
    {
      Vn_double_prime_col.push_back(Vn_double_prime(i, j));
    }
    Vn_double_prime_vec.push_back(Vn_double_prime_col);
  }

  return Vn_double_prime_vec;
}

inline std::vector<std::vector<GRBLinExpr>> SolverGurobi::getMinvoJerkControlPoints(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT3 = deltaT * deltaT * deltaT;

  // Compute the third derivative with respect to u:
  // d³p/du³ = 6*Aₙ.
  // Then, using the chain rule, d³p/dt³ = (d³p/du³) (du/dt)³.
  Eigen::Matrix<GRBLinExpr, 3, 1> Pn_triple_prime;
  Pn_triple_prime << (6 * getAn(interval, 0)) / deltaT3,
      (6 * getAn(interval, 1)) / deltaT3,
      (6 * getAn(interval, 2)) / deltaT3;

  // Here we assume that no further conversion is required for jerk.
  Eigen::Matrix<GRBLinExpr, 3, 1> Vn_triple_prime = Pn_triple_prime;

  // Conver the Eigen matrix to a vector of vectors - and each vector is a column of the matrix because that is the control point
  std::vector<std::vector<GRBLinExpr>> Vn_triple_prime_vec;
  for (int j = 0; j < Vn_triple_prime.cols(); j++)
  {
    std::vector<GRBLinExpr> Vn_triple_prime_col;
    for (int i = 0; i < Vn_triple_prime.rows(); i++)
    {
      Vn_triple_prime_col.push_back(Vn_triple_prime(i, j));
    }
    Vn_triple_prime_vec.push_back(Vn_triple_prime_col);
  }

  return Vn_triple_prime_vec;
}

inline Eigen::Matrix<double, 3, 4> SolverGurobi::getMinvoPosControlPointsDouble(int interval) const
{
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  Pn << getAnDouble(interval, 0), getBnDouble(interval, 0), getCnDouble(interval, 0), getDnDouble(interval, 0),
      getAnDouble(interval, 1), getBnDouble(interval, 1), getCnDouble(interval, 1), getDnDouble(interval, 1),
      getAnDouble(interval, 2), getBnDouble(interval, 2), getCnDouble(interval, 2), getDnDouble(interval, 2);

  // Convert from coefficients of polynomials to MINVO control points using the precomputed conversion matrix.
  Eigen::Matrix<double, 3, 4> Vn = Pn * A_pos_mv_rest_inv_;
  return Vn;
}

inline Eigen::Matrix<double, 3, 3> SolverGurobi::getMinvoVelControlPointsDouble(int interval) const
{
  // Retrieve the duration for segment t.
  double deltaT = dt_[interval];

  // Compute the derivative with respect to the normalized variable u:
  // dp/du = 3*Aₙ*u² + 2*Bₙ*u + Cₙ.
  // Then, using the chain rule, dp/dt = (dp/du) (du/dt).
  // du / dt = 1 / deltaT.
  Eigen::Matrix<double, 3, 3> Pn_prime;
  Pn_prime << (3 * getAnDouble(interval, 0)) / deltaT, (2 * getBnDouble(interval, 0)) / deltaT, getCnDouble(interval, 0) / deltaT,
      (3 * getAnDouble(interval, 1)) / deltaT, (2 * getBnDouble(interval, 1)) / deltaT, getCnDouble(interval, 1) / deltaT,
      (3 * getAnDouble(interval, 2)) / deltaT, (2 * getBnDouble(interval, 2)) / deltaT, getCnDouble(interval, 2) / deltaT;

  // Convert the normalized derivative to MINVO velocity control points.
  Eigen::Matrix<double, 3, 3> Vn_prime = Pn_prime * A_vel_mv_rest_inv_;
  return Vn_prime;
}

inline Eigen::Matrix<double, 3, 2> SolverGurobi::getMinvoAccelControlPointsDouble(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT2 = deltaT * deltaT;

  // Compute the second derivative with respect to u:
  // d²p/du² = 6*Aₙ*u + 2*Bₙ.
  // Then, using the chain rule, d²p/dt² = (d²p/du²) (du/dt)².
  Eigen::Matrix<double, 3, 2> Pn_double_prime;
  Pn_double_prime << (6 * getAnDouble(interval, 0)) / deltaT2, (2 * getBnDouble(interval, 0)) / deltaT2,
      (6 * getAnDouble(interval, 1)) / deltaT2, (2 * getBnDouble(interval, 1)) / deltaT2,
      (6 * getAnDouble(interval, 2)) / deltaT2, (2 * getBnDouble(interval, 2)) / deltaT2;

  // Convert the normalized second derivative to MINVO acceleration control points.
  Eigen::Matrix<double, 3, 2> Vn_double_prime = Pn_double_prime * A_accel_mv_rest_inv_;
  return Vn_double_prime;
}

inline Eigen::Matrix<double, 3, 1> SolverGurobi::getMinvoJerkControlPointsDouble(int interval) const
{
  double deltaT = dt_[interval];
  double deltaT3 = deltaT * deltaT * deltaT;

  // Compute the third derivative with respect to u:
  // d³p/du³ = 6*Aₙ.
  // Then, using the chain rule, d³p/dt³ = (d³p/du³) (du/dt)³.
  Eigen::Matrix<double, 3, 1> Pn_triple_prime;
  Pn_triple_prime << (6 * getAnDouble(interval, 0)) / deltaT3,
      (6 * getAnDouble(interval, 1)) / deltaT3,
      (6 * getAnDouble(interval, 2)) / deltaT3;

  // Here we assume that no further conversion is required for jerk.
  Eigen::Matrix<double, 3, 1> Vn_triple_prime = Pn_triple_prime;
  return Vn_triple_prime;
}

inline Eigen::Matrix<double, 3, 4> SolverGurobi::getPosControlPointsDouble(int interval) const
{
  // Compute the normalized coefficient matrix Pn (for u ∈ [0,1])
  Eigen::Matrix<double, 3, 4> Pn;
  std::vector<double> cp0 = getCP0Double(interval);
  std::vector<double> cp1 = getCP1Double(interval);
  std::vector<double> cp2 = getCP2Double(interval);
  std::vector<double> cp3 = getCP3Double(interval);
  Pn << cp0[0], cp1[0], cp2[0], cp3[0],
      cp0[1], cp1[1], cp2[1], cp3[1],
      cp0[2], cp1[2], cp2[2], cp3[2];
  return Pn;
}

// Returns the velocity control points (as a quadratic Bézier curve) for a given axis in segment 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getVelCP(int interval, int axis) const
{
  // Let T = dt_[interval]
  double T = dt_[interval];
  // The underlying cubic coefficients are stored as:
  // a = x_[axis][4*interval], b = x_[axis][4*interval+1], c = x_[axis][4*interval+2], d = x_[axis][4*interval+3].
  // The velocity polynomial is: v(t) = 3*a*t^2 + 2*b*t + c.
  // Reparameterizing with u = t/T, the power basis coefficients for v(u) become:
  //   A_v = 3*a*T^2, B_v = 2*b*T, C_v = c.
  // Then the quadratic Bézier control points for v(u) are:
  //   V0 = C_v,
  //   V1 = C_v + B_v/2,
  //   V2 = C_v + B_v + A_v.
  GRBLinExpr A_v = 3 * x_[axis][4 * interval] * T * T;
  GRBLinExpr B_v = 2 * x_[axis][4 * interval + 1] * T;
  GRBLinExpr C_v = x_[axis][4 * interval + 2];

  GRBLinExpr V0 = C_v;
  GRBLinExpr V1 = C_v + B_v / 2;
  GRBLinExpr V2 = C_v + B_v + A_v;

  std::vector<GRBLinExpr> cp = {V0, V1, V2};
  return cp;
}

// Returns the acceleration control points (as a linear Bézier curve) for a given axis in segment 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getAccelCP(int interval, int axis) const
{
  // Acceleration: a(t)= 6*a*t + 2*b.
  // With u = t/T, note that:
  // a(0)= 2*b, and a(T)=6*a*T+2*b.
  GRBLinExpr A0_expr = 2 * x_[axis][4 * interval + 1];                       // a(0)
  GRBLinExpr A1_expr = A0_expr + 6 * x_[axis][4 * interval] * dt_[interval]; // a(T)

  std::vector<GRBLinExpr> cp = {A0_expr, A1_expr};
  return cp;
}

// Returns the jerk control point (constant) for a given axis in segment 'interval'
inline std::vector<GRBLinExpr> SolverGurobi::getJerkCP(int interval, int axis) const
{
  // Jerk: j(t)= 6*a, which is constant.
  GRBLinExpr J = 6 * x_[axis][4 * interval];
  std::vector<GRBLinExpr> cp = {J};
  return cp;
}
