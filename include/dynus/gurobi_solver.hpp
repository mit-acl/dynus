/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#ifndef GUROBI_SOLVER_HPP
#define GUROBI_SOLVER_HPP
#include <Eigen/Dense>
#include "gurobi_c++.h"
#include <sstream>
#include <Eigen/Dense>
#include <type_traits>
#include <fstream>
#include "dgp/termcolor.hpp"

#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <unsupported/Eigen/Polynomials>
#include <dynus/dynus_type.hpp>
#include "timer.hpp"

using namespace termcolor;
enum ConstraintType { POSITION, VELOCITY, ACCELERATION, JERK };
typedef timer::Timer MyTimer;

// TODO: This function is the same as solvePolyOrder2 but with other name (weird conflicts...)
inline double solvePolynomialOrder2(Eigen::Vector3f& coeff)
{
  // std::cout << "solving\n" << coeff.transpose() << std::endl;
  double a = coeff(0);
  double b = coeff(1);
  double c = coeff(2);
  double dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    double x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    double x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 >= 0)
    {
      return x1;
    }
    if (x2 >= 0)
    {
      return x2;
    }
  }
  printf("No solution found to the equation\n");
  return std::numeric_limits<float>::max();
}

class mycallback : public GRBCallback
{
public:
  std::atomic<bool> should_terminate_;
  mycallback() : should_terminate_(false) {}
protected:
  void callback();
};

class SolverGurobi
{
public:
  SolverGurobi();

  void setUseRefPoints(bool use_ref_points, int num_ref_sample_points);
  void setClosedFormSolutionParams(int closed_form_time_allocation_adj_iter_max, double closed_form_initial_factor, double closed_form_factor_increment, double closed_form_factor_initial_decrement);
  void setOptimizationType(std::string optimization_type);
  void setN(int N);
  void setX0(const state& data);
  void setT0(double t0);
  void setMapSize(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max, double res);
  void setVehicleType(std::string vehicle_type);
  void setPlannerMode(std::string planner_mode);
  void setXf(const state& data);
  void setDirf(double yawf);
  void initializeGoalSetpoints();
  void setBounds(double max_values[3]);
  bool generateNewTrajectoryWithIteration(bool& gurobi_error_detected, const vec_Vecf<3>& global_path, double& gurobi_computation_time);
  bool generateNewTrajectoryWithFactor(bool &gurobi_error_detected, const vec_Vecf<3> &global_path, double& gurobi_computation_time, double factor);
  bool callOptimizer();
  void getInitialDT();
  void stopExecution();
  void resetToNominalState();

  void setDC(double dc);
  void setPolytopes(std::vector<LinearConstraint3D> polytopes, bool use_closed_form = false);
  void setPolytopesConstraints();
  void setPolyConsts();
  void setMapSizeConstraints();
  void findDT(double factor);
  void findDTForClosedForm(double factor);
  void fillGoalSetPoints();
  void setObjective(const vec_Vecf<3>& global_path);
  void setConstraintsXf();
  void setConstraintsX0();
  void setContinuityConstraints();
  void setWeights(double control_cost_weight, double dynamic_weight, double final_pos_weight, double final_vel_weight, double final_accel_weight, double final_yaw_cost_weight, double total_yaw_diff_weight, double ref_point_weight);
  void setCollisionClearance(double collision_clearance);
  void setFlags(bool use_hard_constr_for_final_yaw, bool debug_verbose);
  void setConstraintFlags(bool use_hard_constr_for_final_state, bool use_hard_dynamic_constraints);
  void setNumObsDistSamples(int num_obs_dist_samples);
  void setInitialGuess(vec_Vecf<3> global_path, std::vector<double> travel_times);
  void findInitialGuessABCDFromRefPoints(double& a, double& b, double& c, double& d, double q0, double q1, double q2, double q3, double dt);
  void checkDynamicViolation(bool &is_dyn_constraints_satisfied);
  void checkCollisionViolation(bool &is_collision_free_corridor_satisfied);
  void createSafeCorridorConstraintsForPolytope(int t);
  void createSafeCorridorConstraintsForPolytopeAtleastOne(int t);
  // void setBoundsForVars(); // Currently not used
  // void setLocalBoxSize(const std::vector<double>& local_box_size); // Currently not used

  // Closed form (N = 3) provided time
  bool findClosedFormSolution();
  void findClosedFormSolutionForEachAxis(double p0, double v0, double a0, double pf, double vf, double af, double T1, double T2, double T3, double &d1, double &c1, double &b1, double &a1, double &d2, double &c2, double &b2, double &a2, double &d3, double &c3, double &b3, double &a3);

  // For the jackal
  void setWMax(double w_max);
  bool isWmaxSatisfied();

  void setDynamicConstraints();
  void createVars();
  void setX();
  void removeVars();
  void setThreads(int threads);
  void setVerbose(int verbose);

  void setDistances(vec_Vecf<3>& samples, std::vector<double> dist_near_obs);

  // set initial and final states
  void getInitialAndFinalConditions(double &P0, double &V0, double &A0, double &Pf, double &Vf, double &Af, int axis);

  // Compute dependent coefficients 
  void computeDependentCoefficientsN4();
  void computeDependentCoefficientsN5();
  void computeDependentCoefficientsN6();

  // (post optimization) get dependent coefficients as double
  void getDependentCoefficientsN4Double();
  void getDependentCoefficientsN5Double();
  void getDependentCoefficientsN6Double();

  void setDistanceConstraints();

  void setMode(int mode);
  void setTimeAllocationParameters(double factor_initial, double factor_final, double factor_gamma_up, double factor_gamma_down, double factor_constant_step_size, double factor_minimum, double factor_delta_step_size, bool use_constant_step_size, int count_to_switch_to_constant_step_size);
  void dynamicallyAdjustFactorInitialAndFinal(double factor);
  void findIntervalIdxAndDt(double time_in_whole_traj, int& interval_idx, double& dt_interval);
  void computeNormalizedTime(double t_target, double &s, int &segmentIndex);

  // TODO: move to utils?
  std::vector<double> computeCubicPolynomial3D(const Eigen::Vector3d& pos0, const Eigen::Vector3d& vel0, const Eigen::Vector3d& posf, const Eigen::Vector3d& velf, double t0, double tf);

  void findClosestIndexFromTime(const double t, int& index, const std::vector<double>& time);
  dynTraj adjustTrajTime(const dynTraj& traj);

  inline GRBLinExpr getPos(int t, double tau, int ii) const;
  inline GRBLinExpr getVel(int t, double tau, int ii) const;
  inline GRBLinExpr getAccel(int t, double tau, int ii) const;
  inline GRBLinExpr getJerk(int t, double tau, int ii) const;

  inline double getPosDouble(int t, double tau, int ii) const;
  inline double getVelDouble(int t, double tau, int ii) const;
  inline double getAccelDouble(int t, double tau, int ii) const;
  inline double getJerkDouble(int t, double tau, int ii) const;

  inline GRBLinExpr getA(int t, int ii) const;
  inline GRBLinExpr getB(int t, int ii) const;
  inline GRBLinExpr getC(int t, int ii) const;
  inline GRBLinExpr getD(int t, int ii) const;

  inline GRBLinExpr getAn(int t, int ii) const;
  inline GRBLinExpr getBn(int t, int ii) const;
  inline GRBLinExpr getCn(int t, int ii) const;
  inline GRBLinExpr getDn(int t, int ii) const;

  inline double getAnDouble(int t, int ii) const;
  inline double getBnDouble(int t, int ii) const;
  inline double getCnDouble(int t, int ii) const;
  inline double getDnDouble(int t, int ii) const;

  inline std::vector<GRBLinExpr> getCP0(int t) const;
  inline std::vector<GRBLinExpr> getCP1(int t) const;
  inline std::vector<GRBLinExpr> getCP2(int t) const;
  inline std::vector<GRBLinExpr> getCP3(int t) const;

  inline std::vector<double> getCP0Double(int t) const;
  inline std::vector<double> getCP1Double(int t) const;
  inline std::vector<double> getCP2Double(int t) const;
  inline std::vector<double> getCP3Double(int t) const;

  inline std::vector<GRBLinExpr> getVelCP(int interval, int axis) const;
  inline std::vector<GRBLinExpr> getAccelCP(int interval, int axis) const;
  inline std::vector<GRBLinExpr> getJerkCP(int interval, int axis) const;

  // Get Minvo Control points given the interval
  inline std::vector<std::vector<GRBLinExpr>> getMinvoPosControlPoints(int t) const;
  inline std::vector<std::vector<GRBLinExpr>> getMinvoVelControlPoints(int t) const;
  inline std::vector<std::vector<GRBLinExpr>> getMinvoAccelControlPoints(int t) const;
  inline std::vector<std::vector<GRBLinExpr>> getMinvoJerkControlPoints(int t) const;

  // Get Minvo Control points given the interval as double
  inline Eigen::Matrix<double, 3, 4> getMinvoPosControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 3> getMinvoVelControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 2> getMinvoAccelControlPointsDouble(int t) const;
  inline Eigen::Matrix<double, 3, 1> getMinvoJerkControlPointsDouble(int t) const;

  // Get Control points given the interval
  inline Eigen::Matrix<double, 3, 4> getPosControlPointsDouble(int t) const;

  // Get coefficients of the polynomial
  void getPieceWisePol(PieceWisePol& pwp);
  // void findTimeParameterizedCoefficients(double a, double b, double c, double d, double L, double T0, Eigen::Matrix<double, 4, 1>& coeff_seg);

  // Helper function that returns true if 'expr' contains d3_var with a nonzero coefficient.
  bool controlPointDepends(ConstraintType type, int seg, int cp);
  bool controlPointDependsOnD3(ConstraintType type, int seg, int cp);
  bool controlPointDependsOnD3OrD4(ConstraintType type, int seg, int cp);
  bool controlPointDependsOnD3OrD4OrD5(ConstraintType type, int seg, int cp);

  // void computeControlPointsN4();
  // void computeControlPointsN5();
  // void computeControlPointsN6();

  // Get minvo control points
  void getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps);

  // Get control points
  void getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>>& cps);

  // Get goal setpoints
  void getGoalSetpoints(std::vector<state>& goal_setpoints);

  // set initial dt
  void setInitialDt(double initial_dt);
  void setClosedFormInitialDt(double closed_form_initial_dt);

  std::vector<state> goal_setpoints_;
  std::vector<double> dt_;  // time step found by the solver
  double total_traj_time_;
  int trials_ = 0;
  int file_t_ = 0;
  double factor_that_worked_ = 0;
  bool use_miqp_ = false;
  int N_ = 6;
  bool use_ref_points_;
  int num_ref_sample_points_;
  std::string optimization_type_;
  mycallback cb_;

protected:

  // parameters
  double cost_;
  double xf_[3 * 3];
  double dirf_[2];
  double x0_[3 * 3];
  double t0_;
  double v_max_;
  double a_max_;
  double j_max_;
  double DC;
  std::vector<dynTraj> trajs_;  // Dynamic trajectory
  double collision_clearance_;
  PieceWisePol pwp_;
  std::string planner_mode_;
  vec_Vecf<3> global_path_;
  std::vector<float> local_box_size_;
  int num_obs_dist_samples_;
  int current_polytopes_size_ = 3;
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;
  double res_;
  int closed_form_time_allocation_adj_iter_max_;
  double closed_form_initial_dt_;
  double closed_form_initial_factor_;
  double closed_form_factor_;
  double closed_form_factor_increment_;
  double closed_form_factor_initial_decrement_;
  double initial_dt_;

  // Basis converter
  BasisConverter basis_converter_;
  std::vector<std::vector<double>> M_be2mv_;
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv_;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv_;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv_;

  // Flags
  bool use_hard_constr_for_final_state_;
  bool use_hard_dynamic_constraints_;
  bool use_hard_constr_for_final_yaw_;
  bool debug_verbose_;
  std::string vehicle_type_ = "uav";

  int N_of_polytopes_ = 3;

  GRBEnv* env = new GRBEnv();
  GRBModel m_ = GRBModel(*env);

  std::vector<GRBConstr> at_least_1_pol_cons_;  // Constraints at least in one polytope
  std::vector<GRBConstr> polytopes_cons_;       // for DYNUS
  std::vector<GRBGenConstr> miqp_polytopes_cons_;    // for MIQP    
  std::vector<GRBConstr> continuity_cons_;
  std::vector<GRBConstr> init_cons_;
  std::vector<GRBConstr> final_cons_;
  std::vector<GRBConstr> map_cons_;
  std::vector<GRBConstr> dyn_cons_;

  std::vector<std::vector<GRBVar>> b_;  // binary variables (only used by the MIQP)
  // std::vector<std::vector<GRBVar>> x_;
  std::vector<std::vector<GRBLinExpr>> x_;
  std::vector<std::vector<double>> x_double_;
  std::vector<std::vector<GRBLinExpr>> p_cp_;
  std::vector<std::vector<GRBLinExpr>> v_cp_;
  std::vector<std::vector<GRBLinExpr>> a_cp_;
  std::vector<std::vector<GRBLinExpr>> j_cp_;
  std::vector<GRBVar> d3_;
  std::vector<GRBVar> d4_;
  std::vector<GRBVar> d5_;
  std::vector<GRBVar> d6_;
  std::vector<GRBVar> d7_;
  std::vector<GRBVar> d8_;

  vec_Vecf<3> samples_;           // Samples along the rescue path
  vec_Vecf<3> samples_penalize_;  // Samples along the rescue path

  std::vector<double> dist_near_obs_;
  std::vector<LinearConstraint3D> polytopes_;

  int mode_;
  bool use_constant_step_size_ = false;
  double factor_initial_ = 0.6;
  double factor_final_ = 2.0;
  double factor_gamma_up_ = 1.0;
  double factor_gamma_down_ = 0.5;
  double factor_constant_step_size_ = 0.1;
  double factor_minimum_ = 0.1;
  double factor_delta_step_size_ = 0.05;
  int count_to_switch_to_constant_step_size_ = 10;
  double original_factor_initial_ = 0.6;
  double original_factor_final_ = 2.0;
  double original_factor_gamma_up_ = 1.0;
  double original_factor_gamma_down_ = 0.5;
  double w_max_ = 1;

  // optimization weights
  double control_cost_weight_;
  double dynamic_weight_;
  double final_pos_weight_;
  double final_vel_weight_;
  double final_accel_weight_;
  double final_yaw_cost_weight_;
  double total_yaw_diff_weight_;
  double ref_point_weight_;

};
#endif