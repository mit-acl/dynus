/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
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
enum ConstraintType
{
    POSITION,
    VELOCITY,
    ACCELERATION,
    JERK
};
typedef timer::Timer MyTimer;

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
    ~SolverGurobi();

    void setPlannerName(const std::string &name);
    void initializeSolver(const parameters &par);
    void setX0(const state &data);
    void setT0(double t0);
    void setXf(const state &data);
    void setDirf(double yawf);
    void getTotalTrajTime(double &total_traj_time);
    void initializeGoalSetpoints();
    bool generateNewTrajectory(bool &gurobi_error_detected, double &gurobi_computation_time, double factor, bool use_single_thread = false);
    bool generateNewTrajectorySequentialFactors(
        bool &gurobi_error_detected,
        double &gurobi_computation_time_ms,
        double &factor_that_worked);
    bool callOptimizer();
    void stopExecution();
    void resetToNominalState();

    void setPolytopes(std::vector<LinearConstraint3D> polytopes);
    void setPolytopesTimeLayered(const std::vector<std::vector<LinearConstraint3D>> &polytopes_by_time);
    void setPolytopesConstraints();
    void setPolyConsts();
    void setMapSizeConstraints();
    void findDT(double factor);
    double getInitialDt();
    void fillGoalSetPoints();
    void setObjective();
    void setConstraintsXf();
    void setConstraintsX0();
    void setContinuityConstraints();
    void findInitialGuessABCDFromRefPoints(double &a, double &b, double &c, double &d, double q0, double q1, double q2, double q3, double dt);
    void checkDynamicViolation(bool &is_dyn_constraints_satisfied);
    void checkCollisionViolation(bool &is_collision_free_corridor_satisfied);
    void createSafeCorridorConstraintsForPolytopeAtleastOne(int t);

    // For the jackal
    void setWMax(double w_max);
    bool isWmaxSatisfied();

    void setDynamicConstraints();
    void createVars();
    void setX();
    void removeVars();
    void setDistances(vec_Vecf<3> &samples, std::vector<double> dist_near_obs);

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

    void findIntervalIdxAndDt(double time_in_whole_traj, int &interval_idx, double &dt_interval);
    void computeNormalizedTime(double t_target, double &s, int &segmentIndex);

    void findClosestIndexFromTime(const double t, int &index, const std::vector<double> &time);
    dynTraj adjustTrajTime(const dynTraj &traj);

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

    inline double getADouble(int interval, int axis) const;
    inline double getBDouble(int interval, int axis) const;
    inline double getCDouble(int interval, int axis) const;
    inline double getDDouble(int interval, int axis) const;

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
    void getPieceWisePol(PieceWisePol &pwp);
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
    void getMinvoControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps);

    // Get control points
    void getControlPoints(std::vector<Eigen::Matrix<double, 3, 4>> &cps);

    // Get goal setpoints
    void getGoalSetpoints(std::vector<state> &goal_setpoints);

    // set initial dt
    void setInitialDt(double initial_dt);
    double getFactorThatWorked();
    double getObjectiveValue() const { return objective_value_; }

    double objective_value_{std::numeric_limits<double>::quiet_NaN()};
    std::vector<state> goal_setpoints_;
    std::vector<double> dt_; // time step found by the solver
    double total_traj_time_;
    int trials_ = 0;
    int file_t_ = 0;
    double factor_that_worked_ = 0;
    int N_ = 6;
    mycallback cb_;

protected:
    std::string planner_name_{"DYNUS"};              // "DYNUS" or "FASTER"
    std::vector<std::vector<GRBVar>> x_faster_vars_; // [axis][4*N] coefficient vars for FASTER
    bool usingFaster_() const;
    void createVarsFaster_();
    void setXFaster_();
    void getCoefficientsDoubleFaster_();
    void setDynamicConstraintsFaster_();
    const LinearConstraint3D &polyAt_(int t, int p) const;
    bool hasPolytopes_() const;
    int numSpatialPolys_() const;

    // parameters
    double cost_;
    double xf_[3 * 3];
    double dirf_[2];
    double x0_[3 * 3];
    double t0_;
    double v_max_;
    double a_max_;
    double j_max_;
    double dc_;
    std::vector<dynTraj> trajs_; // Dynamic trajectory
    PieceWisePol pwp_;
    vec_Vecf<3> global_path_;
    std::vector<float> local_box_size_;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double z_min_;
    double z_max_;
    double initial_dt_;
    bool using_variable_elimination_ = true;

    // Basis converter
    BasisConverter basis_converter_;
    std::vector<std::vector<double>> M_be2mv_;
    Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv_;
    Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv_;
    Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv_;

    // Flags
    bool debug_verbose_;

    int N_of_polytopes_ = 3;

    GRBEnv *env = new GRBEnv();
    GRBModel m_ = GRBModel(*env);

    std::vector<GRBConstr> at_least_1_pol_cons_;    // Constraints at least in one polytope
    std::vector<GRBConstr> polytopes_cons_;         // for DYNUS
    std::vector<GRBGenConstr> miqp_polytopes_cons_; // for MIQP
    std::vector<GRBConstr> continuity_cons_;
    std::vector<GRBConstr> init_cons_;
    std::vector<GRBConstr> final_cons_;
    std::vector<GRBConstr> map_cons_;
    std::vector<GRBConstr> dyn_cons_;

    std::vector<std::vector<GRBVar>> b_; // binary variables (only used by the MIQP)
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

    vec_Vecf<3> samples_;          // Samples along the rescue path
    vec_Vecf<3> samples_penalize_; // Samples along the rescue path

    std::vector<double> dist_near_obs_;
    std::vector<LinearConstraint3D> polytopes_;

    // Optimization weights
    double jerk_smooth_weight_ = 10.0;

    double factor_initial_ = 0.6;
    double factor_final_ = 2.0;
    double factor_constant_step_size_ = 0.1;
    double w_max_ = 1;

    // Time-layered corridor support
    bool use_time_layered_polytopes_{false};
    int P_spatial_{0};
    std::vector<LinearConstraint3D> polytopes_time_layered_; // flattened: [t * P_spatial_ + p]
};
#endif