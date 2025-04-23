/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include "dgp/data_type.hpp"
#include <memory>
#include <octomap/octomap.h>

struct polytope
{
  Eigen::MatrixXd A;
  Eigen::MatrixXd b;
};

struct parameters
{

  // UAV or Ground robot
  std::string vehicle_type;
  bool provide_goal_in_global_frame;
  bool use_hardware;
  
  // Sensor parameters
  bool use_lidar;
  bool use_depth_camera;

  // Flight mode
  std::string flight_mode;

  // Visual level
  int visual_level;

  // Global & Local Planner parameters
  std::string planner_mode;
  bool use_free_space;

  // Global planner parameters
  bool plan_only_in_free_space;
  std::string global_planner;
  bool global_planner_verbose;
  double global_planner_huristic_weight;
  double factor_dgp;
  double inflation_dgp;
  double free_inflation_dgp;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;
  double drone_radius;
  bool use_raw_path;
  double dyn_obst_global_planner_push_k;
  double dyn_obst_global_planner_push_cov_p_alpha;
  double dyn_obst_replusion_max;
  int dgp_timeout_duration_ms;
  bool use_free_start;
  double free_start_factor;
  bool use_free_goal;
  double free_goal_factor;
  bool delete_too_close_points_in_global_path;
  double node_size_factor_for_occupied;
  double node_size_factor_for_free;

  // Path push visualization parameters
  bool use_state_update;
  bool use_random_color_for_global_path;
  bool use_path_push_for_visualization;

  // Static obstacle push parameters
  double dist_discretization;
  double max_dist_threshold_for_static_push;
  double push_force_static;
  int num_lookahead_global_path_for_push;
  double static_push_clustering_threshold;

  // Decomposition parameters
  std::vector<double> local_box_size;
  double min_dist_from_agent_to_traj;
  bool use_shrinked_box;
  double shrinked_box_size;

  // Map parameters
  double map_buffer;
  double failure_map_buffer_increment;
  double map_buffer_velocity_factor;
  double center_shift_factor;
  double initial_wdx;
  double initial_wdy;
  double initial_wdz;
  double max_wdx;
  double max_wdy;
  double max_wdz;
  double min_wdx;
  double min_wdy;
  double min_wdz;
  double lidar_max_range;
  bool use_map_res_adaptation;
  double map_res_adaptation_decrement;
  int map_res_adaptation_threshold;
  double dynus_map_res_min;
  double res;
  double octomap_res;
  double tmap_update_rate;

  // Frontiers parameters
  bool use_frontiers;
  bool use_only_d435_for_frontiers;
  double max_z_diff_from_frontier_to_camera;
  double min_dist_from_frontier_to_camera;
  double frontier_update_alpha;
  double d435_depth_min;
  bool use_mean_based_best_frontier;
  int no_visible_frontiers_threshold;
  double desired_velocity_cost_weight;
  double dist_from_z_axis_weight;
  double dist_to_prev_best_frontier_weight;
  double positive_z_camera_weight;
  double goal_proximity_weight;
  double info_gain_cost_weight;
  double frontier_neighbor_thresh_for_info_gain;
  double frontier_search_buffer;
  int frontier_min_known_free_thresh;
  int frontier_min_unknown_thresh;

  // Communication delay parameters
  bool use_comm_delay_inflation;
  double comm_delay_inflation_alpha;
  double comm_delay_inflation_max;
  double comm_delay_filter_alpha;

  // Safety check parameters
  double safety_check_dt;

  // Simulation parameters
  double depth_camera_depth_max;
  double fov_visual_depth;
  double fov_visual_x_deg;
  double fov_visual_y_deg;

  // Benchmarking parameters
  bool use_z_axis_bottom_inflation;

  // Closed-form trajectory generation parameters
  int closed_form_time_allocation_adj_iter_max;
  double closed_form_initial_factor;
  double closed_form_factor_increment;
  double closed_form_factor_initial_decrement;

  double unknown_cells_weight;
  double dynamic_obsts_weight;
  double dynamic_obsts_prox_weight;
  double dynamic_obsts_count_weight;
  int dynamic_obsts_total_max_num;
  double max_horizon;
  double min_horizon;

  // P, N adaptation parameters
  std::string p_n_mode;
  int num_N;
  int num_N_when_close_to_goal;
  int num_N_max;
  bool use_num_P_adaptation;
  int num_P_min;
  int num_P_max;
  int num_replan_fail_threshold;
  double cool_down_duration;
  double cool_down_duration_factor;

  // v_max adaptation parameters
  bool use_v_max_adaptation;
  double v_max_adaptation_v_min;
  double v_max_adaptation_p_min;
  double v_max_adaptation_p_max;
  int v_max_adaptation_obstacle_threshold;
  
  // max_dist_vertexes adaptation parameters
  bool use_max_dist_vertexes_adaptation;
  double max_dist_vertexes;
  double min_dist_vertexes;
  bool use_min_dist_cutoff;
  int max_dist_vertexes_adaptation_obstacle_threshold;
  int max_dist_vertexes_adaptation_threshold;
  double max_dist_vertexes_adaptation_decrease;

  // Local trajectory optimization parallelization parameters
  bool use_local_traj_parallelization;
  int num_factors;
  double factor_increment;
  double min_factor;
  double max_factor;

  // Optimiztion parameters
  double horizon;
  std::string optimization_type;
  bool use_ref_points;
  int num_ref_sample_points;
  double dc;
  double v_max;
  double a_max;
  double j_max;
  double gurobi_threads;
  bool gurobi_verbose;
  double control_cost_weight;
  double dynamic_weight;
  int num_obs_dist_samples;
  double final_pos_weight;
  double final_vel_weight;
  double final_accel_weight;
  double final_yaw_cost_weight;
  double total_yaw_diff_weight;
  double ref_point_weight;
  double collision_clearance;
  bool use_hard_dynamic_constraints;
  bool use_hard_constr_for_final_state;
  bool use_hard_constr_for_final_yaw;
  std::vector<double> drone_bbox;
  double goal_radius;
  double goal_seen_radius;
  double dist_to_switch_to_g_term;

  // Safe paths parameters
  int num_safe_paths;
  int min_num_safe_paths;
  double min_safe_path_distance;
  double max_safe_path_distance;

  // Contingency paths parameters
  double contingency_lateral_offset;

  // Dynamic obstacles parameters
  double traj_lifetime;
  double tracking_distance_threshold;
  double alpha_cov;
  double dynamic_obstacle_base_inflation;
  double max_dynamic_obstacle_inflation;
  double freq_dynamic_obstacle_update_to_tmap;

  // Time allocation parameters
  double factor_initial;
  double factor_final;
  double factor_gamma_up;
  double factor_gamma_down;
  double factor_constant_step_size;
  double factor_minimum;
  double factor_delta_step_size;
  bool use_constant_step_size;
  int count_to_switch_to_constant_step_size;

  // Dynamic k_value parameters
  int num_replanning_before_adapt;
  int default_k_value;
  double alpha_k_value_filtering;
  double k_value_factor;

  // Yaw-related parameters
  bool use_initial_yawing;
  bool use_yaw;
  double alpha_filter_yaw;
  double alpha_filter_dyaw;
  double w_max;
  double yaw_collision_weight = 10.0;
  double yaw_time_weight = 1.0;
  double yaw_proximity_weight = 30.0;
  double yaw_velocity_weight = 1.0;
  double yaw_change_weight = 0.1;
  double final_yaw_weight = 10.0;
  double cutoff_distance = 4.0;
  int num_yaw_fit_poly = 3;
  int yaw_fit_degree = 3;
  int num_samples_collision_likelihood;
  int num_samples_velocity_score;
  double look_ahead_secs_for_dom;
  bool yaw_optimization_debugging;
  int yaw_spinning_threshold;
  double yaw_spinning_dyaw;

  // ROI parameters
  bool use_roi;
  int num_roi_agents;
  int num_roi_traj_sample;
  double roi_map_share_buffer;

  // Simulation env parameters
  bool force_goal_z;
  double default_goal_z;

  // Debug flag
  bool debug_verbose;
  bool dist_to_term_g_verbose;
};

struct BasisConverter
{

  Eigen::Matrix<double, 4, 4> A_pos_mv_rest;
  Eigen::Matrix<double, 4, 4> A_pos_mv_rest_inv;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest;
  Eigen::Matrix<double, 3, 3> A_vel_mv_rest_inv;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest;
  Eigen::Matrix<double, 2, 2> A_accel_mv_rest_inv;
  Eigen::Matrix<double, 4, 4> A_pos_be_rest;
  Eigen::Matrix<double, 4, 4> A_pos_bs_seg0, A_pos_bs_seg1, A_pos_bs_rest, A_pos_bs_seg_last2, A_pos_bs_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2mv_seg0, M_pos_bs2mv_seg1, M_pos_bs2mv_rest, M_pos_bs2mv_seg_last2,
      M_pos_bs2mv_seg_last;

  Eigen::Matrix<double, 4, 4> M_pos_bs2be_seg0, M_pos_bs2be_seg1, M_pos_bs2be_rest, M_pos_bs2be_seg_last2,
      M_pos_bs2be_seg_last;

  Eigen::Matrix<double, 3, 3> M_vel_bs2mv_seg0, M_vel_bs2mv_rest, M_vel_bs2mv_seg_last;
  Eigen::Matrix<double, 3, 3> M_vel_bs2be_seg0, M_vel_bs2be_rest, M_vel_bs2be_seg_last;

  BasisConverter()
  {
    // See matlab.
    // This is for t \in [0 1];

    //////MATRICES A FOR MINVO POSITION///////// (there is only one)
    A_pos_mv_rest << -3.4416308968564117698463178385282,
        6.9895481477801393310755884158425, -4.4622887507045296828778191411402, 0.91437149978080234369315348885721,
        6.6792587327074839365081970754545, -11.845989901556746914934592496138, 5.2523596690684613008670567069203, 0,
        -6.6792587327074839365081970754545, 8.1917862965657040064115790301003, -1.5981560640774179482548333908198, 0.085628500219197656306846511142794,
        3.4416308968564117698463178385282, -3.3353445427890959784633650997421, 0.80808514571348655231020075007109, -0.0000000000000000084567769453869345852581318467855;

    //////INVERSE OF A_pos_mv_rest
    A_pos_mv_rest_inv = A_pos_mv_rest.inverse();

    //////MATRICES A FOR MINVO VELOCITY///////// (there is only one)
    A_vel_mv_rest << 1.4999999992328318931811281800037, -2.3660254034601951866889635311964, 0.9330127021136816189983420599674,
        -2.9999999984656637863622563600074, 2.9999999984656637863622563600074, 0,
        1.4999999992328318931811281800037, -0.6339745950054685996732928288111, 0.066987297886318325490506708774774;

    //////INVERSE OF A_vel_mv_rest
    A_vel_mv_rest_inv = A_vel_mv_rest.inverse();

    //////MATRICES A FOR MINVO ACCELERATION///////// (there is only one)
    A_accel_mv_rest << -1.0, 1.0,
        1.0, 0.0;

    /////INVERSE OF A_accel_mv_rest
    A_accel_mv_rest_inv = A_accel_mv_rest.inverse();

    //////MATRICES A FOR Bezier POSITION///////// (there is only one)
    A_pos_be_rest <<

        -1.0,
        3.0, -3.0, 1.0,
        3.0, -6.0, 3.0, 0,
        -3.0, 3.0, 0, 0,
        1.0, 0, 0, 0;

    //////MATRICES A FOR BSPLINE POSITION/////////
    A_pos_bs_seg0 <<

        -1.0000,
        3.0000, -3.0000, 1.0000,
        1.7500, -4.5000, 3.0000, 0,
        -0.9167, 1.5000, 0, 0,
        0.1667, 0, 0, 0;

    A_pos_bs_seg1 <<

        -0.2500,
        0.7500, -0.7500, 0.2500,
        0.5833, -1.2500, 0.2500, 0.5833,
        -0.5000, 0.5000, 0.5000, 0.1667,
        0.1667, 0, 0, 0;

    A_pos_bs_rest <<

        -0.1667,
        0.5000, -0.5000, 0.1667,
        0.5000, -1.0000, 0, 0.6667,
        -0.5000, 0.5000, 0.5000, 0.1667,
        0.1667, 0, 0, 0;

    A_pos_bs_seg_last2 <<

        -0.1667,
        0.5000, -0.5000, 0.1667,
        0.5000, -1.0000, 0.0000, 0.6667,
        -0.5833, 0.5000, 0.5000, 0.1667,
        0.2500, 0, 0, 0;

    A_pos_bs_seg_last <<

        -0.1667,
        0.5000, -0.5000, 0.1667,
        0.9167, -1.2500, -0.2500, 0.5833,
        -1.7500, 0.7500, 0.7500, 0.2500,
        1.0000, 0, 0, 0;

    //////BSPLINE to MINVO POSITION/////////

    M_pos_bs2mv_seg0 <<

        1.1023313949144333268037598827505,
        0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457,
        -0.049683556253749178166501110354147, 0.65780347324677179710050722860615, 0.53053863760186903419935333658941, 0.21181027098212013015654520131648,
        -0.047309044211162346038612724896666, 0.015594436894155586093013710069499, 0.5051827557159349613158383363043, 0.63650059656260427054519368539331,
        -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206, 0.057009540927778303009976212933907, 0.18372189915240558222286892942066;

    M_pos_bs2mv_seg1 <<

        0.27558284872860833170093997068761,
        0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644,
        0.6099042761975865811763242163579, 0.63806904207840509091198555324809, 0.29959938009132258684985572472215, 0.12252106674808682651445224109921,
        0.11985166952332682033244282138185, 0.29187180223752445806795208227413, 0.66657381254229419731416328431806, 0.70176522577378930289881964199594,
        -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206, 0.057009540927778303009976212933907, 0.18372189915240558222286892942066;

    M_pos_bs2mv_rest <<

        0.18372189915240555446729331379174,
        0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
        0.70176522577378919187651717948029, 0.66657381254229419731416328431806, 0.29187180223752384744528853843804, 0.11985166952332582113172065874096,
        0.11985166952332682033244282138185, 0.29187180223752445806795208227413, 0.66657381254229419731416328431806, 0.70176522577378930289881964199594,
        -0.0053387944495217444854096022766043, -0.015455155707597083292181849856206, 0.057009540927778303009976212933907, 0.18372189915240558222286892942066;

    M_pos_bs2mv_seg_last2 <<

        0.18372189915240569324517139193631,
        0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988,
        0.70176522577378952494342456702725, 0.66657381254229453038107067186502, 0.29187180223752412500104469472717, 0.11985166952332593215402312125661,
        0.1225210667480875342816304396365, 0.29959938009132280889446064975346, 0.63806904207840497988968309073243, 0.60990427619758624810941682881094,
        -0.0080081916742826154270717964323012, -0.023182733561395621468825822830695, 0.085514311391667444106623463540018, 0.27558284872860833170093997068761;

    M_pos_bs2mv_seg_last <<

        0.18372189915240555446729331379174,
        0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988,
        0.63650059656260415952289122287766, 0.5051827557159349613158383363043, 0.015594436894155294659469745965907, -0.047309044211162887272337229660479,
        0.21181027098212068526805751389475, 0.53053863760186914522165579910506, 0.65780347324677146403359984105919, -0.049683556253749622255710960416764,
        -0.032032766697130461708287185729205, -0.09273093424558248587530329132278, 0.34205724556666977642649385416007, 1.1023313949144333268037598827505;

    //////BSPLINE to BEZIER POSITION/////////

    M_pos_bs2be_seg0 <<

        1.0000,
        0.0000, -0.0000, 0,
        0, 1.0000, 0.5000, 0.2500,
        0, -0.0000, 0.5000, 0.5833,
        0, 0, 0, 0.1667;

    M_pos_bs2be_seg1 <<

        0.2500,
        0.0000, -0.0000, 0,
        0.5833, 0.6667, 0.3333, 0.1667,
        0.1667, 0.3333, 0.6667, 0.6667,
        0, 0, 0, 0.1667;

    M_pos_bs2be_rest <<

        0.1667,
        0.0000, 0, 0,
        0.6667, 0.6667, 0.3333, 0.1667,
        0.1667, 0.3333, 0.6667, 0.6667,
        0, 0, 0, 0.1667;

    M_pos_bs2be_seg_last2 <<

        0.1667,
        0, -0.0000, 0,
        0.6667, 0.6667, 0.3333, 0.1667,
        0.1667, 0.3333, 0.6667, 0.5833,
        0, 0, 0, 0.2500;

    M_pos_bs2be_seg_last <<

        0.1667,
        0.0000, 0, 0,
        0.5833, 0.5000, 0, 0,
        0.2500, 0.5000, 1.0000, 0,
        0, 0, 0, 1.0000;

    /////BSPLINE to MINVO VELOCITY
    M_vel_bs2mv_seg0 <<

        1.077349059083916,
        0.1666702138890985, -0.07735049175615138,
        -0.03867488648729411, 0.7499977187062712, 0.5386802643920123,
        -0.03867417280506149, 0.08333206631563977, 0.538670227146185;

    M_vel_bs2mv_rest <<

        0.538674529541958,
        0.08333510694454926, -0.03867524587807569,
        0.4999996430546639, 0.8333328256508203, 0.5000050185139366,
        -0.03867417280506149, 0.08333206631563977, 0.538670227146185;

    M_vel_bs2mv_seg_last <<

        0.538674529541958,
        0.08333510694454926, -0.03867524587807569,
        0.5386738158597254, 0.7500007593351806, -0.03866520863224832,
        -0.07734834561012298, 0.1666641326312795, 1.07734045429237;

    /////BSPLINE to BEZIER VELOCITY
    M_vel_bs2be_seg0 <<

        1.0000,
        0, 0,
        0, 1.0000, 0.5000,
        0, 0, 0.5000;

    M_vel_bs2be_rest <<

        0.5000,
        0, 0,
        0.5000, 1.0000, 0.5000,
        0, 0, 0.5000;

    M_vel_bs2be_seg_last <<

        0.5000,
        0, 0,
        0.5000, 1.0000, 0,
        0, 0, 1.0000;
  }

  //////MATRIX A FOR MINVO POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestMinvo()
  {
    return A_pos_mv_rest;
  }
  //////MATRIX A FOR Bezier POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBezier()
  {
    return A_pos_be_rest;
  }

  //////MATRIX A FOR BSPLINE POSITION/////////
  Eigen::Matrix<double, 4, 4> getArestBSpline()
  {
    return A_pos_bs_rest;
  }

  //////MATRICES A FOR MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getAMinvo(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_mv; // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      A_pos_mv.push_back(A_pos_mv_rest);
    }
    return A_pos_mv;
  }

  //////MATRICES A FOR Bezier POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABezier(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_be; // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      A_pos_be.push_back(A_pos_be_rest);
    }
    return A_pos_be;
  }

  //////MATRICES A FOR BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getABSpline(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> A_pos_bs; // will have as many elements as num_pol
    A_pos_bs.push_back(A_pos_bs_seg0);
    A_pos_bs.push_back(A_pos_bs_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      A_pos_bs.push_back(A_pos_bs_rest);
    }
    A_pos_bs.push_back(A_pos_bs_seg_last2);
    A_pos_bs.push_back(A_pos_bs_seg_last);
    return A_pos_bs;
  }

  //////BSPLINE to MINVO POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getMinvoPosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2mv; // will have as many elements as num_pol
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg0);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      M_pos_bs2mv.push_back(M_pos_bs2mv_rest);
    }
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last2);
    M_pos_bs2mv.push_back(M_pos_bs2mv_seg_last);
    return M_pos_bs2mv;
  }

  //////BEZIER to MINVO POSITION/////////
  //////Q_{MINVO} = M_{BEZIER2MINVO} * Q_{BEZIER}
  Eigen::Matrix<double, 4, 4> getMinvoPosConverterFromBezier()
  {

    // Compute the conversion matrix for one segment
    Eigen::Matrix<double, 4, 4> M_be2mv = A_pos_mv_rest_inv * A_pos_be_rest;

    return M_be2mv;
  }

  //////BSPLINE to BEZIER POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBezierPosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2be; // will have as many elements as num_pol
    M_pos_bs2be.push_back(M_pos_bs2be_seg0);
    M_pos_bs2be.push_back(M_pos_bs2be_seg1);
    for (int i = 0; i < (num_pol - 4); i++)
    {
      M_pos_bs2be.push_back(M_pos_bs2be_rest);
    }
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last2);
    M_pos_bs2be.push_back(M_pos_bs2be_seg_last);
    return M_pos_bs2be;
  }

  //////BSPLINE to BSPLINE POSITION/////////
  std::vector<Eigen::Matrix<double, 4, 4>> getBSplinePosConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 4, 4>> M_pos_bs2bs; // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      M_pos_bs2bs.push_back(Eigen::Matrix<double, 4, 4>::Identity());
    }
    return M_pos_bs2bs;
  }

  //////BSPLINE to MINVO Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getMinvoVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2mv; // will have as many elements as num_pol
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++)
    {
      M_vel_bs2mv.push_back(M_vel_bs2mv_rest);
    }
    M_vel_bs2mv.push_back(M_vel_bs2mv_seg_last);
    return M_vel_bs2mv;
  }

  //////BSPLINE to BEZIER Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBezierVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2be; // will have as many elements as segments
    M_vel_bs2be.push_back(M_vel_bs2be_seg0);
    for (int i = 0; i < (num_pol - 2 - 1); i++)
    {
      M_vel_bs2be.push_back(M_vel_bs2be_rest);
    }
    M_vel_bs2be.push_back(M_vel_bs2be_seg_last);
    return M_vel_bs2be;
  }

  //////BSPLINE to BSPLINE Velocity/////////
  std::vector<Eigen::Matrix<double, 3, 3>> getBSplineVelConverters(int num_pol)
  {
    std::vector<Eigen::Matrix<double, 3, 3>> M_vel_bs2bs; // will have as many elements as num_pol
    for (int i = 0; i < num_pol; i++)
    {
      M_vel_bs2bs.push_back(Eigen::Matrix<double, 3, 3>::Identity());
    }
    return M_vel_bs2bs;
  }
};

struct PieceWisePol
{
  // Interval 0: t\in[t0, t1)
  // Interval 1: t\in[t1, t2)
  // Interval 2: t\in[t2, t3)
  //...
  // Interval n-1: t\in[tn, tn+1)

  // n intervals in total

  // times has n+1 elements
  std::vector<double> times; // [t0,t1,t2,...,tn+1]

  // coefficients has n elements
  // The coeffients are such that pol(t)=coeff_of_that_interval*[u^3 u^2 u 1]
  // with u=(t-t_min_that_interval)/(t_max_that_interval- t_min_that_interval)
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_x; // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_y; // [a b c d]' of Int0 , [a b c d]' of Int1,...
  std::vector<Eigen::Matrix<double, 4, 1>> coeff_z; // [a b c d]' of Int0 , [a b c d]' of Int1,...

  void clear()
  {
    times.clear();
    coeff_x.clear();
    coeff_y.clear();
    coeff_z.clear();
  }

  // Get the end time of the trajectory
  double getEndTime() const
  {
    return times.back();
  }

  Eigen::Vector3d eval(double t) const
  {
    Eigen::Vector3d result;

    // return the last value of the polynomial in the last interval
    if (t >= times.back())
    {
      Eigen::Matrix<double, 4, 1> tmp;
      // double u = 1;
      double u = times.back() - times[times.size() - 2];
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.back().transpose() * tmp;
      result.y() = coeff_y.back().transpose() * tmp;
      result.z() = coeff_z.back().transpose() * tmp;
      return result;
    }

    // return the first value of the polynomial in the first interval
    if (t < times.front())
    {
      Eigen::Matrix<double, 4, 1> tmp;
      double u = 0;
      tmp << u * u * u, u * u, u, 1.0;
      result.x() = coeff_x.front().transpose() * tmp;
      result.y() = coeff_y.front().transpose() * tmp;
      result.z() = coeff_z.front().transpose() * tmp;
      return result;
    }

    // Find the interval where t is
    //(times - 1) is the number of intervals
    for (int i = 0; i < (times.size() - 1); i++)
    {
      if (times[i] <= t && t < times[i + 1])
      {
        // double u = (t - times[i]) / (times[i + 1] - times[i]);
        double u = t - times[i];

        // TODO: This is hand-coded for a third-degree polynomial
        Eigen::Matrix<double, 4, 1> tmp;
        tmp << u * u * u, u * u, u, 1.0;

        result.x() = coeff_x[i].transpose() * tmp;
        result.y() = coeff_y[i].transpose() * tmp;
        result.z() = coeff_z[i].transpose() * tmp;

        break;
      }
    }
    return result;
  }

  Eigen::Vector3d velocity(double t) const
  {
    Eigen::Vector3d vel;

    // Handle the case where t is after the last interval
    if (t >= times.back())
    {
      double u = times.back() - times[times.size() - 2];
      vel.x() = 3 * coeff_x.back()(0) * u * u + 2 * coeff_x.back()(1) * u + coeff_x.back()(2);
      vel.y() = 3 * coeff_y.back()(0) * u * u + 2 * coeff_y.back()(1) * u + coeff_y.back()(2);
      vel.z() = 3 * coeff_z.back()(0) * u * u + 2 * coeff_z.back()(1) * u + coeff_z.back()(2);
      return vel;
    }

    // Handle the case where t is before the first interval
    if (t < times.front())
    {
      vel.x() = coeff_x.front()(2);
      vel.y() = coeff_y.front()(2);
      vel.z() = coeff_z.front()(2);
      return vel;
    }

    // Find the interval where t lies and calculate velocity
    for (int i = 0; i < (times.size() - 1); i++)
    {
      if (times[i] <= t && t < times[i + 1])
      {
        double u = t - times[i];
        vel.x() = 3 * coeff_x[i](0) * u * u + 2 * coeff_x[i](1) * u + coeff_x[i](2);
        vel.y() = 3 * coeff_y[i](0) * u * u + 2 * coeff_y[i](1) * u + coeff_y[i](2);
        vel.z() = 3 * coeff_z[i](0) * u * u + 2 * coeff_z[i](1) * u + coeff_z[i](2);
        break;
      }
    }

    return vel;
  }

  void print()
  {
    std::cout << "coeff_x.size()= " << coeff_x.size() << std::endl;
    std::cout << "times.size()= " << times.size() << std::endl;
    std::cout << "Note that coeff_x.size() == times.size()-1" << std::endl;

    for (int i = 0; i < times.size(); i++)
    {
      printf("Time: %f\n", times[i]);
    }

    for (int i = 0; i < (times.size() - 1); i++)
    {
      std::cout << "From " << times[i] << " to " << times[i + 1] << std::endl;
      std::cout << "  Coeff_x= " << coeff_x[i].transpose() << std::endl;
      std::cout << "  Coeff_y= " << coeff_y[i].transpose() << std::endl;
      std::cout << "  Coeff_z= " << coeff_z[i].transpose() << std::endl;
    }
  }
};

struct dynTraj
{
  PieceWisePol pwp;
  Eigen::Vector3d ekf_cov_p;
  Eigen::Vector3d ekf_cov_q;
  Eigen::Vector3d poly_cov;
  std::vector<Eigen::Matrix<double, 3, 4>> control_points;
  Eigen::Vector3d bbox;
  Eigen::Vector3d goal;
  bool is_agent = false;
  int id;
  double time_received = 0.0;
  double tracking_utility = 0.0;
  double communication_delay = 0.0;

  // Define print method
  void print()
  {
    std::cout << "id= " << id << std::endl;
    std::cout << "bbox= " << bbox.transpose() << std::endl;
    std::cout << "control_points.size()= " << control_points.size() << std::endl;
    pwp.print();
  }
};

struct state
{

  // time stamp
  double t = 0.0;

  // pos, vel, accel, jerk, yaw, dyaw
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double dyaw = 0.0;

  // flag for tracking
  bool use_tracking_yaw = false;

  void setTimeStamp(const double data)
  {
    t = data;
  }

  void setPos(const double x, const double y, const double z)
  {
    pos << x, y, z;
  }
  void setVel(const double x, const double y, const double z)
  {
    vel << x, y, z;
  }
  void setAccel(const double x, const double y, const double z)
  {
    accel << x, y, z;
  }

  void setJerk(const double x, const double y, const double z)
  {
    jerk << x, y, z;
  }

  void setPos(const Eigen::Vector3d &data)
  {
    pos << data.x(), data.y(), data.z();
  }

  void setVel(const Eigen::Vector3d &data)
  {
    vel << data.x(), data.y(), data.z();
  }

  void setAccel(const Eigen::Vector3d &data)
  {
    accel << data.x(), data.y(), data.z();
  }

  void setJerk(const Eigen::Vector3d &data)
  {
    jerk << data.x(), data.y(), data.z();
  }

  void setState(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &accel, const Eigen::Vector3d &jerk)
  {
    this->pos = pos;
    this->vel = vel;
    this->accel = accel;
    this->jerk = jerk;
  }

  void setYaw(const double data)
  {
    yaw = data;
  }

  void setDYaw(const double data)
  {
    dyaw = data;
  }

  void setZero()
  {
    pos = Eigen::Vector3d::Zero();
    vel = Eigen::Vector3d::Zero();
    accel = Eigen::Vector3d::Zero();
    jerk = Eigen::Vector3d::Zero();
    yaw = 0;
    dyaw = 0;
  }

  void printPos()
  {
    std::cout << "Pos= " << pos.transpose() << std::endl;
  }

  void print()
  {
    std::cout << "Time= " << t << std::endl;
    std::cout << "Pos= " << pos.transpose() << std::endl;
    std::cout << "Vel= " << vel.transpose() << std::endl;
    std::cout << "Accel= " << accel.transpose() << std::endl;
  }

  void printHorizontal()
  {
    std::cout << "Pos, Vel, Accel, Jerk= " << pos.transpose() << " " << vel.transpose() << " " << accel.transpose()
              << " " << jerk.transpose() << std::endl;
  }
};