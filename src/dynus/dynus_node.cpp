/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <dynus/dynus_node.hpp>

// ----------------------------------------------------------------------------

/**
 * @brief Constructor
 */
DYNUS_NODE::DYNUS_NODE() : Node("dynus_node")
{
  // Get id from ns
  ns_ = this->get_namespace();
  ns_ = ns_.substr(ns_.find_last_of("/") + 1);
  id_str_ = ns_.substr(ns_.size() - 2); // ns is like NX01, so we get the last two characters and convert to int
  id_ = std::stoi(id_str_);

  // Declare the parameters
  this->declareParameters();

  // Set the parameters
  this->setParameters();

  // Print the parameters (debug)
  this->printParameters();

  // Qos policy settings
  rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
  sensor_qos.best_effort().durability_volatile();
  rclcpp::QoS critical_qos(rclcpp::KeepLast(10));
  critical_qos.reliable().durability_volatile();

  // Create callbackgroup
  this->cb_group_mu_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_5_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_6_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_7_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_8_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_mu_9_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_re_1_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_2_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_3_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_4_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_5_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_6_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_7_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_8_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_re_9_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  this->cb_group_lidar_pointcloud_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_depth_camera_pointcloud_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_replan_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->cb_group_goal_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Options for callback group
  rclcpp::SubscriptionOptions options_re_1;
  options_re_1.callback_group = this->cb_group_re_1_;
  rclcpp::SubscriptionOptions options_re_2;
  options_re_2.callback_group = this->cb_group_re_2_;
  rclcpp::SubscriptionOptions options_roi_octomap;
  options_roi_octomap.callback_group = this->cb_group_re_7_;
  rclcpp::SubscriptionOptions options_lidar_pointcloud_data;
  options_lidar_pointcloud_data.callback_group = this->cb_group_lidar_pointcloud_;
  rclcpp::SubscriptionOptions options_depth_camera_pointcloud_data;
  options_depth_camera_pointcloud_data.callback_group = this->cb_group_depth_camera_pointcloud_;

  // Publishers
  // Visulaization publishers
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupied_grid", 10);                                                              // visual level 2
  pub_dynamic_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_occupied_grid", 10);                                              // visual level 2 (no longer used)
  pub_dynamic_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_map_marker", 10);                                   // visual level 2
  pub_free_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("free_map_marker", 10);                                         // visual level 2
  pub_unknown_map_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("unknown_map_marker", 10);                                   // visual level 2
  pub_free_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("free_grid", 10);                                                             // visual level 2 (no longer used)
  pub_unknown_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_grid", 10);                                                       // visual level 2 (no longer used)
  pub_dgp_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dgp_path_marker", 10);                                         // visual level 1
  pub_free_dgp_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("free_dgp_path_marker", 10);                               // visual level 1
  pub_local_global_path_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_global_path_marker", 10);                       // visual level 1
  pub_local_global_path_after_push_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("local_global_path_after_push_marker", 10); // visual level 1
  pub_poly_whole_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("poly_whole", 10);                                                  // visual level 1
  pub_poly_safe_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("poly_safe", 10);                                                    // visual level 1
  pub_frontiers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("frontiers", 10);                                                     // visual level 2
  pub_traj_committed_colored_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("traj_committed_colored", 10);                           // visual level 1
  pub_setpoint_ = this->create_publisher<geometry_msgs::msg::PointStamped>("setpoint_vis", 10);                                                       // visual level 1
  pub_actual_traj_ = this->create_publisher<visualization_msgs::msg::Marker>("actual_traj", 10);                                                      // visual level 1
  pub_fov_ = this->create_publisher<visualization_msgs::msg::Marker>("fov", 10);                                                                      // visual level 1
  pub_cp_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("cp", 10);                                                                   // visual level 1
  pub_static_push_points_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_push_points", 10);                                   // visual level 1
  pub_p_points_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("p_points", 10);                                                       // visual level 1
  pub_point_A_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_A", 10);                                                             // visual level 1
  pub_point_G_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_G", 10);                                                             // visual level 1
  pub_point_E_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_E", 10);                                                             // visual level 1
  pub_point_G_term_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_G_term", 10);                                                   // visual level 1
  pub_current_state_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point_current_state", 10);                                           // visual level 1
  pub_vel_text_ = this->create_publisher<visualization_msgs::msg::Marker>("vel_text", 10);                                                            // visual level 1

  // Debug publishers
  pub_yaw_output_ = this->create_publisher<dynus_interfaces::msg::YawOutput>("yaw_output", 10);
  pub_pn_adaptation_ = this->create_publisher<dynus_interfaces::msg::PNAdaptation>("pn_adaptation", 10);

  // Essential publishers
  pub_own_traj_ = this->create_publisher<dynus_interfaces::msg::DynTraj>("/trajs", critical_qos);
  pub_goal_ = this->create_publisher<dynus_interfaces::msg::Goal>("goal", critical_qos);
  pub_goal_reached_ = this->create_publisher<std_msgs::msg::Empty>("goal_reached", critical_qos);

  // Prepare the publishers for ROI
  for (int i = 1; i <= par_.num_roi_agents; i++)
  {
    // Get the vehicle name
    std::string vehicle_name = (i > 9) ? "NX" + std::to_string(i) : "NX0" + std::to_string(i);

    // Create a new publisher and insert it to the map
    pub_roi_octomap_[i] = this->create_publisher<octomap_msgs::msg::Octomap>("/" + vehicle_name + "/roi_octomap", 10);

    // Same for the dynamic obstacles
    pub_roi_trajs_[i] = this->create_publisher<dynus_interfaces::msg::DynTrajArray>("/" + vehicle_name + "/roi_trajs", 10);
  }

  // Subscribers
  sub_traj_ = this->create_subscription<dynus_interfaces::msg::DynTraj>("/trajs", critical_qos, std::bind(&DYNUS_NODE::trajCallback, this, std::placeholders::_1), options_re_1);
  sub_predicted_traj_ = this->create_subscription<dynus_interfaces::msg::DynTraj>("predicted_trajs", critical_qos, std::bind(&DYNUS_NODE::trajCallback, this, std::placeholders::_1), options_re_1);
  // sub_roi_traj_ = this->create_subscription<dynus_interfaces::msg::DynTrajArray>("roi_trajs", critical_qos, std::bind(&DYNUS_NODE::roiTrajCallback, this, std::placeholders::_1), options_re_1);
  sub_state_ = this->create_subscription<dynus_interfaces::msg::State>("state", critical_qos, std::bind(&DYNUS_NODE::stateCallback, this, std::placeholders::_1), options_re_1);
  sub_start_exploration_ = this->create_subscription<std_msgs::msg::Empty>("start_exploration", critical_qos, std::bind(&DYNUS_NODE::startExplorationCallback, this, std::placeholders::_1)); // TODO: make it a service
  sub_terminal_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("term_goal", critical_qos, std::bind(&DYNUS_NODE::terminalGoalCallback, this, std::placeholders::_1));
  sub_roi_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>("roi_octomap", sensor_qos, std::bind(&DYNUS_NODE::octomapCallbackForROI, this, std::placeholders::_1), options_roi_octomap);
  if (par_.flight_mode == "exploration")
    sub_successful_detection_ = this->create_subscription<std_msgs::msg::String>("successful_detection", critical_qos, std::bind(&DYNUS_NODE::successfulDetectionCallback, this, std::placeholders::_1));

  // Timer for callback
  timer_visualize_static_map_ = this->create_wall_timer(500ms, std::bind(&DYNUS_NODE::publishStaticMap, this), this->cb_group_mu_1_);
  timer_visualize_dynamic_map_ = this->create_wall_timer(100ms, std::bind(&DYNUS_NODE::publishDynamicMap, this), this->cb_group_mu_2_);
  timer_replanning_ = this->create_wall_timer(10ms, std::bind(&DYNUS_NODE::replanCallback, this), this->cb_group_replan_);
  timer_goal_ = this->create_wall_timer(std::chrono::duration<double>(par_.dc), std::bind(&DYNUS_NODE::publishGoal, this), this->cb_group_goal_);
  if (par_.use_frontiers)
    timer_frontiers_ = this->create_wall_timer(10ms, std::bind(&DYNUS_NODE::findFrontiersCallback, this), this->cb_group_mu_4_);
  timer_share_roi_map_and_trajs_ = this->create_wall_timer(500ms, std::bind(&DYNUS_NODE::shareROIOctomapAndTrajsCallback, this), this->cb_group_re_4_);
  timer_update_tmap_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / par_.tmap_update_rate), std::bind(&DYNUS_NODE::updateTMapCallback, this), this->cb_group_mu_6_);
  if (use_benchmark_)
    timer_goal_reached_check_ = this->create_wall_timer(100ms, std::bind(&DYNUS_NODE::goalReachedCheckCallback, this), this->cb_group_re_3_);
  timer_cleanup_old_trajs_ = this->create_wall_timer(500ms, std::bind(&DYNUS_NODE::cleanUpOldTrajsCallback, this), this->cb_group_mu_5_);
  timer_trajs_update_for_tmap_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / par_.freq_dynamic_obstacle_update_to_tmap), std::bind(&DYNUS_NODE::updateTrajsForTMapCallback, this), this->cb_group_mu_7_);
  timer_check_plan_safety_ = this->create_wall_timer(10ms, std::bind(&DYNUS_NODE::checkFuturePlanSafetyCallback, this), this->cb_group_mu_8_);
  if (par_.use_hardware)
    timer_initial_pose_ = this->create_wall_timer(100ms, std::bind(&DYNUS_NODE::getInitialPoseHwCallback, this), this->cb_group_mu_9_);

  // Stop the timer for callback
  timer_replanning_->cancel();
  timer_goal_->cancel();
  timer_visualize_static_map_->cancel();
  timer_visualize_dynamic_map_->cancel();
  if (par_.use_frontiers)
    timer_frontiers_->cancel();
  timer_share_roi_map_and_trajs_->cancel();
  if (use_benchmark_)
    timer_goal_reached_check_->cancel();

  // Initialize the DYNUS object
  dynus_ptr_ = std::make_shared<DYNUS>(par_);

  // Initialize the tf2 buffer and listener
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);

  // Initialize the d435 depth frame ID and camera
  d435_depth_frame_id_ = ns_ + "/" + ns_ + "_d435_depth_optical_frame";
  lidar_frame_id_ = ns_ + "/base_link";

  // Construct the FOV marker
  constructFOVMarker();

  // Initialize the initial pose topic name
  initial_pose_topic_ = ns_ + "/init_pose";

  // Later in your initialization code:
  lidar_octree_ = std::make_shared<OcTreeT>(par_.octomap_res, decay_duration_);
  // lidar_octree_->setOccupancyThres(sensor_model_occupancy_thres_);
  // lidar_octree_->setProbHit(sensor_model_hit_);
  // lidar_octree_->setProbMiss(sensor_model_miss_);
  // lidar_octree_->setClampingThresMin(sensor_model_min_);
  // lidar_octree_->setClampingThresMax(sensor_model_max_);
  tree_depth_ = lidar_octree_->getTreeDepth(); // This value is fixed to 16
  max_tree_depth_ = tree_depth_;

  depth_camera_octree_ = std::make_shared<OcTreeT>(par_.octomap_res, decay_duration_);
  // depth_camera_octree_->setOccupancyThres(sensor_model_occupancy_thres_);
  // depth_camera_octree_->setProbHit(sensor_model_hit_);
  // depth_camera_octree_->setProbMiss(sensor_model_miss_);
  // depth_camera_octree_->setClampingThresMin(sensor_model_min_);
  // depth_camera_octree_->setClampingThresMax(sensor_model_max_);

  if (filter_ground_plane_ && (point_cloud_min_z_ > 0.0 || point_cloud_max_z_ < 0.0))
  {
    std::cout << "You enabled ground filtering but incoming pointclouds will be pre-filtered in [" << point_cloud_min_z_ << ", " << point_cloud_max_z_ << "], excluding the ground level z=0. "
                                                                                                                                                          "This will not work."
              << std::endl;
  }

  if (use_height_map_ && use_colored_map_)
  {
    std::cout << "You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map." << std::endl;
    use_colored_map_ = false;
  }

  if (use_colored_map_)
  {
    std::cout << "Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node" << std::endl;
  }

  if (latched_topics_)
  {
    RCLCPP_INFO(
        get_logger(),
        "Publishing latched (single publish will take longer, "
        "all topics are prepared)");
  }
  else
  {
    RCLCPP_INFO(
        get_logger(),
        "Publishing non-latched (topics are only prepared as needed, "
        "will only be re-published on map change");
  }

  // auto qos = latched_topics_ ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
  lidar_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("lidar/occupied_cells_vis_array", 10);
  lidar_free_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("lidar/free_cells_vis_array", 10);
  depth_camera_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("depth_camera/occupied_cells_vis_array", 10);
  depth_camera_free_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("depth_camera/free_cells_vis_array", 10);

  // Create message_filters subscribers with the subscription options.
  lidar_point_cloud_sub_.subscribe(this, "lidar_cloud_in", rmw_qos_profile_sensor_data, options_lidar_pointcloud_data);
  depth_camera_point_cloud_sub_.subscribe(this, "depth_camera_cloud_in", rmw_qos_profile_sensor_data, options_depth_camera_pointcloud_data);

  // Now create the tf2 message filters. They will use the underlying subscriptions,
  // which are assigned to the mutually exclusive callback group.
  lidar_tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      lidar_point_cloud_sub_, *tf2_buffer_, world_frame_id_, 1,
      this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(10));

  depth_camera_tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      depth_camera_point_cloud_sub_, *tf2_buffer_, world_frame_id_, 1,
      this->get_node_logging_interface(), this->get_node_clock_interface(), std::chrono::seconds(10));

  // Register the same callback for both (inside use different octree pointers)
  if (par_.use_lidar)
    lidar_tf_point_cloud_sub_->registerCallback(&DYNUS_NODE::insertLidarCloudCallback, this);
  if (par_.use_depth_camera)
  depth_camera_tf_point_cloud_sub_->registerCallback(&DYNUS_NODE::insertDepthCameraCloudCallback, this);

  last_lidar_callback_time_ = this->now();
  last_depth_camera_callback_time_ = this->now();
}

// ----------------------------------------------------------------------------

/**
 * @brief Destructor
 */
DYNUS_NODE::~DYNUS_NODE()
{
  // release the memory
  dynus_ptr_.reset();
}

// ----------------------------------------------------------------------------

/**
 * @brief Declare the parameters
 */
void DYNUS_NODE::declareParameters()
{
  // UAV or Ground robot
  this->declare_parameter("vehicle_type", "uav");
  this->declare_parameter("provide_goal_in_global_frame", false);
  this->declare_parameter("use_hardware", false);

  // Sensor parameters
  this->declare_parameter("use_lidar", true);
  this->declare_parameter("use_depth_camera", true);

  // Flight mode
  this->declare_parameter("flight_mode", "exploration");

  // Visual
  this->declare_parameter("visual_level", 1);

  // Global & Local planner parameters
  this->declare_parameter("planner_mode", "dynus");
  this->declare_parameter("plan_only_in_free_space", false);
  this->declare_parameter("use_free_space", true);

  // Global planner parameters
  this->declare_parameter("file_path", "/home/kkondo/code/dynus_ws/src/dynus/data/data.txt");
  this->declare_parameter("use_benchmark", false);
  this->declare_parameter("start_yaw", -90.0);
  this->declare_parameter("global_planner", "sjps");
  this->declare_parameter("global_planner_verbose", false);
  this->declare_parameter("global_planner_huristic_weight", 1.0);
  this->declare_parameter("factor_dgp", 1.0);
  this->declare_parameter("inflation_dgp", 0.5);
  this->declare_parameter("free_inflation_dgp", 0.5);
  this->declare_parameter("x_min", -100.0);
  this->declare_parameter("x_max", 100.0);
  this->declare_parameter("y_min", -100.0);
  this->declare_parameter("y_max", 100.0);
  this->declare_parameter("z_min", 0.0);
  this->declare_parameter("z_max", 5.0);
  this->declare_parameter("use_raw_path", false);
  this->declare_parameter("dyn_obst_global_planner_push_k", 2.0);
  this->declare_parameter("dyn_obst_global_planner_push_cov_p_alpha", 100.0);
  this->declare_parameter("dyn_obst_replusion_max", 2.0);
  this->declare_parameter("dgp_timeout_duration_ms", 1000);
  this->declare_parameter("use_free_start", false);
  this->declare_parameter("free_start_factor", 1.0);
  this->declare_parameter("use_free_goal", false);
  this->declare_parameter("free_goal_factor", 1.0);
  this->declare_parameter("delete_too_close_points_in_global_path", false);
  this->declare_parameter("node_size_factor_for_occupied", 0.5);
  this->declare_parameter("node_size_factor_for_free", 0.5);

  // Path push visualization parameters
  this->declare_parameter("use_state_update", true);
  this->declare_parameter("use_random_color_for_global_path", false);
  this->declare_parameter("use_path_push_for_visualization", false);

  // Static obstacle push parameters
  this->declare_parameter("dist_discretization", 0.2);
  this->declare_parameter("max_dist_threshold_for_static_push", 3.0);
  this->declare_parameter("push_force_static", 0.5);
  this->declare_parameter("num_lookahead_global_path_for_push", 3);
  this->declare_parameter("static_push_clustering_threshold", 1.0);

  // Decomposition parameters
  this->declare_parameter("local_box_size", std::vector<float>{2.0, 2.0, 2.0});
  this->declare_parameter("min_dist_from_agent_to_traj", 6.0);
  this->declare_parameter("use_shrinked_box", false);
  this->declare_parameter("shrinked_box_size", 0.2);

  // Map parameters
  this->declare_parameter("map_buffer", 6.0);
  this->declare_parameter("failure_map_buffer_increment", 0.5);
  this->declare_parameter("map_buffer_velocity_factor", 1.0);
  this->declare_parameter("center_shift_factor", 0.5);
  this->declare_parameter("initial_wdx", 1.0);
  this->declare_parameter("initial_wdy", 1.0);
  this->declare_parameter("initial_wdz", 1.0);
  this->declare_parameter("max_wdx", 40.0);
  this->declare_parameter("max_wdy", 40.0);
  this->declare_parameter("max_wdz", 10.0);
  this->declare_parameter("min_wdx", 10.0);
  this->declare_parameter("min_wdy", 10.0);
  this->declare_parameter("min_wdz", 2.0);
  this->declare_parameter("lidar_max_range", 20.0);
  this->declare_parameter("use_map_res_adaptation", true);
  this->declare_parameter("map_res_adaptation_decrement", 0.05);
  this->declare_parameter("map_res_adaptation_threshold", 10);
  this->declare_parameter("dynus_map_res", 0.1);
  this->declare_parameter("dynus_map_res_min", 0.1);
  this->declare_parameter("octomap_res", 0.1);
  this->declare_parameter("tmap_update_rate", 10.0);

  // Frontiers parameters
  this->declare_parameter("use_frontiers", false);
  this->declare_parameter("use_only_d435_for_frontiers", false);
  this->declare_parameter("max_z_diff_from_frontier_to_camera", 1.0);
  this->declare_parameter("min_dist_from_frontier_to_camera", 1.0);
  this->declare_parameter("frontier_update_alpha", 0.8);
  this->declare_parameter("d435_depth_min", 3.0);
  this->declare_parameter("use_mean_based_best_frontier", true);
  this->declare_parameter("no_visible_frontiers_threshold", 10);
  this->declare_parameter("desired_velocity_cost_weight", 1.0);
  this->declare_parameter("dist_from_z_axis_weight", 1.0);
  this->declare_parameter("dist_to_prev_best_frontier_weight", 1.0);
  this->declare_parameter("positive_z_camera_weight", 1.0);
  this->declare_parameter("goal_proximity_weight", 1.0);
  this->declare_parameter("info_gain_cost_weight", 5.0);
  this->declare_parameter("frontier_neighbor_thresh_for_info_gain", 1.0);
  this->declare_parameter("frontier_search_buffer", 1.0);
  this->declare_parameter("frontier_min_known_free_thresh", 1);
  this->declare_parameter("frontier_min_unknown_thresh", 2);

  // Communication delay parameters
  this->declare_parameter("use_comm_delay_inflation", true);
  this->declare_parameter("comm_delay_inflation_alpha", 0.2);
  this->declare_parameter("comm_delay_inflation_max", 0.5);
  this->declare_parameter("comm_delay_filter_alpha", 0.8);

  // Safety check parameters
  this->declare_parameter("safety_check_dt", 0.1);

  // Simulation parameters
  this->declare_parameter("depth_camera_depth_max", 10.0);
  this->declare_parameter("fov_visual_depth", 10.0);
  this->declare_parameter("fov_visual_x_deg", 10.0);
  this->declare_parameter("fov_visual_y_deg", 10.0);

  // Benchmarking parameters
  this->declare_parameter("use_z_axis_bottom_inflation", false);

  // Closed-form trajectory generation parameters
  this->declare_parameter("closed_form_time_allocation_adj_iter_max", 10);
  this->declare_parameter("closed_form_initial_factor", 0.8);
  this->declare_parameter("closed_form_factor_increment", 0.1);
  this->declare_parameter("closed_form_factor_initial_decrement", 0.3);

  // Horizon adaptation parameters
  this->declare_parameter("unknown_cells_weight", 0.5);
  this->declare_parameter("dynamic_obsts_weight", 0.5);
  this->declare_parameter("dynamic_obsts_prox_weight", 0.5);
  this->declare_parameter("dynamic_obsts_count_weight", 0.5);
  this->declare_parameter("dynamic_obsts_total_max_num", 5);
  this->declare_parameter("max_horizon", 20.0);
  this->declare_parameter("min_horizon", 5.0);

  // P, N adaptation parameters
  this->declare_parameter("p_n_mode", "dynus");
  this->declare_parameter("num_N", 3);
  this->declare_parameter("num_N_when_close_to_goal", 5);
  this->declare_parameter("num_N_max", 6);
  this->declare_parameter("use_num_P_adaptation", true);
  this->declare_parameter("num_P_min", 2);
  this->declare_parameter("num_P_max", 2);
  this->declare_parameter("num_replan_fail_threshold", 10);
  this->declare_parameter("cool_down_duration", 10.0);
  this->declare_parameter("cool_down_duration_factor", 3.0);

  // v_max adaptation parameters
  this->declare_parameter("use_v_max_adaptation", true);
  this->declare_parameter("v_max_adaptation_v_min", 1.0);
  this->declare_parameter("v_max_adaptation_p_min", 0.1);
  this->declare_parameter("v_max_adaptation_p_max", 100.0);
  this->declare_parameter("v_max_adaptation_obstacle_threshold", 5);

  // max_dist_veterxes adaptation parameters
  this->declare_parameter("use_max_dist_vertexes_adaptation", true);
  this->declare_parameter("use_min_dist_cutoff", false);
  this->declare_parameter("max_dist_vertexes", 5.0);
  this->declare_parameter("min_dist_vertexes", 0.5);
  this->declare_parameter("max_dist_vertexes_adaptation_obstacle_threshold", 3);
  this->declare_parameter("max_dist_vertexes_adaptation_threshold", 10);
  this->declare_parameter("max_dist_vertexes_adaptation_decrease", 0.1);

  // Local trajectory optimization parallelization parameters
  this->declare_parameter("use_local_traj_parallelization", true);
  this->declare_parameter("num_factors", 4);
  this->declare_parameter("factor_increment", 0.1);
  this->declare_parameter("min_factor", 0.1);
  this->declare_parameter("max_factor", 0.1);

  // Optimization parameters
  this->declare_parameter("horizon", 20.0);
  this->declare_parameter("optimization_type", "hard");
  this->declare_parameter("use_ref_points", true);
  this->declare_parameter("num_ref_sample_points", 5);
  this->declare_parameter("dc", 0.01);
  this->declare_parameter("v_max", 1.0);
  this->declare_parameter("a_max", 1.0);
  this->declare_parameter("j_max", 1.0);
  this->declare_parameter("gurobi_threads", 0);
  this->declare_parameter("gurobi_verbose", false);
  this->declare_parameter("control_cost_weight", 1.0);
  this->declare_parameter("dynamic_weight", 1.0);
  this->declare_parameter("num_obs_dist_samples", 10);
  this->declare_parameter("final_pos_weight", 1.0);
  this->declare_parameter("final_vel_weight", 1.0);
  this->declare_parameter("final_accel_weight", 1.0);
  this->declare_parameter("final_yaw_cost_weight", 1.0);
  this->declare_parameter("total_yaw_diff_weight", 1.0);
  this->declare_parameter("ref_point_weight", 1.0);
  this->declare_parameter("collision_clearance", 1.0);
  this->declare_parameter("use_hard_constr_for_final_state", false);
  this->declare_parameter("use_hard_dynamic_constraints", false);
  this->declare_parameter("use_hard_constr_for_final_yaw", false);
  this->declare_parameter("verbose_computation_time", false);
  this->declare_parameter("drone_bbox", std::vector<double>{0.5, 0.5, 0.5});
  this->declare_parameter("goal_radius", 0.5);
  this->declare_parameter("goal_seen_radius", 2.0);
  this->declare_parameter("dist_to_switch_to_g_term", 5.0);

  // Safe paths parameters
  this->declare_parameter("num_safe_paths", 5);
  this->declare_parameter("min_num_safe_paths", 3);
  this->declare_parameter("min_safe_path_distance", 1.0);
  this->declare_parameter("max_safe_path_distance", 2.0);

  // Contingency paths parameters
  this->declare_parameter("contingency_lateral_offset", 0.5);

  // Dynamic obstacles parameters
  this->declare_parameter("traj_lifetime", 10.0);
  this->declare_parameter("tracking_distance_threshold", 5.0);
  this->declare_parameter("alpha_cov", 0.1);
  this->declare_parameter("dynamic_obstacle_base_inflation", 0.5);
  this->declare_parameter("max_dynamic_obstacle_inflation", 1.0);
  this->declare_parameter("freq_dynamic_obstacle_update_to_tmap", 1.0);

  // Time allocation parameters
  this->declare_parameter("factor_initial", 1.0);
  this->declare_parameter("factor_final", 1.5);
  this->declare_parameter("factor_gamma_up", 1.2);
  this->declare_parameter("factor_gamma_down", 0.8);
  this->declare_parameter("factor_constant_step_size", 0.1);
  this->declare_parameter("factor_minimum", 0.5);
  this->declare_parameter("factor_delta_step_size", 0.1);
  this->declare_parameter("use_constant_step_size", false);
  this->declare_parameter("count_to_switch_to_constant_step_size", 10);

  // Dynamic k_value parameters
  this->declare_parameter("num_replanning_before_adapt", 10);
  this->declare_parameter("default_k_value", 150);
  this->declare_parameter("alpha_k_value_filtering", 0.8);
  this->declare_parameter("k_value_factor", 1.2);

  // Yaw-related parameters
  this->declare_parameter("use_initial_yawing", false);
  this->declare_parameter("use_yaw", false);
  this->declare_parameter("alpha_filter_yaw", 0.8);
  this->declare_parameter("alpha_filter_dyaw", 0.8);
  this->declare_parameter("w_max", 0.5);
  this->declare_parameter("yaw_collision_weight", 10.0);
  this->declare_parameter("yaw_time_weight", 1.0);
  this->declare_parameter("yaw_proximity_weight", 30.0);
  this->declare_parameter("yaw_velocity_weight", 1.0);
  this->declare_parameter("yaw_change_weight", 0.1);
  this->declare_parameter("final_yaw_weight", 10.0);
  this->declare_parameter("cutoff_distance", 4.0);
  this->declare_parameter("num_yaw_fit_poly", 3);
  this->declare_parameter("yaw_fit_degree", 3);
  this->declare_parameter("num_samples_collision_likelihood", 5);
  this->declare_parameter("num_samples_velocity_score", 5);
  this->declare_parameter("look_ahead_secs_for_dom", 5.0);
  this->declare_parameter("yaw_optimization_debugging", false);
  this->declare_parameter("yaw_spinning_threshold", 10);
  this->declare_parameter("yaw_spinning_dyaw", 0.1);

  // ROI parameters
  this->declare_parameter("use_roi", true);
  this->declare_parameter("num_roi_agents", 5);
  this->declare_parameter("num_roi_traj_sample", 5);
  this->declare_parameter("roi_map_share_buffer", 2.0);

  // Simulation env parameters
  this->declare_parameter("force_goal_z", true);
  this->declare_parameter("default_goal_z", 2.5);

  // Debug flag
  this->declare_parameter("debug_verbose", false);
  this->declare_parameter("dist_to_term_g_verbose", false);

  // Octomap parameters
  this->declare_parameter("world_frame_id", "map");
  this->declare_parameter("use_height_map", false);
  this->declare_parameter("use_colored_map", false);
  this->declare_parameter("color_factor", 0.8);
  this->declare_parameter("point_cloud_min_x", -200.0);
  this->declare_parameter("point_cloud_max_x", 200.0);
  this->declare_parameter("point_cloud_min_y", -200.0);
  this->declare_parameter("point_cloud_max_y", 200.0);
  this->declare_parameter("point_cloud_min_z", 0.0);  // Minimum height of points to consider for insertion
  this->declare_parameter("point_cloud_max_z", 10.0); // Maximum height of points to consider for insertion
  this->declare_parameter("filter_speckles", false);
  this->declare_parameter("filter_ground_plane", false);
  this->declare_parameter("ground_filter.distance", 0.04); // Distance threshold to consider a point as ground
  this->declare_parameter("ground_filter.angle", 0.15);
  this->declare_parameter("ground_filter.plane_distance", 0.07);
  this->declare_parameter("use_decay", false);
  this->declare_parameter("decay_duration", 5.0);
  this->declare_parameter("sensor_model.occupancy_thres", 0.5); // Threshold for occupancy
  this->declare_parameter("sensor_model.hit", 0.7);             // Probabilities for hits in the sensor model when dynamically building a map
  this->declare_parameter("sensor_model.miss", 0.4);            // Probabilities for misses in the sensor model when dynamically building a map
  this->declare_parameter("sensor_model.min", 0.12);            // Minimum probability for clamping when dynamically building a map
  this->declare_parameter("sensor_model.max", 0.97);            // Maximum probability for clamping when dynamically building a map
  this->declare_parameter("compress_map", true);                // Compresses the map losslessly
  this->declare_parameter("color.r", 0.0);
  this->declare_parameter("color.g", 0.0);
  this->declare_parameter("color.b", 1.0);
  this->declare_parameter("color.a", 1.0);
  this->declare_parameter("color_free.r", 0.0);
  this->declare_parameter("color_free.g", 1.0);
  this->declare_parameter("color_free.b", 0.0);
  this->declare_parameter("color_free.a", 0.5);
  this->declare_parameter("latched_topics", false);
  this->declare_parameter("publish_free_space", true);
  this->declare_parameter("lidar_insert_hz", 5.0);
  this->declare_parameter("depth_camera_insert_hz", 5.0);
}

// ----------------------------------------------------------------------------

/**
 * @brief Set the parameters
 */
void DYNUS_NODE::setParameters()
{
  // Set the parameters

  // Vehicle type (UAV, Wheeled Robit, or Quadruped)
  par_.vehicle_type = this->get_parameter("vehicle_type").as_string();
  par_.provide_goal_in_global_frame = this->get_parameter("provide_goal_in_global_frame").as_bool();
  par_.use_hardware = this->get_parameter("use_hardware").as_bool();

  // Sensor parameters
  par_.use_lidar = this->get_parameter("use_lidar").as_bool();
  par_.use_depth_camera = this->get_parameter("use_depth_camera").as_bool();

  // Flight mode
  par_.flight_mode = this->get_parameter("flight_mode").as_string();

  // Visual level
  par_.visual_level = this->get_parameter("visual_level").as_int();

  // Global & Local Planner parameters
  par_.planner_mode = this->get_parameter("planner_mode").as_string();
  par_.plan_only_in_free_space = this->get_parameter("plan_only_in_free_space").as_bool();
  par_.use_free_space = this->get_parameter("use_free_space").as_bool();

  // Global Planner parameters
  file_path_ = this->get_parameter("file_path").as_string();
  use_benchmark_ = this->get_parameter("use_benchmark").as_bool();
  par_.global_planner = this->get_parameter("global_planner").as_string();
  par_.global_planner_verbose = this->get_parameter("global_planner_verbose").as_bool();
  par_.global_planner_huristic_weight = this->get_parameter("global_planner_huristic_weight").as_double();
  par_.factor_dgp = this->get_parameter("factor_dgp").as_double();
  par_.inflation_dgp = this->get_parameter("inflation_dgp").as_double();
  par_.free_inflation_dgp = this->get_parameter("free_inflation_dgp").as_double();
  par_.x_min = this->get_parameter("x_min").as_double();
  par_.x_max = this->get_parameter("x_max").as_double();
  par_.y_min = this->get_parameter("y_min").as_double();
  par_.y_max = this->get_parameter("y_max").as_double();
  par_.z_min = this->get_parameter("z_min").as_double();
  par_.z_max = this->get_parameter("z_max").as_double();
  par_.use_raw_path = this->get_parameter("use_raw_path").as_bool();
  par_.dyn_obst_global_planner_push_k = this->get_parameter("dyn_obst_global_planner_push_k").as_double();
  par_.dyn_obst_global_planner_push_cov_p_alpha = this->get_parameter("dyn_obst_global_planner_push_cov_p_alpha").as_double();
  par_.dyn_obst_replusion_max = this->get_parameter("dyn_obst_replusion_max").as_double();
  par_.dgp_timeout_duration_ms = this->get_parameter("dgp_timeout_duration_ms").as_int();
  par_.use_free_start = this->get_parameter("use_free_start").as_bool();
  par_.free_start_factor = this->get_parameter("free_start_factor").as_double();
  par_.use_free_goal = this->get_parameter("use_free_goal").as_bool();
  par_.free_goal_factor = this->get_parameter("free_goal_factor").as_double();
  par_.delete_too_close_points_in_global_path = this->get_parameter("delete_too_close_points_in_global_path").as_bool();
  par_.node_size_factor_for_occupied = this->get_parameter("node_size_factor_for_occupied").as_double();
  par_.node_size_factor_for_free = this->get_parameter("node_size_factor_for_free").as_double();

  // Path push visualization parameters
  par_.use_state_update = this->get_parameter("use_state_update").as_bool();
  par_.use_random_color_for_global_path = this->get_parameter("use_random_color_for_global_path").as_bool();
  par_.use_path_push_for_visualization = this->get_parameter("use_path_push_for_visualization").as_bool();

  // Static obstacle push parameters
  par_.dist_discretization = this->get_parameter("dist_discretization").as_double();
  par_.max_dist_threshold_for_static_push = this->get_parameter("max_dist_threshold_for_static_push").as_double();
  par_.push_force_static = this->get_parameter("push_force_static").as_double();
  par_.num_lookahead_global_path_for_push = this->get_parameter("num_lookahead_global_path_for_push").as_int();
  par_.static_push_clustering_threshold = this->get_parameter("static_push_clustering_threshold").as_double();

  // Decomposition parameters
  par_.local_box_size = this->get_parameter("local_box_size").as_double_array();
  par_.min_dist_from_agent_to_traj = this->get_parameter("min_dist_from_agent_to_traj").as_double();
  par_.use_shrinked_box = this->get_parameter("use_shrinked_box").as_bool();
  par_.shrinked_box_size = this->get_parameter("shrinked_box_size").as_double();

  // Map parameters
  par_.map_buffer = this->get_parameter("map_buffer").as_double();
  par_.failure_map_buffer_increment = this->get_parameter("failure_map_buffer_increment").as_double();
  par_.map_buffer_velocity_factor = this->get_parameter("map_buffer_velocity_factor").as_double();
  par_.center_shift_factor = this->get_parameter("center_shift_factor").as_double();
  par_.initial_wdx = this->get_parameter("initial_wdx").as_double();
  par_.initial_wdy = this->get_parameter("initial_wdy").as_double();
  par_.initial_wdz = this->get_parameter("initial_wdz").as_double();
  par_.max_wdx = this->get_parameter("max_wdx").as_double();
  par_.max_wdy = this->get_parameter("max_wdy").as_double();
  par_.max_wdz = this->get_parameter("max_wdz").as_double();
  par_.min_wdx = this->get_parameter("min_wdx").as_double();
  par_.min_wdy = this->get_parameter("min_wdy").as_double();
  par_.min_wdz = this->get_parameter("min_wdz").as_double();
  par_.lidar_max_range = this->get_parameter("lidar_max_range").as_double();
  par_.use_map_res_adaptation = this->get_parameter("use_map_res_adaptation").as_bool();
  par_.map_res_adaptation_decrement = this->get_parameter("map_res_adaptation_decrement").as_double();
  par_.map_res_adaptation_threshold = this->get_parameter("map_res_adaptation_threshold").as_int();
  par_.res = this->get_parameter("dynus_map_res").as_double();
  par_.dynus_map_res_min = this->get_parameter("dynus_map_res_min").as_double();
  par_.octomap_res = this->get_parameter("octomap_res").as_double();
  par_.tmap_update_rate = this->get_parameter("tmap_update_rate").as_double();

  // Frontiers parameters
  par_.use_frontiers = this->get_parameter("use_frontiers").as_bool();
  par_.use_only_d435_for_frontiers = this->get_parameter("use_only_d435_for_frontiers").as_bool();
  par_.max_z_diff_from_frontier_to_camera = this->get_parameter("max_z_diff_from_frontier_to_camera").as_double();
  par_.min_dist_from_frontier_to_camera = this->get_parameter("min_dist_from_frontier_to_camera").as_double();
  par_.frontier_update_alpha = this->get_parameter("frontier_update_alpha").as_double();
  par_.d435_depth_min = this->get_parameter("d435_depth_min").as_double();
  par_.use_mean_based_best_frontier = this->get_parameter("use_mean_based_best_frontier").as_bool();
  par_.no_visible_frontiers_threshold = this->get_parameter("no_visible_frontiers_threshold").as_int();
  par_.desired_velocity_cost_weight = this->get_parameter("desired_velocity_cost_weight").as_double();
  par_.dist_from_z_axis_weight = this->get_parameter("dist_from_z_axis_weight").as_double();
  par_.dist_to_prev_best_frontier_weight = this->get_parameter("dist_to_prev_best_frontier_weight").as_double();
  par_.positive_z_camera_weight = this->get_parameter("positive_z_camera_weight").as_double();
  par_.goal_proximity_weight = this->get_parameter("goal_proximity_weight").as_double();
  par_.info_gain_cost_weight = this->get_parameter("info_gain_cost_weight").as_double();
  par_.frontier_neighbor_thresh_for_info_gain = this->get_parameter("frontier_neighbor_thresh_for_info_gain").as_double();
  par_.frontier_search_buffer = this->get_parameter("frontier_search_buffer").as_double();
  par_.frontier_min_known_free_thresh = this->get_parameter("frontier_min_known_free_thresh").as_int();
  par_.frontier_min_unknown_thresh = this->get_parameter("frontier_min_unknown_thresh").as_int();

  // Communication delay parameters
  par_.use_comm_delay_inflation = this->get_parameter("use_comm_delay_inflation").as_bool();
  par_.comm_delay_inflation_alpha = this->get_parameter("comm_delay_inflation_alpha").as_double();
  par_.comm_delay_inflation_max = this->get_parameter("comm_delay_inflation_max").as_double();
  par_.comm_delay_filter_alpha = this->get_parameter("comm_delay_filter_alpha").as_double();

  // Safety check parameters
  par_.safety_check_dt = this->get_parameter("safety_check_dt").as_double();

  // Simulation parameters
  par_.depth_camera_depth_max = this->get_parameter("depth_camera_depth_max").as_double();
  par_.fov_visual_depth = this->get_parameter("fov_visual_depth").as_double();
  par_.fov_visual_x_deg = this->get_parameter("fov_visual_x_deg").as_double();
  par_.fov_visual_y_deg = this->get_parameter("fov_visual_y_deg").as_double();

  // Benchmarking parameters
  par_.use_z_axis_bottom_inflation = this->get_parameter("use_z_axis_bottom_inflation").as_bool();

  // Closed-form trajectory generation parameters
  par_.closed_form_time_allocation_adj_iter_max = this->get_parameter("closed_form_time_allocation_adj_iter_max").as_int();
  par_.closed_form_initial_factor = this->get_parameter("closed_form_initial_factor").as_double();
  par_.closed_form_factor_increment = this->get_parameter("closed_form_factor_increment").as_double();
  par_.closed_form_factor_initial_decrement = this->get_parameter("closed_form_factor_initial_decrement").as_double();

  // Horizon adaptation parameters
  par_.unknown_cells_weight = this->get_parameter("unknown_cells_weight").as_double();
  par_.dynamic_obsts_weight = this->get_parameter("dynamic_obsts_weight").as_double();
  par_.dynamic_obsts_prox_weight = this->get_parameter("dynamic_obsts_prox_weight").as_double();
  par_.dynamic_obsts_count_weight = this->get_parameter("dynamic_obsts_count_weight").as_double();
  par_.dynamic_obsts_total_max_num = this->get_parameter("dynamic_obsts_total_max_num").as_int();
  par_.max_horizon = this->get_parameter("max_horizon").as_double();
  par_.min_horizon = this->get_parameter("min_horizon").as_double();

  // P, N adaptation parameters
  par_.p_n_mode = this->get_parameter("p_n_mode").as_string();
  par_.num_N = this->get_parameter("num_N").as_int();
  par_.num_N_when_close_to_goal = this->get_parameter("num_N_when_close_to_goal").as_int();
  par_.num_N_max = this->get_parameter("num_N_max").as_int();
  par_.use_num_P_adaptation = this->get_parameter("use_num_P_adaptation").as_bool();
  par_.num_P_min = this->get_parameter("num_P_min").as_int();
  par_.num_P_max = this->get_parameter("num_P_max").as_int();
  par_.num_replan_fail_threshold = this->get_parameter("num_replan_fail_threshold").as_int();
  par_.cool_down_duration = this->get_parameter("cool_down_duration").as_double();
  par_.cool_down_duration_factor = this->get_parameter("cool_down_duration_factor").as_double();

  // v_max adaptation parameters
  par_.use_v_max_adaptation = this->get_parameter("use_v_max_adaptation").as_bool();
  par_.v_max_adaptation_v_min = this->get_parameter("v_max_adaptation_v_min").as_double();
  par_.v_max_adaptation_p_min = this->get_parameter("v_max_adaptation_p_min").as_double();
  par_.v_max_adaptation_p_max = this->get_parameter("v_max_adaptation_p_max").as_double();
  par_.v_max_adaptation_obstacle_threshold = this->get_parameter("v_max_adaptation_obstacle_threshold").as_int();

  // max_dist_veterxes adaptation parameters
  par_.use_max_dist_vertexes_adaptation = this->get_parameter("use_max_dist_vertexes_adaptation").as_bool();
  par_.use_min_dist_cutoff = this->get_parameter("use_min_dist_cutoff").as_bool();
  par_.max_dist_vertexes = this->get_parameter("max_dist_vertexes").as_double();
  par_.min_dist_vertexes = this->get_parameter("min_dist_vertexes").as_double();
  par_.max_dist_vertexes_adaptation_obstacle_threshold = this->get_parameter("max_dist_vertexes_adaptation_obstacle_threshold").as_int();
  par_.max_dist_vertexes_adaptation_threshold = this->get_parameter("max_dist_vertexes_adaptation_threshold").as_int();
  par_.max_dist_vertexes_adaptation_decrease = this->get_parameter("max_dist_vertexes_adaptation_decrease").as_double();

  // Local trajectory optimization parallelization parameters
  par_.use_local_traj_parallelization = this->get_parameter("use_local_traj_parallelization").as_bool();
  par_.num_factors = this->get_parameter("num_factors").as_int();
  par_.factor_increment = this->get_parameter("factor_increment").as_double();
  par_.min_factor = this->get_parameter("min_factor").as_double();
  par_.max_factor = this->get_parameter("max_factor").as_double();

  // Optimization parameters
  par_.horizon = this->get_parameter("horizon").as_double();
  par_.optimization_type = this->get_parameter("optimization_type").as_string();
  par_.use_ref_points = this->get_parameter("use_ref_points").as_bool();
  par_.num_ref_sample_points = this->get_parameter("num_ref_sample_points").as_int();
  par_.dc = this->get_parameter("dc").as_double();
  par_.v_max = this->get_parameter("v_max").as_double();
  par_.a_max = this->get_parameter("a_max").as_double();
  par_.j_max = this->get_parameter("j_max").as_double();
  par_.gurobi_threads = this->get_parameter("gurobi_threads").as_int();
  par_.gurobi_verbose = this->get_parameter("gurobi_verbose").as_bool();
  par_.control_cost_weight = this->get_parameter("control_cost_weight").as_double();
  par_.dynamic_weight = this->get_parameter("dynamic_weight").as_double();
  par_.num_obs_dist_samples = this->get_parameter("num_obs_dist_samples").as_int();
  par_.final_pos_weight = this->get_parameter("final_pos_weight").as_double();
  par_.final_vel_weight = this->get_parameter("final_vel_weight").as_double();
  par_.final_accel_weight = this->get_parameter("final_accel_weight").as_double();
  par_.final_yaw_cost_weight = this->get_parameter("final_yaw_cost_weight").as_double();
  par_.total_yaw_diff_weight = this->get_parameter("total_yaw_diff_weight").as_double();
  par_.ref_point_weight = this->get_parameter("ref_point_weight").as_double();
  par_.collision_clearance = this->get_parameter("collision_clearance").as_double();
  par_.use_hard_constr_for_final_state = this->get_parameter("use_hard_constr_for_final_state").as_bool();
  par_.use_hard_dynamic_constraints = this->get_parameter("use_hard_dynamic_constraints").as_bool();
  par_.use_hard_constr_for_final_yaw = this->get_parameter("use_hard_constr_for_final_yaw").as_bool();
  verbose_computation_time_ = this->get_parameter("verbose_computation_time").as_bool();
  par_.drone_bbox = this->get_parameter("drone_bbox").as_double_array();
  par_.drone_radius = par_.drone_bbox[0] / 2.0;
  par_.goal_radius = this->get_parameter("goal_radius").as_double();
  par_.goal_seen_radius = this->get_parameter("goal_seen_radius").as_double();
  par_.dist_to_switch_to_g_term = this->get_parameter("dist_to_switch_to_g_term").as_double();

  // Safe paths parameters
  par_.num_safe_paths = this->get_parameter("num_safe_paths").as_int();
  par_.min_num_safe_paths = this->get_parameter("min_num_safe_paths").as_int();
  par_.min_safe_path_distance = this->get_parameter("min_safe_path_distance").as_double();
  par_.max_safe_path_distance = this->get_parameter("max_safe_path_distance").as_double();

  // Contingency paths parameters
  par_.contingency_lateral_offset = this->get_parameter("contingency_lateral_offset").as_double();

  // Dynamic obstacles parameters
  par_.traj_lifetime = this->get_parameter("traj_lifetime").as_double();
  par_.tracking_distance_threshold = this->get_parameter("tracking_distance_threshold").as_double();
  par_.alpha_cov = this->get_parameter("alpha_cov").as_double();
  par_.dynamic_obstacle_base_inflation = this->get_parameter("dynamic_obstacle_base_inflation").as_double();
  par_.max_dynamic_obstacle_inflation = this->get_parameter("max_dynamic_obstacle_inflation").as_double();
  par_.freq_dynamic_obstacle_update_to_tmap = this->get_parameter("freq_dynamic_obstacle_update_to_tmap").as_double();

  // Time allocation parameters
  par_.factor_initial = this->get_parameter("factor_initial").as_double();
  par_.factor_final = this->get_parameter("factor_final").as_double();
  par_.factor_gamma_up = this->get_parameter("factor_gamma_up").as_double();
  par_.factor_gamma_down = this->get_parameter("factor_gamma_down").as_double();
  par_.factor_constant_step_size = this->get_parameter("factor_constant_step_size").as_double();
  par_.factor_minimum = this->get_parameter("factor_minimum").as_double();
  par_.factor_delta_step_size = this->get_parameter("factor_delta_step_size").as_double();
  par_.use_constant_step_size = this->get_parameter("use_constant_step_size").as_bool();
  par_.count_to_switch_to_constant_step_size = this->get_parameter("count_to_switch_to_constant_step_size").as_int();

  // Dynamic k_value parameters
  par_.num_replanning_before_adapt = this->get_parameter("num_replanning_before_adapt").as_int();
  par_.default_k_value = this->get_parameter("default_k_value").as_int();
  par_.alpha_k_value_filtering = this->get_parameter("alpha_k_value_filtering").as_double();
  par_.k_value_factor = this->get_parameter("k_value_factor").as_double();

  // Yaw-related parameters
  if (par_.vehicle_type == "uav") // drone
  {
    par_.use_initial_yawing = this->get_parameter("use_initial_yawing").as_bool();
  }
  else // ground robot
  {
    par_.use_initial_yawing = false;
  }
  par_.use_yaw = this->get_parameter("use_yaw").as_bool();
  par_.alpha_filter_yaw = this->get_parameter("alpha_filter_yaw").as_double();
  par_.alpha_filter_dyaw = this->get_parameter("alpha_filter_dyaw").as_double();
  par_.w_max = this->get_parameter("w_max").as_double();
  par_.yaw_collision_weight = this->get_parameter("yaw_collision_weight").as_double();
  par_.yaw_time_weight = this->get_parameter("yaw_time_weight").as_double();
  par_.yaw_proximity_weight = this->get_parameter("yaw_proximity_weight").as_double();
  par_.yaw_velocity_weight = this->get_parameter("yaw_velocity_weight").as_double();
  par_.yaw_change_weight = this->get_parameter("yaw_change_weight").as_double();
  par_.final_yaw_weight = this->get_parameter("final_yaw_weight").as_double();
  par_.cutoff_distance = this->get_parameter("cutoff_distance").as_double();
  par_.num_yaw_fit_poly = this->get_parameter("num_yaw_fit_poly").as_int();
  par_.yaw_fit_degree = this->get_parameter("yaw_fit_degree").as_int();
  par_.num_samples_collision_likelihood = this->get_parameter("num_samples_collision_likelihood").as_int();
  par_.num_samples_velocity_score = this->get_parameter("num_samples_velocity_score").as_int();
  par_.look_ahead_secs_for_dom = this->get_parameter("look_ahead_secs_for_dom").as_double();
  par_.yaw_optimization_debugging = this->get_parameter("yaw_optimization_debugging").as_bool();
  par_.yaw_spinning_threshold = this->get_parameter("yaw_spinning_threshold").as_int();
  par_.yaw_spinning_dyaw = this->get_parameter("yaw_spinning_dyaw").as_double();

  // ROI parameters
  par_.use_roi = this->get_parameter("use_roi").as_bool();
  par_.num_roi_agents = this->get_parameter("num_roi_agents").as_int();
  par_.num_roi_traj_sample = this->get_parameter("num_roi_traj_sample").as_int();
  par_.roi_map_share_buffer = this->get_parameter("roi_map_share_buffer").as_double();

  // Simulation env parameters
  par_.force_goal_z = this->get_parameter("force_goal_z").as_bool();
  par_.default_goal_z = this->get_parameter("default_goal_z").as_double();

  if (par_.default_goal_z <= par_.z_min)
  {
    RCLCPP_ERROR(this->get_logger(), "Default goal z is lower than the ground level");
  }

  if (par_.default_goal_z >= par_.z_max)
  {
    RCLCPP_ERROR(this->get_logger(), "Default goal z is higher than the max level");
  }

  // Debug flag
  par_.debug_verbose = this->get_parameter("debug_verbose").as_bool();
  par_.dist_to_term_g_verbose = this->get_parameter("dist_to_term_g_verbose").as_bool();

  // Octomap parameters
  world_frame_id_ = this->get_parameter("world_frame_id").as_string();
  use_height_map_ = this->get_parameter("use_height_map").as_bool();
  use_colored_map_ = this->get_parameter("use_colored_map").as_bool();
  color_factor_ = this->get_parameter("color_factor").as_double();
  point_cloud_min_x_ = this->get_parameter("point_cloud_min_x").as_double();
  point_cloud_max_x_ = this->get_parameter("point_cloud_max_x").as_double();
  point_cloud_min_y_ = this->get_parameter("point_cloud_min_y").as_double();
  point_cloud_max_y_ = this->get_parameter("point_cloud_max_y").as_double();
  point_cloud_min_z_ = this->get_parameter("point_cloud_min_z").as_double();
  point_cloud_max_z_ = this->get_parameter("point_cloud_max_z").as_double();
  filter_speckles_ = this->get_parameter("filter_speckles").as_bool();
  filter_ground_plane_ = this->get_parameter("filter_ground_plane").as_bool();
  ground_filter_distance_ = this->get_parameter("ground_filter.distance").as_double();
  ground_filter_angle_ = this->get_parameter("ground_filter.angle").as_double();
  ground_filter_plane_distance_ = this->get_parameter("ground_filter.plane_distance").as_double();
  use_decay_ = this->get_parameter("use_decay").as_bool();
  decay_duration_ = this->get_parameter("decay_duration").as_double();
  sensor_model_occupancy_thres_ = this->get_parameter("sensor_model.occupancy_thres").as_double();
  sensor_model_hit_ = this->get_parameter("sensor_model.hit").as_double();
  sensor_model_miss_ = this->get_parameter("sensor_model.miss").as_double();
  sensor_model_min_ = this->get_parameter("sensor_model.min").as_double();
  sensor_model_max_ = this->get_parameter("sensor_model.max").as_double();
  compress_map_ = this->get_parameter("compress_map").as_bool();
  color_.r = this->get_parameter("color.r").as_double();
  color_.g = this->get_parameter("color.g").as_double();
  color_.b = this->get_parameter("color.b").as_double();
  color_.a = this->get_parameter("color.a").as_double();
  color_free_.r = this->get_parameter("color_free.r").as_double();
  color_free_.g = this->get_parameter("color_free.g").as_double();
  color_free_.b = this->get_parameter("color_free.b").as_double();
  color_free_.a = this->get_parameter("color_free.a").as_double();
  latched_topics_ = this->get_parameter("latched_topics").as_bool();
  publish_free_space_ = this->get_parameter("publish_free_space").as_bool();
  lidar_insert_hz_ = this->get_parameter("lidar_insert_hz").as_double();
  depth_camera_insert_hz_ = this->get_parameter("depth_camera_insert_hz").as_double();
}

// ----------------------------------------------------------------------------

/**
 * @brief Print the parameters
 */
void DYNUS_NODE::printParameters()
{
  // Print the parameters

  // Vehicle type (UAV, Wheeled Robit, or Quadruped)
  RCLCPP_INFO(this->get_logger(), "Vehicle Type: %d", par_.vehicle_type);
  RCLCPP_INFO(this->get_logger(), "Provide Goal in Global Frame: %d", par_.provide_goal_in_global_frame);
  RCLCPP_INFO(this->get_logger(), "Use Hardware: %d", par_.use_hardware);

  // Sensor parameters
  RCLCPP_INFO(this->get_logger(), "Use Lidar: %d", par_.use_lidar);
  RCLCPP_INFO(this->get_logger(), "Use Depth Camera: %d", par_.use_depth_camera);

  // Flight mode
  RCLCPP_INFO(this->get_logger(), "Flight Mode: %s", par_.flight_mode.c_str());

  // Visual
  RCLCPP_INFO(this->get_logger(), "Visual Level: %d", par_.visual_level);

  // Global & Local planner parameters
  RCLCPP_INFO(this->get_logger(), "Planner Mode: %s", par_.planner_mode.c_str());
  RCLCPP_INFO(this->get_logger(), "Use Free Space: %d", par_.use_free_space);

  // DGP parameters
  RCLCPP_INFO(this->get_logger(), "File Path: %s", file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Perform Benchmark?: %d", use_benchmark_);
  RCLCPP_INFO(this->get_logger(), "Plan Only in Free Space?: %d", par_.plan_only_in_free_space);
  RCLCPP_INFO(this->get_logger(), "Initial Guess Planner: %s", par_.global_planner.c_str());
  RCLCPP_INFO(this->get_logger(), "DGP Planner Verbose: %d", par_.global_planner_verbose);
  RCLCPP_INFO(this->get_logger(), "Global Planner Huristic Weight: %f", par_.global_planner_huristic_weight);
  RCLCPP_INFO(this->get_logger(), "Factor DGP: %f", par_.factor_dgp);
  RCLCPP_INFO(this->get_logger(), "Inflation DGP: %f", par_.inflation_dgp);
  RCLCPP_INFO(this->get_logger(), "Free Inflation DGP: %f", par_.free_inflation_dgp);
  RCLCPP_INFO(this->get_logger(), "X Min: %f", par_.x_min);
  RCLCPP_INFO(this->get_logger(), "X Max: %f", par_.x_max);
  RCLCPP_INFO(this->get_logger(), "Y Min: %f", par_.y_min);
  RCLCPP_INFO(this->get_logger(), "Y Max: %f", par_.y_max);
  RCLCPP_INFO(this->get_logger(), "Z Ground: %f", par_.z_min);
  RCLCPP_INFO(this->get_logger(), "Z Max: %f", par_.z_max);
  RCLCPP_INFO(this->get_logger(), "Use Raw Path?: %d", par_.use_raw_path);
  RCLCPP_INFO(this->get_logger(), "Global Planner Push K: %f", par_.dyn_obst_global_planner_push_k);
  RCLCPP_INFO(this->get_logger(), "Global Planner Push Cov P Alpha: %f", par_.dyn_obst_global_planner_push_cov_p_alpha);
  RCLCPP_INFO(this->get_logger(), "Global Planner Repulsion Max: %f", par_.dyn_obst_replusion_max);
  RCLCPP_INFO(this->get_logger(), "DGP Timeout Duration: %d", par_.dgp_timeout_duration_ms);
  RCLCPP_INFO(this->get_logger(), "Use Free Start?: %d", par_.use_free_start);
  RCLCPP_INFO(this->get_logger(), "Free Start Factor: %f", par_.free_start_factor);
  RCLCPP_INFO(this->get_logger(), "Use Free Goal?: %d", par_.use_free_goal);
  RCLCPP_INFO(this->get_logger(), "Free Goal Factor: %f", par_.free_goal_factor);
  RCLCPP_INFO(this->get_logger(), "Delete Too Close Points in Global Path?: %d", par_.delete_too_close_points_in_global_path);
  RCLCPP_INFO(this->get_logger(), "Node Size Factor for Occupied: %f", par_.node_size_factor_for_occupied);
  RCLCPP_INFO(this->get_logger(), "Node Size Factor for Free: %f", par_.node_size_factor_for_free);

  // Path push visualization parameters
  RCLCPP_INFO(this->get_logger(), "Use State Update?: %d", par_.use_state_update);
  RCLCPP_INFO(this->get_logger(), "Use Random Color for Global Path?: %d", par_.use_random_color_for_global_path);
  RCLCPP_INFO(this->get_logger(), "Use Path Push for Paper?: %d", par_.use_path_push_for_visualization);

  // Static obstacle push parameters
  RCLCPP_INFO(this->get_logger(), "Dist Discretization: %f", par_.dist_discretization);
  RCLCPP_INFO(this->get_logger(), "Max Dist Threshold for Static Push: %f", par_.max_dist_threshold_for_static_push);
  RCLCPP_INFO(this->get_logger(), "Push Force Static: %f", par_.push_force_static);
  RCLCPP_INFO(this->get_logger(), "Num Lookahead Global Path for Push: %d", par_.num_lookahead_global_path_for_push);
  RCLCPP_INFO(this->get_logger(), "Static Push Clustering Threshold: %f", par_.static_push_clustering_threshold);

  RCLCPP_INFO(this->get_logger(), "Local Box Size: (%f, %f, %f)", par_.local_box_size[0], par_.local_box_size[1], par_.local_box_size[2]);
  RCLCPP_INFO(this->get_logger(), "Min Dist from Agent to Traj: %f", par_.min_dist_from_agent_to_traj);
  RCLCPP_INFO(this->get_logger(), "Use Shrinked Box: %d", par_.use_shrinked_box);
  RCLCPP_INFO(this->get_logger(), "Shrinked Box Size: %f", par_.shrinked_box_size);

  // Map parameters
  RCLCPP_INFO(this->get_logger(), "Local Map Buffer: %f", par_.map_buffer);
  RCLCPP_INFO(this->get_logger(), "Failure Map Buffer Increment: %f", par_.failure_map_buffer_increment);
  RCLCPP_INFO(this->get_logger(), "Map Buffer Velocity Factor: %f", par_.map_buffer_velocity_factor);
  RCLCPP_INFO(this->get_logger(), "Center Shift Factor: %f", par_.center_shift_factor);
  RCLCPP_INFO(this->get_logger(), "initial_wdx: %f", par_.initial_wdx);
  RCLCPP_INFO(this->get_logger(), "initial_wdy: %f", par_.initial_wdy);
  RCLCPP_INFO(this->get_logger(), "initial_wdz: %f", par_.initial_wdz);
  RCLCPP_INFO(this->get_logger(), "max_wdx: %f", par_.max_wdx);
  RCLCPP_INFO(this->get_logger(), "max_wdy: %f", par_.max_wdy);
  RCLCPP_INFO(this->get_logger(), "max_wdz: %f", par_.max_wdz);
  RCLCPP_INFO(this->get_logger(), "min_wdx: %f", par_.min_wdx);
  RCLCPP_INFO(this->get_logger(), "min_wdy: %f", par_.min_wdy);
  RCLCPP_INFO(this->get_logger(), "min_wdz: %f", par_.min_wdz);
  RCLCPP_INFO(this->get_logger(), "Lidar Max Range: %f", par_.lidar_max_range);
  RCLCPP_INFO(this->get_logger(), "Use Map Res Adaptation: %d", par_.use_map_res_adaptation);
  RCLCPP_INFO(this->get_logger(), "Map Res Adaptation Decrement: %f", par_.map_res_adaptation_decrement);
  RCLCPP_INFO(this->get_logger(), "Map Res Adaptation Threshold: %d", par_.map_res_adaptation_threshold);
  RCLCPP_INFO(this->get_logger(), "Res: %f", par_.res);
  RCLCPP_INFO(this->get_logger(), "Dynus Map Res Min: %f", par_.dynus_map_res_min);
  RCLCPP_INFO(this->get_logger(), "Octomap Res: %f", par_.octomap_res);
  RCLCPP_INFO(this->get_logger(), "TMap Update Rate: %f", par_.tmap_update_rate);

  // Frontiers parameters
  RCLCPP_INFO(this->get_logger(), "Use Frontiers: %d", par_.use_frontiers);
  RCLCPP_INFO(this->get_logger(), "Use D435 for Frontiers: %d", par_.use_only_d435_for_frontiers);
  RCLCPP_INFO(this->get_logger(), "Min Z Diff from Frontier to Camera: %f", par_.max_z_diff_from_frontier_to_camera);
  RCLCPP_INFO(this->get_logger(), "Min Dist from Frontier to Camera: %f", par_.min_dist_from_frontier_to_camera);
  RCLCPP_INFO(this->get_logger(), "Frontier Update Alpha: %f", par_.frontier_update_alpha);
  RCLCPP_INFO(this->get_logger(), "D435 Depth Min: %f", par_.d435_depth_min);
  RCLCPP_INFO(this->get_logger(), "Use Mean Based Best Frontier: %d", par_.use_mean_based_best_frontier);
  RCLCPP_INFO(this->get_logger(), "No Visible Frontiers Threshold: %d", par_.no_visible_frontiers_threshold);
  RCLCPP_INFO(this->get_logger(), "Desired Velocity Cost Weight: %f", par_.desired_velocity_cost_weight);
  RCLCPP_INFO(this->get_logger(), "Dist from Z Axis Weight: %f", par_.dist_from_z_axis_weight);
  RCLCPP_INFO(this->get_logger(), "Dist to Prev Best Frontier Weight: %f", par_.dist_to_prev_best_frontier_weight);
  RCLCPP_INFO(this->get_logger(), "Positive Z Camera Weight: %f", par_.positive_z_camera_weight);
  RCLCPP_INFO(this->get_logger(), "Goal Proximity Weight: %f", par_.goal_proximity_weight);
  RCLCPP_INFO(this->get_logger(), "Info Gain Cost Weight: %f", par_.info_gain_cost_weight);
  RCLCPP_INFO(this->get_logger(), "Frontier Neighbor Thresh for Info Gain: %f", par_.frontier_neighbor_thresh_for_info_gain);
  RCLCPP_INFO(this->get_logger(), "Frontier Search Buffer: %f", par_.frontier_search_buffer);
  RCLCPP_INFO(this->get_logger(), "Frontier Min Known Free Thresh: %d", par_.frontier_min_known_free_thresh);
  RCLCPP_INFO(this->get_logger(), "Frontier Min Unknown Thresh: %d", par_.frontier_min_unknown_thresh);

  // Communication delay parameters
  RCLCPP_INFO(this->get_logger(), "Use Comm Delay Inflation: %d", par_.use_comm_delay_inflation);
  RCLCPP_INFO(this->get_logger(), "Comm Delay Inflation Alpha: %f", par_.comm_delay_inflation_alpha);
  RCLCPP_INFO(this->get_logger(), "Comm Delay Inflation Max: %f", par_.comm_delay_inflation_max);
  RCLCPP_INFO(this->get_logger(), "Comm Delay Filter Alpha: %f", par_.comm_delay_filter_alpha);

  // Safety check parameters
  RCLCPP_INFO(this->get_logger(), "Safety Check Dt: %f", par_.safety_check_dt);

  // Simulation parameters
  RCLCPP_INFO(this->get_logger(), "D435 Depth Max: %f", par_.depth_camera_depth_max);
  RCLCPP_INFO(this->get_logger(), "FOV Visual Depth: %f", par_.fov_visual_depth);
  RCLCPP_INFO(this->get_logger(), "FOV Visual X Deg: %f", par_.fov_visual_x_deg);
  RCLCPP_INFO(this->get_logger(), "FOV Visual Y Deg: %f", par_.fov_visual_y_deg);

  // Benchmarking parameters
  RCLCPP_INFO(this->get_logger(), "Use Z Axis Bottom Inflation: %d", par_.use_z_axis_bottom_inflation);

  // Closed-form trajectory generation parameters
  RCLCPP_INFO(this->get_logger(), "Closed Form Time Allocation Adj Iter Max: %d", par_.closed_form_time_allocation_adj_iter_max);
  RCLCPP_INFO(this->get_logger(), "Closed Form Initial Factor: %f", par_.closed_form_initial_factor);
  RCLCPP_INFO(this->get_logger(), "Closed Form Factor Increment: %f", par_.closed_form_factor_increment);
  RCLCPP_INFO(this->get_logger(), "Closed Form Factor Initial Decrement: %f", par_.closed_form_factor_initial_decrement);

  // Horizon adaptation parameters
  RCLCPP_INFO(this->get_logger(), "Unknown Cells Weight: %f", par_.unknown_cells_weight);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obsts Weight: %f", par_.dynamic_obsts_weight);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obsts Prox Weight: %f", par_.dynamic_obsts_prox_weight);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obsts Count Weight: %f", par_.dynamic_obsts_count_weight);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obsts Total Max Num: %d", par_.dynamic_obsts_total_max_num);
  RCLCPP_INFO(this->get_logger(), "Max Horizon: %f", par_.max_horizon);
  RCLCPP_INFO(this->get_logger(), "Min Horizon: %f", par_.min_horizon);

  // P, N adaptation parameters
  RCLCPP_INFO(this->get_logger(), "P, N Mode: %s", par_.p_n_mode.c_str());
  RCLCPP_INFO(this->get_logger(), "Num N: %d", par_.num_N);
  RCLCPP_INFO(this->get_logger(), "Num N When Close to Goal: %d", par_.num_N_when_close_to_goal);
  RCLCPP_INFO(this->get_logger(), "Num N Max: %d", par_.num_N_max);
  RCLCPP_INFO(this->get_logger(), "Use Num P Adaptation: %d", par_.use_num_P_adaptation);
  RCLCPP_INFO(this->get_logger(), "Num P Min: %d", par_.num_P_min);
  RCLCPP_INFO(this->get_logger(), "Num P Max: %d", par_.num_P_max);
  RCLCPP_INFO(this->get_logger(), "Num Replan Fail Threshold: %d", par_.num_replan_fail_threshold);
  RCLCPP_INFO(this->get_logger(), "Cool Down Duration: %f", par_.cool_down_duration);
  RCLCPP_INFO(this->get_logger(), "Cool Down Duration Factor: %f", par_.cool_down_duration_factor);

  // v_max adaptation parameters
  RCLCPP_INFO(this->get_logger(), "Use v_max Adaptation: %d", par_.use_v_max_adaptation);
  RCLCPP_INFO(this->get_logger(), "v_max adaptation v_min: %f", par_.v_max_adaptation_v_min);
  RCLCPP_INFO(this->get_logger(), "v_max adaptation p_min: %f", par_.v_max_adaptation_p_min);
  RCLCPP_INFO(this->get_logger(), "v_max adaptation p_max: %f", par_.v_max_adaptation_p_max);
  RCLCPP_INFO(this->get_logger(), "v_max adaptation obstacle threshold: %d", par_.v_max_adaptation_obstacle_threshold);

  // max_dist_vertexes adaptation parameters
  RCLCPP_INFO(this->get_logger(), "Use max_dist_vertexes Adaptation: %d", par_.use_max_dist_vertexes_adaptation);
  RCLCPP_INFO(this->get_logger(), "Use min_dist_cutoff: %d", par_.use_min_dist_cutoff);
  RCLCPP_INFO(this->get_logger(), "max_dist_vertexes: %f", par_.max_dist_vertexes);
  RCLCPP_INFO(this->get_logger(), "min_dist_vertexes: %f", par_.min_dist_vertexes);
  RCLCPP_INFO(this->get_logger(), "max_dist_vertexes adaptation obstacle threshold: %d", par_.max_dist_vertexes_adaptation_obstacle_threshold);
  RCLCPP_INFO(this->get_logger(), "max_dist_vertexes adaptation threshold: %d", par_.max_dist_vertexes_adaptation_threshold);
  RCLCPP_INFO(this->get_logger(), "max_dist_vertexes adaptation decrease: %f", par_.max_dist_vertexes_adaptation_decrease);

  // Local trajectory optimization parallelization parameters
  RCLCPP_INFO(this->get_logger(), "Use Parallelization: %d", par_.use_local_traj_parallelization);
  RCLCPP_INFO(this->get_logger(), "Num Factors: %d", par_.num_factors);
  RCLCPP_INFO(this->get_logger(), "Factor Increment: %f", par_.factor_increment);
  RCLCPP_INFO(this->get_logger(), "Min Factor: %f", par_.min_factor);
  RCLCPP_INFO(this->get_logger(), "Max Factor: %f", par_.max_factor);

  // Optimization parameters
  RCLCPP_INFO(this->get_logger(), "Horizon: %f", par_.horizon);
  RCLCPP_INFO(this->get_logger(), "Optimization Type: %s", par_.optimization_type.c_str());
  RCLCPP_INFO(this->get_logger(), "Use Ref Points: %d", par_.use_ref_points);
  RCLCPP_INFO(this->get_logger(), "Num Ref Sample Points: %d", par_.num_ref_sample_points);
  RCLCPP_INFO(this->get_logger(), "DC: %f", par_.dc);
  RCLCPP_INFO(this->get_logger(), "V Max: %f", par_.v_max);
  RCLCPP_INFO(this->get_logger(), "A Max: %f", par_.a_max);
  RCLCPP_INFO(this->get_logger(), "J Max: %f", par_.j_max);
  RCLCPP_INFO(this->get_logger(), "Gurobi Threads: %d", par_.gurobi_threads);
  RCLCPP_INFO(this->get_logger(), "Gurobi Verbose: %d", par_.gurobi_verbose);
  RCLCPP_INFO(this->get_logger(), "Control Cost Weight: %f", par_.control_cost_weight);
  RCLCPP_INFO(this->get_logger(), "Obstacles and Agents Distance Weight: %f", par_.dynamic_weight);
  RCLCPP_INFO(this->get_logger(), "Num Obs Dist Samples: %d", par_.num_obs_dist_samples);
  RCLCPP_INFO(this->get_logger(), "Final State Weight: %f", par_.final_pos_weight);
  RCLCPP_INFO(this->get_logger(), "Final Vel Weight: %f", par_.final_vel_weight);
  RCLCPP_INFO(this->get_logger(), "Final Accel Weight: %f", par_.final_accel_weight);
  RCLCPP_INFO(this->get_logger(), "Final Yaw Cost Weight: %f", par_.final_yaw_cost_weight);
  RCLCPP_INFO(this->get_logger(), "Total Yaw Diff Weight: %f", par_.total_yaw_diff_weight);
  RCLCPP_INFO(this->get_logger(), "Ref Point Weight: %f", par_.ref_point_weight);
  RCLCPP_INFO(this->get_logger(), "Collision Clearance: %f", par_.collision_clearance);
  RCLCPP_INFO(this->get_logger(), "Use Hard Constr For Final State: %d", par_.use_hard_constr_for_final_state);
  RCLCPP_INFO(this->get_logger(), "Use Hard Dynamic Constraints: %d", par_.use_hard_dynamic_constraints);
  RCLCPP_INFO(this->get_logger(), "Use Hard Constr For Final Yaw: %d", par_.use_hard_constr_for_final_yaw);
  RCLCPP_INFO(this->get_logger(), "Verbose Computation Time: %d", verbose_computation_time_);
  RCLCPP_INFO(this->get_logger(), "Drone Bbox: (%f, %f, %f)", par_.drone_bbox[0], par_.drone_bbox[1], par_.drone_bbox[2]);
  RCLCPP_INFO(this->get_logger(), "Goal Radius: %f", par_.goal_radius);
  RCLCPP_INFO(this->get_logger(), "Goal Seen Radius: %f", par_.goal_seen_radius);
  RCLCPP_INFO(this->get_logger(), "Dist to Switch to G Term: %f", par_.dist_to_switch_to_g_term);

  // Safe paths parameters
  RCLCPP_INFO(this->get_logger(), "Num Safe Paths Disc: %d", par_.num_safe_paths);
  RCLCPP_INFO(this->get_logger(), "Min Num Safe Paths: %f", par_.min_num_safe_paths);
  RCLCPP_INFO(this->get_logger(), "Min Safe Path Distance: %f", par_.min_safe_path_distance);
  RCLCPP_INFO(this->get_logger(), "Max Safe Path Distance: %f", par_.max_safe_path_distance);

  // Contingency paths parameters
  RCLCPP_INFO(this->get_logger(), "Contingency Lateral Offset: %f", par_.contingency_lateral_offset);

  // Dynamic obstacles parameters
  RCLCPP_INFO(this->get_logger(), "Traj Lifetime: %f", par_.traj_lifetime);
  RCLCPP_INFO(this->get_logger(), "Tracking Distance Threshold: %f", par_.tracking_distance_threshold);
  RCLCPP_INFO(this->get_logger(), "Alpha Cov: %f", par_.alpha_cov);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obstacle Base Inflation: %f", par_.dynamic_obstacle_base_inflation);
  RCLCPP_INFO(this->get_logger(), "Dynamic Obstacle Inflation: %f", par_.max_dynamic_obstacle_inflation);
  RCLCPP_INFO(this->get_logger(), "Freq Dynamic Obstacle Updata to TMap: %f", par_.freq_dynamic_obstacle_update_to_tmap);

  // Time allocation parameters
  RCLCPP_INFO(this->get_logger(), "Factor Initial: %f", par_.factor_initial);
  RCLCPP_INFO(this->get_logger(), "Num Factor Increments: %d", par_.factor_final);
  RCLCPP_INFO(this->get_logger(), "Factor Gamma Up: %f", par_.factor_gamma_up);
  RCLCPP_INFO(this->get_logger(), "Factor Gamma Down: %f", par_.factor_gamma_down);
  RCLCPP_INFO(this->get_logger(), "Factor Constant Step Size: %f", par_.factor_constant_step_size);
  RCLCPP_INFO(this->get_logger(), "Factor Minimum: %f", par_.factor_minimum);
  RCLCPP_INFO(this->get_logger(), "Factor Delta Step Size: %f", par_.factor_delta_step_size);
  RCLCPP_INFO(this->get_logger(), "Use Constant Step Size: %d", par_.use_constant_step_size);
  RCLCPP_INFO(this->get_logger(), "Count to Switch to Constant Step Size: %d", par_.count_to_switch_to_constant_step_size);

  // Dynamic k_value parameters
  RCLCPP_INFO(this->get_logger(), "Num Replanning Before Adapt: %d", par_.num_replanning_before_adapt);
  RCLCPP_INFO(this->get_logger(), "Default K Value End: %d", par_.default_k_value);
  RCLCPP_INFO(this->get_logger(), "Alpha K Value: %f", par_.alpha_k_value_filtering);
  RCLCPP_INFO(this->get_logger(), "K Value Inflation: %f", par_.k_value_factor);

  // Yaw-related parameters
  RCLCPP_INFO(this->get_logger(), "Use Initial Yawing: %d", par_.use_initial_yawing);
  RCLCPP_INFO(this->get_logger(), "Use Yaw: %d", par_.use_yaw);
  RCLCPP_INFO(this->get_logger(), "Alpha Filter Yaw: %f", par_.alpha_filter_yaw);
  RCLCPP_INFO(this->get_logger(), "Alpha Filter Dyaw: %f", par_.alpha_filter_dyaw);
  RCLCPP_INFO(this->get_logger(), "W Max: %f", par_.w_max);
  RCLCPP_INFO(this->get_logger(), "Yaw Collision Weight: %f", par_.yaw_collision_weight);
  RCLCPP_INFO(this->get_logger(), "Yaw Time Weight: %f", par_.yaw_time_weight);
  RCLCPP_INFO(this->get_logger(), "Yaw Proximity Weight: %f", par_.yaw_proximity_weight);
  RCLCPP_INFO(this->get_logger(), "Yaw Velocity Weight: %f", par_.yaw_velocity_weight);
  RCLCPP_INFO(this->get_logger(), "Yaw Change Weight: %f", par_.yaw_change_weight);
  RCLCPP_INFO(this->get_logger(), "Final Yaw Weight: %f", par_.final_yaw_weight);
  RCLCPP_INFO(this->get_logger(), "Cutoff Distance: %f", par_.cutoff_distance);
  RCLCPP_INFO(this->get_logger(), "Num Yaw Fit Poly: %d", par_.num_yaw_fit_poly);
  RCLCPP_INFO(this->get_logger(), "Yaw Fit Degree: %d", par_.yaw_fit_degree);
  RCLCPP_INFO(this->get_logger(), "Num Samples Collision Likelihood: %d", par_.num_samples_collision_likelihood);
  RCLCPP_INFO(this->get_logger(), "Num Samples Velocity Score: %d", par_.num_samples_velocity_score);
  RCLCPP_INFO(this->get_logger(), "Look Ahead Secs for Direction of Motion: %f", par_.look_ahead_secs_for_dom);
  RCLCPP_INFO(this->get_logger(), "Yaw Optimization Debugging: %d", par_.yaw_optimization_debugging);
  RCLCPP_INFO(this->get_logger(), "Yaw Spinning Threshold: %d", par_.yaw_spinning_threshold);
  RCLCPP_INFO(this->get_logger(), "Yaw Spinning Dyaw: %f", par_.yaw_spinning_dyaw);

  // ROI parameters
  RCLCPP_INFO(this->get_logger(), "Use ROI: %d", par_.use_roi);
  RCLCPP_INFO(this->get_logger(), "Num ROI Agents: %d", par_.num_roi_agents);
  RCLCPP_INFO(this->get_logger(), "Num ROI Traj Sample: %d", par_.num_roi_traj_sample);
  RCLCPP_INFO(this->get_logger(), "ROI Map Share Buffer: %f", par_.roi_map_share_buffer);

  // Simulation env parameters
  RCLCPP_INFO(this->get_logger(), "Force Goal Z: %d", par_.force_goal_z);
  RCLCPP_INFO(this->get_logger(), "Default Goal Z: %f", par_.default_goal_z);

  // Debug flag
  RCLCPP_INFO(this->get_logger(), "Debug Verbose: %d", par_.debug_verbose);
  RCLCPP_INFO(this->get_logger(), "Dist to Term G Verbose: %d", par_.dist_to_term_g_verbose);

  // Octomap parameters
  RCLCPP_INFO(this->get_logger(), "World Frame ID: %s", world_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "Use Height Map: %d", use_height_map_);
  RCLCPP_INFO(this->get_logger(), "Use Colored Map: %d", use_colored_map_);
  RCLCPP_INFO(this->get_logger(), "Color Factor: %f", color_factor_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Min X: %f", point_cloud_min_x_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Max X: %f", point_cloud_max_x_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Min Y: %f", point_cloud_min_y_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Max Y: %f", point_cloud_max_y_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Min Z: %f", point_cloud_min_z_);
  RCLCPP_INFO(this->get_logger(), "Point Cloud Max Z: %f", point_cloud_max_z_);
  RCLCPP_INFO(this->get_logger(), "Filter Speckles: %d", filter_speckles_);
  RCLCPP_INFO(this->get_logger(), "Filter Ground Plane: %d", filter_ground_plane_);
  RCLCPP_INFO(this->get_logger(), "Ground Filter Distance: %f", ground_filter_distance_);
  RCLCPP_INFO(this->get_logger(), "Ground Filter Angle: %f", ground_filter_angle_);
  RCLCPP_INFO(this->get_logger(), "Ground Filter Plane Distance: %f", ground_filter_plane_distance_);
  RCLCPP_INFO(this->get_logger(), "Use Decay: %d", use_decay_);
  RCLCPP_INFO(this->get_logger(), "Decay Duration: %f", decay_duration_);
  RCLCPP_INFO(this->get_logger(), "Sensor Model Occupancy Thres: %f", sensor_model_occupancy_thres_);
  RCLCPP_INFO(this->get_logger(), "Sensor Model Hit: %f", sensor_model_hit_);
  RCLCPP_INFO(this->get_logger(), "Sensor Model Miss: %f", sensor_model_miss_);
  RCLCPP_INFO(this->get_logger(), "Sensor Model Min: %f", sensor_model_min_);
  RCLCPP_INFO(this->get_logger(), "Sensor Model Max: %f", sensor_model_max_);
  RCLCPP_INFO(this->get_logger(), "Compress Map: %d", compress_map_);
  RCLCPP_INFO(this->get_logger(), "Color: (%f, %f, %f, %f)", color_.r, color_.g, color_.b, color_.a);
  RCLCPP_INFO(this->get_logger(), "Color Free: (%f, %f, %f, %f)", color_free_.r, color_free_.g, color_free_.b, color_free_.a);
  RCLCPP_INFO(this->get_logger(), "Latched Topics: %d", latched_topics_);
  RCLCPP_INFO(this->get_logger(), "Publish Free Space: %d", publish_free_space_);
  RCLCPP_INFO(this->get_logger(), "LiDAR Insert Hz: %f", lidar_insert_hz_);
  RCLCPP_INFO(this->get_logger(), "Depth Camera Insert Hz: %f", depth_camera_insert_hz_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to clean up old trajs in DYNUS
 */
void DYNUS_NODE::cleanUpOldTrajsCallback()
{
  // Get current time
  double current_time = this->now().seconds();

  // Clean up old trajs
  dynus_ptr_->cleanUpOldTrajs(current_time);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to update trajs for tmap
 */
void DYNUS_NODE::updateTrajsForTMapCallback()
{
  // Update trajs for tmap
  dynus_ptr_->updateTrajsForTmap();
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function ro Region of Interest (ROI) trajectories
 * @param msg ROI trajectories
 */
void DYNUS_NODE::roiTrajCallback(const dynus_interfaces::msg::DynTrajArray::SharedPtr msg)
{

  // Loop through the array
  for (int i = 0; i < msg->trajs.size(); i++)
  {

    // Filter out its own traj
    if (msg->trajs[i].id == id_)
    {
      continue;
    }

    // Current time
    double current_time = this->now().seconds();

    // Get the dynTraj
    dynTraj traj;
    convertDynTrajMsg2DynTraj(msg->trajs[i], traj, current_time);

    // Pass the dynTraj
    dynus_ptr_->addTraj(traj, current_time);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to update the ROI octomap
 * @param msg ROI octomap
 */
void DYNUS_NODE::trajCallback(const dynus_interfaces::msg::DynTraj::SharedPtr msg)
{

  // Filter out its own traj
  if (msg->id == id_)
    return;

  // Check if trajectory id exists in pub_roi_octomap_ (unordered_map<id, octomap>)
  if (msg->is_agent && pub_roi_octomap_.find(msg->id) == pub_roi_octomap_.end())
  {
    // Get the vehicle name
    std::string vehicle_name = (msg->id > 9) ? "NX" + std::to_string(msg->id) : "NX0" + std::to_string(msg->id);

    // Create a new publisher and insert it to the map
    pub_roi_octomap_[msg->id] = this->create_publisher<octomap_msgs::msg::Octomap>("/" + vehicle_name + "/roi_octomap", 10);

    // Same for the dynamic obstacles
    pub_roi_trajs_[msg->id] = this->create_publisher<dynus_interfaces::msg::DynTrajArray>("/" + vehicle_name + "/roi_trajs", 10);
  }

  // Get current time
  double current_time = this->now().seconds();

  // Get dynTraj from the message
  dynTraj traj;
  convertDynTrajMsg2DynTraj(*msg, traj, current_time);

  // Pass the dynTraj to dynus.cpp
  dynus_ptr_->addTraj(traj, current_time);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to share ROI octomap and trajectories
 */
void DYNUS_NODE::shareROIOctomapAndTrajsCallback()
{

  // Get current time
  double current_time = this->now().seconds();

  // Share a partial map if useful
  std::unordered_map<int, std::shared_ptr<octomap::TimedOcTree>> roi_octrees;
  // TODO: we might need to implement masking for another agent to enable this - although another agent in gazebo is not so visible and since we share trajectories, we can perform collision avoidance quite well
  dynus_ptr_->shareStaticMap(roi_octrees, current_time);

  // Share dynamic obstacles
  std::unordered_map<int, std::vector<dynTraj>> roi_trajs;
  // TODO: not sure but seems like the agent receives its own trajectory if i turn this on
  // dynus_ptr_->shareDynamicObstacles(roi_trajs, current_time);

  // Publish the partial static map and dynamic obstacles to the agent who needs the ROI
  publishROIOctomapAndTrajs(roi_octrees, roi_trajs);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function for the state of the agent
 * @param msg State message
 */
void DYNUS_NODE::stateCallback(const dynus_interfaces::msg::State::SharedPtr msg)
{

  if (par_.use_state_update)
  {
    state current_state;
    current_state.setPos(msg->pos.x, msg->pos.y, msg->pos.z);
    current_state.setVel(msg->vel.x, msg->vel.y, msg->vel.z);
    current_state.setAccel(0.0, 0.0, 0.0);
    double roll, pitch, yaw;
    quaternion2Euler(msg->quat, roll, pitch, yaw);
    current_state.setYaw(yaw);
    dynus_ptr_->updateState(current_state);

    // publish the state
    publishCurrentState(current_state);

    // Publish the velocity in text
    if (par_.visual_level >= 1)
      publishVelocityInText(current_state.pos, current_state.vel.norm());
  }

  if (!state_initialized_)
  {

    // If we don't use state update, we need to initialize the state
    if (!par_.use_state_update)
    {
      state current_state;
      current_state.setPos(msg->pos.x, msg->pos.y, msg->pos.z);
      current_state.setVel(msg->vel.x, msg->vel.y, msg->vel.z);
      current_state.setAccel(0.0, 0.0, 0.0);
      double roll, pitch, yaw;
      quaternion2Euler(msg->quat, roll, pitch, yaw);
      current_state.setYaw(yaw);
      current_state.t = this->now().seconds();
      dynus_ptr_->updateState(current_state);
    }

    RCLCPP_INFO(this->get_logger(), "State initialized");
    state_initialized_ = true;
    timer_goal_->reset();
  }

  if (par_.visual_level >= 1)
    publishActualTraj();
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function for replanning
 */
void DYNUS_NODE::replanCallback()
{

  // Get the current time as double
  double current_time = this->now().seconds(); // TODO: this needs to be the time when the agent is at A

  // Set computation times to zero
  setComputationTimesToZero();

  // Replan (TODO: clean up)
  auto [replanning_result, dgp_result, new_P_or_N_set] = dynus_ptr_->replan(replanning_computation_time_, current_time);

  // Get computation time (used to find point A) - note this value is not updated in the replan function
  if (replanning_result)
  {
    // Get the replanning computation time
    replanning_computation_time_ = this->now().seconds() - current_time;
    if (par_.debug_verbose)
      printf("Total Replanning: %f ms\n", replanning_computation_time_ * 1000.0);
  }

  // To share trajectory with other agents
  if (replanning_result)
    publishOwnTraj();

  // For visualization of global path
  if (dgp_result && par_.visual_level >= 1)
    publishGlobalPath();

  // For visualization of free global path
  if (dgp_result && par_.visual_level >= 1)
    publishFreeGlobalPath();

  // For visualization of local_global_path and local_global_path_after_push_
  if (dgp_result && par_.visual_level >= 1)
    publishLocalGlobalPath();

  // For visualization of the local trajectory
  if (replanning_result && par_.visual_level >= 1)
    publishTraj();

  // For visualization of the safe corridor
  if (dgp_result && par_.visual_level >= 1)
    publishPoly();

  // For visualization of point G and point A
  if (replanning_result && par_.visual_level >= 1)
  {
    publishPointG();
    publishPointE();
    publishPointA();
  }

  // For visualization of control points
  if (replanning_result && par_.visual_level >= 1)
    publisCps();

  // For visualization of static push points and P points
  if (replanning_result && par_.visual_level >= 1)
  {
    dynus_ptr_->getStaticPushPoints(static_push_points_);
    publishStaticPushPoints();
    // dynus_ptr_->getPpoints(p_points_); // P points are no longer used
    // publishPPoints(); // P points are no longer used
  }

  // If verbose_computation_time_ or use_benchmark_ is true, we need to retrieve data from dynus_ptr_
  // if (replanning_result && (verbose_computation_time_ || use_benchmark_))
  if (verbose_computation_time_ || use_benchmark_)
    retrieveData();

  // Verbose computation time to the terminal
  if (verbose_computation_time_)
    printComputationTime(replanning_result);

  // Record the data
  // if (replanning_result && use_benchmark_)
  if (use_benchmark_)
    recordData(replanning_result);

  // Publish yaw sequence and b-spline
  if (par_.use_yaw && replanning_result && par_.yaw_optimization_debugging)
    publishYawSequenceAndBSpline();

  // If new P or N set, publish it
  if (new_P_or_N_set)
    publishPandN();

  // Usually this is done is goal callback but becuase we don't call that in push path test, we need to call it here
  if (par_.use_path_push_for_visualization)
    publishFOV();
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function for the terminal goal
 * @param msg Empty message to start the exploration (you can also give a goal to terminalGoalCallback, and as long as flight_mode is set to "exploration" in dynus.yaml, it will start the exploration)
 */
void DYNUS_NODE::startExplorationCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  // Start the exploration
  dynus_ptr_->startExploration();
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function for the terminal goal
 * @param msg Terminal goal message
 */
void DYNUS_NODE::terminalGoalCallback(const geometry_msgs::msg::PoseStamped &msg)
{

  // Set the terminal goal
  state G_term;
  double goal_z;

  // If force_goal_z is true, set the goal_z to default_goal_z
  if (par_.force_goal_z)
    goal_z = par_.default_goal_z;
  else
    goal_z = msg.pose.position.z;

  // Check if the goal_z is within the limits
  if (goal_z < par_.z_min || goal_z > par_.z_max)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal z is out of bounds: %f", goal_z);
    return;
  }

  // Set the terminal goal
  G_term.setPos(msg.pose.position.x, msg.pose.position.y, goal_z);

  // Update the terminal goal
  dynus_ptr_->setTerminalGoal(G_term);

  // Publish the term goal for visualization
  publishState(G_term, pub_point_G_term_);

  // Start replanning
  if (!terminal_goal_set_)
    terminal_goal_set_ = true;

  // clear all the trajectories on we receive a new goal
  // clearMarkerActualTraj();
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::publishVelocityInText(const Eigen::Vector3d &position, double velocity)
{

  // Set velocity's precision to 2 decimal points
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity;

  // Make a string
  std::string text = oss.str() + "m/s";

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.ns = "velocity";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.scale.z = 1.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.text = text;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z() + 5.0;
  marker.pose.orientation.w = 1.0;
  pub_vel_text_->publish(marker);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to update the TMap
 */
void DYNUS_NODE::updateTMapCallback()
{

  // Check if the octomap is received
  if (!octomap_initialized_)
  {
    // RCLCPP_WARN(this->get_logger(), "Octomap is not initialized yet.");
    return;
  }

  // Debug - to make sure Tmap is updated at the correct rate
  static double current_time_for_debug_ = this->now().seconds();
  double elapsed_time = this->now().seconds() - current_time_for_debug_;
  if (elapsed_time > 1 / par_.tmap_update_rate * 2.0)
    std::cerr << "Elapsed time for updating TMap: " << elapsed_time * 1000.0 << " ms" << std::endl;
  current_time_for_debug_ = this->now().seconds();

  // Check if the octree is empty
  if (lidar_octree_->size() <= 1 && depth_camera_octree_->size() <= 1)
  {
    RCLCPP_WARN(get_logger(), "octree is empty in updateTMapCallback");
    return;
  }

  // Get the current time
  double current_time = this->now().seconds();

  // Update the TMap
  // Mutex of the octree octree for planning
  octree_mutex_.lock();
  dynus_ptr_->updateMapCallback(current_time, lidar_octree_, depth_camera_octree_);
  octree_mutex_.unlock();

  // Housekeeping

  // check if this is the first map received
  if (!map_initialized_)
  {
    map_initialized_ = true;

    // Start region of interest share timer
    if (par_.use_roi)
      timer_share_roi_map_and_trajs_->reset();

    // Start frontiers timer
    if (par_.use_frontiers)
      timer_frontiers_->reset();

    if (par_.visual_level == 2 || par_.use_hardware)
    {
      // timer_visualize_static_map_->reset();
      timer_visualize_dynamic_map_->reset();
    }
  }

  // Start planning
  if (!replan_timer_started_ && map_initialized_)
  {
    replan_timer_started_ = true;
    timer_replanning_->reset();
    if (use_benchmark_)
      timer_goal_reached_check_->reset();
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to check if the goal is reached
 */
void DYNUS_NODE::goalReachedCheckCallback()
{
  if (dynus_ptr_->goalReachedCheck())
  {
    logData();
    pub_goal_reached_->publish(std_msgs::msg::Empty());
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to find frontiers
 */
void DYNUS_NODE::findFrontiersCallback()
{

  // First find the transformation matrix from map to camera
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer_->lookupTransform(d435_depth_frame_id_, "map", latest_d435_octomap_time_stamp_, rclcpp::Duration::from_seconds(0.5));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  Eigen::Matrix4d camera_transform = dynus_utils::transformStampedToMatrix(transform_stamped);

  // Find frontiers
  octree_mutex_.lock();
  bool result = dynus_ptr_->findFrontiers(best_frontier_, camera_transform);
  octree_mutex_.unlock();

  // Clear
  if (par_.visual_level == 2)
    clearMarkerArray(frontiers_marker_, pub_frontiers_);

  // Publish frontiers
  if (par_.visual_level == 2 && result)
    publishFrontiers();
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::getInitialPoseHwCallback()
{
  // First find the transformation matrix from map to camera
  try
  {
    init_pose_transform_stamped_ = tf2_buffer_->lookupTransform("map", initial_pose_topic_, tf2::TimePointZero);

    // Print out the initial pose
    RCLCPP_INFO(this->get_logger(), "Initial pose received: (%f, %f, %f)", init_pose_transform_stamped_.transform.translation.x,
                init_pose_transform_stamped_.transform.translation.y, init_pose_transform_stamped_.transform.translation.z);

    // Push the initial pose to dynus
    dynus_ptr_->setInitialPose(init_pose_transform_stamped_);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  // flag
  if (!initial_pose_received_)
  {
    initial_pose_received_ = true;
    timer_initial_pose_->cancel();
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to receive successful detection from YOLO (only for exploration)
 */
void DYNUS_NODE::successfulDetectionCallback(const std_msgs::msg::String::SharedPtr msg)
{

  // Get string
  std::string str = msg->data;

  // Update the time of the last successful detection
  dynus_ptr_->detectSuccessfulDetection(str);
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function to check if the future trajectory (plan) is collision-free
 */
void DYNUS_NODE::checkFuturePlanSafetyCallback()
{
  dynus_ptr_->checkFuturePlanSafety();
}

// ----------------------------------------------------------------------------

/**
 * @brief Callback function for ROI octomap
 */
void DYNUS_NODE::octomapCallbackForROI(const octomap_msgs::msg::Octomap::SharedPtr msg)
{

  std::cout << "Received ROI Octomap" << std::endl;

  // Create a pointer for the octomap to be filled
  std::shared_ptr<octomap::TimedOcTree> octree(nullptr);

  // Convert the binary message to an OcTree map
  octree.reset(dynamic_cast<octomap::TimedOcTree *>(octomap_msgs::binaryMsgToMap(*msg)));

  if (octree)
  {
    // Update the octomap
    dynus_ptr_->setOctomap(octree, "roi", msg->id);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert octomap binary message to OcTree.");
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Convert dynTraj message to dynTraj
 * @param msg dynTraj message
 * @param traj dynTraj
 * @param current_time current time
 */
void DYNUS_NODE::convertDynTrajMsg2DynTraj(const dynus_interfaces::msg::DynTraj &msg, dynTraj &traj, double current_time)
{

  // Inflate bbox using drone_bbox
  // We need to use the obstacle's bbox as well as ego drone's bbox
  traj.bbox << msg.bbox[0] / 2.0 + par_.drone_bbox[0] / 2.0, msg.bbox[1] / 2.0 + par_.drone_bbox[1] / 2.0, msg.bbox[2] / 2.0 + par_.drone_bbox[2] / 2.0;

  // Get id
  traj.id = msg.id;

  // Get pwp
  traj.pwp = dynus_utils::convertPwpMsg2Pwp(msg.pwp);

  // Get covariances
  if (!msg.is_agent)
  {
    traj.ekf_cov_p = dynus_utils::convertCovMsg2Cov(msg.ekf_cov_p); // ekf cov
    traj.ekf_cov_q = dynus_utils::convertCovMsg2Cov(msg.ekf_cov_q); // ekf cov
    traj.poly_cov = dynus_utils::convertCovMsg2Cov(msg.poly_cov);   // future traj cov
  }

  // Record received time
  traj.time_received = current_time;

  // Get is_agent
  traj.is_agent = msg.is_agent;

  // Get terminal goal
  if (traj.is_agent)
    traj.goal << msg.goal[0], msg.goal[1], msg.goal[2];

  // Get communication delay
  if (traj.is_agent && par_.use_comm_delay_inflation)
  {
    // Get the delay (current time - msg time)
    traj.communication_delay = this->now().seconds();
    -msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

    // Sanity check - if the delay is negative, set it to 0 - send warning message: it's probably due to the clock synchronization issue
    if (traj.communication_delay < 0)
    {
      traj.communication_delay = 0;
      RCLCPP_WARN(this->get_logger(), "Communication delay is negative. Setting it to 0.");
    }
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish control points
 */

void DYNUS_NODE::publisCps()
{

  // Retrieve control points
  dynus_ptr_->retrieveCPs(cps_);

  // Create a marker array
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.resize(cps_.size());

  // Loop through the control points (std::vector<Eigen::Matrix<double, 3, 4>>)
  for (int seg = 0; seg < cps_.size(); seg++)
  {

    // Create a marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "cp";
    marker.id = seg;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;

    // Set different colors for different segments
    if (seg % 3 == 0)
    {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    else if (seg % 3 == 1)
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
    }

    auto cp = cps_[seg];

    // Loop through the control points for each segment
    for (int i = 0; i < cp.cols(); i++)
    {
      geometry_msgs::msg::Point point;
      point.x = cp(0, i);
      point.y = cp(1, i);
      point.z = cp(2, i);
      marker.points.push_back(point);
    }

    // Add the marker to the marker array
    marker_array.markers[seg] = marker;
  }

  // Publish
  pub_cp_->publish(marker_array);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish static push points
 */
void DYNUS_NODE::publishStaticPushPoints()
{

  // Create a marker array
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "static_push_points";
  marker.id = static_push_points_id_;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 3.0;
  marker.scale.y = 3.0;
  marker.scale.z = 3.0;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.925;
  marker.color.b = 1.0;

  // Loop through the static push points
  for (int idx = 0; idx < static_push_points_.size(); idx++)
  {
    geometry_msgs::msg::Point point;
    point.x = static_push_points_[idx](0);
    point.y = static_push_points_[idx](1);
    point.z = static_push_points_[idx](2);
    marker.points.push_back(point);
  }

  // Add the single marker to the marker array
  marker_array.markers.push_back(marker);

  // Publish
  pub_static_push_points_->publish(marker_array);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish P points
 */
void DYNUS_NODE::publishPPoints()
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->now();
  marker.ns = "p_points";
  marker.id = p_points_id_; // Use one id.
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 0.667;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  // Add all points to the marker's points list.
  for (size_t idx = 0; idx < p_points_.size(); idx++)
  {
    geometry_msgs::msg::Point point;
    point.x = p_points_[idx](0);
    point.y = p_points_[idx](1);
    point.z = p_points_[idx](2);
    marker.points.push_back(point);
  }

  // Add the single marker to the marker array.
  marker_array.markers.push_back(marker);

  pub_p_points_->publish(marker_array);
}

// ----------------------------------------------------------------------------
/**
 * @brief Publish ROI octomap and trajectories
 */
void DYNUS_NODE::publishROIOctomapAndTrajs(const std::unordered_map<int, std::shared_ptr<octomap::TimedOcTree>> &roi_octrees, std::unordered_map<int, std::vector<dynTraj>> &roi_trajs)
{

  if (!roi_octrees.empty())
  {

    // Cycle through the octrees
    for (auto &[id, octree] : roi_octrees)
    {

      // Check if the publisher is initialized
      if (!pub_roi_octomap_[id])
      {
        std::cerr << "Error: pub_roi_octomap_[" << id << "] is not initialized." << std::endl;
        continue;
      }

      // Check if the octree is valid
      if (!octree)
      {
        std::cerr << "Error: octree is not valid." << std::endl;
        continue;
      }

      octomap_msgs::msg::Octomap octomap_msg;

      // header
      octomap_msg.header.stamp = this->now();
      octomap_msg.header.frame_id = "map";
      octomap_msg.binary = true;
      octomap_msg.id = id_str_;
      octomap_msg.resolution = par_.octomap_res;

      if (!octomap_msgs::binaryMapToMsg(*octree, octomap_msg))
      {
        std::cerr << "Error: Failed to convert octree to Octomap message." << std::endl;
        continue;
      }

      pub_roi_octomap_[id]->publish(octomap_msg);

    } // end for octrees

  } // end if octrees

  if (!roi_trajs.empty())
  {

    // Cycle through the dynamic obstacles
    for (auto &[id, trajs] : roi_trajs)
    {

      // Check if the publisher is initialized
      if (!pub_roi_trajs_[id])
      {
        std::cerr << "Error: pub_roi_trajs_[" << id << "] is not initialized." << std::endl;
        continue;
      }

      // Publish the dynamic obstacles
      dynus_interfaces::msg::DynTrajArray dyn_traj_array_msg;
      dyn_traj_array_msg.header.stamp = this->now();
      dyn_traj_array_msg.header.frame_id = "map";

      for (int i = 0; i < trajs.size(); i++)
      {

        // We don't wanna share the receiver's own traj
        if (trajs[i].id == id)
        {
          continue;
        }

        dynus_interfaces::msg::DynTraj dyn_traj_msg;
        dyn_traj_msg.header.stamp = this->now();
        dyn_traj_msg.header.frame_id = "map";
        dyn_traj_msg.bbox.push_back(trajs[i].bbox[0]);
        dyn_traj_msg.bbox.push_back(trajs[i].bbox[1]);
        dyn_traj_msg.bbox.push_back(trajs[i].bbox[2]);
        dyn_traj_msg.id = trajs[i].id;
        dyn_traj_msg.pwp = dynus_utils::convertPwp2PwpMsg(trajs[i].pwp);
        dyn_traj_msg.ekf_cov_p = dynus_utils::convertCov2CovMsg(trajs[i].ekf_cov_p);
        dyn_traj_msg.ekf_cov_q = dynus_utils::convertCov2CovMsg(trajs[i].ekf_cov_q);
        dyn_traj_msg.poly_cov = dynus_utils::convertCov2CovMsg(trajs[i].poly_cov);
        dyn_traj_msg.is_agent = trajs[i].is_agent;

        dyn_traj_array_msg.trajs.push_back(dyn_traj_msg);
      }

      pub_roi_trajs_[id]->publish(dyn_traj_array_msg);
    }
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Set computation times to zero
 */
void DYNUS_NODE::setComputationTimesToZero()
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
}

// ----------------------------------------------------------------------------

/**
 * @brief Retrive computation times from dynus_ptr_
 */
void DYNUS_NODE::retrieveData()
{
  dynus_ptr_->retrieveData(final_g_,
                           global_planning_time_,
                           dgp_static_jps_time_,
                           dgp_check_path_time_,
                           dgp_dynamic_astar_time_,
                           dgp_recover_path_time_,
                           cvx_decomp_time_,
                           gurobi_computation_time_,
                           safety_check_time_,
                           safe_paths_time_,
                           yaw_sequence_time_,
                           yaw_fitting_time_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish P and N when they are changed
 */
void DYNUS_NODE::publishPandN()
{

  // Get the P and N from DYNUS
  int new_P, new_N, old_P, old_N;
  dynus_ptr_->getPandN(new_P, new_N, old_P, old_N);

  // Create the message
  dynus_interfaces::msg::PNAdaptation pn_msg;
  pn_msg.header.stamp = this->now();
  pn_msg.header.frame_id = "map";
  pn_msg.new_p = new_P;
  pn_msg.new_n = new_N;
  pn_msg.old_p = old_P;
  pn_msg.old_n = old_N;

  // Publish
  pub_pn_adaptation_->publish(pn_msg);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish yaw sequence and b-spline
 */
void DYNUS_NODE::publishYawSequenceAndBSpline()
{

  // Retrieve yaw data
  dynus_ptr_->retrieveYawData(optimal_yaw_sequence_, yaw_control_points_, yaw_knots_);

  // Publish yaw sequence
  if (!optimal_yaw_sequence_.empty() && !yaw_control_points_.empty() && !yaw_knots_.empty())
  {

    // Create yaw output message
    dynus_interfaces::msg::YawOutput yaw_msg;

    // Header
    yaw_msg.header.stamp = this->now();
    yaw_msg.header.frame_id = "map";
    yaw_msg.sequence = optimal_yaw_sequence_;
    yaw_msg.control_points = yaw_control_points_;
    yaw_msg.knots = yaw_knots_;

    // Publish
    pub_yaw_output_->publish(yaw_msg);

    // Clear the vectors
    optimal_yaw_sequence_.clear();
    yaw_control_points_.clear();
    yaw_knots_.clear();
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Print the computation times
 */
void DYNUS_NODE::printComputationTime(bool result)
{
  // Print the computation times
  RCLCPP_INFO(this->get_logger(), "Planner: %s", par_.global_planner.c_str());
  RCLCPP_INFO(this->get_logger(), "Result: %d", result);
  RCLCPP_INFO(this->get_logger(), "Cost (final node's g): %f", final_g_);
  RCLCPP_INFO(this->get_logger(), "Total replanning time [ms]: %f", replanning_computation_time_ * 1000.0);
  RCLCPP_INFO(this->get_logger(), "Global Planning Time [ms]: %f", global_planning_time_);
  RCLCPP_INFO(this->get_logger(), "CVX Decomposition Time [ms]: %f", cvx_decomp_time_);
  RCLCPP_INFO(this->get_logger(), "Gurobi Time [ms]: %f", gurobi_computation_time_);
  RCLCPP_INFO(this->get_logger(), "Safe Paths Time [ms]: %f", safe_paths_time_);
  RCLCPP_INFO(this->get_logger(), "Safety Check Time [ms]: %f", safety_check_time_);
  RCLCPP_INFO(this->get_logger(), "Yaw Sequence Time [ms]: %f", yaw_sequence_time_);
  RCLCPP_INFO(this->get_logger(), "Yaw Fitting Time [ms]: %f", yaw_fitting_time_);
  if (par_.global_planner == "dgp")
  {
    RCLCPP_INFO(this->get_logger(), "Static JPS Time [ms]: %f", dgp_static_jps_time_);
    RCLCPP_INFO(this->get_logger(), "Check Path Time [ms]: %f", dgp_check_path_time_);
    RCLCPP_INFO(this->get_logger(), "Dynamic A* Time [ms]: %f", dgp_dynamic_astar_time_);
    RCLCPP_INFO(this->get_logger(), "Recover Path Time [ms]: %f", dgp_recover_path_time_);
  }
  RCLCPP_INFO(this->get_logger(), "------------------------");
}

// ----------------------------------------------------------------------------

/**
 * @brief Record the data
 * @param result result of the replanning
 */
void DYNUS_NODE::recordData(bool result)
{

  // Record all the data into global_path_benchmark_
  std::tuple<bool, double, double, double, double, double, double, double, double, double, double, double, double, double> data;
  if (par_.global_planner == "dgp")
  {
    data = std::make_tuple(result, final_g_, replanning_computation_time_, global_planning_time_, cvx_decomp_time_, gurobi_computation_time_, safe_paths_time_, safety_check_time_, yaw_sequence_time_, yaw_fitting_time_, dgp_static_jps_time_, dgp_check_path_time_, dgp_dynamic_astar_time_, dgp_recover_path_time_);
  }
  else
  {
    data = std::make_tuple(result, final_g_, replanning_computation_time_, global_planning_time_, cvx_decomp_time_, gurobi_computation_time_, safe_paths_time_, safety_check_time_, yaw_sequence_time_, yaw_fitting_time_, 0.0, 0.0, 0.0, 0.0);
  }

  global_path_benchmark_.push_back(data);
}

// ----------------------------------------------------------------------------

/**
 * @brief Log the data to a csv file
 */
void DYNUS_NODE::logData()
{

  // Loc the computation times to csv file
  // std::ofstream log_file(file_path_, std::ios_base::app); // Open the file in append mode
  std::ofstream log_file(file_path_); // Open the file in overwrite mode
  if (log_file.is_open())
  {
    if (par_.global_planner == "dgp")
    {
      // Header
      log_file << "Planner,Result,Cost (final node's g),Total replanning time [ms],Global Planning Time [ms],CVX Decomposition Time [ms],Gurobi Time [ms],Safe Paths Time [ms],Safety Check Time [ms],Yaw Sequence Time [ms],Yaw Fitting Time [ms],Static JPS Time [ms],Check Path Time [ms],Dynamic A* Time [ms],Recover Path Time [ms]\n";
    }
    else
    {
      // Header
      log_file << "Planner,Result,Cost (final node's g),Total replanning time [ms],Global Planning Time [ms],CVX Decomposition Time [ms],Gurobi Time [ms],Safe Paths Time [ms],Safety Check Time [ms],Yaw Sequence Time [ms],Yaw Fitting Time [ms]\n";
    }

    // Data
    for (const auto &row : global_path_benchmark_)
    {
      if (par_.global_planner == "dgp")
      {
        log_file << par_.global_planner << "," << std::get<0>(row) << "," << std::get<1>(row) << "," << std::get<2>(row) * 1000.0 << "," << std::get<3>(row) << "," << std::get<4>(row) << "," << std::get<5>(row) << "," << std::get<6>(row) << "," << std::get<7>(row) << "," << std::get<8>(row) << "," << std::get<9>(row) << "," << std::get<10>(row) << "," << std::get<11>(row) << "," << std::get<12>(row) << "," << std::get<13>(row) << "\n";
      }
      else
      {
        log_file << par_.global_planner << "," << std::get<0>(row) << "," << std::get<1>(row) << "," << std::get<2>(row) * 1000.0 << "," << std::get<3>(row) << "," << std::get<4>(row) << "," << std::get<5>(row) << "," << std::get<6>(row) << "," << std::get<7>(row) << "," << std::get<8>(row) << "," << std::get<9>(row) << "\n";
      }
    }

    log_file.close();
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the Point G (sub goal)
 */
void DYNUS_NODE::publishPointG() const
{

  // get projected goal (G)
  state G;
  dynus_ptr_->getG(G);

  // Publish the goal for visualization
  publishState(G, pub_point_G_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the Point E (sub goal)
 */
void DYNUS_NODE::publishPointE() const
{

  // get projected goal (E)
  state E;
  dynus_ptr_->getE(E);

  // Publish the goal for visualization
  publishState(E, pub_point_E_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the Point A (trajectory start point)
 */
void DYNUS_NODE::publishPointA() const
{

  // get projected goal (A)
  state A;
  dynus_ptr_->getA(A);

  // Publish the goal for visualization
  publishState(A, pub_point_A_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the current state
 */
void DYNUS_NODE::publishCurrentState(const state &state) const
{
  // Publish the goal for visualization
  publishState(state, pub_current_state_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish state
 */
void DYNUS_NODE::publishState(const state &data, const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &publisher) const
{
  geometry_msgs::msg::PointStamped p;
  p.header.frame_id = "map";
  p.point = eigen2point(data.pos);
  publisher->publish(p);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish its own trajectory for deconfliction
 */
void DYNUS_NODE::publishOwnTraj()
{

  // Retrieve pwp_to_share_
  dynus_ptr_->retrievePwpToShare(pwp_to_share_);

  dynus_interfaces::msg::DynTraj msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.bbox.push_back(par_.drone_bbox[0]);
  msg.bbox.push_back(par_.drone_bbox[1]);
  msg.bbox.push_back(par_.drone_bbox[2]);
  msg.id = id_;
  msg.pwp = dynus_utils::convertPwp2PwpMsg(pwp_to_share_);
  msg.is_agent = true;

  // Get the terminal goal
  state G;
  dynus_ptr_->getG(G);
  msg.goal.push_back(G.pos(0));
  msg.goal.push_back(G.pos(1));
  msg.goal.push_back(G.pos(2));

  // Publish the trajectory
  pub_own_traj_->publish(msg);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the trajectory the agent actually followed for visualization
 */
void DYNUS_NODE::publishActualTraj()
{
  // Initialize the previous point
  static geometry_msgs::msg::Point prev_p = pointOrigin();

  // Get the current state and position
  state current_state;
  dynus_ptr_->getState(current_state);
  Eigen::Vector3d current_pos = current_state.pos;

  if (current_pos.norm() < 1e-2)
    return; // because the state is not updated yet

  // If we use UAV, we can just use the state topic published by fake_sim, but if we use ground robot, since we use TF for state publisher, we cannot get velocity info from the state topic. So we will approximiate
  if (par_.vehicle_type != "uav")
  {

    // Initialize the previous position and time
    if (!publish_actual_traj_called_)
    {
      actual_traj_prev_pos_ = current_pos;
      actual_traj_prev_time_ = this->now().seconds();
      publish_actual_traj_called_ = true;
      return;
    }

    // Get the velocity
    current_state.vel = (current_pos - actual_traj_prev_pos_) / (this->now().seconds() - actual_traj_prev_time_);
  }

  // Set up the marker
  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.id = actual_traj_id_;
  m.ns = "actual_traj_" + id_str_;
  m.color = getColorJet(current_state.vel.norm(), 0, par_.v_max); // note that par_.v_max is per axis
  m.scale.x = 0.15;
  m.scale.y = 0.0001;
  m.scale.z = 0.0001;
  m.header.stamp = this->now();
  m.header.frame_id = "map";

  // pose is actually not used in the marker, but if not RVIZ complains about the quaternion
  m.pose.position = pointOrigin();
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  // Set the points
  geometry_msgs::msg::Point p;
  p = dynus_utils::convertEigen2Point(current_pos);
  m.points.push_back(prev_p);
  m.points.push_back(p);
  prev_p = p;

  // Return if the actual_traj_id_ is 0 - avoid publishing the first point which goes from the origin to the first point
  if (actual_traj_id_ == 0)
  {
    actual_traj_id_++;
    return;
  }
  actual_traj_id_++;

  // Publish the marker
  pub_actual_traj_->publish(m);
}

// ----------------------------------------------------------------------------

/**
 * @brief Clear the marker array
 */
void DYNUS_NODE::clearMarkerActualTraj()
{
  visualization_msgs::msg::Marker m;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::DELETEALL;
  m.id = 0;
  m.scale.x = 0.02;
  m.scale.y = 0.04;
  m.scale.z = 1;
  pub_actual_traj_->publish(m);
  actual_traj_id_ = 0;
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish goal (setpoint)
 */
void DYNUS_NODE::publishGoal()
{

  // Initialize the goal
  state next_goal;

  // Get the next goal
  if (dynus_ptr_->getNextGoal(next_goal) && par_.use_state_update)
  {

    // Publish the goal (actual setpoint)
    dynus_interfaces::msg::Goal quadGoal;
    quadGoal.header.stamp = this->now();
    quadGoal.header.frame_id = "map";
    quadGoal.p = eigen2rosvector(next_goal.pos);
    quadGoal.v = eigen2rosvector(next_goal.vel);
    quadGoal.a = eigen2rosvector(next_goal.accel);
    quadGoal.j = eigen2rosvector(next_goal.jerk);
    quadGoal.yaw = next_goal.yaw;
    quadGoal.dyaw = next_goal.dyaw;
    pub_goal_->publish(quadGoal);

    // Publish the goal (setpoint) for visualization
    if (par_.visual_level >= 1)
      publishState(next_goal, pub_setpoint_);
  }

  // Publish FOV
  if (par_.visual_level >= 1)
    publishFOV();
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish frontiers
 */
void DYNUS_NODE::publishFrontiers()
{

  // Get all the visible frontiers
  // vec_Vecf<3> visible_frontiers;
  // dynus_ptr_->getVisibleFrontiers(visible_frontiers);
  // if (!visible_frontiers.empty())
  // {
  //     createFrontiersMarker(visible_frontiers, color(ORANGE_TRANS), 0, 0.1);
  // }

  // Get all the frontiers
  vec_Vecf<3> all_frontiers;
  dynus_ptr_->getAllFrontiers(all_frontiers);

  if (!all_frontiers.empty())
  {
    createFrontiersMarker(all_frontiers, color(RED), 0, 0.1);
  }

  // Get the best frontier
  // vec_Vecf<3> best_frontier_vec;
  // best_frontier_vec.push_back(best_frontier_);
  // createFrontiersMarker(best_frontier_vec, color(BLUE), all_frontiers.size(), 0.3);
  pub_frontiers_->publish(frontiers_marker_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Create frontiers marker
 * @param frontiers frontiers
 * @param color color
 * @param start_id start id
 * @param scale scale
 */
void DYNUS_NODE::createFrontiersMarker(const vec_Vecf<3> &frontiers, std_msgs::msg::ColorRGBA color, int start_id, double scale)
{

  for (int i = 0; i < frontiers.size(); i++)
  {
    visualization_msgs::msg::Marker frontier;
    frontier.header.frame_id = "map";
    frontier.header.stamp = this->now();
    frontier.ns = "frontiers";
    frontier.id = start_id + i;
    frontier.type = visualization_msgs::msg::Marker::SPHERE;
    frontier.action = visualization_msgs::msg::Marker::ADD;
    frontier.pose.position.x = frontiers[i](0);
    frontier.pose.position.y = frontiers[i](1);
    frontier.pose.position.z = frontiers[i](2);
    frontier.pose.orientation.x = 0.0;
    frontier.pose.orientation.y = 0.0;
    frontier.pose.orientation.z = 0.0;
    frontier.pose.orientation.w = 1.0;
    frontier.scale.x = scale;
    frontier.scale.y = scale;
    frontier.scale.z = scale;
    frontier.color = color;
    frontiers_marker_.markers.push_back(frontier);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish Sefe Corridor Polyhedra
 */
void DYNUS_NODE::publishPoly()
{

  // retrieve the polyhedra
  dynus_ptr_->retrievePolytopes(poly_whole_, poly_safe_);

  // For whole trajectory
  if (!poly_whole_.empty())
  {
    decomp_ros_msgs::msg::PolyhedronArray poly_whole_msg = DecompROS::polyhedron_array_to_ros(poly_whole_);
    poly_whole_msg.header.stamp = this->now();
    poly_whole_msg.header.frame_id = "map";
    poly_whole_msg.lifetime = rclcpp::Duration::from_seconds(1.0);
    pub_poly_whole_->publish(poly_whole_msg);
  }

  // For safe trajectory
  if (!poly_safe_.empty())
  {
    decomp_ros_msgs::msg::PolyhedronArray poly_safe_msg = DecompROS::polyhedron_array_to_ros(poly_safe_);
    poly_safe_msg.header.stamp = this->now();
    poly_safe_msg.header.frame_id = "map";
    poly_safe_msg.lifetime = rclcpp::Duration::from_seconds(1.0);
    pub_poly_safe_->publish(poly_safe_msg);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish trajectory for visualization
 */
void DYNUS_NODE::publishTraj()
{

  // Retrieve goal setpoints
  dynus_ptr_->retrieveGoalSetpoints(goal_setpoints_);

  // Trajectory
  nav_msgs::msg::Path traj;
  traj.poses.clear();
  traj.header.stamp = this->now();
  traj.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped temp_path;

  for (int i = 0; i < goal_setpoints_.size(); i = i + 8)
  {
    // get time stamp and add to the path
    int int_time = (int)goal_setpoints_[i].t + 0.5 - (goal_setpoints_[i].t < 0);
    double frac_time = goal_setpoints_[i].t - int_time;
    temp_path.header.stamp.sec = int_time;
    temp_path.header.stamp.nanosec = frac_time * 1e9;
    temp_path.header.frame_id = "map";
    temp_path.pose.position.x = goal_setpoints_[i].pos(0);
    temp_path.pose.position.y = goal_setpoints_[i].pos(1);
    temp_path.pose.position.z = goal_setpoints_[i].pos(2);
    temp_path.pose.orientation.w = 1;
    temp_path.pose.orientation.x = 0;
    temp_path.pose.orientation.y = 0;
    temp_path.pose.orientation.z = 0;
    traj.poses.push_back(temp_path);
  }

  // Publish the trajectory
  traj_committed_colored_ = stateVector2ColoredMarkerArray(goal_setpoints_, 1, par_.v_max);
  pub_traj_committed_colored_->publish(traj_committed_colored_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the global path (that can go through unknown space)
 */
void DYNUS_NODE::publishGlobalPath()
{

  int global_path_color = RED;

  // Generate random integer from 1 to 10 to generate random color
  if (par_.use_random_color_for_global_path)
    global_path_color = rand() % 10 + 1;

  // Get global_path
  vec_Vecf<3> global_path;
  dynus_ptr_->getGlobalPath(global_path);

  if (global_path.empty())
    return;

  // Publish global_path
  clearMarkerArray(dgp_path_marker_, pub_dgp_path_marker_);
  vectorOfVectors2MarkerArray(global_path, &dgp_path_marker_, color(global_path_color));
  pub_dgp_path_marker_->publish(dgp_path_marker_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the free global path (that only goes through free space)
 */
void DYNUS_NODE::publishFreeGlobalPath()
{

  // Get free_global_path
  vec_Vecf<3> free_global_path;
  dynus_ptr_->getFreeGlobalPath(free_global_path);

  if (free_global_path.empty())
    return;

  // Publish free_global_path
  clearMarkerArray(dgp_free_path_marker_, pub_free_dgp_path_marker_);
  vectorOfVectors2MarkerArray(free_global_path, &dgp_free_path_marker_, color(GREEN));
  pub_free_dgp_path_marker_->publish(dgp_free_path_marker_);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the local_global_path and local_global_path_after_push_
 */
void DYNUS_NODE::publishLocalGlobalPath()
{

  // Get the local global path and local global path after push
  vec_Vecf<3> local_global_path;
  vec_Vecf<3> local_global_path_after_push;
  dynus_ptr_->getLocalGlobalPath(local_global_path, local_global_path_after_push);

  if (!local_global_path.empty())
  {
    // Publish local_global_path
    clearMarkerArray(dgp_local_global_path_marker_, pub_local_global_path_marker_);
    vectorOfVectors2MarkerArray(local_global_path, &dgp_local_global_path_marker_, color(BLUE));
    pub_local_global_path_marker_->publish(dgp_local_global_path_marker_);
  }

  if (!local_global_path_after_push.empty())
  {
    // Publish local_global_path_after_push
    clearMarkerArray(dgp_local_global_path_after_push_marker_, pub_local_global_path_after_push_marker_);
    vectorOfVectors2MarkerArray(local_global_path_after_push, &dgp_local_global_path_after_push_marker_, color(ORANGE));
    pub_local_global_path_after_push_marker_->publish(dgp_local_global_path_after_push_marker_);
  }
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the static map
 */
void DYNUS_NODE::publishStaticMap()
{

  // Get the occupied cells (debug)
  double current_time = this->now().seconds();
  vec_Vecf<3> occupied_cells;
  dynus_ptr_->getMap(occupied_cells);

  // Get the PointCloud2 from the occupied cells (debug)
  sensor_msgs::msg::PointCloud2 pc2_msg;
  this->getPointCloud2FromVec_Vecf3(occupied_cells, pc2_msg);
  pc2_msg.header.frame_id = "map";
  pc2_msg.header.stamp = this->now();

  // Publish the PointCloud2 (debug)
  pub_map_->publish(pc2_msg);
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the dynamic map
 */
void DYNUS_NODE::publishDynamicMap()
{

  // check if state is initialized
  if (!state_initialized_)
  {
    RCLCPP_INFO(this->get_logger(), "State not initialized yet");
    return;
  }

  // Get the current_time
  double current_time = this->now().seconds();

  vec_Vecf<3> occupied_cells;
  vec_Vecf<3> free_cells;
  vec_Vecf<3> unknown_cells;
  dynus_ptr_->getDynamicMap(occupied_cells, free_cells, unknown_cells, current_time);

  // Get MarkerArray for occupied cells
  visualization_msgs::msg::MarkerArray occupied_cells_marker;
  this->createMarkerArrayFromVec_Vec3f(occupied_cells, color(RED_TRANS_TRANS), 0, 0.1, &occupied_cells_marker);
  // this->createMarkerArrayFromVec_Vec3f(occupied_cells, color(RED), 0, 0.1, &occupied_cells_marker);
  pub_dynamic_map_marker_->publish(occupied_cells_marker);

  // Get MarkerArray for free cells
  visualization_msgs::msg::MarkerArray free_cells_marker;
  this->createMarkerArrayFromVec_Vec3f(free_cells, color(GREEN_TRANS_TRANS), 0, 0.1, &free_cells_marker);
  pub_free_map_marker_->publish(free_cells_marker);

  // Get MarkerArray for unknown cells
  visualization_msgs::msg::MarkerArray unknown_cells_marker;
  this->createMarkerArrayFromVec_Vec3f(unknown_cells, color(ORANGE_TRANS), 0, 0.1, &unknown_cells_marker);
  pub_unknown_map_marker_->publish(unknown_cells_marker);

  // // Get the PointCloud2 from the occupied cells (debug)
  // sensor_msgs::msg::PointCloud2 pc2_msg;
  // this->getPointCloud2FromVec_Vecf3(occupied_cells, pc2_msg);
  // pc2_msg.header.frame_id = "map";
  // pc2_msg.header.stamp = this->now();

  // // Get the PointCloud2 from the free cells (debug)
  // sensor_msgs::msg::PointCloud2 pc2_msg_free;
  // this->getPointCloud2FromVec_Vecf3(free_cells, pc2_msg_free);
  // pc2_msg_free.header.frame_id = "map";
  // pc2_msg_free.header.stamp = this->now();

  // // Get the PointCloud2 from the unknown cells (debug)
  // sensor_msgs::msg::PointCloud2 pc2_msg_unknown;
  // this->getPointCloud2FromVec_Vecf3(unknown_cells, pc2_msg_unknown);
  // pc2_msg_unknown.header.frame_id = "map";
  // pc2_msg_unknown.header.stamp = this->now();

  // // Publish the PointCloud2 (debug)
  // pub_dynamic_map_->publish(pc2_msg);
  // pub_free_map_->publish(pc2_msg_free);
  // pub_unknown_map_->publish(pc2_msg_unknown);
}

// ----------------------------------------------------------------------------

/**
 * @brief Create MarkerArray from vec_Vec3f
 */
void DYNUS_NODE::createMarkerArrayFromVec_Vec3f(
    const vec_Vec3f &occupied_cells, const std_msgs::msg::ColorRGBA &color, int namespace_id, double scale, visualization_msgs::msg::MarkerArray *marker_array)
{

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";              // Set the appropriate frame
  marker.header.stamp = rclcpp::Clock().now(); // Use ROS2 clock
  marker.ns = "namespace_" + std::to_string(namespace_id);
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST; // Each point will be visualized as a cube
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = par_.res;
  marker.scale.y = par_.res;
  marker.scale.z = par_.res;
  marker.color = color;

  for (const auto &cell : occupied_cells)
  {
    geometry_msgs::msg::Point point;
    point.x = cell(0);
    point.y = cell(1);
    point.z = cell(2);
    marker.points.push_back(point);
  }

  marker_array->markers.push_back(marker);
}

// ----------------------------------------------------------------------------

/**
 * @brief Get PointCloud2 from vec_Vecf3
 */
void DYNUS_NODE::getPointCloud2FromVec_Vecf3(const vec_Vecf<3> &vec, sensor_msgs::msg::PointCloud2 &pc2_msg)
{
  pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2());

  // Convert the vector to PointCloud2
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(vec.size());

  for (int i = 0; i < vec.size(); i++)
  {
    cloud->points[i].x = vec[i](0);
    cloud->points[i].y = vec[i](1);
    cloud->points[i].z = vec[i](2);
  }
  pcl::toPCLPointCloud2(*cloud, *pcl_cloud);

  // Convert the PointCloud2 to sensor_msgs::msg::PointCloud2
  pcl_conversions::fromPCL(*pcl_cloud, pc2_msg);
}

// ----------------------------------------------------------------------------

/**
 * @brief Clear any marker array
 */
void DYNUS_NODE::clearMarkerArray(visualization_msgs::msg::MarkerArray &path_marker, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher)
{

  // If the marker array is empty, return
  if (path_marker.markers.size() == 0)
    return;

  // Clear the marker array
  int id_begin = path_marker.markers[0].id;

  for (int i = 0; i < path_marker.markers.size(); i++)
  {
    visualization_msgs::msg::Marker m;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::DELETE;
    m.id = i + id_begin;
    path_marker.markers[i] = m;
  }

  publisher->publish(path_marker);
  path_marker.markers.clear();
}

// ----------------------------------------------------------------------------

/**
 * @brief Construct the FOV marker for visualization
 */
void DYNUS_NODE::constructFOVMarker()
{

  marker_fov_.header.stamp = this->now();
  marker_fov_.header.frame_id = d435_depth_frame_id_;
  marker_fov_.ns = "marker_fov";
  marker_fov_.id = marker_fov_id_++;
  marker_fov_.frame_locked = true;
  marker_fov_.type = marker_fov_.LINE_LIST;
  marker_fov_.action = marker_fov_.ADD;
  marker_fov_.pose = dynus_utils::identityGeometryMsgsPose();

  double delta_y = par_.fov_visual_depth * fabs(tan((par_.fov_visual_x_deg * M_PI / 180) / 2.0));
  double delta_z = par_.fov_visual_depth * fabs(tan((par_.fov_visual_y_deg * M_PI / 180) / 2.0));

  geometry_msgs::msg::Point v0 = eigen2point(Eigen::Vector3d(0.0, 0.0, 0.0));
  geometry_msgs::msg::Point v1 = eigen2point(Eigen::Vector3d(-delta_y, delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v2 = eigen2point(Eigen::Vector3d(delta_y, delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v3 = eigen2point(Eigen::Vector3d(delta_y, -delta_z, par_.fov_visual_depth));
  geometry_msgs::msg::Point v4 = eigen2point(Eigen::Vector3d(-delta_y, -delta_z, par_.fov_visual_depth));

  marker_fov_.points.clear();

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v1);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v0);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v1);
  marker_fov_.points.push_back(v2);

  // Line
  marker_fov_.points.push_back(v2);
  marker_fov_.points.push_back(v3);

  // Line
  marker_fov_.points.push_back(v3);
  marker_fov_.points.push_back(v4);

  // Line
  marker_fov_.points.push_back(v4);
  marker_fov_.points.push_back(v1);

  marker_fov_.scale.x = 0.03;
  marker_fov_.scale.y = 0.00001;
  marker_fov_.scale.z = 0.00001;
  marker_fov_.color.a = 1.0;
  marker_fov_.color.r = 0.0;
  marker_fov_.color.g = 1.0;
  marker_fov_.color.b = 0.0;
}

// ----------------------------------------------------------------------------

/**
 * @brief Publish the FOV marker for visualization
 */
void DYNUS_NODE::publishFOV()
{
  marker_fov_.header.stamp = this->now();
  pub_fov_->publish(marker_fov_);
  return;
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::insertLidarCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  auto now = this->now();
  if ((now - last_lidar_callback_time_) < rclcpp::Duration(0, (1.0 / lidar_insert_hz_) * 1e9))
  {
    RCLCPP_DEBUG(get_logger(), "insertLidarCloudCallback throttled (%f Hz limit)", lidar_insert_hz_);
    return;
  }
  last_lidar_callback_time_ = now;

  processCloud(cloud, "LiDAR");
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::insertDepthCameraCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{

  auto now = this->now();
  if ((now - last_depth_camera_callback_time_) < rclcpp::Duration(0, (1.0 / depth_camera_insert_hz_) * 1e9))
  {
    RCLCPP_DEBUG(get_logger(), "insertDepthCameraCloudCallback throttled (%f Hz limit)", depth_camera_insert_hz_);
    return;
  }
  last_depth_camera_callback_time_ = now;

  processCloud(cloud, "Depth Camera");
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud, const std::string &sensor_type)
{

  const auto start_time = rclcpp::Clock{}.now();

  // Get base_frame_id
  std::string base_frame_id;
  if (sensor_type == "LiDAR")
  {
    base_frame_id = lidar_frame_id_;
  }
  else if (sensor_type == "Depth Camera")
  {
    base_frame_id = d435_depth_frame_id_;
  }

  //
  // ground filtering in base frame
  //

  pcl::PointCloud<pcl::PointXYZ> pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
  try
  {
    sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
        world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);

  pcl::PointCloud<pcl::PointXYZ> pc_ground;    // segmented ground plane
  pcl::PointCloud<pcl::PointXYZ> pc_nonground; // everything else

  if (filter_ground_plane_)
  {
    geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped;
    geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
    try
    {
      tf2_buffer_->canTransform(
          base_frame_id, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(2.0));
      sensor_to_base_transform_stamped = tf2_buffer_->lookupTransform(
          base_frame_id, cloud->header.frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(2.0));
      base_to_world_transform_stamped = tf2_buffer_->lookupTransform(
          world_frame_id_, base_frame_id, cloud->header.stamp,
          rclcpp::Duration::from_seconds(2.0));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR_STREAM(
          get_logger(),
          "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                                                                      "You need to set the base_frame_id or disable filter_ground.");
    }

    // transform pointcloud from sensor frame to fixed robot frame
    pcl_ros::transformPointCloud(pc, pc, sensor_to_base_transform_stamped);
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl_ros::transformPointCloud(pc_ground, pc_ground, base_to_world_transform_stamped);
    pcl_ros::transformPointCloud(pc_nonground, pc_nonground, base_to_world_transform_stamped);
  }
  else
  {
    // directly transform to map frame:
    pcl_ros::transformPointCloud(pc, pc, sensor_to_world_transform_stamped);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }

  // Get current time
  double current_time = rclcpp::Clock{}.now().seconds();

  const auto &t = sensor_to_world_transform_stamped.transform.translation;
  tf2::Vector3 sensor_to_world_vec3{t.x, t.y, t.z};

  insertScan(sensor_to_world_vec3, pc_ground, pc_nonground, sensor_type);

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(
      get_logger(),
      "Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)",
      pc_ground.size(), pc_nonground.size(), total_elapsed);

  // Get current time
  current_time = rclcpp::Clock{}.now().seconds();

  publishAll(cloud->header.stamp, sensor_type);

  // If using frontiers, we need to give the octree to dynus_ptr_
  if (par_.use_frontiers)
    dynus_ptr_->setOctree(lidar_octree_, depth_camera_octree_);

  if (!octomap_initialized_)
  {
    octomap_initialized_ = true;
    std::cout << bold << green << "Octomap initialized" << reset << std::endl;
  }

  // Print out the time taken
  // RCLCPP_INFO(get_logger(), "Time taken for processing the cloud: %f ms", (rclcpp::Clock{}.now() - start_time).seconds() * 1000);
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::insertScan(
    const tf2::Vector3 &sensor_origin_tf,
    const pcl::PointCloud<pcl::PointXYZ> &ground,
    const pcl::PointCloud<pcl::PointXYZ> &nonground,
    const std::string &sensor_type)
{

  double sensor_max_range = 0.0;
  // create local octree for each sensor
  std::shared_ptr<OcTreeT> local_octree;
  if (sensor_type == "LiDAR")
  {
    sensor_max_range = par_.lidar_max_range;
    local_octree = std::shared_ptr<OcTreeT>(lidar_octree_);
  }
  else if (sensor_type == "Depth Camera")
  {
    sensor_max_range = par_.depth_camera_depth_max;
    local_octree = std::shared_ptr<OcTreeT>(depth_camera_octree_);
  }

  // Convert sensor origin
  const auto sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);
  if (!local_octree->coordToKeyChecked(sensor_origin, update_bbox_min_) ||
      !local_octree->coordToKeyChecked(sensor_origin, update_bbox_max_))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for origin " << sensor_origin);
  }

  // Global containers for the keys.
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

// --- Process ground points in parallel ---
#pragma omp parallel
  {
    // Thread-local containers.
    octomap::KeySet local_free_cells;
    // Copy the global bounding box keys into thread-local ones.
    octomap::OcTreeKey local_bbox_min = update_bbox_min_;
    octomap::OcTreeKey local_bbox_max = update_bbox_max_;
    octomap::KeyRay local_key_ray; // Local instance for computeRayKeys.

// Use OpenMP for loop over ground points.
#pragma omp for nowait
    for (size_t i = 0; i < ground.size(); ++i)
    {
      const auto &pt = ground[i];
      octomap::point3d point(pt.x, pt.y, pt.z);

      // Check for max range:
      if ((sensor_max_range > 0.0) &&
          ((point - sensor_origin).norm() > sensor_max_range))
      {
        point = sensor_origin + (point - sensor_origin).normalized() * sensor_max_range;
      }

      // Insert free cells along the ray.
      if (local_octree->computeRayKeys(sensor_origin, point, local_key_ray))
      {
        local_free_cells.insert(local_key_ray.begin(), local_key_ray.end());
      }

      // Update bounding box using the endpoint.
      octomap::OcTreeKey end_key;
      if (local_octree->coordToKeyChecked(point, end_key))
      {
        updateMinKey(end_key, local_bbox_min);
        updateMaxKey(end_key, local_bbox_max);
      }
      else
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << point);
      }
    }

// Merge thread-local results into global containers.
#pragma omp critical
    {
      free_cells.insert(local_free_cells.begin(), local_free_cells.end());
      updateMinKey(local_bbox_min, update_bbox_min_);
      updateMaxKey(local_bbox_max, update_bbox_max_);
    }
  } // End parallel region for ground

// --- Process nonground points in parallel ---
#pragma omp parallel
  {
    octomap::KeySet local_free_cells;
    octomap::KeySet local_occupied_cells;
    octomap::OcTreeKey local_bbox_min = update_bbox_min_;
    octomap::OcTreeKey local_bbox_max = update_bbox_max_;
    octomap::KeyRay local_key_ray;

#pragma omp for nowait
    for (size_t i = 0; i < nonground.size(); ++i)
    {
      const auto &pt = nonground[i];
      octomap::point3d point(pt.x, pt.y, pt.z);

      if ((sensor_max_range < 0.0) ||
          ((point - sensor_origin).norm() <= sensor_max_range))
      {
        // Insert free cells along the ray.
        if (local_octree->computeRayKeys(sensor_origin, point, local_key_ray))
        {
          local_free_cells.insert(local_key_ray.begin(), local_key_ray.end());
        }
        // Occupied endpoint.
        octomap::OcTreeKey key;
        if (local_octree->coordToKeyChecked(point, key))
        {
          local_occupied_cells.insert(key);
          updateMinKey(key, local_bbox_min);
          updateMaxKey(key, local_bbox_max);
        }
      }
      else
      {
        // Ray exceeds max range: compute new endpoint.
        octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * sensor_max_range;
        if (local_octree->computeRayKeys(sensor_origin, new_end, local_key_ray))
        {
          local_free_cells.insert(local_key_ray.begin(), local_key_ray.end());
          octomap::OcTreeKey end_key;
          if (local_octree->coordToKeyChecked(new_end, end_key))
          {
            local_free_cells.insert(end_key);
            updateMinKey(end_key, local_bbox_min);
            updateMaxKey(end_key, local_bbox_max);
          }
          else
          {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << new_end);
          }
        }
      }
    }

// Merge thread-local results into the global sets.
#pragma omp critical
    {
      free_cells.insert(local_free_cells.begin(), local_free_cells.end());
      occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
      updateMinKey(local_bbox_min, update_bbox_min_);
      updateMaxKey(local_bbox_max, update_bbox_max_);
    }
  } // End parallel region for nonground

  double current_time = rclcpp::Clock{}.now().seconds();

  // --- Mark free cells (serial update) ---
  // Process free cells in parallel.
  std::for_each(std::execution::par, free_cells.begin(), free_cells.end(),
                [&](const auto &cell)
                {
                  // Check if the cell is not marked as occupied.
                  if (occupied_cells.find(cell) == occupied_cells.end())
                  {
                    // Protect the update with a lock if necessary.
                    std::lock_guard<std::mutex> lock(octree_mutex_);
                    local_octree->updateNode(cell, false);
                    if (use_decay_)
                    {
                      dynamic_cast<OcTreeT *>(local_octree.get())->updateNodeTimestamp(cell, current_time);
                    }
                  }
                });

  // Process occupied cells in parallel.
  std::for_each(std::execution::par, occupied_cells.begin(), occupied_cells.end(),
                [&](const auto &cell)
                {
                  std::lock_guard<std::mutex> lock(octree_mutex_);
                  local_octree->updateNode(cell, true);
                  if (use_decay_)
                  {
                    dynamic_cast<OcTreeT *>(local_octree.get())->updateNodeTimestamp(cell, current_time);
                  }
                });

  // --- Log bounding box information ---
  octomap::point3d min_pt = local_octree->keyToCoord(update_bbox_min_);
  octomap::point3d max_pt = local_octree->keyToCoord(update_bbox_max_);
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Bounding box keys (before): " << update_bbox_min_[0] << " " << update_bbox_min_[1] << " " << update_bbox_min_[2]
                                                     << " / " << update_bbox_max_[0] << " " << update_bbox_max_[1] << " " << update_bbox_max_[2]);
  RCLCPP_DEBUG_STREAM(get_logger(), "Updated area bounding box: " << min_pt << " - " << max_pt);
  RCLCPP_DEBUG_STREAM(get_logger(),
                      "Bounding box keys (after): " << update_bbox_min_[0] << " " << update_bbox_min_[1] << " " << update_bbox_min_[2]
                                                    << " / " << update_bbox_max_[0] << " " << update_bbox_max_[1] << " " << update_bbox_max_[2]);

  if (compress_map_)
  {
    octree_mutex_.lock();
    local_octree->prune();
    octree_mutex_.unlock();
  }

  // If decay is enabled, update node decay first (sequentially).
  if (use_decay_)
  {
    try
    {
      octree_mutex_.lock();

      auto *octree = dynamic_cast<OcTreeT *>(local_octree.get());
      if (!octree)
      {
        throw std::runtime_error("Invalid octree type in publishAll");
      }
      octree->decayNodes(this->get_clock()->now().seconds());

      octree_mutex_.unlock();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error during node decay: %s", e.what());
    }
  }
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::filterGroundPlane(
    const PCLPointCloud &pc, PCLPointCloud &ground,
    PCLPointCloud &nonground) const
{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50)
  {
    RCLCPP_WARN(
        get_logger(), "Pointcloud in OctomapServer too small, skipping ground plane extraction");
    nonground = pc;
  }
  else
  {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients(true);
    // TODO(someone): maybe a filtering based on the surface normals
    // might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(ground_filter_distance_);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(ground_filter_angle_);

    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool ground_plane_found = false;

    while (cloud_filtered.size() > 10 && !ground_plane_found)
    {
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        RCLCPP_INFO(get_logger(), "PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < ground_filter_plane_distance_)
      {
        RCLCPP_DEBUG(
            get_logger(),
            "Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f",
            inliers->indices.size(), cloud_filtered.size(),
            coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative(false);
        extract.filter(ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size())
        {
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        ground_plane_found = true;
      }
      else
      {
        RCLCPP_DEBUG(
            get_logger(),
            "Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
            inliers->indices.size(), cloud_filtered.size(),
            coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative(false);
        extract.filter(cloud_out);
        nonground += cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size())
        {
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        }
        else
        {
          cloud_filtered.points.clear();
        }
      }
    }
    // TODO(someone): also do this if overall starting pointcloud too small?
    if (!ground_plane_found)
    { // no plane found or remaining points too small
      RCLCPP_WARN(get_logger(), "No ground plane found in scan");
      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-ground_filter_plane_distance_, ground_filter_plane_distance_);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);
      second_pass.setNegative(true);
      second_pass.filter(nonground);
    }
    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);
  }
}

// ----------------------------------------------------------------------------

void DYNUS_NODE::publishAll(const rclcpp::Time &rostime, const std::string &sensor_type)
{

  std::shared_ptr<OcTreeT> local_octree;
  if (sensor_type == "LiDAR")
  {
    local_octree = std::shared_ptr<OcTreeT>(lidar_octree_);
  }
  else if (sensor_type == "Depth Camera")
  {
    local_octree = std::shared_ptr<OcTreeT>(depth_camera_octree_);
  }

  const auto start_time = rclcpp::Clock{}.now();
  const size_t octomap_size = local_octree->size();
  if (octomap_size <= 1)
  {
    RCLCPP_WARN(get_logger(), "Nothing to publish, octree is empty");
    return;
  }

  // For this example, we assume these publishers should publish.
  bool publish_free_marker_array_ = true;
  bool publish_marker_array = true;

  // Initialize global MarkerArrays. Each array holds one Marker per depth level.
  visualization_msgs::msg::MarkerArray free_nodes_vis;
  free_nodes_vis.markers.resize(tree_depth_ + 1);
  visualization_msgs::msg::MarkerArray occupied_nodes_vis;
  occupied_nodes_vis.markers.resize(tree_depth_ + 1);

  // --- Gather leaves within vicinity into a vector ---
  double min_x, min_y, min_z, max_x, max_y, max_z;
  double wdx, wdy, wdz;
  state local_A;
  dynus_ptr_->getCurrentWds(wdx, wdy, wdz);
  dynus_ptr_->getA(local_A);

  min_x = local_A.pos.x() - wdx / 2.0;
  min_y = local_A.pos.y() - wdy / 2.0;
  min_z = local_A.pos.z() - wdz / 2.0;
  max_x = local_A.pos.x() + wdx / 2.0;
  max_y = local_A.pos.y() + wdy / 2.0;
  max_z = local_A.pos.z() + wdz / 2.0;

  std::vector<OcTreeT::leaf_bbx_iterator> leaves;
  for (OcTreeT::leaf_bbx_iterator it = local_octree->begin_leafs_bbx(octomap::point3d(min_x, min_y, min_z), octomap::point3d(max_x, max_y, max_z)),
                                  end = local_octree->end_leafs_bbx();
       it != end; ++it)
  {
    leaves.push_back(it);
  }

  // --- Prepare thread-local storage for marker arrays ---
  // We'll allocate, for each thread, a pair of MarkerArrays (occupied and free), each with tree_depth_+1 markers.
  int num_threads = omp_get_max_threads();
  std::vector<std::vector<visualization_msgs::msg::MarkerArray>> thread_local_markers(num_threads, std::vector<visualization_msgs::msg::MarkerArray>(2));
  // Index 0: occupied markers, Index 1: free markers.
  for (int t = 0; t < num_threads; t++)
  {
    thread_local_markers[t][0].markers.resize(tree_depth_ + 1);
    thread_local_markers[t][1].markers.resize(tree_depth_ + 1);
  }

  // If using a height map for color, precompute metric bounds once.
  if (use_height_map_)
  {
    local_octree->getMetricMin(min_x, min_y, min_z);
    local_octree->getMetricMax(max_x, max_y, max_z);
  }

// --- Process all leaves in parallel ---
#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < leaves.size(); i++)
  {
    int tid = omp_get_thread_num();
    auto it = leaves[i];
    // For readability, define local references to the thread-local MarkerArrays:
    auto &local_occ_markers = thread_local_markers[tid][0].markers;
    auto &local_free_markers = thread_local_markers[tid][1].markers;

    if (local_octree->isNodeOccupied(*it))
    {
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      // Only process nodes within the z-range.
      if (z + half_size > par_.z_min && z - half_size < par_.z_max)
      {
        double x = it.getX();
        double y = it.getY();
        // Optionally filter out speckle nodes.
        if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey(), sensor_type))
        {
          continue;
        }
        if (publish_marker_array)
        {
          unsigned idx = it.getDepth();
          if (idx < local_occ_markers.size())
          {
            geometry_msgs::msg::Point cube_center;
            cube_center.x = x;
            cube_center.y = y;
            cube_center.z = z;
            local_occ_markers[idx].points.push_back(cube_center);
            if (use_height_map_)
            {
              double h = (1.0 - std::min(std::max((cube_center.z - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor_;
              local_occ_markers[idx].colors.push_back(heightMapColor(h));
            }
          }
        }
      }
    }
    else
    { // Node is free.
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > par_.z_min && z - half_size < par_.z_max)
      {
        if (publish_free_space_)
        {
          double x = it.getX();
          double y = it.getY();
          if (publish_free_marker_array_)
          {
            unsigned idx = it.getDepth();
            if (idx < local_free_markers.size())
            {
              geometry_msgs::msg::Point cube_center;
              cube_center.x = x;
              cube_center.y = y;
              cube_center.z = z;
              local_free_markers[idx].points.push_back(cube_center);
            }
          }
        }
      }
    }
  } // End parallel for

  // --- Merge thread-local marker arrays into the global MarkerArrays ---
  for (int t = 0; t < num_threads; t++)
  {
    // Merge occupied markers.
    for (size_t idx = 0; idx < tree_depth_ + 1; idx++)
    {
      auto &local_occ = thread_local_markers[t][0].markers[idx].points;
      auto &global_occ = occupied_nodes_vis.markers[idx].points;
      global_occ.insert(global_occ.end(), local_occ.begin(), local_occ.end());

      auto &local_colors = thread_local_markers[t][0].markers[idx].colors;
      auto &global_colors = occupied_nodes_vis.markers[idx].colors;
      global_colors.insert(global_colors.end(), local_colors.begin(), local_colors.end());
    }
    // Merge free markers.
    for (size_t idx = 0; idx < tree_depth_ + 1; idx++)
    {
      auto &local_free = thread_local_markers[t][1].markers[idx].points;
      auto &global_free = free_nodes_vis.markers[idx].points;
      global_free.insert(global_free.end(), local_free.begin(), local_free.end());
    }
  }

  // --- Finalize marker configuration and publish ---
  if (publish_marker_array)
  {
    for (size_t i = 0; i < occupied_nodes_vis.markers.size(); ++i)
    {
      double size = local_octree->getNodeSize(i);
      occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      occupied_nodes_vis.markers[i].header.stamp = rostime;
      occupied_nodes_vis.markers[i].ns = "occupied_nodes";
      occupied_nodes_vis.markers[i].id = i;
      occupied_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      occupied_nodes_vis.markers[i].scale.x = size;
      occupied_nodes_vis.markers[i].scale.y = size;
      occupied_nodes_vis.markers[i].scale.z = size;
      if (!use_colored_map_)
      {
        occupied_nodes_vis.markers[i].color = color_;
      }
      occupied_nodes_vis.markers[i].action =
          (occupied_nodes_vis.markers[i].points.size() > 0) ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    }

    if (sensor_type == "LiDAR")
      lidar_marker_pub_->publish(occupied_nodes_vis);
    else if (sensor_type == "Depth Camera")
      depth_camera_marker_pub_->publish(occupied_nodes_vis);
  }

  if (publish_free_marker_array_)
  {
    for (size_t i = 0; i < free_nodes_vis.markers.size(); ++i)
    {
      double size = local_octree->getNodeSize(i);
      free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      free_nodes_vis.markers[i].header.stamp = rostime;
      free_nodes_vis.markers[i].ns = "free_nodes";
      free_nodes_vis.markers[i].id = i;
      free_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      free_nodes_vis.markers[i].scale.x = size;
      free_nodes_vis.markers[i].scale.y = size;
      free_nodes_vis.markers[i].scale.z = size;
      free_nodes_vis.markers[i].color = color_free_;
      free_nodes_vis.markers[i].action =
          (free_nodes_vis.markers[i].points.size() > 0) ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    }
    if (sensor_type == "LiDAR")
      lidar_free_marker_pub_->publish(free_nodes_vis);
    else if (sensor_type == "Depth Camera")
      depth_camera_free_marker_pub_->publish(free_nodes_vis);
  }

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(get_logger(), "Map publishing in OctomapServer took %f sec", total_elapsed);
}

// ----------------------------------------------------------------------------

std_msgs::msg::ColorRGBA DYNUS_NODE::heightMapColor(double h)
{
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i{};
  double m{};
  double n{};
  double f{};

  i = floor(h);
  f = h - i;
  if (!(i & 1))
  {
    f = 1 - f; // if i is even
  }
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
  case 6:
  case 0:
    color.r = v;
    color.g = n;
    color.b = m;
    break;
  case 1:
    color.r = n;
    color.g = v;
    color.b = m;
    break;
  case 2:
    color.r = m;
    color.g = v;
    color.b = n;
    break;
  case 3:
    color.r = m;
    color.g = n;
    color.b = v;
    break;
  case 4:
    color.r = n;
    color.g = m;
    color.b = v;
    break;
  case 5:
    color.r = v;
    color.g = m;
    color.b = n;
    break;
  default:
    color.r = 1;
    color.g = 0.5;
    color.b = 0.5;
    break;
  }

  return color;
}

// ----------------------------------------------------------------------------

bool DYNUS_NODE::isSpeckleNode(const octomap::OcTreeKey &n_key, const std::string &sensor_type)
{

  std::shared_ptr<OcTreeT> local_octree;
  if (sensor_type == "LiDAR")
  {
    local_octree = std::shared_ptr<OcTreeT>(lidar_octree_);
  }
  else if (sensor_type == "Depth Camera")
  {
    local_octree = std::shared_ptr<OcTreeT>(depth_camera_octree_);
  }

  octomap::OcTreeKey key;
  bool neighbor_found = false;
  for (key[2] = n_key[2] - 1; !neighbor_found && key[2] <= n_key[2] + 1; ++key[2])
  {
    for (key[1] = n_key[1] - 1; !neighbor_found && key[1] <= n_key[1] + 1; ++key[1])
    {
      for (key[0] = n_key[0] - 1; !neighbor_found && key[0] <= n_key[0] + 1; ++key[0])
      {
        if (key != n_key)
        {
          octomap::TimedOcTreeNode *node = local_octree->search(key);
          if (node && local_octree->isNodeOccupied(node))
          {
            // we have a neighbor=> break!
            neighbor_found = true;
          }
        }
      }
    }
  }

  return neighbor_found;
}

// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  // Initialize multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // add node to executor
  auto node = std::make_shared<dynus::DYNUS_NODE>();
  executor.add_node(node);

  // spin
  executor.spin();

  rclcpp::shutdown();
  return 0;
}