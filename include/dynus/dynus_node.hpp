/* ----------------------------------------------------------------------------
 * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "rclcpp/rclcpp.hpp"
#include "dynus/dynus_type.hpp"
#include "dynus/dynus.hpp"
#include "dgp/utils.hpp"
#include <dynus_interfaces/msg/dyn_traj.hpp>
#include <dynus_interfaces/msg/dyn_traj_array.hpp>
#include "dynus_interfaces/msg/state.hpp"
#include "dynus_interfaces/msg/goal.hpp"
#include "dynus_interfaces/msg/yaw_output.hpp"
#include "dynus_interfaces/msg/pn_adaptation.hpp"
#include <dynus/utils.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <fstream>                                         // for flie operations
#include "ament_index_cpp/get_package_share_directory.hpp" // for getting the package path
#include <vector>
#include <unordered_map>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeKey.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations" // pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0 // NOLINT
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/srv/bounding_box_query.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <memory>
#include <string>
#include <dgp/TimedOcTree.h>
#include <dgp/TimedOcTreeNode.h>
#include <execution>

// prefix
using namespace std::chrono_literals;

// Define the synchronization policy
typedef message_filters::sync_policies::ApproximateTime<octomap_msgs::msg::Octomap, octomap_msgs::msg::Octomap> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> Sync;

// ROS2 component (run-time composition)
// common practice is to use namespace for components
namespace dynus
{

    using PCLPoint = pcl::PointXYZ;
    using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
    // using OcTreeT = octomap::OcTree;
    using OcTreeT = octomap::TimedOcTree;
    // #endif
    using OctomapSrv = octomap_msgs::srv::GetOctomap;
    using BBoxSrv = octomap_msgs::srv::BoundingBoxQuery;
    using ResetSrv = std_srvs::srv::Empty;

    class DYNUS_NODE : public rclcpp::Node
    {

    public:
        DYNUS_NODE();  // Constructor
        ~DYNUS_NODE(); // Destructor

    private:
        // Callbacks
        void replanCallback(); // Replan the path
        void trajCallback(const dynus_interfaces::msg::DynTraj::SharedPtr msg);
        void stateCallback(const dynus_interfaces::msg::State::SharedPtr msg);
        void startExplorationCallback(const std_msgs::msg::Empty::SharedPtr msg);
        void terminalGoalCallback(const geometry_msgs::msg::PoseStamped &msg);
        void updateTMapCallback();
        void octomapCallbackForROI(const octomap_msgs::msg::Octomap::SharedPtr msg);
        void roiTrajCallback(const dynus_interfaces::msg::DynTrajArray::SharedPtr msg);
        void findFrontiersCallback();
        void shareROIOctomapAndTrajsCallback();
        void goalReachedCheckCallback();
        void convertDynTrajMsg2DynTraj(const dynus_interfaces::msg::DynTraj &msg, dynTraj &traj, double current_time);
        void cleanUpOldTrajsCallback();
        void updateTrajsForTMapCallback();
        void successfulDetectionCallback(const std_msgs::msg::String::SharedPtr msg);
        void checkFuturePlanSafetyCallback();
        void getInitialPoseHwCallback();

        // Others
        void declareParameters();                                                                       // Declare the parameters
        void setParameters();                                                                           // Set the parameters
        void printParameters();                                                                         // Print the parameters
        void getPointCloud2FromVec_Vecf3(const vec_Vecf<3> &vec, sensor_msgs::msg::PointCloud2 &cloud); // Get PointCloud2 from Vec_Vecf3
        void createMarkerArrayFromVec_Vec3f(const vec_Vec3f &occupied_cells, const std_msgs::msg::ColorRGBA &color, int namespace_id, double scale, visualization_msgs::msg::MarkerArray *marker_array);
        void clearMarkerArray(visualization_msgs::msg::MarkerArray &path_marker, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher); // Clear MarkerArray
        void clearMarkerActualTraj();                                                                                                                           // Clear the actual trajectory marker
        void runSim();                                                                                                                                          // Run the simulation
        void printComputationTime(bool result);                                                                                                                 // Print computation time
        void recordData(bool result);                                                                                                                           // Record data for benchmarking
        void logData();                                                                                                                                         // Log data for benchmarking
        void createFrontiersMarker(const vec_Vecf<3> &frontiers, std_msgs::msg::ColorRGBA color, int start_id, double scale = 0.5);
        void setComputationTimesToZero();
        void constructFOVMarker();
        void retrieveData();
        void retrieveDataForVisualizationAndTopics();

        // Functions to publish
        void publishStaticMap();  // Publish the map
        void publishDynamicMap(); // Publish the dynamic map
        void publishGlobalPath();
        void publishFreeGlobalPath();
        void publishPoly();
        void publishTraj();
        void publishOwnTraj();
        void publishActualTraj();
        void publishGoal();         // Publish the goal (trajectory points)
        void publishPointG() const; // Publish the point G (projected terminal goal)
        void publishPointE() const; // Publish the point E (Local trajectory's goal)
        void publishPointA() const; // Publish the point A (starting point)
        void publishCurrentState(const state &state) const;
        void publishFrontiers();
        void publishState(const state &data, const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &publisher) const;
        void publishROIOctomapAndTrajs(const std::unordered_map<int, std::shared_ptr<octomap::TimedOcTree>> &roi_octree, std::unordered_map<int, std::vector<dynTraj>> &roi_trajs);
        void publishYawSequenceAndBSpline();
        void publishPandN();
        void publishFOV();
        void publisCps();
        void publishStaticPushPoints();
        void publishPPoints();
        void publishLocalGlobalPath();
        void publishVelocityInText(const Eigen::Vector3d &position, double velocity);

        // Octomap functions
        void insertLidarCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
        void insertDepthCameraCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
        void processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,  const std::string& sensor_type);
        void insertScan( const tf2::Vector3 &sensor_origin, const pcl::PointCloud<pcl::PointXYZ> &ground,
            const pcl::PointCloud<pcl::PointXYZ> &nonground, const std::string& sensor_type);
        void filterGroundPlane(const PCLPointCloud &pc, PCLPointCloud &ground,
            PCLPointCloud &nonground) const;
        void publishAll(const rclcpp::Time &rostime, const std::string& sensor_type);
        /**
         * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
         * @param key
         * @return
         */
        bool isSpeckleNode(const octomap::OcTreeKey &key, const std::string& sensor_type);

        /// Test if key is within update area of map (2D, ignores height)
        inline bool isInUpdateBBX(const OcTreeT::iterator & it) const
        {
            // 2^(tree_depth-depth) voxels wide:
            unsigned voxelWidth = (1 << (max_tree_depth_ - it.getDepth()));
            octomap::OcTreeKey key = it.getIndexKey();  // lower corner of voxel
            return key[0] + voxelWidth >= update_bbox_min_[0] &&
                key[1] + voxelWidth >= update_bbox_min_[1] &&
                key[0] <= update_bbox_max_[0] &&
                key[1] <= update_bbox_max_[1];
        }

        static std_msgs::msg::ColorRGBA heightMapColor(double h);

        // Timers for callback
        rclcpp::TimerBase::SharedPtr timer_visualize_static_map_;
        rclcpp::TimerBase::SharedPtr timer_visualize_dynamic_map_;
        rclcpp::TimerBase::SharedPtr timer_replanning_;
        rclcpp::TimerBase::SharedPtr timer_goal_;
        rclcpp::TimerBase::SharedPtr timer_frontiers_;
        rclcpp::TimerBase::SharedPtr timer_share_roi_map_and_trajs_;
        rclcpp::TimerBase::SharedPtr timer_update_tmap_;
        rclcpp::TimerBase::SharedPtr timer_goal_reached_check_;
        rclcpp::TimerBase::SharedPtr timer_cleanup_old_trajs_;
        rclcpp::TimerBase::SharedPtr timer_trajs_update_for_tmap_;
        rclcpp::TimerBase::SharedPtr timer_check_plan_safety_;
        rclcpp::TimerBase::SharedPtr timer_initial_pose_;

        // Callback groups
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_1_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_2_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_3_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_4_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_5_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_6_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_7_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_8_;
        rclcpp::CallbackGroup::SharedPtr cb_group_mu_9_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_1_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_2_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_3_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_4_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_5_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_6_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_7_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_8_;
        rclcpp::CallbackGroup::SharedPtr cb_group_re_9_;
        rclcpp::CallbackGroup::SharedPtr cb_group_lidar_pointcloud_;
        rclcpp::CallbackGroup::SharedPtr cb_group_depth_camera_pointcloud_;
        rclcpp::CallbackGroup::SharedPtr cb_group_replan_;
        rclcpp::CallbackGroup::SharedPtr cb_group_goal_;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dgp_path_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_free_dgp_path_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_local_global_path_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_local_global_path_after_push_marker_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_map_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_free_map_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unknown_map_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dynamic_map_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_free_map_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_unknown_map_marker_;
        rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_whole_;
        rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr pub_poly_safe_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_traj_committed_colored_;
        rclcpp::Publisher<dynus_interfaces::msg::DynTraj>::SharedPtr pub_own_traj_;
        rclcpp::Publisher<dynus_interfaces::msg::Goal>::SharedPtr pub_goal_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_E_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_G_term_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_point_A_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_current_state_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frontiers_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_goal_reached_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_setpoint_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_actual_traj_;
        rclcpp::Publisher<dynus_interfaces::msg::YawOutput>::SharedPtr pub_yaw_output_;
        rclcpp::Publisher<dynus_interfaces::msg::PNAdaptation>::SharedPtr pub_pn_adaptation_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_fov_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cp_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_static_push_points_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_p_points_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vel_text_;

        // Octomap-related publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr depth_camera_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lidar_free_marker_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr depth_camera_free_marker_pub_;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_map_pub_;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_map_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

        // Unordered map for publishers
        std::unordered_map<int, rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr> pub_roi_octomap_;
        std::unordered_map<int, rclcpp::Publisher<dynus_interfaces::msg::DynTrajArray>::SharedPtr> pub_roi_trajs_;

        // Subscribers
        rclcpp::Subscription<dynus_interfaces::msg::DynTraj>::SharedPtr sub_traj_;
        rclcpp::Subscription<dynus_interfaces::msg::DynTraj>::SharedPtr sub_predicted_traj_;
        rclcpp::Subscription<dynus_interfaces::msg::DynTrajArray>::SharedPtr sub_roi_traj_;
        rclcpp::Subscription<dynus_interfaces::msg::State>::SharedPtr sub_state_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_start_exploration_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_could_map_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_terminal_goal_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_lidar_octomap_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_depth_camera_octomap_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_roi_octomap_;
        rclcpp::Subscription<dynus_interfaces::msg::DynTrajArray>::SharedPtr sub_roi_trajs_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_successful_detection_;

        // Octomap subscribers
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> lidar_tf_point_cloud_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> depth_camera_tf_point_cloud_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_point_cloud_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> depth_camera_point_cloud_sub_;

        // Time synchronizer
        std::shared_ptr<Sync> octomap_sync_;

        // Visualization
        visualization_msgs::msg::MarkerArray dgp_path_marker_;
        visualization_msgs::msg::MarkerArray dgp_free_path_marker_;
        visualization_msgs::msg::MarkerArray dgp_local_global_path_marker_;
        visualization_msgs::msg::MarkerArray dgp_local_global_path_after_push_marker_;
        visualization_msgs::msg::MarkerArray traj_committed_colored_;
        visualization_msgs::msg::MarkerArray frontiers_marker_;
        visualization_msgs::msg::Marker marker_fov_;

        // Mutex
        std::mutex cloud_callback_mutex_; // Mutex for cloud callback
        std::mutex octree_mutex_; // Mutex for octree

        // Parameters
        int id_;
        std::string ns_;
        std::string id_str_;
        parameters par_;
        double final_g_ = 0.0;                  // only for debugging
        bool verbose_computation_time_ = false; // only for debugging
        int marker_fov_id_ = 0;

        // DYNUS pointer
        std::shared_ptr<DYNUS> dynus_ptr_;

        // Global Path Benchmarking
        std::string file_path_;        // only for benchmarking
        std::vector<std::tuple<bool,   // Result
                               double, // Cost
                               double, // Total replanning time
                               double, // Global planning time
                               double, // CVX decomposition time
                               double, // Gurobi time
                               double, // Safe paths time
                               double, // Safety Check time
                               double, // Yaw sequence time
                               double, // Yaw fitting time
                               double, // Static JPS time in DGP
                               double, // Check path time in DGP
                               double, // Dynamic A* time in DGP
                               double  // Recover path time in DGP
                               >>
            global_path_benchmark_; // only for benchmarking

        // debug
        double octree_last_time_received_ = 0.0;
        double replan_last_time_called_ = 0.0;

        // Record computation time
        double global_planning_time_ = 0.0;
        double dgp_static_jps_time_ = 0.0;
        double dgp_check_path_time_ = 0.0;
        double dgp_dynamic_astar_time_ = 0.0;
        double dgp_recover_path_time_ = 0.0;
        double cvx_decomp_time_ = 0.0;
        double gurobi_computation_time_ = 0.0;
        double safe_paths_time_ = 0.0;
        double safety_check_time_ = 0.0;
        double yaw_sequence_time_ = 0.0;
        double yaw_fitting_time_ = 0.0;
        double replanning_computation_time_ = 0.0;
        double current_time_for_debug_ = 0.0;

        // Visualization
        vec_E<Polyhedron<3>> poly_whole_;
        vec_E<Polyhedron<3>> poly_safe_;
        std::vector<state> goal_setpoints_;
        int actual_traj_id_ = 0;

        // Yaw Optimization Debugging
        std::vector<double> optimal_yaw_sequence_;
        std::vector<double> yaw_control_points_;
        std::vector<double> yaw_knots_;

        // Local trajectory debugging
        std::vector<Eigen::Matrix<double, 3, 4>> cps_;

        // Static push points and p points
        vec_Vecf<3> static_push_points_;
        vec_Vecf<3> p_points_;
        int static_push_points_id_ = 0;
        int p_points_id_ = 0;

        // Trajectory sharing
        PieceWisePol pwp_to_share_; // Piecewise polynomial

        // Flags
        bool state_initialized_ = false;             // State initialized
        bool map_initialized_ = false;               // Map initialized
        bool replan_timer_started_ = false;          // Replan timer started
        bool lidar_octomap_received_ = false;        // Mid360 octomap received
        bool depth_camera_octomap_received_ = false; // D435 octomap received
        bool publish_actual_traj_called_ = false;    // Publish actual trajectory called
        bool use_benchmark_ = false;                 // Use benchmark
        bool terminal_goal_set_ = false;             // Terminal goal set
        bool octomap_initialized_ = false;           // Octomap initialized

        // Actual trajectory variables for ground robots
        Eigen::Vector3d actual_traj_prev_pos_;
        double actual_traj_prev_time_;

        // Frontiers
        Eigen::Vector3d best_frontier_;

        // D435 parameters
        std::string d435_depth_frame_id_;
        std::string lidar_frame_id_;
        std::string d435_camera_info_topic_;

        // TF2 buffer and listener
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        // Octomap update time stamp
        rclcpp::Time latest_d435_octomap_time_stamp_;
        rclcpp::Time latest_mid360_octomap_time_stamp_;
        
        // initial pose (for hardware)
        bool initial_pose_received_ = false;
        std::string initial_pose_topic_;
        geometry_msgs::msg::TransformStamped init_pose_transform_stamped_;

        // Timer to make sure we don't sample point cloud too often
        rclcpp::Time last_lidar_callback_time_;
        rclcpp::Time last_depth_camera_callback_time_;

        // Octomap mutex-related variables
        std::shared_ptr<OcTreeT> lidar_octree_;
        std::shared_ptr<OcTreeT> depth_camera_octree_;

        // Octomap parameters
        octomap::OcTreeKey update_bbox_min_;
        octomap::OcTreeKey update_bbox_max_;
        std::string world_frame_id_ = "map"; // the map frame
        bool use_height_map_ = false;
        bool use_colored_map_ = true;
        double color_factor_;
        double point_cloud_min_x_;
        double point_cloud_max_x_;
        double point_cloud_min_y_;
        double point_cloud_max_y_;
        double point_cloud_min_z_;
        double point_cloud_max_z_;
        bool filter_speckles_;
        bool filter_ground_plane_;
        double ground_filter_distance_;
        double ground_filter_angle_;
        double ground_filter_plane_distance_;
        bool use_decay_;
        double decay_duration_;
        bool latched_topics_ = false;
        bool publish_free_space_;
        size_t tree_depth_;
        size_t max_tree_depth_;
        bool compress_map_;
        std_msgs::msg::ColorRGBA color_;
        std_msgs::msg::ColorRGBA color_free_;
        double sensor_model_occupancy_thres_;
        double sensor_model_hit_;
        double sensor_model_miss_;
        double sensor_model_min_;
        double sensor_model_max_;
        double lidar_insert_hz_;
        double depth_camera_insert_hz_;

    protected:
        inline static void updateMinKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &min)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                min[i] = std::min(in[i], min[i]);
            }
        }

        inline static void updateMaxKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &max)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                max[i] = std::max(in[i], max[i]);
            }
        }
    };

} // namespace dynus