/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

// ROS2
#include <rclcpp/rclcpp.hpp>

// Convex Decomposition includes
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_util/seed_decomp.h>

// Map includes
#include <dgp/read_map.hpp>

// DGP includes
#include <dgp/data_utils.hpp>
#include <dgp/dgp_planner.hpp>
#include <dgp/utils.hpp>

// Other includes
#include <Eigen/Dense>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "timer.hpp"
#include <dgp/TimedOcTree.h>
#include <dgp/TimedOcTreeNode.h>

// prefix
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class DGPManager
{
public:
    // Constructor
    DGPManager();

    // Set parameters
    void setParameters(const parameters &par);

    // Update map resolution
    void updateMapRes(double res);

    // DGP initialization methods
    void setupDGPPlanner(const std::string &global_planner, bool global_planner_verbose, double res, double v_max, double a_max, double j_max, int timeout_duration_ms);

    // Map updates
    void getOccupiedCells(vec_Vecf<3> &occupied_cells);
    void getOccupiedCellsForCvxDecomp(vec_Vecf<3> &occupied_cells, const vec_Vecf<3> &path, bool use_for_safe_path);
    void getDynamicOccupiedCellsForCvxDecompTemporal(vec_Vecf<3> &occupied_cells, const std::vector<double> &current_times, std::vector<double> times_elapsed_from_plan_start, const vec_Vecf<3> &path, bool use_for_safe_path);
    void getDynamicOccupiedCellsForVis(vec_Vecf<3> &occupied_cells, vec_Vecf<3> &free_cells, vec_Vecf<3> &unknown_cells, double current_time);
    void updateMapCallback(double wdx, double wdy, double wdz, const Vec3f &center_map, const Vec3f &start, const Vec3f &goal, double octmap_received_time, const std::shared_ptr<octomap::TimedOcTree>& lidar_octree, const std::shared_ptr<octomap::TimedOcTree>& depth_camera_octree);
    void setOctomap(const std::shared_ptr<octomap::TimedOcTree> &octree, std::string octomap_name);

    // Free start and goal
    void freeStart(Vec3f &start_sent, double factor);
    void freeGoal(Vec3f &goal_sent, double factor);

    // Check if the point is occupied
    bool checkIfPointOccupied(const Vec3f &point);

    // DGP planning methods
    bool solveDGP(const Vec3f &start_sent, const Vec3f &start_vel, const Vec3f &goal_sent, double &final_g, double weight, double current_time, vec_Vecf<3> &path);
    // Check if the path is within free space
    bool checkIfPathInFree(const vec_Vecf<3> &path, vec_Vecf<3> &free_path);
    // Get computation time
    void getComputationTime(double &global_planning_time, double &dgp_static_jps_time, double &dgp_check_path_time, double &dgp_dynamic_astar_time, double &dgp_recover_path_time);

    // Convex decomposition
    bool cvxEllipsoidDecomp(const state &A, const vec_Vecf<3> &path, std::vector<LinearConstraint3D> &l_constraints, vec_E<Polyhedron<3>> &poly_out, bool use_for_safe_path);                                                                         // only including static obstacles
    bool cvxEllipsoidDecompTemporal(const state &A, const vec_Vecf<3> &path, std::vector<LinearConstraint3D> &l_constraints, vec_E<Polyhedron<3>> &poly_out, double current_time, const std::vector<double> &travel_times, bool use_for_safe_path); // including dynamic obstacles

    // Update trajectories
    void updateTrajs(const std::vector<dynTraj> &trajs);

    // Check if the point is free
    inline bool checkIfPointFree(const Vec3f &point) const;

    // Update map util
    void updateReadMapUtil();

    // Find the closest free point
    void pushPathIntoFreeSpace(const vec_Vecf<3> &path, vec_Vecf<3> &free_path);
    void findClosestFreePoint(const Vec3f &point, Vec3f &closest_free_point);

    // Push vectors for static obstacles
    bool computeStaticPushPoints(const vec_Vecf<3> &path, double discretization_dist, Vecf<3> &mean_point, int num_lookahead_global_path_for_push);

    // count unknown cells
    int countUnknownCells() const;
    int getTotalNumCells() const;

    // Update maximum velocity
    void updateVmax(double v_max);

    // Update max_dist_vertexes
    void updateMaxDistVertexes(double max_dist_vertexes);

    // Clean up the path
    void cleanUpPath(vec_Vecf<3> &path);

    bool isMapInitialized() const;

    // Get P point for static push
    void getPpoints(const Vec3f &global_path_point, const Vec3f &static_push_point, Vec3f &p_point);

    // check if the point has an occupied neighbour
    bool checkIfPointHasNonFreeNeighbour(const Vec3f& point) const;

    // Shared pointers
    std::shared_ptr<dynus::VoxelMapUtil> write_map_util_;
    std::shared_ptr<dynus::VoxelMapUtil> read_map_util_;
    std::shared_ptr<dynus::VoxelMapUtil> map_util_for_planning_;
    std::unique_ptr<DGPPlanner> planner_ptr_;

private:
    // Mutex
    std::mutex mtx_write_map_util_;
    std::mutex mtx_read_map_util_;

    // Convex decomposition
    EllipsoidDecomp3D ellip_decomp_util_;
    std::vector<float> local_box_size_;

    // Parameters
    parameters par_;
    double weight_;
    double res_, drone_radius_;
    Vec3f center_map_; // only for debugging
    double v_max_, a_max_, j_max_;
    Eigen::Vector3d v_max_3d_, a_max_3d_, j_max_3d_;
    double max_dist_vertexes_ = 2.0;
    bool use_raw_path_ = false;
    bool use_shrinked_box_ = false;
    double shrinked_box_size_ = 0.0;

    // Constants
    const Eigen::Vector3d unitX_ = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d unitY_ = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d unitZ_ = Eigen::Vector3d::UnitZ();

    // Flats
    bool map_initialized_ = false;
};
