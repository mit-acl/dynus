/**
 * @file dgp_planner.h
 * @brief DGPPlanner
 */
#ifndef DGP_PLANNER_BASE_H
#define DGP_PLANNER_BASE_H

#include <dgp/data_type.hpp>
#include <dgp/graph_search.hpp>
#include <dgp/map_util.hpp>

/**
 * @brief Abstract base for planning
 */
class DGPPlanner
{
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  DGPPlanner(std::string global_planner, bool verbose, double v_max, double a_max, double j_max, int dgp_timeout_duration_ms);

  // Set map util for collistion checking
  void setMapUtil(const std::shared_ptr<dynus::MapUtil<3>> &map_util);
  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  int status();
  // Get the modified path
  vec_Vecf<3> getPath();
  // Get the raw path
  vec_Vecf<3> getRawPath();
  // Set timeout duration
  void setTimeOutDurationMs(int timeout_duration_ms);
  // remove redundant points on the same line
  vec_Vecf<3> removeLinePts(const vec_Vecf<3> &path);
  // Remove some corner waypoints
  vec_Vecf<3> removeCornerPts(const vec_Vecf<3> &path);
  // Clean up path
  void cleanUpPath(vec_Vecf<3>& path);
  // Planning function
  bool plan(const Vecf<3> &start, const Vecf<3> &start_vel, const Vecf<3> &goal, double &final_g, double current_time, decimal_t eps = 1);
  // Get the nodes in open set
  vec_Vecf<3> getOpenSet() const;
  // Get the nodes in close set
  vec_Vecf<3> getCloseSet() const;
  // Get all the nodes
  vec_Vecf<3> getAllSet() const;
  // Get computation time
  double getInitialGuessPlanningTime();
  double getStaticJPSPlanningTime();
  double getCheckPathTime();
  double getDynamicAstarTime();
  double getRecoverPathTime();
  void updateVmax(double v_max);

protected:
  // Assume using 3D voxel map for all 2d and 3d planning
  std::shared_ptr<dynus::VoxelMapUtil> map_util_;
  // The planner
  std::shared_ptr<dynus::GraphSearch> graph_search_;
  // Raw path from planner
  vec_Vecf<3> raw_path_;
  // Modified path for future usage
  vec_Vecf<3> path_;
  // Flag indicating the success of planning
  int status_ = 0;
  // Enabled for printing info
  bool planner_verbose_;

  // Parameters
  std::string global_planner_;

  // For benchmarking time recording
  double global_planning_time_ = 0.0;
  double dgp_static_jps_time_ = 0.0;
  double dgp_check_path_time_ = 0.0;
  double dgp_dynamic_astar_time_ = 0.0;
  double dgp_recover_path_time_ = 0.0;

  // time out duration
  int dgp_timeout_duration_ms_ = 1000;

  // max values
  double v_max_;
  double a_max_;
  double j_max_;
};

#endif
