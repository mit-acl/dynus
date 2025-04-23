/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include <dgp/dgp_planner.hpp>

using namespace termcolor;

DGPPlanner::DGPPlanner(std::string global_planner, bool verbose, double v_max, double a_max, double j_max, int dgp_timeout_duration_ms) : global_planner_(global_planner), planner_verbose_(verbose), v_max_(v_max), a_max_(a_max), j_max_(j_max), dgp_timeout_duration_ms_(dgp_timeout_duration_ms)
{
  if (planner_verbose_)
    printf(ANSI_COLOR_CYAN "JPS PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

void DGPPlanner::updateVmax(double v_max)
{
  v_max_ = v_max;
}

void DGPPlanner::setMapUtil(const std::shared_ptr<dynus::MapUtil<3>> &map_util)
{
  // Deep copy the map_util
  map_util_ = std::make_shared<dynus::MapUtil<3>>(*map_util);
  // map_util_ = map_util;
}

int DGPPlanner::status()
{
  return status_;
}

vec_Vecf<3> DGPPlanner::getPath()
{
  return path_;
}

vec_Vecf<3> DGPPlanner::getRawPath()
{
  return raw_path_;
}

vec_Vecf<3> DGPPlanner::removeCornerPts(const vec_Vecf<3> &path)
{
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  vec_Vecf<3> optimized_path;
  Vecf<3> pose1 = path[0];
  Vecf<3> pose2 = path[1];
  Vecf<3> prev_pose = pose1;
  optimized_path.push_back(pose1);
  decimal_t cost1, cost2, cost3;

  if (!map_util_->isBlocked(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++)
  {
    pose1 = path[i];
    pose2 = path[i + 1];
    if (!map_util_->isBlocked(pose1, pose2))
      cost2 = (pose1 - pose2).norm();
    else
      cost2 = std::numeric_limits<decimal_t>::infinity();

    if (!map_util_->isBlocked(prev_pose, pose2))
      cost3 = (prev_pose - pose2).norm();
    else
      cost3 = std::numeric_limits<decimal_t>::infinity();

    if (cost3 < cost1 + cost2)
      cost1 = cost3;
    else
    {
      optimized_path.push_back(path[i]);
      cost1 = (pose1 - pose2).norm();
      prev_pose = pose1;
    }
  }

  optimized_path.push_back(path.back());
  return optimized_path;
}

vec_Vecf<3> DGPPlanner::removeLinePts(const vec_Vecf<3> &path)
{
  if (path.size() < 3)
    return path;

  vec_Vecf<3> new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++)
  {
    Vecf<3> p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
    if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
      new_path.push_back(path[i]);
  }
  new_path.push_back(path.back());
  return new_path;
}

vec_Vecf<3> DGPPlanner::getOpenSet() const
{
  vec_Vecf<3> ps;
  const auto ss = graph_search_->getOpenSet();
  for (const auto &it : ss)
  {
    Veci<3> pn;
    pn << it->x, it->y, it->z;
    ps.push_back(map_util_->intToFloat(pn));
  }
  return ps;
}

vec_Vecf<3> DGPPlanner::getCloseSet() const
{
  vec_Vecf<3> ps;
  const auto ss = graph_search_->getCloseSet();
  for (const auto &it : ss)
  {
    Veci<3> pn;
    pn << it->x, it->y, it->z;
    ps.push_back(map_util_->intToFloat(pn));
  }
  return ps;
}

vec_Vecf<3> DGPPlanner::getAllSet() const
{
  vec_Vecf<3> ps;
  const auto ss = graph_search_->getAllSet();
  for (const auto &it : ss)
  {
    Veci<3> pn;
    pn << it->x, it->y, it->z;
    ps.push_back(map_util_->intToFloat(pn));
  }
  return ps;
}

bool DGPPlanner::plan(const Vecf<3> &start, const Vecf<3> &start_vel, const Vecf<3> &goal, double &final_g, double current_time, decimal_t eps)
{

  if (map_util_->map_.size() == 0)
  {
    std::cout << "map size: " << map_util_->map_.size() << std::endl;
    printf(ANSI_COLOR_RED "need to set the map!\n" ANSI_COLOR_RESET);
    return false;
  }

  if (planner_verbose_)
  {
    std::cout << "Start: " << start.transpose() << std::endl;
    std::cout << "Goal:  " << goal.transpose() << std::endl;
    std::cout << "Epsilon:  " << eps << std::endl;
  }

  path_.clear();
  raw_path_.clear();
  status_ = 0;

  const Veci<3> start_int = map_util_->floatToInt(start);
  // if (!map_util_->isFree(start_int))
  if (map_util_->isOccupied(start_int))
  {
    if (planner_verbose_)
    {
      if (map_util_->isOccupied(start_int))
        printf(ANSI_COLOR_RED "start is occupied!\n" ANSI_COLOR_RESET);
      else if (map_util_->isUnknown(start_int))
        printf(ANSI_COLOR_RED "start is unknown!\n" ANSI_COLOR_RESET);
      else
      {
        printf(ANSI_COLOR_RED "start is outside!\n" ANSI_COLOR_RESET);
        std::cout << "start: " << start.transpose() << std::endl;
        std::cout << "start_int: " << start_int.transpose() << std::endl;
        std::cout << "Map origin: " << map_util_->getOrigin().transpose() << std::endl;
        std::cout << "Map dim: " << map_util_->getDim().transpose() << std::endl;
      }
    }
    status_ = 1;
    std::cout << bold << red << "Start is not free" << reset << std::endl;
    return false;
  }

  const Veci<3> goal_int = map_util_->floatToInt(goal);
  // if (!map_util_->isFree(goal_int) && !map_util_->isUnknown(goal_int))
  if (map_util_->isOccupied(goal_int))
  {
    if (planner_verbose_)
    {
      printf(ANSI_COLOR_RED "goal_int: %d %d %d\n" ANSI_COLOR_RESET, goal_int(0), goal_int(1), goal_int(2));
      printf(ANSI_COLOR_RED "is outside in x: %d\n" ANSI_COLOR_RESET, map_util_->isOutsideXYZ(goal_int, 0));
      printf(ANSI_COLOR_RED "is outside in y: %d\n" ANSI_COLOR_RESET, map_util_->isOutsideXYZ(goal_int, 1));
      printf(ANSI_COLOR_RED "is outside in z: %d\n" ANSI_COLOR_RESET, map_util_->isOutsideXYZ(goal_int, 2));
      std::cout << "Map origin: " << map_util_->getOrigin().transpose() << std::endl;
    }
    status_ = 2;
    std::cout << bold << red << "goal is not free!" << reset << std::endl;
    return false;
  }

  if ((map_util_->map_).empty())
  {
    if (planner_verbose_)
      printf(ANSI_COLOR_RED "need to set the map!\n" ANSI_COLOR_RESET);
    return -1;
  }

  const Veci<3> dim = map_util_->getDim();

  // compute initial g value (this is due to the fact that the actual initial position is not on the grid)
  double initial_g = (start - map_util_->intToFloat(start_int)).norm();

  // should we initialize the planner in constructor?
  graph_search_ = std::make_shared<dynus::GraphSearch>((map_util_->map_).data(), map_util_, dim(0), dim(1), dim(2), eps, planner_verbose_, global_planner_);
  graph_search_->setStartAndGoal(start, goal);
  double max_values[3] = {v_max_, a_max_, j_max_};
  graph_search_->setBounds(max_values);

  // Run Initial guess module
  int max_expand = 10000;
  graph_search_->plan(start_int(0), start_int(1), start_int(2), goal_int(0), goal_int(1), goal_int(2), initial_g, global_planning_time_, dgp_static_jps_time_, dgp_check_path_time_, dgp_dynamic_astar_time_, dgp_recover_path_time_, current_time, start_vel, max_expand, dgp_timeout_duration_ms_);

  const auto path = graph_search_->getPath();

  if (path.size() < 1)
  {
    std::cout << ANSI_COLOR_RED "Cannot find a path from " << start.transpose() << " to " << goal.transpose() << " Abort!" ANSI_COLOR_RESET << std::endl;
    status_ = -1;
    return false;
  }

  // print the path
  // std::cout << "initial guess path found" << std::endl;
  // for (int i = 0; i < path.size(); i++)
  // {
  //   Vecf<3> p = map_util_->intToFloat(Veci<3>(path[i]->x, path[i]->y, path[i]->z));
  //   printf("path[%d]: %f %f %f and g: %f\n", i, p(0), p(1), p(2), path[i]->g);
  // }

  // get the final g value
  final_g = path.front()->g;

  //**** raw path, s --> g
  vec_Vecf<3> ps;
  for (const auto &it : path)
  {
    Veci<3> pn;
    pn << it->x, it->y, it->z;
    ps.push_back(map_util_->intToFloat(pn));
  }

  raw_path_ = ps;
  std::reverse(std::begin(raw_path_), std::end(raw_path_));

  // // Simplify the raw path
  // path_ = removeLinePts(raw_path_);

  // // If you activate removeCornerPts, the path will be too simplified for dynamic A*
  // // TODO: only apply this for the static JPS's path? not for the dynamic A*'s path?
  // path_ = removeCornerPts(path_);
  // std::reverse(std::begin(path_), std::end(path_));
  // path_ = removeCornerPts(path_);
  // std::reverse(std::begin(path_), std::end(path_));

  path_ = raw_path_;

  return true;
}

void DGPPlanner::cleanUpPath(vec_Vecf<3>& path)
{

  // Simplify the raw path
  path = removeLinePts(path);

  // If you activate removeCornerPts, the path will be too simplified for dynamic A*
  // TODO: only apply this for the static JPS's path? not for the dynamic A*'s path?
  path = removeCornerPts(path);
  std::reverse(std::begin(path), std::end(path));
  path = removeCornerPts(path);
  std::reverse(std::begin(path), std::end(path));

}

double DGPPlanner::getInitialGuessPlanningTime()
{
  return global_planning_time_;
}

double DGPPlanner::getStaticJPSPlanningTime()
{
  return dgp_static_jps_time_;
}

double DGPPlanner::getCheckPathTime()
{
  return dgp_check_path_time_;
}

double DGPPlanner::getDynamicAstarTime()
{
  return dgp_dynamic_astar_time_;
}

double DGPPlanner::getRecoverPathTime()
{
  return dgp_recover_path_time_;
}
