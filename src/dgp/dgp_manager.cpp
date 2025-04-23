/* ----------------------------------------------------------------------------
 * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
 * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
 * All Rights Reserved
 * Authors: XXXXX XXXXX, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#include "dgp/dgp_manager.hpp"

/// The type of map data Tmap is defined as a 1D array
using Tmap = std::vector<char>;
using namespace dynus;
using namespace termcolor;

typedef timer::Timer MyTimer;

DGPManager::DGPManager() {}

void DGPManager::setParameters(const parameters &par)
{
    // Get the parameter
    par_ = par;

    // Set the parameters
    res_ = par.res;
    use_raw_path_ = par.use_raw_path;
    drone_radius_ = par.drone_radius;
    max_dist_vertexes_ = par.max_dist_vertexes;
    use_shrinked_box_ = par.use_shrinked_box;
    shrinked_box_size_ = par.shrinked_box_size;
    // local_box_size_ is std::vector<float> but par.local_box_size is std::vector<double>
    local_box_size_ = {static_cast<float>(par.local_box_size[0]), static_cast<float>(par.local_box_size[1]), static_cast<float>(par.local_box_size[2])};

    // shared pointer to the map util for actual planning
    write_map_util_ = std::make_shared<dynus::VoxelMapUtil>(par.factor_dgp * par.res, par.x_min, par.x_max, par.y_min, par.y_max, par.z_min, par.z_max, par.inflation_dgp, par.free_inflation_dgp, par.octomap_res, par.alpha_cov, par_.dynamic_obstacle_base_inflation, par.max_dynamic_obstacle_inflation, par.map_buffer, par.use_lidar, par.use_depth_camera, par.use_free_space, par.use_z_axis_bottom_inflation, par.node_size_factor_for_occupied, par.node_size_factor_for_free);
}

void DGPManager::updateMapRes(double res)
{
    // Update the resolution
    res_ = res;
    write_map_util_->setResolution(res);
}

void DGPManager::updateMaxDistVertexes(double max_dist_vertexes)
{
    max_dist_vertexes_ = max_dist_vertexes;
}

void DGPManager::updateReadMapUtil()
{
    mtx_write_map_util_.lock();
    mtx_read_map_util_.lock();
    read_map_util_ = std::make_shared<dynus::VoxelMapUtil>(*write_map_util_);
    mtx_read_map_util_.unlock();
    mtx_write_map_util_.unlock();
}

void DGPManager::cleanUpPath(vec_Vecf<3>& path)
{
    planner_ptr_->cleanUpPath(path);
}

void DGPManager::setupDGPPlanner(const std::string &global_planner, bool global_planner_verbose, double res, double v_max, double a_max, double j_max, int dgp_timeout_duration_ms)
{

    // Get the parameters
    v_max_ = v_max;
    v_max_3d_ = Eigen::Vector3d(v_max, v_max, v_max);
    a_max_ = a_max;
    a_max_3d_ = Eigen::Vector3d(a_max, a_max, a_max);
    j_max_ = j_max;
    j_max_3d_ = Eigen::Vector3d(j_max, j_max, j_max);

    // Create the DGP planner
    planner_ptr_ = std::unique_ptr<DGPPlanner>(new DGPPlanner(global_planner, global_planner_verbose, v_max, a_max, j_max, dgp_timeout_duration_ms));

    // Create the map_util_for_planning
    // This is the beginning of the planning, so we fetch the map_util_ and don't update it for the entire planning process (updating while planning makes the planner slower)
    mtx_read_map_util_.lock();
    map_util_for_planning_ = std::make_shared<dynus::VoxelMapUtil>(*read_map_util_);
    mtx_read_map_util_.unlock();
}

void DGPManager::updateVmax(double v_max)
{
    // Update the maximum velocity
    v_max_ = v_max;
    v_max_3d_ = Eigen::Vector3d(v_max, v_max, v_max);
    planner_ptr_->updateVmax(v_max);
}

void DGPManager::updateTrajs(const std::vector<dynTraj> &trajs)
{
    mtx_write_map_util_.lock();
    write_map_util_->updateTrajs(trajs);
    mtx_write_map_util_.unlock();

    // Update the read_map_util_ with the write_map_util_
    updateReadMapUtil();
}

void DGPManager::freeStart(Vec3f &start, double factor)
{
    // Set start free
    Veci<3> start_int = map_util_for_planning_->floatToInt(start);
    map_util_for_planning_->setFreeVoxelAndSurroundings(start_int, factor * res_);
    // map_util_for_planning_->setFree(start_int);
}

void DGPManager::freeGoal(Vec3f &goal, double factor)
{
    // Set goal free
    Veci<3> goal_int = map_util_for_planning_->floatToInt(goal);
    map_util_for_planning_->setFreeVoxelAndSurroundings(goal_int, factor * res_);
    // map_util_for_planning_->setFree(goal_int);
}

bool DGPManager::checkIfPointOccupied(const Vec3f &point)
{
    // Check if the point is free
    Veci<3> point_int = map_util_for_planning_->floatToInt(point);

    return map_util_for_planning_->isOccupied(point_int);
}

bool DGPManager::solveDGP(const Vec3f &start_sent, const Vec3f &start_vel, const Vec3f &goal_sent, double &final_g, double weight, double current_time, vec_Vecf<3> &path)
{
    // Set start and goal
    Eigen::Vector3d start(start_sent(0), start_sent(1), start_sent(2));
    Eigen::Vector3d goal(goal_sent(0), goal_sent(1), goal_sent(2));

    //
    // Run DGP
    //

    // Set collision checking function
    planner_ptr_->setMapUtil(map_util_for_planning_);

    // DGP Plan
    bool result = false;

    // Attempt to plan
    result = planner_ptr_->plan(start, start_vel, goal, final_g, current_time, weight);

    // If there is a solution
    if (result)
    {
        // Process path
        if (use_raw_path_)
        {
            path = planner_ptr_->getRawPath(); // Get raw path
        }
        else
        {
            path = planner_ptr_->getPath();
        }

        // Add more vertices if necessary
        dynus_utils::createMoreVertexes(path, max_dist_vertexes_);

        if (path.size() > 1)
        {
            path[0] = start;
            path[path.size() - 1] = goal; // Ensure path starts and ends at the correct points
        }
        else
        { // Handle the case where start and goal are in the same voxel
            vec_Vecf<3> tmp;
            tmp.push_back(start);
            tmp.push_back(goal);
            path = tmp;
        }
    }
    else
    {
        // std::cout << bold << red << "DGP didn't find a solution from " << start.transpose() << " to " << goal.transpose() << reset << std::endl;
    }

    // Clean up path
    // planner_ptr_->cleanUpPath(path);

    return result;
}

bool DGPManager::checkIfPathInFree(const vec_Vecf<3> &path, vec_Vecf<3> &free_path)
{
    // Initialize result
    free_path.clear();
    free_path.push_back(path[0]);

    // Plan only in free space if required
    for (size_t i = 1; i < path.size(); i++)
    {
        Veci<3> path_int = map_util_for_planning_->floatToInt(path[i]);
        if (map_util_for_planning_->isFree(path_int))
        {
            free_path.push_back(path[i]);
        }
        else
        {
            break;
        }
    }

    if (free_path.size() <= 1)
        return false;

    return true;
}

void DGPManager::pushPathIntoFreeSpace(const vec_Vecf<3> &path, vec_Vecf<3> &free_path)
{

    // Initialize result
    free_path.clear();
    free_path.push_back(path[0]);

    // Plan only in free space if required
    for (size_t i = 1; i < path.size(); i++)
    {
        Vec3f free_point;
        map_util_for_planning_->findClosestFreePoint(path[i], free_point);
        free_path.push_back(free_point);
    }
}

inline bool DGPManager::checkIfPointFree(const Vec3f &point) const
{
    // Check if the point is free
    Veci<3> point_int = map_util_for_planning_->floatToInt(point);
    return map_util_for_planning_->isFree(point_int);
}

void DGPManager::getPpoints(const Vec3f& global_path_point, const Vec3f& static_push_point, Vec3f& p_point)
{

    // starting from static_push_point, we increase the distance by some amount until it detects a free point
    double push_detection_dist = 0.1;
    double total_dist = (global_path_point - static_push_point).norm();
    Vec3f direction = (global_path_point - static_push_point).normalized();

    // Initialize p_point
    p_point = static_push_point;

    // Get the direction
    for (double dist = 0.0; dist < total_dist; dist += push_detection_dist)
    {
        Vec3f point = static_push_point + dist * direction;
        if (checkIfPointFree(point))
        {
            // get p point (which is in occupied space)
            p_point = point - push_detection_dist * direction;
            break;
        }
    }
}

bool DGPManager::checkIfPointHasNonFreeNeighbour(const Vec3f& point) const
{
    // Check if the point has an occupied neighbour
    Veci<3> point_int = map_util_for_planning_->floatToInt(point);
    return map_util_for_planning_->checkIfPointHasNonFreeNeighbour(point_int);
}

void DGPManager::getOccupiedCells(vec_Vecf<3> &occupied_cells)
{
    // Get the occupied cells
    mtx_read_map_util_.lock();
    auto local_read_map_util = std::make_shared<dynus::VoxelMapUtil>(*read_map_util_);
    mtx_read_map_util_.unlock();
    occupied_cells = local_read_map_util->getOccupiedCloud();
}

void DGPManager::getOccupiedCellsForCvxDecomp(vec_Vecf<3> &occupied_cells, const vec_Vecf<3> &path, bool use_for_safe_path)
{
    // Get the occupied cells
    // We use map_util_for_planning_ (whose start and goal positions are freed by setFreeVoxelAndSurroundings() in solveDGP())

    if (!use_for_safe_path) // for whole trajectory planning
    {
        occupied_cells = map_util_for_planning_->getOccupiedCloud(path, local_box_size_);
    }
    else // for safe path planning
    {
        occupied_cells = map_util_for_planning_->getOccupiedCloudWithUnknownAsOccupied(path, local_box_size_);
    }

    // occupied_cells = map_util_for_planning_->getUninflatedStaticCloud(); // It will be inflated later using drone_bbox
}

void DGPManager::getDynamicOccupiedCellsForCvxDecompTemporal(vec_Vecf<3> &occupied_cells, const std::vector<double> &current_times, std::vector<double> times_elapsed_from_plan_start, const vec_Vecf<3> &path, bool use_for_safe_path)
{
    // Get the occupied cells
    if (!use_for_safe_path) // for whole trajectory planning
    {
        occupied_cells = map_util_for_planning_->getDynamicCloudBasedOnTimesForCvxDecompTemporal(current_times, times_elapsed_from_plan_start, path, local_box_size_);
    }
    else
    {
        occupied_cells = map_util_for_planning_->getDynamicCloudBasedOnTimesForCvxDecompTemporalWithUnknownAsOccupied(current_times, times_elapsed_from_plan_start, path, local_box_size_);
    }
}

void DGPManager::getDynamicOccupiedCellsForVis(vec_Vecf<3> &occupied_cells, vec_Vecf<3> &free_cells, vec_Vecf<3> &unknown_cells, double current_time)
{
    // Lock the mutex for map_util_
    mtx_read_map_util_.lock();
    auto local_read_map_util = std::make_shared<dynus::VoxelMapUtil>(*read_map_util_);
    mtx_read_map_util_.unlock();
    local_read_map_util->getDynamicCloudBasedOnTimeForVis(occupied_cells, free_cells, unknown_cells, current_time);

}

void DGPManager::getComputationTime(double &global_planning_time, double &dgp_static_jps_time, double &dgp_check_path_time, double &dgp_dynamic_astar_time, double &dgp_recover_path_time)
{
    // Get the computation time
    global_planning_time = planner_ptr_->getInitialGuessPlanningTime();
    dgp_static_jps_time = planner_ptr_->getStaticJPSPlanningTime();
    dgp_check_path_time = planner_ptr_->getCheckPathTime();
    dgp_dynamic_astar_time = planner_ptr_->getDynamicAstarTime();
    dgp_recover_path_time = planner_ptr_->getRecoverPathTime();
}

bool DGPManager::cvxEllipsoidDecomp(const state &A, const vec_Vecf<3> &path,
                                      std::vector<LinearConstraint3D> &l_constraints,
                                      vec_E<Polyhedron<3>> &poly_out,
                                      bool use_for_safe_path)
{

    // MyTimer cvx_decomp_timer(true);

    // Initialize result.
    bool result = true;

    // Get static occupied cells.
    vec_Vecf<3> occupied_static_cells;
    getOccupiedCellsForCvxDecomp(occupied_static_cells, path, use_for_safe_path);    
    ellip_decomp_util_.set_obs(occupied_static_cells);

    // Set the local bounding box and z constraints.
    ellip_decomp_util_.set_local_bbox(Vec3f(local_box_size_[0], local_box_size_[1], local_box_size_[2]));
    ellip_decomp_util_.set_z_min_and_max(par_.z_min + 2 * par_.drone_bbox[2],
                                         par_.z_max - 2 * par_.drone_bbox[2]);

    // Set the inflate distance.
    ellip_decomp_util_.set_inflate_distance(drone_radius_);

    // Find convex polyhedra.
    ellip_decomp_util_.dilate(path, result);

    if (!result)
        return false;
    // Optionally shrink polyhedra.
    if (use_shrinked_box_)
        ellip_decomp_util_.shrink_polyhedrons(shrinked_box_size_);

    // Get the polyhedra.
    auto polys = ellip_decomp_util_.get_polyhedrons();

    // Preallocate the constraints vector.
    size_t numConstraints = (path.size() > 0) ? (path.size() - 1) : 0;
    l_constraints.clear();
    l_constraints.resize(numConstraints);

    // Flag to record if any thread finds an error.
    bool errorFound = false;

    // Parallelize the constraint computation loop.
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < static_cast<int>(numConstraints); i++)
    {

        // Compute the midpoint between consecutive path points.
        auto pt_inside = (path[i] + path[i + 1]) / 2.0;
        LinearConstraint3D cs(pt_inside, polys[i].hyperplanes(), polys[i]);

        // If either matrix A_ or vector b_ contains NaN, mark an error.
        if (cs.A_.hasNaN() || cs.b_.hasNaN())
        {
            #pragma omp atomic write
            errorFound = true;
        }
        else
        {
            l_constraints[i] = cs;
        }

    }

    // If an error was detected, report and exit.
    if (errorFound)
    {
        std::cout << "A_ or b_ has NaN" << std::endl;
        return false;
    }

    // Return the computed polyhedra.
    poly_out = std::move(polys);

    // std::cout << "cvxEllipsoidDecomp took " << cvx_decomp_timer.getElapsedMicros() / 1000.0 << " ms" << std::endl;

    return true;
}


bool DGPManager::cvxEllipsoidDecompTemporal(const state &A, const vec_Vecf<3> &path, std::vector<LinearConstraint3D> &l_constraints, vec_E<Polyhedron<3>> &poly_out, double current_time, const std::vector<double> &travel_times, bool use_for_safe_path)
{

    // Initialize result
    bool result = true;

    // Initialization
    l_constraints.clear();
    poly_out.clear();

    // Reserve memory to avoid repeated allocations
    l_constraints.reserve(path.size());
    poly_out.reserve(path.size());

    // Discretize the path and get the estimated time
    Eigen::Vector3d next_point;
    double delta_dist = max_dist_vertexes_;
    double dist_to_goal;
    double travel_time_start, travel_time_end;
    state start, end;

    // Set the local bounding box
    // Only try to find cvx decomp in the Mikowsski sum of JPS and this box (I think) par_.drone_radius
    ellip_decomp_util_.set_local_bbox(Vec3f(local_box_size_[0], local_box_size_[1], local_box_size_[2]));
    ellip_decomp_util_.set_z_min_and_max(par_.z_min + 2* par_.drone_bbox[2], 
                                         par_.z_max - 2* par_.drone_bbox[2]); // buffer for the drone size

    // Set the inflate distance
    ellip_decomp_util_.set_inflate_distance(drone_radius_);

    // Add 0.0 to the first element of travel_times
    std::vector<double> travel_times_with_start;
    travel_times_with_start.push_back(0.0);
    travel_times_with_start.insert(travel_times_with_start.end(), travel_times.begin(), travel_times.end());

    // Generate accumulated travel times
    std::vector<double> accumulated_travel_times;
    accumulated_travel_times.push_back(travel_times_with_start[0] + current_time);
    for (int i = 1; i < travel_times_with_start.size(); i++)
    {
        accumulated_travel_times.push_back(travel_times_with_start[i] + accumulated_travel_times[i - 1]);
    }

    // Sanity checks
    assert(path.size() == travel_times.size() + 1); // travel_times do not include the first state's travel time (because it is 0)
    assert(path.size() == travel_times_with_start.size());
    assert(path.size() == accumulated_travel_times.size());

    for (int i = 0; i < path.size() - 1; i++)
    {

        // Update the dynamic map
        vec_Vecf<3> occupied_dynamic_cells;
        getDynamicOccupiedCellsForCvxDecompTemporal(occupied_dynamic_cells, {accumulated_travel_times[i], accumulated_travel_times[i + 1]}, {travel_times_with_start[i], travel_times_with_start[i + 1]}, {path[i], path[i + 1]}, use_for_safe_path);
        ellip_decomp_util_.set_obs(occupied_dynamic_cells);

        // Find convex polyhedra for the current segment (path[i] to path[i+1])
        ellip_decomp_util_.dilate({path[i], path[i + 1]}, result);
        if (!result)
            return false;

        // Shrink polyhedra by the drone radius. NOT RECOMMENDED (leads to lack of continuity in path sometimes)
        if (use_shrinked_box_)
            ellip_decomp_util_.shrink_polyhedrons(shrinked_box_size_);

        // Convert to inequality constraints Ax < b
        auto polys = ellip_decomp_util_.get_polyhedrons();

        // Get the constraints
        const auto pt_inside = (path[i] + path[i + 1]) / 2;
        LinearConstraint3D cs(pt_inside, polys[0].hyperplanes(), polys[0]);

        // Check if A_ or b_ has NaN
        if (cs.A_.hasNaN() || cs.b_.hasNaN())
        {
            std::cout << "A_ or b_ has NaN" << std::endl;
            return false;
        }

        // Store the results
        l_constraints.push_back(cs);
        poly_out.insert(std::end(poly_out), std::begin(polys), std::end(polys));
    }

    return true;
}

void DGPManager::updateMapCallback(double wdx, double wdy, double wdz, const Vec3f &center_map, const Vec3f &start, const Vec3f &goal, double octmap_received_time, const std::shared_ptr<octomap::TimedOcTree>& lidar_octree, const std::shared_ptr<octomap::TimedOcTree>& depth_camera_octree) 
{

    mtx_write_map_util_.lock();
    write_map_util_->updateMapCallback((int)wdx/res_, (int)wdy/res_, (int)wdz/res_, center_map, start, goal, octmap_received_time, lidar_octree, depth_camera_octree);
    mtx_write_map_util_.unlock();

    // Update the read_map_util_ with the write_map_util_
    updateReadMapUtil();

    // Map is initialized
    if (!map_initialized_)
        map_initialized_ = true;
}

bool DGPManager::isMapInitialized() const
{
    return map_initialized_;
}

void DGPManager::setOctomap(const std::shared_ptr<octomap::TimedOcTree> &octree, std::string octomap_name)
{
    // Set the ROI octomap
    mtx_write_map_util_.lock();

    if (octomap_name == "roi")
    {
        write_map_util_->updateROIOctree(octree);
    }
    else
    {
        std::cout << "Unknown octomap name: " << octomap_name << std::endl;
    }

    mtx_write_map_util_.unlock();

    // Update the read_map_util_ with the write_map_util_
    updateReadMapUtil();
}

void DGPManager::findClosestFreePoint(const Vec3f &point, Vec3f &closest_free_point)
{
    mtx_read_map_util_.lock();
    auto local_read_map_util = std::make_shared<dynus::VoxelMapUtil>(*read_map_util_);
    mtx_read_map_util_.unlock();
    local_read_map_util->findClosestFreePoint(point, closest_free_point);
}

bool DGPManager::computeStaticPushPoints(const vec_Vecf<3> &path, double discretization_dist, Vecf<3> &mean_point, int num_lookahead_global_path_for_push)
{
    return map_util_for_planning_->computeStaticPushPoints(path, discretization_dist, mean_point, num_lookahead_global_path_for_push);
}

int DGPManager::countUnknownCells() const
{
    return map_util_for_planning_->countUnknownCells();
}

int DGPManager::getTotalNumCells() const
{
    return map_util_for_planning_->getTotalNumCells();
}