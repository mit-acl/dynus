/**
 * @file map_util.h
 * @brief MapUtil classes
 */
#ifndef DGP_MAP_UTIL_H
#define DGP_MAP_UTIL_H

#include <iostream>
#include "dgp/data_type.hpp"
#include <dynus/dynus_type.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <octomap/octomap.h>
#include <dgp/TimedOcTree.h>
#include <dgp/TimedOcTreeNode.h>
#include "timer.hpp"
#include <omp.h>

namespace dynus
{

  // The type of map data Tmap is defined as a 1D array
  using Tmap = std::vector<int>;
  typedef timer::Timer MyTimer;

  /**
   * @brief The map util class for collision checking
   * @param Dim is the dimension of the workspace
   */
  template <int Dim>
  class MapUtil
  {
  public:
    // Constructor
    MapUtil(float res, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float inflation, float free_inflation, float octomap_res, float alpha_cov, float dynamic_obstacle_base_inflation, float max_dynamic_obstacle_inflation, float map_buffer, bool use_lidar, bool use_depth_camera, bool use_free_space, bool use_z_axis_bottom_inflation, float node_size_factor_for_occupied, float node_size_factor_for_free)
    {

      /* --------- Initialize parameters --------- */
      setInflation(inflation);                                                                                                                           // Set inflation
      setResolution(res);                                                                                                                                // Set the resolution
      setMapSize(x_min, x_max, y_min, y_max, z_min, z_max);                                                                                              // Set the cells and z_boundaries
      use_free_space_ = use_free_space;                                                                                                                  // Set use_free_space
      octomap_res_ = octomap_res;                                                                                                                        // Set octomap resolution
      cells_inflation_ = std::ceil(inflation / res_);                                                                                                    // Prepare for inflation
      free_cells_inflation_ = std::floor(free_inflation / res_);                                                                                         // Prepare for free cell inflation
      alpha_cov_ = alpha_cov;                                                                                                                            // Set dynamic obstacle tracking parameters
      dynamic_obstacle_base_inflation_ = Eigen::Vector3d(dynamic_obstacle_base_inflation, dynamic_obstacle_base_inflation, dynamic_obstacle_base_inflation); // Set dynamic obstacle tracking parameters
      max_dynamic_obstacle_inflation_ = Eigen::Vector3d(max_dynamic_obstacle_inflation, max_dynamic_obstacle_inflation, max_dynamic_obstacle_inflation); // Set dynamic obstacle tracking parameters
      map_buffer_ = map_buffer;                                                                                                                          // Set local map buffer

      /* --------- Set flags --------- */
      use_lidar_ = use_lidar;
      use_depth_camera_ = use_depth_camera;
      use_z_axis_bottom_inflation_ = use_z_axis_bottom_inflation;

      /* --------- Node size factor for occupied and free --------- */
      node_size_factor_for_occupied_ = node_size_factor_for_occupied;
      node_size_factor_for_free_ = node_size_factor_for_free;
    }

    // Destructor
    ~MapUtil()
    {
      // Clear the map
      map_.clear();
      static_map_.clear();
      dynamic_map_.clear();
    }

    void updateROIOctree(const std::shared_ptr<octomap::TimedOcTree> &roi_octree)
    {
      // We copy the octree since roi_octree could get updated
      roi_octree_ = std::make_shared<octomap::TimedOcTree>(*roi_octree);
    }

    /**
     * @brief [static] Read octree from both lidar and depth camera and update static_map_ and map_
     * @param octree is the octree
     * @param cells_x is the number of cells in the x direction
     * @param cells_y is the number of cells in the y direction
     * @param cells_z is the number of cells in the z direction
     * @param res is the resolution of the map
     * @param center_map is the center of the map (current state of agent)
     * @param z_min is the ground level
     * @param z_max is the maximum height
     * @param inflation is the inflation radius
     * @return void
     */
    void updateMapCallback(int cells_x, int cells_y, int cells_z, const Vec3f &center_map, const Vec3f &start, const Vec3f &goal, float octmap_received_time, const std::shared_ptr<octomap::TimedOcTree>& lidar_octree, const std::shared_ptr<octomap::TimedOcTree>& depth_camera_octree)
    {

      /* --------- House keeping --------- */
      setCellSize(cells_x, cells_y, cells_z);              // Set the number of cells
      setOriginDAndDimAndTotalSize(center_map);            // Set the map size
      if (!map_initialized_)                               // Initialize the map
      {
        // Clear and resize all maps
        map_.clear();
        static_map_.clear();
        dynamic_map_.clear();
        map_.resize(total_size_, val_unknown_);
        static_map_.resize(total_size_, val_unknown_);
        dynamic_map_.resize(total_size_, val_unknown_);
      }

      /* --------- Update octree pointer --------- */

      lidar_octree_ = lidar_octree;
      depth_camera_octree_ = depth_camera_octree;

      /* --------- Update static map --------- */

      // Clear up the static map
      static_map_.assign(total_size_, val_unknown_);

      updateStaticMap(static_map_);
      // if (roi_octree_)
      //   updateStaticMap(static_map_, roi_octree_);

      // Merge maps
      mergeMaps(true);

      // Map is initialized
      if (!map_initialized_)
        map_initialized_ = true;

    }

    // Utility function to get the closest grid point
    inline Vecf<Dim> findClosestGridPoint(const Vecf<Dim> &pt) const
    {
      Vec3f closest_grid_point;

      for (int i = 0; i < 3; ++i)
      {
        // Find the closest grid index
        int grid_index = std::round(pt[i] / res_ - 0.5);

        // Convert back to the grid point (center of the grid)
        closest_grid_point[i] = grid_index * res_ + res_ / 2.0;
      }

      return closest_grid_point;
    }

    // Utility function to get the closest grid point
    inline float findClosestGridPoint(const float pt) const
    {

      // Find the closest grid index
      int grid_index = std::round(pt / res_);

      // Convert back to the grid point (center of the grid)
      return grid_index * res_ + res_ / 2.0;
    }

    // Update static map: now accepts two octree pointers (second defaults to nullptr)
    void updateStaticMap(Tmap &map_to_update)
    {
      // Use the primary tree pointer for resolution (assume at least one is non-null)
      int inflation_cells_ele = std::ceil(inflation_ / res_);
      Veci<3> inflation_cells(inflation_cells_ele, inflation_cells_ele, inflation_cells_ele);
      // Precompute offsets once
      vec_Veci<3> inflation_offsets = computeInflationOffsets(inflation_cells);

      // Container for nodes to inflate (in integer grid coordinates)
      std::vector<Veci<3>> nodes_to_inflate;

      // Loop over all grid cells (parallelized)
      #pragma omp parallel for collapse(3)
      for (int ix = 0; ix < dim_(0); ++ix) {
        for (int iy = 0; iy < dim_(1); ++iy) {
          for (int iz = 0; iz < dim_(2); ++iz) {
            // Convert grid indices to world coordinates.
            float x = x_min_ + ix * res_;
            float y = y_min_ + iy * res_;
            float z = z_min_ + iz * res_;

            // Query point
            octomap::point3d query_point(x, y, z);

            // std::cout << "query_point: " << query_point << std::endl;

            // Check first octree pointer if available.
            octomap::TimedOcTreeNode* lidar_node = lidar_octree_->search(query_point);
            octomap::TimedOcTreeNode* depth_camera_node = depth_camera_octree_->search(query_point);

            bool lidar_occupied = false;
            bool depth_camera_occupied = false;
            bool lidar_free = false;
            bool depth_camera_free = false;

            if (lidar_node)
            {
              if (lidar_octree_->isNodeOccupied(lidar_node)) // occupied
              {
                lidar_occupied = true;
              }
              else // free
              {
                lidar_free = true;
              }
            }

            if (depth_camera_node)
            {
              if (depth_camera_octree_->isNodeOccupied(depth_camera_node)) // occupied
              {
                depth_camera_occupied = true;
              }
              else // free
              {
                depth_camera_free = true;
              }
            }

            // Update the map
            // Lidar is a bit noisier than depth camera
            if (depth_camera_occupied || lidar_occupied)
            {
              #pragma omp critical
              {
                nodes_to_inflate.push_back(Veci<3>(ix, iy, iz));
              }
            }
            else if (depth_camera_free || lidar_free)
            {
              map_to_update[getIndex(Veci<3>(ix, iy, iz))] = val_free_;
            }
            else // depth camera unknown
            {
              if (lidar_occupied)
              {
                #pragma omp critical
                {
                  nodes_to_inflate.push_back(Veci<3>(ix, iy, iz));
                }
              }
              else if (lidar_free)
              {
                map_to_update[getIndex(Veci<3>(ix, iy, iz))] = val_free_;
              }
            }
          }
        }
      }

      // Now update the map sequentially using the nodes to inflate.
      #pragma omp parallel for collapse(2)
      for (size_t i = 0; i < nodes_to_inflate.size(); ++i) {
        // For each node, loop over inflation offsets.
        for (size_t j = 0; j < inflation_offsets.size(); ++j) {
          Veci<3> inflated_point = nodes_to_inflate[i] + inflation_offsets[j];

          // Get the corresponding index in the map.
          if (!isOutside(inflated_point)) {
            map_to_update[getIndex(inflated_point)] = val_occ_;
          }
        }
      }
    } // updateStaticMap

    // Pre-compute inflation
    // Precompute offsets for inflation
    vec_Veci<3> computeInflationOffsets(const Veci<3> &inflation_cells)
    {
      vec_Veci<3> offsets;
      
      // include diagonal offsets
      for (int dx = -inflation_cells(0); dx <= inflation_cells(0); ++dx)
      {
        for (int dy = -inflation_cells(1); dy <= inflation_cells(1); ++dy)
        {
          for (int dz = -inflation_cells(2); dz <= inflation_cells(2); ++dz)
          {
            offsets.push_back(Veci<3>(dx, dy, dz));
          }
        }
      }

      return offsets;
    }

    /**
     * @brief Computes push vectors to adjust a given path away from obstacles.
     * @param path A vector of Eigen::Vector3d representing the path waypoints.
     * @param discretization_dist The distance of discretization for the path.
     * @param mean_point The mean point of the occupied points.
     * @return success A boolean indicating if the computation was successful.
     */
    bool computeStaticPushPoints(const vec_Vecf<3> &path, float discretization_dist, Vecf<3> &mean_point, int num_lookahead_global_path_for_push)
    {

      // Get the start and end points
      const Eigen::Vector3d start = path.front();
      const Eigen::Vector3d end = (num_lookahead_global_path_for_push < path.size()) ? path[num_lookahead_global_path_for_push] : path.back();
      // const Eigen::Vector3d end = path.back();

      // Discretize the line segment
      Eigen::Vector3d diff = end - start;
      int steps = std::ceil(diff.norm() / discretization_dist);
      Eigen::Vector3d step = diff / static_cast<float>(steps);

      // Initialize the occupied points
      std::vector<Eigen::Vector3d> points;

      for (int j = 0; j <= steps; ++j)
      {
        Eigen::Vector3d point = start + j * step;

        // Convert point to map index
        int map_index = getIndex(floatToInt(point));

        // Check if point is within map bounds
        if (!(map_index >= 0 && map_index < total_size_))
          continue;

        // Get map index and check if it's occupied or unknown
        if (!static_map_[map_index] == val_free_)
        {
          points.push_back(point);
        }
      }

      if (points.empty())
      {
        return false;
      }

      // Compute the mean location of the points
      mean_point = Vecf<3>::Zero();
      for (const auto &point : points)
      {
        mean_point += point;
      }
      mean_point /= points.size();

      return true;
    }

    // Apply inflation around a specific index
    void applyInflationToDyanmicMap(Tmap &map, const Veci<3> &position, const Eigen::Vector3d &inflation, bool for_occ = true)
    {

      int cells_inflation_x = std::ceil(inflation(0) / res_);
      int cells_inflation_y = std::ceil(inflation(1) / res_);
      int cells_inflation_z = std::ceil(inflation(2) / res_);

      int min_x = std::max(0, position[0] - cells_inflation_x);
      int max_x = std::min(dim_(0) - 1, position[0] + cells_inflation_x);
      int min_y = std::max(0, position[1] - cells_inflation_y);
      int max_y = std::min(dim_(1) - 1, position[1] + cells_inflation_y);
      int min_z = std::max(0, position[2] - cells_inflation_z);
      int max_z = std::min(dim_(2) - 1, position[2] + cells_inflation_z);

      // #pragma omp parallel for collapse(3)
      for (int dz = min_z; dz <= max_z; ++dz)
      {
        for (int dy = min_y; dy <= max_y; ++dy)
        {
          for (int dx = min_x; dx <= max_x; ++dx)
          {
            int new_index = getIndex(Veci<3>(dx, dy, dz));
            if (new_index >= 0 && new_index < total_size_)
            {
              if (for_occ || map[new_index] != val_occ_)
              {
                map[new_index] = for_occ ? val_occ_ : val_free_;
              }
            }
          }
        }
      }
    }

    Veci<3> indexToVeci3(int index)
    {
      Veci<3> position;
      position[0] = index % dim_(0);
      position[1] = (index / dim_(0)) % dim_(1);
      position[2] = index / (dim_(0) * dim_(1));
      return position;
    }

    void setCellSize(int cells_x, int cells_y, int cells_z)
    {
      // Set cells
      cells_x_ = cells_x;
      cells_y_ = cells_y;
      cells_z_ = cells_z;
    }

    void setMapSize(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    {
      // Set map boundaries
      x_map_min_ = x_min;
      x_map_max_ = x_max;
      y_map_min_ = y_min;
      y_map_max_ = y_max;
      z_map_min_ = z_min;
      z_map_max_ = z_max;
    }

    void setOriginDAndDimAndTotalSize(const Vec3f &center_map)
    {
    // Define the dimensions of the map.
    Vec3i dim(cells_x_, cells_y_, cells_z_);
    dim(0) = dim(0) + static_cast<int>(inflation_ / res_); // add inflation buffer
    dim(1) = dim(1) + static_cast<int>(inflation_ / res_); // add inflation buffer
    dim(2) = dim(2) + static_cast<int>(inflation_ / res_); // add inflation buffer

    // they need to be even numbers (if the resolution is even), so if they are odd, we add 1
    for (int i = 0; i < 3; i++)
    {
      if (dim(i) % 2 != 0)
        dim(i) += 1;
    }

    // Split the vertical (z) dimension around the center.
    int dim2_down = dim(2) / 2;
    int dim2_up   = dim(2) / 2;

    // Convert the center of the map to grid coordinates.
    Vec3f center_map_grid = findClosestGridPoint(center_map);

    // Adjust if the lower part of the map goes below z_map_min_
    if ((center_map_grid[2] - res_ * dim(2) / 2.0) < z_map_min_) {
      dim2_down = std::max(static_cast<int>((center_map_grid(2) - z_map_min_) / res_), 0);
    }

    // Adjust if the upper part of the map goes above z_map_max_
    if (center_map_grid[2] + res_ * dim(2) / 2.0 > z_map_max_) {
      dim2_up = static_cast<int>((z_map_max_ - center_map_grid(2)) / res_);
    }

    dim(2) = dim2_down + dim2_up;

    // Compute the origin so that the cell centers align with the map center.
    // We add res_/2.0 to shift from a corner-based coordinate to a center-based one.
    origin_d_(0) = center_map_grid(0) - res_ * dim(0) / 2.0;
    origin_d_(1) = center_map_grid(1) - res_ * dim(1) / 2.0;
    origin_d_(2) = center_map_grid(2) - res_ * dim2_down;

    // Optionally align origin_d_ to the nearest discretized grid point.
    // origin_d_ = findClosestGridPoint(origin_d_);

    // Update the internal dimension vector.
    for (unsigned int i = 0; i < 3; i++) {
      dim_(i) = dim(i);
    }

    // Compute the total size.
    total_size_ = dim(0) * dim(1) * dim(2);

    // Store the map center.
    center_map_ = center_map_grid;

    // Compute the bounds in floating point. (these all need to be odd numbers (if resolution is even))
    x_min_ = std::max(static_cast<float>(center_map_grid[0] - dim(0) / 2.0 * res_), x_map_min_);
    x_max_ = std::min(static_cast<float>(center_map_grid[0] + dim(0) / 2.0 * res_), x_map_max_);
    y_min_ = std::max(static_cast<float>(center_map_grid[1] - dim(1) / 2.0 * res_), y_map_min_);
    y_max_ = std::min(static_cast<float>(center_map_grid[1] + dim(1) / 2.0 * res_), y_map_max_);
    z_min_ = std::max(static_cast<float>(center_map_grid[2] - dim2_down * res_), z_map_min_);
    z_max_ = std::min(static_cast<float>(center_map_grid[2] + dim2_up * res_), z_map_max_);

  }

    /**
     * @brief  [static] Update the dynamic map based on the current time (mainly for initial guess planning)
     *         This function is used to update dynamic_map_ based of trajs_ at the current time
     *
     * @param  {double} current_time : The current time
     * @return {void}
     */
    void updateMap(double current_time, double time_elapsed_from_plan_start)
    {

      // Check if the map is initialized
      if (!map_initialized_)
      {
        std::cout << "Map is not initialized" << std::endl;
        return;
      }

      // Clear the dynamic map
      dynamic_map_.assign(total_size_, val_unknown_);

      if (!trajs_initialized_)
      {
        mergeMaps(false);
        return;
      }

      // update the dynamic obstacle
      for (dynTraj &traj : trajs_)
        updateDynamicMapFromTrajs(traj, current_time, time_elapsed_from_plan_start);

      // merge the dynamic map with the static map
      mergeMaps(true);
    }

    /**
     * @brief  [static] Update the dynamic map based on the current times (mainly for convex decomposition)
     *         This function is used to update dynamic_map_ based of trajs_ at the current times
     *
     * @param  {std::vector<float>} current_times : The current times
     * @return {void}
     */
    void updateMapConvexDecompForCvxDecompTemporal(std::vector<double> current_times, std::vector<double> times_elapsed_from_plan_start)
    {

      // Check if the map is initialized
      if (!map_initialized_)
        return;

      // Clear the dynamic map
      dynamic_map_.assign(total_size_, val_unknown_);

      if (!trajs_initialized_)
      {
        // std::cout << "Trajectories are not initialized" << std::endl;
        mergeMaps(false);
        return;
      }

      // update the dynamic obstacle
      for (dynTraj &traj : trajs_)
      {
        for (int i = 0; i < current_times.size(); i++)
        {
          updateDynamicMapFromTrajs(traj, current_times[i], times_elapsed_from_plan_start[i]);
        }
      }

      // merge the dynamic map with the static map
      mergeMaps(true);
    }

    void updateDynamicMapFromTrajs(dynTraj &traj, double current_time, double time_elapsed_from_plan_start)
    {

      // If current time is not in the trajectory, return
      // TODO: Could be dangerous - only need this for visulaization but not actual deconfliction
      // if (current_time < traj.pwp.times.front() || current_time > traj.pwp.times.back())
      // {
      //   return;
      // }

      // Get trajectory's position in the map
      Eigen::Vector3d traj_position_world = traj.pwp.eval(current_time);

      // Check if the trajectory is inside the map
      if (traj_position_world.z() < z_min_ || traj_position_world.z() > z_max_ || traj_position_world.x() < x_min_ || traj_position_world.x() > x_max_ || traj_position_world.y() < y_min_ || traj_position_world.y() > y_max_)
      {
        // std::cout << "Trajectory is outside the map" << std::endl;
        return;
      }

      // Get the position of the trajectory in int
      Veci<3> traj_position_int = floatToInt(Vec3f(traj_position_world.x(), traj_position_world.y(), traj_position_world.z()));

      // Get the index
      int index = getIndex(traj_position_int);

      if (index >= 0 && index < total_size_)
      {
        // Set occupied value
        dynamic_map_[index] = val_occ_;

        // get the inlated points
        if (traj.is_agent)
        {
          applyInflationToDyanmicMap(dynamic_map_, traj_position_int, traj.bbox, true);
        }
        else
        {
          // use ekf P for inflation and increase the size as the time goes
          Eigen::Vector3d inflation = ((traj.bbox / 2.0) + dynamic_obstacle_base_inflation_ + time_elapsed_from_plan_start * alpha_cov_ * traj.ekf_cov_p)
                                          .cwiseMin(max_dynamic_obstacle_inflation_);
          applyInflationToDyanmicMap(dynamic_map_, traj_position_int, inflation, true);
        }
      }
    }

    /**
     * @brief  Find a free point in the map that is closest to the given point
     * @param  vec_Vecf<3> point : The given point
     * @param  vec_Vecf<3> free_point : The free point that is closest to the given point
     * @return void
     */
    void findClosestFreePoint(const Vec3f &point, Vec3f &closest_free_point)
    {

      // Initialize the closest free point
      closest_free_point = point;

      // Check if the map is initialized
      if (!map_initialized_)
      {
        std::cout << "Map is not initialized" << std::endl;
        return;
      }

      // Get the position of the point in int
      Veci<3> point_int = floatToInt(point);

      // Get the index
      int index = getIndex(point_int);

      if (index >= 0 && index < total_size_)
      {
        // Check if the point is free
        if (static_map_[index] == val_free_)
        {
          closest_free_point = point;
          return;
        }

        // Get the neighboring indices
        std::vector<int> neighbor_indices;

        // Increase the radius until a free point is found
        for (float radius = 1.0; radius < 5.0; radius += 0.5) // TODO: expose the radius as a parameter
        {
          neighbor_indices.clear();
          getNeighborIndices(point_int, neighbor_indices, radius);

          // Find the closest free point
          float min_dist = std::numeric_limits<float>::max();
          for (int neighbor_index : neighbor_indices)
          {
            if (neighbor_index >= 0 && neighbor_index < total_size_)
            {
              if (static_map_[neighbor_index] == val_free_)
              {
                Veci<3> neighbor_int = indexToVeci3(neighbor_index);
                Vec3f neighbor = intToFloat(neighbor_int);
                float dist = (neighbor - point).norm();
                if (dist < min_dist)
                {
                  min_dist = dist;
                  closest_free_point = neighbor;
                }
              }
            }
          }

          // Check if a free point is found
          if (min_dist < std::numeric_limits<float>::max())
          {
            return;
          }
        }
      }
    }

    /**
     * @brief Get indices of the neighbors of a point given the radius
     * @param Veci<3> point_int : The given point
     * @param std::vector<int>& neighbor_indices : The indices of the neighbors
     * @param float radius : The radius
     * @return void
     * */
    void getNeighborIndices(const Veci<3> &point_int, std::vector<int> &neighbor_indices, float radius)
    {
      // Get the radius in int
      float radius_int = radius / res_;
      Veci<3> radius_int_vec(radius_int, radius_int, radius_int);

      // Get the min and max positions
      Veci<3> min_pos = point_int - radius_int_vec;
      Veci<3> max_pos = point_int + radius_int_vec;

      // Iterate over the neighbors
      for (int x = min_pos[0]; x <= max_pos[0]; ++x)
      {
        for (int y = min_pos[1]; y <= max_pos[1]; ++y)
        {
          for (int z = min_pos[2]; z <= max_pos[2]; ++z)
          {

            // Check if the neighbor is inside the map
            if (x >= 0 && x < dim_(0) && y >= 0 && y < dim_(1) && z >= 0 && z < dim_(2))
            {

              Veci<3> neighbor_int(x, y, z);
              int index = getIndex(neighbor_int);
              if (index >= 0 && index < total_size_)
              {
                neighbor_indices.push_back(index);
              }
            }
          }
        }
      }
    }

    // Check if the given point has any occupied neighbors.
    // Returns true if at least one neighboring cell is non-free.
    inline bool checkIfPointHasNonFreeNeighbour(const Veci<Dim> &pt) const
    {
      if constexpr (Dim == 2)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          for (int dy = -1; dy <= 1; ++dy)
          {
            // Skip the center point
            if (dx == 0 && dy == 0)
              continue;
            Veci<2> neighbor = pt;
            neighbor(0) += dx;
            neighbor(1) += dy;
            // Check if the neighbor is within the map and non-free.
            if (!isOutside(neighbor) && !isFree(neighbor))
              return true;
          }
        }
      }
      else if constexpr (Dim == 3)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          for (int dy = -1; dy <= 1; ++dy)
          {
            for (int dz = -1; dz <= 1; ++dz)
            {
              // Skip the center point
              if (dx == 0 && dy == 0 && dz == 0)
                continue;
              Veci<3> neighbor = pt;
              neighbor(0) += dx;
              neighbor(1) += dy;
              neighbor(2) += dz;
              // Check if the neighbor is within the map and non-free.
              if (!isOutside(neighbor) && !isFree(neighbor))
                return true;
            }
          }
        }
      }
      return false;
    }

    void setInflation(float inflation)
    {
      inflation_ = inflation;
      inflation_vec_ = Eigen::Vector3d(inflation, inflation, inflation);
      if (inflation_ < 1e-6)
        use_inflation_ = false;
    }

    void setResolution(float res)
    {
      res_ = res;
    }

    /**
     * @brief  [static] Merge the static_map_ and dynamic_map_ into map_
     * @return {void}
     */
    void mergeMaps(bool use_dynamic_map)
    {

      // Make sure static_map_ and map_ are initialized only once, outside this function
      if (!map_initialized_)
        return;

      // Initialize map_ with static_map_
      map_ = static_map_;

      if (use_dynamic_map)
      {

// Combine the static and dynamic maps into map_
#pragma omp parallel for
        for (int i = 0; i < dynamic_map_.size(); i++)
        {
          if (dynamic_map_[i] == val_occ_)
          {
            map_[i] = val_occ_;
          }
        }
      }
    }

    // Check if a point is inside the box formed by 8 points (convex hull of 8 points)
    bool isPointInBox(const Vec3f &p, const std::vector<Vec3f> &vertices)
    {
      // Assuming vertices contains 8 points representing the corners of the box in 3D space.
      // The box is axis-aligned and the vertices are provided in any order.

      // Find the minimum and maximum x, y, and z coordinates among the vertices.
      Vec3f min = vertices[0];
      Vec3f max = vertices[0];

      for (const auto &vertex : vertices)
      {
        min.x() = std::min(min.x(), vertex.x());
        min.y() = std::min(min.y(), vertex.y());
        min.z() = std::min(min.z(), vertex.z());

        max.x() = std::max(max.x(), vertex.x());
        max.y() = std::max(max.y(), vertex.y());
        max.z() = std::max(max.z(), vertex.z());
      }

      // Add buffer to the min and max coordinates to give some margin around the box.
      // TODO: expose this buffer as a parameter.
      min -= Vec3f(0.5, 0.5, 0.5); // min -= Vec3f
      max += Vec3f(0.5, 0.5, 0.5); // max += Vec3f

      // Check if the point lies within the bounds defined by the min and max coordinates.
      return (p.x() >= min.x() && p.x() <= max.x()) &&
             (p.y() >= min.y() && p.y() <= max.y()) &&
             (p.z() >= min.z() && p.z() <= max.z());
    }

    // Given a set of 8 points, create a convex hull (a cube or rectangular cuboid) and fill the cells inside the convex hull
    void updateDynamicMap(const std::vector<Vec3f> &points)
    {

      if (points.size() != 8)
      {
        std::cerr << "Error: Exactly 8 points are required to form a cube or rectangular cuboid." << std::endl;
        return;
      }

      // Create a bounding box around the points to limit the search space
      Vec3f min_point = points[0];
      Vec3f max_point = points[0];

      for (const auto &point : points)
      {
        min_point = min_point.cwiseMin(point);
        max_point = max_point.cwiseMax(point);
      }

      // Convert the bounding box to map coordinates
      Vec3i min_coords = floatToInt(min_point);
      Vec3i max_coords = floatToInt(max_point);

      // Iterate through all points in the bounding box and check if they are inside the convex hull
      for (int x = min_coords.x(); x <= max_coords.x(); ++x)
      {
        for (int y = min_coords.y(); y <= max_coords.y(); ++y)
        {
          for (int z = min_coords.z(); z <= max_coords.z(); ++z)
          {
            // Convert map coordinates back to world coordinates
            Vec3f point_world = intToFloat(Vec3i(x, y, z));

            // Check if the point is inside the hexahedron (convex hull of 8 points)
            if (isPointInBox(point_world, points))
            {

              // Get the id of the point in the map
              int id = x + dim_.x() * y + dim_.x() * dim_.y() * z;

              // Set the point as occupied
              if (id >= 0 && id < total_size_)
              {
                dynamic_map_[id] = val_occ_;

                // Inflate the voxels around this point
                int m = static_cast<int>(std::floor(inflation_ / res_));
                for (int ix = x - m; ix <= x + m; ++ix)
                {
                  for (int iy = y - m; iy <= y + m; ++iy)
                  {
                    for (int iz = z - m; iz <= z + m; ++iz)
                    {
                      int id_infl = ix + dim_.x() * iy + dim_.x() * dim_.y() * iz;
                      if (id_infl >= 0 && id_infl < total_size_) // Ensure we are inside the map bounds
                      {
                        dynamic_map_[id_infl] = val_occ_;
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    // Update dynamic trajectories
    void updateTrajs(const std::vector<dynTraj> &trajs)
    {
      if (!trajs_initialized_)
        trajs_initialized_ = true;

      trajs_ = trajs;
    }

    // Get resolution
    decimal_t getRes()
    {
      return res_;
    }
    // Get dimensions
    Veci<Dim> getDim()
    {
      return dim_;
    }
    // Get origin
    Vecf<Dim> getOrigin()
    {
      return origin_d_;
    }
    // Get index of a cell
    inline int getIndex(const Veci<Dim> &pn) const
    {
      return Dim == 2 ? pn(0) + dim_(0) * pn(1) : pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
    }
    // Get index of a cell in old map
    inline int getOldIndex(const Veci<Dim> &pn) const
    {
      return Dim == 2 ? pn(0) + prev_dim_(0) * pn(1) : pn(0) + prev_dim_(0) * pn(1) + prev_dim_(0) * prev_dim_(1) * pn(2);
    }

    ///
    Veci<Dim> getVoxelPos(int idx)
    {
      Veci<Dim> pn;
      if (Dim == 2)
      {
        pn(0) = idx % dim_(0);
        pn(1) = idx / dim_(0);
      }
      else
      {
        pn(0) = idx % dim_(0);
        pn(1) = (idx / dim_(0)) % dim_(1);
        pn(2) = idx / (dim_(0) * dim_(1));
      }
      return pn;
    }
    // Check if the given cell is outside of the map in i-the dimension
    inline bool isOutsideXYZ(const Veci<Dim> &n, int i) const
    {
      return n(i) < 0 || n(i) >= dim_(i);
    }
    // Check if the cell is free by index
    inline bool isFree(int idx) const
    {
      return map_[idx] == val_free_;
    }
    // Check if the cell is unknown by index
    inline bool isUnknown(int idx) const
    {
      return map_[idx] == val_unknown_;
    }
    // Check if the cell is occupied by index
    inline bool isOccupied(int idx) const
    {
      return map_[idx] > val_free_;
    }
    // Check if the cell is occupied by index
    inline bool isStaticOccupied(int idx) const
    {
      return static_map_[idx] > val_free_;
    }
    // Check if the cell is occupied by index in uninflated map
    inline bool isStaticOccupiedUninflated(int idx) const
    {
      return uninflated_static_map_[idx] > val_free_;
    }

    inline void setOccupied(const Veci<Dim> &pn)
    {
      int index = getIndex(pn);
      if (index >= 0 && index < total_size_)
      { // check that the point is inside the map
        map_[getIndex(pn)] = 100;
      }
    }

    inline void setFree(const Veci<Dim> &pn)
    {
      int index = getIndex(pn);
      if (index >= 0 && index < total_size_)
      { // check that the point is inside the map
        static_map_[index] = val_free_;
        map_[index] = val_free_;
      }
    }

    // set Free all the voxels that are in a 3d cube centered at center and with side/2=d
    inline void setFreeVoxelAndSurroundings(const Veci<Dim> &center, const float d)
    {
      int n_voxels = std::round(d / res_ + 0.5); // convert distance to number of voxels
      for (int ix = -n_voxels; ix <= n_voxels; ix++)
      {
        for (int iy = -n_voxels; iy <= n_voxels; iy++)
        {
          for (int iz = -n_voxels; iz <= n_voxels; iz++)
          {
            Veci<Dim> voxel = center + Veci<Dim>(ix, iy, iz); // Int coordinates of the voxel I'm going to clear

            // std::cout << "Clearing" << voxel.transpose() << std::endl;
            setFree(voxel);
          }
        }
      }
    }

    // Check if the cell is outside by coordinate
    inline bool isOutsideOldMap(const Veci<Dim> &pn) const
    {
      for (int i = 0; i < Dim; i++)
        if (pn(i) < 0 || pn(i) >= prev_dim_(i))
          return true;
      return false;
    }
    // Check if the cell is outside by coordinate
    inline bool isOutside(const Veci<Dim> &pn) const
    {
      for (int i = 0; i < Dim; i++)
        if (pn(i) < 0 || pn(i) >= dim_(i))
          return true;
      return false;
    }
    inline bool isOutside(const Vecf<Dim> &pt) const
    {
      return isOutside(floatToInt(pt));
    }
    // Check if the given cell is free by coordinate
    inline bool isFree(const Veci<Dim> &pn) const
    {
      if (isOutside(pn))
        return false;
      else
        return isFree(getIndex(pn));
    }
    inline bool isFree(const Vecf<Dim> &pt) const
    {
      return isFree(floatToInt(pt));
    }
    // Check if the given cell is occupied by coordinate
    inline bool isOccupied(const Veci<Dim> &pn) const
    {
      if (isOutside(pn))
        return false;
      else
        return isOccupied(getIndex(pn));
    }
    inline bool isOccupied(const Vecf<Dim> &pt) const
    {
      return isOccupied(floatToInt(pt));
    }
    inline bool isStaticOccupied(const Veci<Dim> &pn) const
    {
      if (isOutside(pn))
        return false;
      else
        return isStaticOccupied(getIndex(pn));
    }
    inline bool isStaticOccupied(const Vecf<Dim> &pt) const
    {
      return isStaticOccupied(floatToInt(pt));
    }
    // Check if the given cell is unknown by coordinate
    inline bool isUnknown(const Veci<Dim> &pn) const
    {
      if (isOutside(pn))
        return false;
      return map_[getIndex(pn)] == val_unknown_;
    }
    inline bool isUnknown(const Vecf<Dim> &pt) const
    {
      return isUnknown(floatToInt(pt));
    }

    // Print basic information about the util
    void info()
    {
      Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
      std::cout << "MapUtil Info ========================== " << std::endl;
      std::cout << "   res: [" << res_ << "]" << std::endl;
      std::cout << "   origin: [" << origin_d_.transpose() << "]" << std::endl;
      std::cout << "   range: [" << range.transpose() << "]" << std::endl;
      std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
    };

    // Float position to discrete cell coordinate
    inline Veci<Dim> floatToInt(const Vecf<Dim> &pt) const
    {
      Veci<Dim> pn;

      Vecf<Dim> ptg = findClosestGridPoint(pt);

      for (int i = 0; i < Dim; i++)
        pn(i) = std::round((ptg(i) - origin_d_(i)) / res_);
        // pn(i) = std::round((ptg(i) - origin_d_(i)) / res_ - 0.5);
      return pn;
    }
    // Discrete cell coordinate to float position
    inline Vecf<Dim> intToFloat(const Veci<Dim> &pn) const
    {
      // return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) * res_ + origin_d_;
      return pn.template cast<decimal_t>() * res_ + origin_d_;
      // return findClosestGridPoint(pn.template cast<decimal_t>() * res_ + origin_d_);
    }

    // Raytrace from float point pt1 to pt2
    inline vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2) const
    {
      Vecf<Dim> diff = pt2 - pt1;
      decimal_t k = 0.1;
      int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
      decimal_t s = 1.0 / max_diff;
      Vecf<Dim> step = diff * s;

      vec_Veci<Dim> pns;
      Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
      for (int n = 1; n < max_diff; n++)
      {
        Vecf<Dim> pt = pt1 + step * n;
        Veci<Dim> new_pn = floatToInt(pt);
        if (isOutside(new_pn))
          break;
        if (new_pn != prev_pn)
          pns.push_back(new_pn);
        prev_pn = new_pn;
      }
      return pns;
    }

    // Check if the ray from p1 to p2 is occluded
    inline bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2, int8_t val = 100) const
    {
      vec_Veci<Dim> pns = rayTrace(p1, p2);
      for (const auto &pn : pns)
      {
        if (map_[getIndex(pn)] >= val)
          return true;
      }
      return false;
    }

    // Get occupied dynamic voxels based on time
    void getDynamicCloudBasedOnTimeForVis(vec_Vecf<3> &occupied_cells, vec_Vecf<3> &free_cells, vec_Vecf<3> &unknown_cells, double current_time)
    {

      // update map using current_time
      updateMap(current_time, 0.0); // time_elapsed_from_plan_start is only used for planning to increasingly inflate the dynamic obstacles - for visualization, it only visualizes at the current moment anyways, so we can just set it to 0

      // get the cloud
      occupied_cells = getOccupiedCloud();
      free_cells = getFreeCloud();
      unknown_cells = getUnknownCloud();
    }

    // Get occupied dynamic voxels based on times
    vec_Vecf<Dim> getDynamicCloudBasedOnTimesForCvxDecompTemporalWithUnknownAsOccupied(std::vector<double> current_times, std::vector<double> times_elapsed_from_plan_start, const vec_Vecf<3> &path, const std::vector<float> local_box_size)
    {

      // update map using current_times
      updateMapConvexDecompForCvxDecompTemporal(current_times, times_elapsed_from_plan_start);

      // get the cloud
      return getOccupiedCloudWithUnknownAsOccupied(path, local_box_size);
    }

    // Get occupied dynamic voxels based on times
    vec_Vecf<Dim> getDynamicCloudBasedOnTimesForCvxDecompTemporal(std::vector<double> current_times, std::vector<double> times_elapsed_from_plan_start, const vec_Vecf<3> &path, const std::vector<float> local_box_size)
    {

      // update map using current_times
      updateMapConvexDecompForCvxDecompTemporal(current_times, times_elapsed_from_plan_start);

      // get the cloud
      return getOccupiedCloud(path, local_box_size);
    }

    // Compute vicinities
    void computeVicinityMapInteger(const vec_Vecf<3> &path, const std::vector<float> &local_box_size, Veci<3> &min_point_int, Veci<3> &max_point_int) const
    {
      // 1. Compute the global bounding box around the path.
      Vecf<3> min_point_float = Vecf<3>::Constant(std::numeric_limits<float>::max());
      Vecf<3> max_point_float = Vecf<3>::Constant(std::numeric_limits<float>::lowest());

      for (const auto &point : path)
      {
        // Inflate the local box size (using factor 1.5 as in your example)
        Vecf<3> inflated_local_box_size(1.5 * local_box_size[0], 1.5 * local_box_size[1], 1.5 * local_box_size[2]);
        Vecf<3> local_min = point - inflated_local_box_size;
        Vecf<3> local_max = point + inflated_local_box_size;

        // Update global bounds
        min_point_float = min_point_float.cwiseMin(local_min);
        max_point_float = max_point_float.cwiseMax(local_max);
      }

      // 2. Generate min and max points in integer coordinates
      min_point_int = floatToInt(min_point_float);
      max_point_int = floatToInt(max_point_float);
    }

    // Cloud-getting actually happens here
    template <typename CheckFunc>
    vec_Vecf<Dim> getCloud_(CheckFunc check, const Veci<3>& min_point_int, const Veci<3>& max_point_int) const
    {
      vec_Vecf<Dim> cloud;

      // Reserve an estimated size (optional, just to reduce reallocations)
      cloud.reserve(static_cast<size_t>((max_point_int - min_point_int).prod()));

      if (Dim == 3)
      {
        #pragma omp parallel
        {
          vec_Vecf<Dim> local_cloud;
          // Collapse the three nested loops into one for OpenMP
          #pragma omp for collapse(3) nowait
          for (int i = min_point_int(0); i < max_point_int(0); ++i)
          {
            for (int j = min_point_int(1); j < max_point_int(1); ++j)
            {
              for (int k = min_point_int(2); k < max_point_int(2); ++k)
              {
                Veci<3> pti(i, j, k);
                // Use the provided check function
                if (check(pti) && !isOutside(pti))
                {
                  Vecf<3> ptf = intToFloat(pti);
                  local_cloud.push_back(ptf);
                }
              }
            }
          }
          #pragma omp critical
          {
            cloud.insert(cloud.end(), local_cloud.begin(), local_cloud.end());
          }
        }
      }
      else if (Dim == 2)
      {
        #pragma omp parallel
        {
          vec_Vecf<Dim> local_cloud;
          #pragma omp for collapse(2) nowait
          for (int i = min_point_int(0); i < max_point_int(0); ++i)
          {
            for (int j = min_point_int(1); j < max_point_int(1); ++j)
            {
              Veci<3> pti(i, j, 0);
              if (check(pti) && !isOutside(pti))
              {
                Vecf<3> ptf = intToFloat(pti);
                local_cloud.push_back(ptf);
              }
            }
          }
          #pragma omp critical
          {
            cloud.insert(cloud.end(), local_cloud.begin(), local_cloud.end());
          }
        }
      }

      return cloud;
    }

    // Cloud-getter
    template <typename CheckFunc>
    vec_Vecf<Dim> getCloud(CheckFunc check, const vec_Vecf<3> &path, const std::vector<float> local_box_size) const
    {
      vec_Vecf<Dim> cloud;

      // Compute vicinty map integer
      Veci<3> min_point_int, max_point_int;
      computeVicinityMapInteger(path, local_box_size, min_point_int, max_point_int);

      return getCloud_(check, min_point_int, max_point_int);
    }

    // Cloud-getter
    template <typename CheckFunc>
    vec_Vecf<Dim> getCloud(CheckFunc check) const
    {
      vec_Vecf<Dim> cloud;

      // Get the minimum and maximum points in the current map
      Vecf<3> min_point_float, max_point_float;
      min_point_float(0) = x_min_;
      min_point_float(1) = y_min_;
      min_point_float(2) = z_min_;
      max_point_float(0) = x_max_;
      max_point_float(1) = y_max_;
      max_point_float(2) = z_max_;
      Veci<3> min_point_int = floatToInt(min_point_float);
      Veci<3> max_point_int = floatToInt(max_point_float);

      return getCloud_(check, min_point_int, max_point_int);
    }

    // Get occupied voxels (useful for convex decomposition)
    inline vec_Vecf<Dim> getOccupiedCloud(const vec_Vecf<3> &path, const std::vector<float> local_box_size) const
    {
      // Get cloud for occupied cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isOccupied(pti); }, path, local_box_size);
    }

    // Get occupied voxels for the entire map (useful for visualization)
    inline vec_Vecf<Dim> getOccupiedCloud() const
    {
      // Get cloud for occupied cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isOccupied(pti); });
    }

    // Get free voxels
    inline vec_Vecf<Dim> getFreeCloud(const vec_Vecf<3> &path, const std::vector<float> local_box_size) const
    {
      // Get cloud for free cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isFree(pti); }, path, local_box_size);
    }

    // Get free voxels for the entire map (useful for visualization)
    inline vec_Vecf<Dim> getFreeCloud() const
    {
      // Get cloud for free cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isFree(pti); });
    }

    // Get unknown voxels
    inline vec_Vecf<Dim> getUnknownCloud(const vec_Vecf<3> &path, const std::vector<float> local_box_size) const
    {
      // Get cloud for unknown cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isUnknown(pti); }, path, local_box_size);
    }

    // Get unknown voxels for the entire map (useful for visualization)
    inline vec_Vecf<Dim> getUnknownCloud() const
    {
      // Get cloud for unknown cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isUnknown(pti); });
    }

    // Get static occupied voxels with unknown as occupied for a specific region (useful for convex decomposition)
    inline vec_Vecf<Dim> getOccupiedCloudWithUnknownAsOccupied(const vec_Vecf<3> &path, const std::vector<float> local_box_size) const
    {
      // Get cloud for occupied cells
      return getCloud([this](const Veci<Dim> &pti) -> bool
                      { return isOccupied(pti) || isUnknown(pti); }, path, local_box_size);
    }

    // Get number of unknown cells
    int countUnknownCells() const
    {
      return std::count(map_.begin(), map_.end(), val_unknown_);
    }

    // Get number of occupied cells
    int countOccupiedCells() const
    {
      return std::count(map_.begin(), map_.end(), val_occ_);
    }

    // Get number of free cells
    int countFreeCells() const
    {
      return std::count(map_.begin(), map_.end(), val_free_);
    }

    // Get the total number of cells
    int getTotalNumCells() const
    {
      return total_size_;
    }

    // Octrees
    std::shared_ptr<octomap::TimedOcTree> lidar_octree_;
    std::shared_ptr<octomap::TimedOcTree> depth_camera_octree_;
    std::shared_ptr<octomap::TimedOcTree> roi_octree_;

    // Map entity
    Tmap map_;
    Tmap static_map_;
    Tmap previous_static_map_;
    Tmap uninflated_static_map_;
    Tmap dynamic_map_;

  protected:
    // Resolution
    decimal_t res_;
    // Octomap resolution
    decimal_t octomap_res_;
    // Total size of the map
    int total_size_ = 0;
    // Inflation
    float inflation_;
    // Use inflation
    bool use_inflation_ = true;
    // Inflation vector
    Eigen::Vector3d inflation_vec_;
    // Cell's inflation
    int cells_inflation_;
    // Free cells inflation
    int free_cells_inflation_;
    // Use free space
    bool use_free_space_ = true;
    // Origin, float type
    Vecf<Dim> origin_d_;
    // Center, float type
    Vecf<Dim> center_map_;
    // Dimension, int type
    Veci<Dim> dim_, prev_dim_;
    // Map values
    float x_map_min_, x_map_max_, y_map_min_, y_map_max_, z_map_min_, z_map_max_; 
    float x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    float prev_x_min_, prev_x_max_, prev_y_min_, prev_y_max_, prev_z_min_, prev_z_max_;
    // Cells size
    int cells_x_, cells_y_, cells_z_;
    // Assume occupied cell has value 100
    int8_t val_occ_ = 100;
    // Assume free cell has value 0
    int8_t val_free_ = 0;
    // Assume unknown cell has value -1
    int8_t val_unknown_ = -1;

    // Dynamic obstacle tracking
    float alpha_cov_ = 0.1;
    Eigen::Vector3d dynamic_obstacle_base_inflation_;
    Eigen::Vector3d max_dynamic_obstacle_inflation_;

    // Flags
    bool trajs_initialized_ = false;
    bool map_initialized_ = false;
    bool use_lidar_ = false;
    bool use_depth_camera_ = false;
    bool use_z_axis_bottom_inflation_ = false;

    // Node size factor
    float node_size_factor_for_occupied_;
    float node_size_factor_for_free_;

    // Dynamic trajectories
    std::vector<dynTraj> trajs_;

    // small map buffer
    float map_buffer_;
    Vec3f min_point_;
    Vec3f max_point_;
  };

  typedef MapUtil<3> VoxelMapUtil;

} // namespace dynus

#endif
