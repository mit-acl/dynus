#include "dgp/graph_search.hpp"
#include <cmath>

using namespace dynus;
using namespace termcolor;

typedef timer::Timer MyTimer;

GraphSearch::GraphSearch(const int* cMap, const std::shared_ptr<dynus::VoxelMapUtil>& map_util, int xDim, int yDim, int zDim, double eps, bool verbose, std::string global_planner) :
  cMap_(cMap), map_util_(map_util), xDim_(xDim), yDim_(yDim), zDim_(zDim), eps_(eps), verbose_(verbose), global_planner_(global_planner)
{
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  // Set 3D neighbors
  for(int x = -1; x <= 1; x ++) {
    for(int y = -1; y <= 1; y ++) {
      for(int z = -1; z <= 1; z ++) {
        if(x == 0 && y == 0 && z == 0) continue;
        ns_.push_back(std::vector<int>{x, y, z});
      }
    }
  }

  // Set neighbors (not including the diagonal neighbors)
  // ns_.push_back(std::vector<int>{1, 0, 0});
  // ns_.push_back(std::vector<int>{-1, 0, 0});
  // ns_.push_back(std::vector<int>{0, 1, 0});
  // ns_.push_back(std::vector<int>{0, -1, 0});
  // ns_.push_back(std::vector<int>{0, 0, 1});
  // ns_.push_back(std::vector<int>{0, 0, -1});

  jn3d_ = std::make_shared<JPS3DNeib>();
}

inline int GraphSearch::coordToId(int x, int y) const {
  return x + y*xDim_;
}

inline int GraphSearch::coordToId(int x, int y, int z) const {
  return x + y*xDim_ + z*xDim_*yDim_;
}

inline bool GraphSearch::isFree(int x, int y) const {
  // return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && cMap_[coordToId(x, y)] == val_free_;
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && cMap_[coordToId(x, y)] <= val_free_;
}

inline bool GraphSearch::isFree(int x, int y, int z) const {
  // return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ && cMap_[coordToId(x, y, z)] == val_free_;
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ && cMap_[coordToId(x, y, z)] <= val_free_;
}

// inline bool GraphSearch::isOccupied(const StatePtr &state) 
// {
//   if (Dim == 2)
//     return isOccupied(state->x, state->y);
//   else
//     return isOccupied(state->x, state->y, state->z);
// }

inline bool GraphSearch::isOccupied(int x, int y) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && cMap_[coordToId(x, y)] > val_free_;
}

inline bool GraphSearch::isOccupied(int x, int y, int z) const {
  return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ && cMap_[coordToId(x, y, z)] > val_free_;
}

inline double GraphSearch::getHeur(int x, int y) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_));
}

inline double GraphSearch::getHeur(int x, int y, int z) const {
  return eps_ * std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) + (z - zGoal_) * (z - zGoal_));
}

bool GraphSearch::plan(int xStart, int yStart, int zStart, int xGoal, int yGoal, int zGoal, double initial_g, double& global_planning_time, double& dgp_static_jps_time, double& dgp_check_path_time, double& dgp_dynamic_astar_time, double& dgp_recover_path_time, double current_time, const Vec3f& start_vel, int max_expand, int timeout_duration_ms)
{
  use_2d_ = false;
  pq_.clear();
  path_.clear();
  hm_.resize(xDim_ * yDim_ * zDim_);
  seen_.resize(xDim_ * yDim_ * zDim_, false);

  start_vel_ = start_vel;

  // Set goal
  int goal_id = coordToId(xGoal, yGoal, zGoal);
  xGoal_ = xGoal; yGoal_ = yGoal; zGoal_ = zGoal;
  // Set start node

  int start_id = coordToId(xStart, yStart, zStart);
  StatePtr currNode_ptr = std::make_shared<State>(State(start_id, xStart, yStart, zStart, 0, 0, 0, 0));

  currNode_ptr->g = initial_g;
  currNode_ptr->h = getHeur(xStart, yStart, zStart);

  // Set current time
  current_time_ = current_time;

  // Start a timer
  MyTimer global_planning_timer(true);
  
  // Run the planner
  bool result = select_planner(currNode_ptr, max_expand, start_id, goal_id, std::chrono::milliseconds(timeout_duration_ms));

  // Print the time taken [ms]
  global_planning_time_ = global_planning_timer.getElapsedMicros() / 1000.0;

  // Assign the time taken 
  global_planning_time = global_planning_time_;
  dgp_static_jps_time = dgp_static_jps_time_;
  dgp_check_path_time = dgp_check_path_time_;
  dgp_dynamic_astar_time = dgp_dynamic_astar_time_;
  dgp_recover_path_time = dgp_recover_path_time_;

  return result;
}

bool GraphSearch::select_planner(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id, std::chrono::milliseconds timeout_duration)
{
  
  if (global_planner_ == "sjps") // Static JPS planner
  { 
    return static_jps_plan(currNode_ptr, max_expand, start_id, goal_id, timeout_duration);
  }
  else if (global_planner_ == "dastar") // Dynamic A* planner
  {
    currNode_ptr->t = current_time_;
    return dynamic_astar_plan(currNode_ptr, max_expand, start_id, goal_id, timeout_duration);
  }
  else if (global_planner_ == "dgp") // Dynus Global Planenr
  {
    return dgp_plan(currNode_ptr, max_expand, start_id, goal_id, timeout_duration);
  }
  else // Error
    printf("Unknown planner: %s\n", global_planner_.c_str());
}

bool GraphSearch::static_jps_plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id, std::chrono::milliseconds timeout_duration)
{

  // Record the start time
  auto start_time = std::chrono::steady_clock::now();

  if (!currNode_ptr) 
  {
    std::cerr << "Error: currNode_ptr is null!" << std::endl;
    return false;
  }

  // Insert start node
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;

  int expand_iteration = 0;
  while(true)
  {
    expand_iteration++;

    // Check if timeout has occurred
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time) > timeout_duration) {
        std::cerr << "Timeout occurred in dynamic_astar_plan. Exiting safely.\n";
        return false;
    }

    if (pq_.empty()) 
    {
        std::cerr << "Error: Priority queue is empty!" << std::endl;
        return false;
    }

  
    // get element with smallest cost
    currNode_ptr = pq_.top(); 
    pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if(currNode_ptr->id == goal_id) 
    {
      if(verbose_) printf("Goal Reached!\n");
      break;
    }

    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    
    getJpsSucc(currNode_ptr, succ_ids, succ_costs);

    // Process successors
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      
      //see if we can improve the value of succstate
      StatePtr& child_ptr = hm_[succ_ids[s]];

      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;  // Assign new parent
        child_ptr->g = tentative_gval;    // Update gval

        //double fval = child_ptr->g + child_ptr->h;

        // if currently in OPEN, update
        if( child_ptr->opened && !child_ptr->closed) {
          pq_.increase( child_ptr->heapkey );       // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
           if(child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // if currently in CLOSED
        else if( child_ptr->opened && child_ptr->closed)
        {
          printf("STATIC JPS ERROR!\n");
        }
        else // new node, add to heap
        {
          //printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    } // Process successors

    if(max_expand > 0 && expand_iteration >= max_expand) {
      if(verbose_){
        printf("max_expandStep [%d] Reached\n\n", max_expand);
      }
      return false;
    }

  }

  if(verbose_) {
    // printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    // printf("Expand [%d] nodes!\n", expand_iteration);
  }

  path_ = recoverPath(currNode_ptr, start_id);
  updateGValues(); // this gives correct g values

  return true;
}

bool GraphSearch::dynamic_astar_plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id, std::chrono::milliseconds timeout_duration) 
{

  // Record the start time
  auto start_time = std::chrono::steady_clock::now();

  // Check null pointer
  if (!currNode_ptr) 
  {
    std::cerr << "[Dynamic A*] Error: currNode_ptr is null!" << std::endl;
    return false;
  }

  // Insert start node
  currNode_ptr->heapkey = pq_.push(currNode_ptr);
  currNode_ptr->opened = true;
  hm_[currNode_ptr->id] = currNode_ptr;
  seen_[currNode_ptr->id] = true;
  currNode_ptr->vel = start_vel_;

  int expand_iteration = 0;
  while(true)
  {

    expand_iteration++;

    // Check if timeout has occurred
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time) > timeout_duration) {
        std::cerr << "[Dynamic A*] Timeout occurred in dynamic_astar_plan. Exiting safely.\n";
        return false;
    }

    // get element with smallest cost
    if (pq_.empty()) {
        if(verbose_) printf("[Dynamic A*] Priority queue is empty\n\n");
        return false;
    }

    currNode_ptr = pq_.top(); pq_.pop();
    currNode_ptr->closed = true; // Add to closed list

    if(currNode_ptr->id == goal_id) {
      if(verbose_)
        printf("Goal Reached!\n");
      break;
    }

    // Update dynamic cMap
    updateDynamicCMap(currNode_ptr, start_id);

    // Check if the current node is occupied
    if (isOccupied(currNode_ptr->x, currNode_ptr->y, currNode_ptr->z)) {
      if(verbose_) printf("[Dynamic A*] Current node is occupied!\n");
      continue;
    }

    // Get successors
    std::vector<int> succ_ids;
    std::vector<double> succ_costs;
    getSucc(currNode_ptr, succ_ids, succ_costs); // this is getJpsSucc(currNode_ptr, succ_ids, succ_costs); in JPS

    // Process successors
    for( int s = 0; s < (int) succ_ids.size(); s++ )
    {
      //see if we can improve the value of succstate
      StatePtr& child_ptr = hm_[succ_ids[s]];

      double tentative_gval = currNode_ptr->g + succ_costs[s];

      if( tentative_gval < child_ptr->g )
      {
        child_ptr->parentId = currNode_ptr->id;  // Assign new parent
        child_ptr->g = tentative_gval;    // Update gval

        //double fval = child_ptr->g + child_ptr->h;

        // if currently in OPEN, update
        if( child_ptr->opened && !child_ptr->closed) {
          pq_.increase( child_ptr->heapkey );       // update heap
          child_ptr->dx = (child_ptr->x - currNode_ptr->x);
          child_ptr->dy = (child_ptr->y - currNode_ptr->y);
          child_ptr->dz = (child_ptr->z - currNode_ptr->z);
          if(child_ptr->dx != 0)
            child_ptr->dx /= std::abs(child_ptr->dx);
          if(child_ptr->dy != 0)
            child_ptr->dy /= std::abs(child_ptr->dy);
           if(child_ptr->dz != 0)
            child_ptr->dz /= std::abs(child_ptr->dz);
        }
        // if currently in CLOSED
        else if( child_ptr->opened && child_ptr->closed)
        {
          printf("[Dynamic A*] DYANMIC ASTAR ERROR!\n");
        }
        else // new node, add to heap
        {
          if(verbose_) printf("add to open set: %d, %d\n", child_ptr->x, child_ptr->y);
          child_ptr->heapkey = pq_.push(child_ptr);
          child_ptr->opened = true;
        }
      } //
    } // Process successors

    if(max_expand > 0 && expand_iteration >= max_expand) {
      // printf("[Dynamic A*] max_expandStep [%d] Reached!!!!!!\n\n", max_expand);
      return false;
    }

  }

  if(verbose_) 
  {
    printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
    printf("Expand [%d] nodes!\n", expand_iteration);
  }

  path_ = recoverPath(currNode_ptr, start_id);

  // print out the path
  for (int i = 0; i < path_.size(); i++)
  {
    Vecf<3> p = map_util_->intToFloat(Veci<3>(path_[i]->x, path_[i]->y, path_[i]->z));
    if(verbose_) printf("path[%d]: %f %f %f and g: %f\n", i, p(0), p(1), p(2), path_[i]->g);
  }

  updateGValues(); // this gives correct g values

  return true;
}

bool GraphSearch::dgp_plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id, std::chrono::milliseconds timeout_duration)
{

  // first run static jps without considering the dynamic obstacles at all
  cMap_ = (map_util_->static_map_).data();

  // Start a timer
  MyTimer dgp_static_jps_timer(true);

  // Run the static JPS planner
  bool static_jps_result = static_jps_plan(currNode_ptr, max_expand, start_id, goal_id, timeout_duration);

  // get the computation time of the static JPS
  dgp_static_jps_time_ = dgp_static_jps_timer.getElapsedMicros() / 1000.0;

  // if the static planner fails, return false
  if (static_jps_result)
  {
    if (verbose_)
      std::cout << bold << green << "Static JPS succeeded" << reset << std::endl;
  }
  else
  {
    if (verbose_)
      std::cout << bold << red << "Static JPS failed" << reset << std::endl;
    return false;
  }

  // simplify the path
  // path_ = removeLinePts(path_);
  // path_ = removeCornerPts(path_);
  // std::reverse(std::begin(path_), std::end(path_));
  // path_ = removeCornerPts(path_);
  // std::reverse(std::begin(path_), std::end(path_));

  // copy the path
  auto path_to_be_checked = path_;
  // get the path in the right order
  std::reverse(path_to_be_checked.begin(), path_to_be_checked.end());

  // This path might have nodes that are too far apart - we need to add intermediate nodes between them
  double max_dist = 0.5; // 0.5m
  std::vector<StatePtr> finer_path_to_be_checked;
  finer_path_to_be_checked.push_back(path_to_be_checked[0]); // add the first node

  for (int i = 0; i < path_to_be_checked.size() - 1; i++)
  {
    Vecf<3> p0 = map_util_->intToFloat(Veci<3>(path_to_be_checked[i]->x, path_to_be_checked[i]->y, path_to_be_checked[i]->z));
    Vecf<3> pf = map_util_->intToFloat(Veci<3>(path_to_be_checked[i + 1]->x, path_to_be_checked[i + 1]->y, path_to_be_checked[i + 1]->z));
    double dist = (pf - p0).norm();
    int parent_id = path_to_be_checked[i]->id;
    if (dist > max_dist)
    {
      // add intermediate nodes
      int num_intermediate_nodes = std::ceil(dist / max_dist);
      for (int j = 1; j < num_intermediate_nodes; j++)
      {
        Vecf<3> p = p0 + (pf - p0) * ((double)j / (double)num_intermediate_nodes);
        // convert to int
        Veci<3> p_int = map_util_->floatToInt(p);
        // Get int id
        int id = coordToId(p_int(0), p_int(1), p_int(2));
        StatePtr new_node_ptr = std::make_shared<State>(State(id, p_int(0), p_int(1), p_int(2), 0, 0, 0));
        new_node_ptr->g = path_to_be_checked[i]->g + (path_to_be_checked[i + 1]->g - path_to_be_checked[i]->g) * ((double)j / (double)num_intermediate_nodes);
        new_node_ptr->h = getHeur(p_int(0), p_int(1), p_int(2));
        new_node_ptr->t = path_to_be_checked[i]->t + (path_to_be_checked[i + 1]->t - path_to_be_checked[i]->t) * ((double)j / (double)num_intermediate_nodes);
        new_node_ptr->parentId = parent_id;
        finer_path_to_be_checked.push_back(new_node_ptr);
        parent_id = id; // update the parent id
      }
    }
    
    // add the next node
    Veci<3> p_int = Veci<3>(path_to_be_checked[i + 1]->x, path_to_be_checked[i + 1]->y, path_to_be_checked[i + 1]->z);
    int id = coordToId(p_int(0), p_int(1), p_int(2));
    StatePtr new_node_ptr = std::make_shared<State>(State(id, p_int(0), p_int(1), p_int(2), 0, 0, 0));
    new_node_ptr->g = path_to_be_checked[i + 1]->g;
    new_node_ptr->h = getHeur(p_int(0), p_int(1), p_int(2));
    new_node_ptr->t = path_to_be_checked[i + 1]->t;
    new_node_ptr->parentId = parent_id;
    finer_path_to_be_checked.push_back(new_node_ptr);
  }

  // add the last node
  finer_path_to_be_checked.push_back(path_to_be_checked.back());

  // update the path_to_be_checked
  path_to_be_checked.clear();
  path_to_be_checked = finer_path_to_be_checked;

  // check if the path doesn't collide with dynamic obstacles
  int iteration_count = 0;
  int max_while = 10;
  while (true)
  {

    // If the path has no length, return false
    if (path_to_be_checked.size() == 0) return false;

    if (++iteration_count >= max_while) 
    {
      std::cout << "Exceeded max iterations in dgp_plan." << std::endl;
      return false;
    }

    // check if the path doesn't collide with dynamic obstacles
    StatePtr new_start_ptr; int new_start_idx; StatePtr new_goal_ptr; int new_goal_idx;

    // Start a timer
    MyTimer dgp_check_path_timer(true);

    // check if the path doesn't collide with dynamic obstacles
    // Note: path_to_be_checked is updated in checkPathCollide
    double dynamic_a_star_start_time = 0.0;
    bool is_collision = checkPathCollide(path_to_be_checked, new_start_ptr, new_start_idx, new_goal_ptr, new_goal_idx, dynamic_a_star_start_time);

    // get the computation time of the checkPathCollide [ms]
    dgp_check_path_time_ += dgp_check_path_timer.getElapsedMicros() / 1000.0;

    // if the path doesn't collide with dynamic obstacles, return the path_to_be_checked
    if(!is_collision) {
      std::reverse(path_to_be_checked.begin(), path_to_be_checked.end()); // put the path in the original (goal to start) order
      path_ = path_to_be_checked; // update the path
      updateGValues(); // this gives correct g values
      if (verbose_) std::cout << bold << green << "Path doesn't collide with dynamic obstacles" << reset << std::endl;
      return true;
    }

    if (verbose_) printf("Path collides with dynamic obstacles\n");

    // Get the start and goal points in float
    Vecf<3> new_start = map_util_->intToFloat(Veci<3>(new_start_ptr->x, new_start_ptr->y, new_start_ptr->z));
    Vecf<3> new_goal = map_util_->intToFloat(Veci<3>(new_goal_ptr->x, new_goal_ptr->y, new_goal_ptr->z));

    // if the path collides with dynamic obstacles, replan using dyanmic A* and come back to the original path
    // get start_id and goal_id from the new_start_ptr and new_goal_ptr
    Veci<3> new_start_int = Veci<3>(new_start_ptr->x, new_start_ptr->y, new_start_ptr->z);
    Veci<3> new_goal_int = Veci<3>(new_goal_ptr->x, new_goal_ptr->y, new_goal_ptr->z);
    int new_start_id = new_start_ptr->id;
    int new_goal_id = new_goal_ptr->id;

    // reset the graph search
    pq_.clear(); path_.clear(); hm_.clear(); seen_.clear();
    hm_.resize(xDim_ * yDim_ * zDim_); seen_.resize(xDim_ * yDim_ * zDim_, false);

    // Set start and goal
    StatePtr new_curr_node_ptr = std::make_shared<State>(State(new_start_id, new_start_int(0), new_start_int(1), new_start_int(2), 0, 0, 0));
    new_curr_node_ptr->g = new_start_ptr->g;
    new_curr_node_ptr->h = getHeur(new_start_int(0), new_start_int(1), new_start_int(2));
    xGoal_ = new_goal_int(0); yGoal_ = new_goal_int(1); zGoal_ = new_goal_int(2);
    start_vel_ = Eigen::Vector3d(0.0, 0.0, 0.0); // reset the start velocity

    // Start a timer
    MyTimer dgp_dynamic_astar_timer(true);
    
    // Set the current time
    current_time_ = dynamic_a_star_start_time;
    new_curr_node_ptr->t = dynamic_a_star_start_time;

    if (verbose_)
    {
      std::cout << "plan using dynamic A*" << std::endl;
      std::cout << "max_expand: " << max_expand << std::endl;
      std::cout << "new_start_id: " << new_start_id << std::endl;
      std::cout << "new_goal_id: " << new_goal_id << std::endl;
      printf("current_time_: %f\n", current_time_);
    }

    if (!dynamic_astar_plan(new_curr_node_ptr, max_expand, new_start_id, new_goal_id, timeout_duration))
    {
      if (verbose_) std::cout << bold << red << "Dynamic A* failed" << reset << std::endl;
      return false;
    }

    if (verbose_)
      std::cout << bold << green << "Dynamic A* succeeded" << reset << std::endl;

    // get the computation time of the dynamic A* [ms]
    dgp_dynamic_astar_time_ += dgp_dynamic_astar_timer.getElapsedMicros() / 1000.0;

    // Start a timer
    MyTimer dgp_recover_path_timer(true);

    // if replan is successful, insert the new path to the original path (from new_start_ptr to new_goal_ptr)
    auto detoured_part = path_; // get the detoured part from dynamic A*

    // reverse the original path and detoured part
    std::reverse(detoured_part.begin(), detoured_part.end());

    // print out detoured part
    if (verbose_)
    {
      std::cout << "detoured part: " << std::endl;
      for (int i = 0; i < detoured_part.size(); i++)
      {
        Vecf<3> p = map_util_->intToFloat(Veci<3>(detoured_part[i]->x, detoured_part[i]->y, detoured_part[i]->z));
        printf("detoured part[%d]: %f %f %f and g: %f\n", i, p(0), p(1), p(2), detoured_part[i]->g);
      }
    }

    auto new_path = std::vector<StatePtr>(path_to_be_checked.begin(), path_to_be_checked.begin() + new_start_idx); // get the original path up to new_start_idx
    auto path_after_new_goal = std::vector<StatePtr>(path_to_be_checked.begin() + new_goal_idx + 1, path_to_be_checked.end()); // get the original path after new_goal_idx

    // update g values of the path_after_new_goal
    for (int i = 0; i < path_after_new_goal.size(); i++)
    {
      Vecf<3> p1;
      if (i == 0){
        p1 = Vecf<3>(detoured_part[detoured_part.size() - 1]->x, detoured_part[detoured_part.size() - 1]->y, detoured_part[detoured_part.size() - 1]->z);
        Vecf<3> p2 = Vecf<3>(path_after_new_goal[i]->x, path_after_new_goal[i]->y, path_after_new_goal[i]->z);
        path_after_new_goal[i]->g = detoured_part[detoured_part.size() - 1]->g + (p2 - p1).norm();
      }
      else
      {
        p1 = Vecf<3>(path_after_new_goal[i-1]->x, path_after_new_goal[i-1]->y, path_after_new_goal[i-1]->z);
        Vecf<3> p2 = Vecf<3>(path_after_new_goal[i]->x, path_after_new_goal[i]->y, path_after_new_goal[i]->z);
        path_after_new_goal[i]->g = path_after_new_goal[i - 1]->g + (p2 - p1).norm();
      }
    }

    if (verbose_)
    {
      std::cout << "new_path size: " << new_path.size() << std::endl;
      std::cout << "detoured_part size: " << detoured_part.size() << std::endl;
      std::cout << "path_after_new_goal size: " << path_after_new_goal.size() << std::endl;
    }

    new_path.insert(new_path.end(), detoured_part.begin(), detoured_part.end());                 // add the detoured new path
    new_path.insert(new_path.end(), path_after_new_goal.begin(), path_after_new_goal.end());     // add the original path after new_goal_idx

    // get the computation time of the recoverPath [ms]
    dgp_recover_path_time_ += dgp_recover_path_timer.getElapsedMicros() / 1000.0;

    // update the path_to_be_checked
    path_to_be_checked = new_path;

    if (verbose_)
    {
      std::cout << "Path length: " << path_to_be_checked.size() << std::endl;
      std::cout << "new path: " << std::endl;
      for (int i = 0; i < path_to_be_checked.size(); i++)
      {
        Vecf<3> p = map_util_->intToFloat(Veci<3>(path_to_be_checked[i]->x, path_to_be_checked[i]->y, path_to_be_checked[i]->z));
        printf("new path[%d]: %f %f %f and g: %f\n", i, p(0), p(1), p(2), path_to_be_checked[i]->g);
      }
    }

  } // End of while loop

}

std::vector<StatePtr> GraphSearch::removeLinePts(const std::vector<StatePtr>& path)
{
  if (path.size() < 3)
    return path;

  std::vector<StatePtr> new_path;
  new_path.push_back(path.front());
  for (unsigned int i = 1; i < path.size() - 1; i++)
  {
    Vecf<3> path_i_p_1 = Vecf<3>(path[i + 1]->x, path[i + 1]->y, path[i + 1]->z);
    Vecf<3> path_i = Vecf<3>(path[i]->x, path[i]->y, path[i]->z);
    Vecf<3> path_i_m_1 = Vecf<3>(path[i - 1]->x, path[i - 1]->y, path[i - 1]->z);
    Vecf<3> p = (path_i_p_1 - path_i) - (path_i - path_i_m_1);
    if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
    {
      new_path.push_back(path[i]);
    }
  }
  new_path.push_back(path.back());
  return new_path;
}

std::vector<StatePtr> GraphSearch::removeCornerPts(const std::vector<StatePtr> &path)
{
  if (path.size() < 2)
    return path;

  // cut zigzag segment
  std::vector<StatePtr> optimized_path;
  Vecf<3> pose1 = Vecf<3>(path[0]->x, path[0]->y, path[0]->z);
  Vecf<3> pose2 = Vecf<3>(path[1]->x, path[1]->y, path[1]->z);
  Vecf<3> prev_pose = pose1;
  optimized_path.push_back(path[0]);
  decimal_t cost1, cost2, cost3;

  if (!map_util_->isBlocked(pose1, pose2))
    cost1 = (pose1 - pose2).norm();
  else
    cost1 = std::numeric_limits<decimal_t>::infinity();

  for (unsigned int i = 1; i < path.size() - 1; i++)
  {
    pose1 = Vecf<3>(path[i]->x, path[i]->y, path[i]->z);
    pose2 = Vecf<3>(path[i + 1]->x, path[i + 1]->y, path[i + 1]->z);
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

bool GraphSearch::checkPathCollide(std::vector<StatePtr>& path_int, StatePtr& new_start_ptr, int& new_start_idx, StatePtr& new_goal_ptr, int& new_goal_idx, double& dynamic_a_star_start_time)
{

  // interpolate the path
  // interpolatePath(path_int);

  // flags
  bool is_collission_detected = false;

  // compute float values of the path
  vec_Vecf<3> path_float;
  for (int i = 0; i < path_int.size(); i++)
  {
    path_float.push_back(map_util_->intToFloat(Veci<3>(path_int[i]->x, path_int[i]->y, path_int[i]->z)));
  }

  // Initialize start state
  state s;
  s.setState(start_, start_vel_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  // compute travel times
  std::vector<double> travel_times = dynus_utils::getTravelTimes(path_float, s, false, v_max_3d_, a_max_3d_);

  // Generate accumulated travel times
  std::vector<double> accumulated_travel_times;
  accumulated_travel_times.push_back(0.0); // Add the first state's travel time
  for (size_t i = 0; i < travel_times.size(); i++)
  {
      accumulated_travel_times.push_back(accumulated_travel_times.back() + travel_times[i]);
  }

  // Sanity check
  assert(path_int.size() == path_float.size());
  assert(path_int.size() == travel_times.size() + 1); // travel_times do not include the first state's travel time (because it is 0)
  assert(path_int.size() == accumulated_travel_times.size());

  // check if the path collides with the map
  for (int i = 1; i < accumulated_travel_times.size(); i++) // The first point is the start point so skip it
  {

    // update the map
    map_util_->updateMap(current_time_ + accumulated_travel_times[i], accumulated_travel_times[i]);

    // update the cMap in graph search
    cMap_ = (map_util_->map_).data();

    // check if the path_int collides with the map
    if (isOccupied(path_int[i]->x, path_int[i]->y, path_int[i]->z) && !is_collission_detected)
    {
      
      if(verbose_)
      {
        Vecf<3> temp = map_util_->intToFloat(Veci<3>(path_int[i]->x, path_int[i]->y, path_int[i]->z));
        printf("collision detected at id %d, (%f, %f, %f)\n", path_int[i]->id, temp(0), temp(1), temp(2));
        printf("i: %d\n", i);
      }

      // Deep copy the state and set the new_start_ptr and new_goal_ptr
      new_start_ptr = path_int[i - 1];
      new_start_idx = i - 1;
      dynamic_a_star_start_time = current_time_ + accumulated_travel_times[i - 1];
      is_collission_detected = true;
    }

    // find the tmp goal when collision is detected
    if (is_collission_detected && !isOccupied(path_int[i] -> x, path_int[i] -> y, path_int[i] -> z)) // instead of !isOccupied, we could use isFree()?
    // if (is_collission_detected && isFree(path_int[i] -> x, path_int[i] -> y, path_int[i] -> z)) // instead of !isOccupied, we could use isFree()?
    {
      
      if(verbose_)
      {
        Vecf<3> temp = map_util_->intToFloat(Veci<3>(path_int[i]->x, path_int[i]->y, path_int[i]->z));
        printf("collision free at id %d, (%f, %f, %f)\n", path_int[i]->id, temp(0), temp(1), temp(2));
        printf("i: %d\n", i);
      }

      new_goal_ptr = path_int[i];
      new_goal_idx = i;
      return true;
    }
  }

  return false;
}

//// RIGHT NOW THE JPS GIVES BACK A VERY SPARSE PATH, SO TO CHECK COLLISION AGAINST DYNAMIC OBSTACLES, WE NEED TO INTERPOLATE THE PATH
void GraphSearch::updateGValues()
{

  // first reverse the path
  std::reverse(path_.begin(), path_.end());

  // first create a path with float values
  std::vector<Vecf<3>> path_float;
  for (int i = 0; i < path_.size(); i++)
  {
    path_float.push_back(map_util_->intToFloat(Veci<3>(path_[i]->x, path_[i]->y, path_[i]->z)));
  }

  // convert the path back to int and update g values
  std::vector<StatePtr> new_path_int;
  for (int i = 0; i < path_float.size(); i++)
  {
    Veci<3> temp = map_util_->floatToInt(Vecf<3>(path_float[i](0), path_float[i](1), path_float[i](2)));
    StatePtr temp_ptr = std::make_shared<State>(State(coordToId(temp(0), temp(1), temp(2)), temp(0), temp(1), temp(2), 0, 0, 0)); // for now dx, dy, dz are 0 (TODO: this could be problematic?)
    
    // compute total path length and update g
    if (i > 0)
    {
      Vecf<3> p1 = path_float[i - 1];
      Vecf<3> p2 = path_float[i];
      temp_ptr->g = new_path_int[i - 1]->g + (p2 - p1).norm();
    }
    else
    {
      temp_ptr->g = path_[0]->g;
    }
    
    new_path_int.push_back(temp_ptr);
  }

  // update the path
  path_ = new_path_int;

  // reverse the path
  std::reverse(path_.begin(), path_.end());

}

//// RIGHT NOW THE JPS GIVES BACK A VERY SPARSE PATH, SO TO CHECK COLLISION AGAINST DYNAMIC OBSTACLES, WE NEED TO INTERPOLATE THE PATH
void GraphSearch::interpolatePath(std::vector<StatePtr>& path)
{

  // If path's size is 1, then return
  if (path.size() == 1)
  {
    return;
  }

  // first create a path with float values
  std::vector<Vecf<3>> path_float;
  for (int i = 0; i < path.size(); i++)
  {
    path_float.push_back(map_util_->intToFloat(Veci<3>(path[i]->x, path[i]->y, path[i]->z)));
  }

  // interpolate the path
  std::vector<Vecf<3>> interpolated_path_float;

  // std::cout << "path_float.size(): " << path_float.size() << std::endl;

  for (int i = 0; i < path_float.size() - 1; i++)
  {

      double num_interpolations = (path_float[i + 1] - path_float[i]).norm();
      Vec3f path_step = (path_float[i + 1] - path_float[i]) / num_interpolations;
      for (int j = 0; j < num_interpolations; j++)
      {
          interpolated_path_float.push_back(path_float[i] + j * path_step);
      }

      // add the last point
      if (i == path_float.size() - 2)
      {
          interpolated_path_float.push_back(path_float[path_float.size() - 1]);
          break;
      }
  }

  // convert the path back to int
  std::vector<StatePtr> new_path_int;

  // std::cout << "interpolated_path_float.size(): " << interpolated_path_float.size() << std::endl;

  // to check if the last point is the same as the last_node_int
  Veci<3> last_node_int = map_util_->floatToInt(Vecf<3>(interpolated_path_float[0](0), interpolated_path_float[0](1), interpolated_path_float[0](2)));

  for (int i = 0; i < interpolated_path_float.size(); i++)
  {
    Veci<3> temp = map_util_->floatToInt(Vecf<3>(interpolated_path_float[i](0), interpolated_path_float[i](1), interpolated_path_float[i](2)));
    
    // if p1 and p2 are the same as the last point, don't add it (due to interpolation this could happen)
    if (i > 0 && temp == last_node_int)
    {
      continue;
    }
    
    StatePtr temp_ptr = std::make_shared<State>(State(coordToId(temp(0), temp(1), temp(2)), temp(0), temp(1), temp(2), 0, 0, 0)); // for now dx, dy, dz are 0 (TODO: this could be problematic?)
    
    // compute total path length and update g
    if (i > 0)
    {
      Vecf<3> p1 = interpolated_path_float[i - 1];
      Vecf<3> p2 = interpolated_path_float[i];
      temp_ptr->g = new_path_int.back()->g + (p2 - p1).norm();
    }
    else
    {
      temp_ptr->g = path[0]->g;
    }

    new_path_int.push_back(temp_ptr);

    // update the last_node_int
    last_node_int = temp;
  }

  // update the path
  path = new_path_int;

}

// Update dynamic cMap
void GraphSearch::updateDynamicCMap(StatePtr& currNode_ptr, const int& start_id) 
{

    // 1. get the obstacle position from double integrator estimator
    
    // if the current node is the start node, then we will use node_time = 0
    if (currNode_ptr->id == start_id)
    {
      // update the map
      map_util_->updateMap(current_time_, 0.0);
      // update the cMap in graph search
      cMap_ = (map_util_->map_).data();
      return;
    }

    // First we need to get the previous node
    StatePtr prevNode_ptr = hm_[currNode_ptr->parentId];

    // Get the travel time from the previous node to the current node

    // get p0 from prevNode_ptr
    Vecf<3> p0_vecf3 = map_util_->intToFloat(Veci<3>(prevNode_ptr->x, prevNode_ptr->y, prevNode_ptr->z));
    Eigen::Vector3d p0 = Eigen::Vector3d(p0_vecf3(0), p0_vecf3(1), p0_vecf3(2));

    // get v0 
    Eigen::Vector3d v0 = prevNode_ptr->vel;

    // get pf from currNode_ptr
    Vecf<3> pf_vecf3 = map_util_->intToFloat(Veci<3>(currNode_ptr->x, currNode_ptr->y, currNode_ptr->z));
    Eigen::Vector3d pf = Eigen::Vector3d(pf_vecf3(0), pf_vecf3(1), pf_vecf3(2));

    // compute node time
    double node_time_from_prev_to_current = computeNodeTime(pf, p0, v0, currNode_ptr);

    // // get the time to reach the goal
    // double node_time = dynus_utils::getMinTimeDoubleIntegrator3D(p0, v0, pf, vf, v_max_3d_, a_max_3d_);

    // update travel time
    currNode_ptr->t = prevNode_ptr->t + node_time_from_prev_to_current;

    // update the map
    map_util_->updateMap(current_time_ + currNode_ptr->t, currNode_ptr->t);

    // update the cMap in graph search
    cMap_ = (map_util_->map_).data();
}

/// Compute the node time
double GraphSearch::computeNodeTime(const Eigen::Vector3d& pf,
                                    const Eigen::Vector3d& p0,
                                    const Eigen::Vector3d& v0,
                                    StatePtr& currNode_ptr)
{
  // Initialize final velocity vector (for each axis)
  Eigen::Vector3d vf = Eigen::Vector3d::Zero();

  // Absolute displacement between nodes
  Eigen::Vector3d dist = (pf - p0).cwiseAbs();

  // Will store travel time for each dimension; the overall node time is the max.
  std::vector<double> node_times;

  // Small tolerance for checking if v0 is nearly equal to v_max.
  double epsilon = 1e-6;

  // Iterate over the dimensions (x, y, z)
  for (int i = 0; i < 3; i++)
  {
    double a_max = a_max_3d_(i);       // maximum acceleration in this dimension
    double v_max = v_max_3d_(i);       // maximum velocity in this dimension (assumed defined)
    double delta = pf(i) - p0(i);      // signed displacement along axis i
    double node_time = 0;              // time to traverse this dimension

    // std::cout << "----- Dimension " << i << " -----" << std::endl;
    // std::cout << "delta = " << delta << ", dist = " << dist(i) << std::endl;
    
    // If delta is very small, we can assume no motion is needed.
    if (std::abs(delta) < epsilon)
    {
      // std::cout << "Delta is negligible, no motion needed." << std::endl;
      node_time = 0;
      vf(i) = v0(i);
    }
    else if (delta > 0)  // Desired displacement is positive.
    {
      // std::cout << "Positive displacement branch." << std::endl;
      if (v0(i) >= 0)  // Already moving in the positive direction
      {
        // std::cout << "v0(" << i << ") > 0: v0 = " << v0(i) << std::endl;
        // If already cruising (and no deceleration is needed)
        if (v0(i) >= (v_max - epsilon))
        {
          // std::cout << "Already cruising: v0(" << i << ") = " << v0(i) << " >= v_max - epsilon (" << v_max - epsilon << ")" << std::endl;
          vf(i) = v_max;
          node_time = dist(i) / v_max;
          // std::cout << "Computed node_time (cruise) = " << node_time << std::endl;
        }
        else
        {
          double v_candidate = std::sqrt(v0(i) * v0(i) + 2 * a_max * dist(i));
          // std::cout << "Computed v_candidate = " << v_candidate << std::endl;
          if (v_candidate <= v_max)
          {
            // std::cout << "v_candidate <= v_max" << std::endl;
            vf(i) = v_candidate;
            node_time = (vf(i) - v0(i)) / a_max;
            // std::cout << "Computed node_time (acceleration only) = " << node_time << std::endl;
          }
          else
          {
            // std::cout << "v_candidate > v_max, splitting into acceleration and cruise phases." << std::endl;
            double d_accel = (v_max * v_max - v0(i) * v0(i)) / (2 * a_max);
            double t_accel = (v_max - v0(i)) / a_max;
            // std::cout << "d_accel = " << d_accel << ", t_accel = " << t_accel << std::endl;
            if (dist(i) > d_accel)
            {
              double d_cruise = dist(i) - d_accel;
              double t_cruise = d_cruise / v_max;
              node_time = t_accel + t_cruise;
              // std::cout << "d_cruise = " << d_cruise << ", t_cruise = " << t_cruise << std::endl;
              // std::cout << "Total node_time = " << node_time << std::endl;
            }
            else
            {
              // Fallback if numerical issues arise.
              // std::cout << "Fallback branch: dist <= d_accel." << std::endl;
              node_time = (std::sqrt(v0(i) * v0(i) + 2 * a_max * dist(i)) - v0(i)) / a_max;
              // std::cout << "Computed fallback node_time = " << node_time << std::endl;
            }
            vf(i) = v_max;
          }
        }
      }
      else  // v0(i) < 0: initially moving in the wrong (or zero) direction.
      {
        // std::cout << "v0(" << i << ") <= 0: initial velocity = " << v0(i) << " (wrong direction or stationary)" << std::endl;
        double d_stop = (v0(i) * v0(i)) / (2 * a_max);
        double t_stop = (-v0(i)) / a_max;
        // std::cout << "d_stop = " << d_stop << ", t_stop = " << t_stop << std::endl;
        double d_accel = dist(i) + d_stop;
        double v_candidate = std::sqrt(2 * a_max * d_accel);
        // std::cout << "Phase 2: d_accel = " << d_accel << ", v_candidate = " << v_candidate << std::endl;
        if (v_candidate <= v_max)
        {
          double t_accel = std::sqrt(2 * d_accel / a_max);
          node_time = t_stop + t_accel;
          vf(i) = v_candidate;
          // std::cout << "Computed node_time (decelerate then accelerate) = " << node_time << std::endl;
        }
        else
        {
          // std::cout << "v_candidate > v_max in deceleration branch, splitting phases." << std::endl;
          double d_to_vmax = (v_max * v_max) / (2 * a_max);
          double t_accel = v_max / a_max;
          // std::cout << "d_to_vmax = " << d_to_vmax << ", t_accel = " << t_accel << std::endl;
          if (d_accel > d_to_vmax)
          {
            double d_cruise = d_accel - d_to_vmax;
            double t_cruise = d_cruise / v_max;
            node_time = t_stop + t_accel + t_cruise;
            // std::cout << "d_cruise = " << d_cruise << ", t_cruise = " << t_cruise << std::endl;
          }
          else
          {
            node_time = t_stop + t_accel;
          }
          vf(i) = v_max;
          // std::cout << "Total node_time = " << node_time << std::endl;
        }
      }
    }
    else  // delta <= 0: Desired displacement is negative.
    {
      // std::cout << "Negative displacement branch." << std::endl;
      if (v0(i) <= 0)  // Already moving in the negative direction.
      {
        // std::cout << "v0(" << i << ") < 0: already moving negative, v0 = " << v0(i) << std::endl;
        if (std::fabs(v0(i)) >= (v_max - epsilon))
        {
          // std::cout << "Already cruising in the negative direction: |v0| >= v_max - epsilon" << std::endl;
          vf(i) = -v_max;
          node_time = dist(i) / v_max;
          // std::cout << "Computed node_time (cruise negative) = " << node_time << std::endl;
        }
        else
        {
          double v_candidate = std::sqrt(v0(i) * v0(i) + 2 * a_max * dist(i));
          v_candidate = -v_candidate;  // assign the negative sign
          // std::cout << "Computed negative v_candidate = " << v_candidate << std::endl;
          if (std::fabs(v_candidate) <= v_max)
          {
            vf(i) = v_candidate;
            node_time = (std::fabs(vf(i)) - std::fabs(v0(i))) / a_max;
            // std::cout << "Computed node_time (negative acceleration only) = " << node_time << std::endl;
          }
          else
          {
            // std::cout << "v_candidate exceeds v_max in negative branch, splitting phases." << std::endl;
            double d_accel = ((v_max * v_max) - (v0(i) * v0(i))) / (2 * a_max);
            double t_accel = (v_max - std::fabs(v0(i))) / a_max;
            // std::cout << "d_accel = " << d_accel << ", t_accel = " << t_accel << std::endl;
            if (dist(i) > d_accel)
            {
              double d_cruise = dist(i) - d_accel;
              double t_cruise = d_cruise / v_max;
              node_time = t_accel + t_cruise;
              // std::cout << "d_cruise = " << d_cruise << ", t_cruise = " << t_cruise << std::endl;
            }
            else
            {
              node_time = (std::fabs(std::sqrt(v0(i) * v0(i) + 2 * a_max * dist(i))) - std::fabs(v0(i))) / a_max;
              // std::cout << "Computed fallback node_time (negative) = " << node_time << std::endl;
            }
            vf(i) = -v_max;
          }
        }
      }
      else  // v0(i) > 0: initially moving in the positive direction while desired displacement is negative.
      {
        // std::cout << "v0(" << i << ") >= 0: Need to decelerate and reverse, v0 = " << v0(i) << std::endl;
        double d_stop = (v0(i) * v0(i)) / (2 * a_max);
        double t_stop = v0(i) / a_max;
        // std::cout << "d_stop = " << d_stop << ", t_stop = " << t_stop << std::endl;
        double d_accel = dist(i) + d_stop;
        double v_candidate = std::sqrt(2 * a_max * d_accel);
        v_candidate = -v_candidate;  // final velocity is negative.
        // std::cout << "d_accel = " << d_accel << ", negative v_candidate = " << v_candidate << std::endl;
        if (std::fabs(v_candidate) <= v_max)
        {
          double t_accel = std::sqrt(2 * d_accel / a_max);
          node_time = t_stop + t_accel;
          vf(i) = v_candidate;
          // std::cout << "Computed node_time (decelerate then reverse) = " << node_time << std::endl;
        }
        else
        {
          // std::cout << "v_candidate exceeds v_max in reversal branch, splitting phases." << std::endl;
          double d_to_vmax = (v_max * v_max) / (2 * a_max);
          double t_accel = v_max / a_max;
          // std::cout << "d_to_vmax = " << d_to_vmax << ", t_accel = " << t_accel << std::endl;
          if (d_accel > d_to_vmax)
          {
            double d_cruise = d_accel - d_to_vmax;
            double t_cruise = d_cruise / v_max;
            node_time = t_stop + t_accel + t_cruise;
            // std::cout << "d_cruise = " << d_cruise << ", t_cruise = " << t_cruise << std::endl;
          }
          else
          {
            node_time = t_stop + t_accel;
          }
          vf(i) = -v_max;
          // std::cout << "Computed total node_time = " << node_time << std::endl;
        }
      }
    }

    // Save the computed time for this axis.
    node_times.push_back(node_time);
    // std::cout << "Final node_time for dimension " << i << " = " << node_time << std::endl;
    // std::cout << "Final velocity for dimension " << i << " = " << vf(i) << std::endl;
  } // end for loop over dimensions

  // Update the current node's velocity with the computed final velocity vector.
  currNode_ptr->vel = vf;

  // Return the overall node time (the maximum among x, y, and z dimensions)
  double overall_time = *std::max_element(node_times.begin(), node_times.end());
  // std::cout << "Overall node time: " << overall_time << std::endl;
  return overall_time;
}


void GraphSearch::setStartAndGoal(const Vecf<3>& start, const Vecf<3>& goal) 
{
  start_ = start;
  goal_ = goal;
}

void GraphSearch::setBounds(double max_values[3])
{
  v_max_ = max_values[0];
  v_max_3d_ << max_values[0], max_values[0], max_values[0];
  a_max_ = max_values[1];
  a_max_3d_ << max_values[1], max_values[1], max_values[1];
  j_max_ = max_values[2];
  j_max_3d_ << max_values[2], max_values[2], max_values[2];
}


std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id) {
  std::vector<StatePtr> path;
  path.push_back(node);
  while(node && node->id != start_id) {
    node = hm_[node->parentId];
    path.push_back(node);
  }

  return path;
}

void GraphSearch::getSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    for(const auto& d: ns_) {
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      if(!isFree(new_x, new_y))
        continue;

      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, d[0], d[1]);
        hm_[new_id]->h = getHeur(new_x, new_y);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]));
    }
  }
  else {
    for(const auto& d: ns_) {
      int new_x = curr->x + d[0];
      int new_y = curr->y + d[1];
      int new_z = curr->z + d[2];
      if(!isFree(new_x, new_y, new_z))
        continue;

      int new_id = coordToId(new_x, new_y, new_z);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, d[0], d[1], d[2]);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]));
    }
  }
}

void GraphSearch::getJpsSucc(const StatePtr& curr, std::vector<int>& succ_ids, std::vector<double>& succ_costs) {
  if(use_2d_) {
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy);
    int num_neib = jn2d_->nsz[norm1][0];
    int num_fneib = jn2d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1);

    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y;
      int dx, dy;
      if(dev < num_neib) {
        dx = jn2d_->ns[id][0][dev];
        dy = jn2d_->ns[id][1][dev];
        if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
      }
      else {
        int nx = curr->x + jn2d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn2d_->f1[id][1][dev-num_neib];
        if(isOccupied(nx,ny)) {
          dx = jn2d_->f2[id][0][dev-num_neib];
          dy = jn2d_->f2[id][1][dev-num_neib];
          if(!jump(curr->x, curr->y, dx, dy, new_x, new_y)) continue;
        }
        else
          continue;
      }

      int new_id = coordToId(new_x, new_y);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, dx, dy);
        hm_[new_id]->h = getHeur(new_x, new_y);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y)));
    }
  }
  else {
    const int norm1 = std::abs(curr->dx)+std::abs(curr->dy)+std::abs(curr->dz);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int id = (curr->dx+1)+3*(curr->dy+1)+9*(curr->dz+1);

    for( int dev = 0; dev < num_neib+num_fneib; ++dev) {
      int new_x, new_y, new_z;
      int dx, dy, dz;
      if(dev < num_neib) {
        dx = jn3d_->ns[id][0][dev];
        dy = jn3d_->ns[id][1][dev];
        dz = jn3d_->ns[id][2][dev];
        if(!jump(curr->x, curr->y, curr->z,
              dx, dy, dz, new_x, new_y, new_z)) continue;
      }
      else {
        int nx = curr->x + jn3d_->f1[id][0][dev-num_neib];
        int ny = curr->y + jn3d_->f1[id][1][dev-num_neib];
        int nz = curr->z + jn3d_->f1[id][2][dev-num_neib];
        if(isOccupied(nx,ny,nz)) {
          dx = jn3d_->f2[id][0][dev-num_neib];
          dy = jn3d_->f2[id][1][dev-num_neib];
          dz = jn3d_->f2[id][2][dev-num_neib];
          if(!jump(curr->x, curr->y, curr->z,
                dx, dy, dz, new_x, new_y, new_z)) continue;
        }
        else
          continue;
      }

      int new_id = coordToId(new_x, new_y, new_z);
      if(!seen_[new_id]) {
        seen_[new_id] = true;
        hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y, new_z, dx, dy, dz);
        hm_[new_id]->h = getHeur(new_x, new_y, new_z);
      }

      succ_ids.push_back(new_id);
      succ_costs.push_back(std::sqrt((new_x - curr->x) * (new_x - curr->x) +
            (new_y - curr->y) * (new_y - curr->y) +
            (new_z - curr->z) * (new_z - curr->z)));
    }

  }
}


bool GraphSearch::jump(int x, int y, int dx, int dy, int& new_x, int& new_y ) {
  new_x = x + dx;
  new_y = y + dy;
  if (!isFree(new_x, new_y))
    return false;

  if (new_x ==  xGoal_ && new_y == yGoal_)
    return true;


  if (hasForced(new_x, new_y, dx, dy))
    return true;

  const int id = (dx+1)+3*(dy+1);
  const int norm1 = std::abs(dx) + std::abs(dy);
  int num_neib = jn2d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y;
    if(jump(new_x, new_y, jn2d_->ns[id][0][k], jn2d_->ns[id][1][k],
        new_new_x, new_new_y)) return true;
  }

  return jump(new_x, new_y, dx, dy, new_x, new_y);
}


bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z) {
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;
  if (!isFree(new_x, new_y, new_z))
    return false;

  if (new_x ==  xGoal_ && new_y == yGoal_ && new_z == zGoal_)
    return true;

  if (hasForced(new_x, new_y, new_z, dx, dy, dz))
    return true;

  const int id = (dx+1)+3*(dy+1)+9*(dz+1);
  const int norm1 = std::abs(dx) + std::abs(dy) +std::abs(dz);
  int num_neib = jn3d_->nsz[norm1][0];
  for( int k = 0; k < num_neib-1; ++k )
  {
    int new_new_x, new_new_y, new_new_z;
    if(jump(new_x,new_y,new_z,
          jn3d_->ns[id][0][k], jn3d_->ns[id][1][k], jn3d_->ns[id][2][k],
        new_new_x, new_new_y, new_new_z)) return true;
  }

  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}

inline bool GraphSearch::hasForced(int x, int y, int dx, int dy) {
  const int id = (dx+1)+3*(dy+1);
  for( int fn = 0; fn < 2; ++fn )
  {
    int nx = x + jn2d_->f1[id][0][fn];
    int ny = y + jn2d_->f1[id][1][fn];
    if( isOccupied(nx,ny) )
      return true;
  }
  return false;
}


inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz) {
  int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
  int id = (dx+1)+3*(dy+1)+9*(dz+1);
  switch(norm1)
  {
    case 1:
      // 1-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 2:
      // 2-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    case 3:
      // 3-d move, check 6 neighbors
      for( int fn = 0; fn < 6; ++fn )
      {
        int nx = x + jn3d_->f1[id][0][fn];
        int ny = y + jn3d_->f1[id][1][fn];
        int nz = z + jn3d_->f1[id][2][fn];
        if( isOccupied(nx,ny,nz) )
          return true;
      }
      return false;
    default:
      return false;
  }
}


std::vector<StatePtr> GraphSearch::getPath() const {
  return path_;
}

std::vector<StatePtr> GraphSearch::getOpenSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->opened && !it->closed)
      ss.push_back(it);
  }
  return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it && it->closed)
      ss.push_back(it);
  }
  return ss;
}


std::vector<StatePtr> GraphSearch::getAllSet() const {
  std::vector<StatePtr> ss;
  for(const auto& it: hm_) {
    if(it)
      ss.push_back(it);
  }
  return ss;
}

constexpr int JPS2DNeib::nsz[3][2];

JPS2DNeib::JPS2DNeib() {
  int id = 0;
  for(int dy = -1; dy <= 1; ++dy) {
    for(int dx = -1; dx <= 1; ++dx) {
      int norm1 = std::abs(dx) + std::abs(dy);
      for(int dev = 0; dev < nsz[norm1][0]; ++dev)
        Neib(dx,dy,norm1,dev, ns[id][0][dev], ns[id][1][dev]);
      for(int dev = 0; dev < nsz[norm1][1]; ++dev)
      {
        FNeib(dx,dy,norm1,dev,
            f1[id][0][dev],f1[id][1][dev],
            f2[id][0][dev],f2[id][1][dev]);
      }
      id ++;
    }
  }
}

void JPS2DNeib::print() {
  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      int id = (dx+1)+3*(dy+1);
      printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
      for(unsigned int i = 0; i < sizeof(f1[id][0])/sizeof(f1[id][0][0]); i++)
        printf("                f1: [%d, %d]\n", f1[id][0][i], f1[id][1][i]);
    }
  }
}

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; return;
        case 1: tx=-1; ty=0; return;
        case 2: tx=0; ty=1; return;
        case 3: tx=1; ty=1; return;
        case 4: tx=-1; ty=1; return;
        case 5: tx=0; ty=-1; return;
        case 6: tx=1; ty=-1; return;
        case 7: tx=-1; ty=-1; return;
     }
    case 1:
      tx = dx; ty = dy; return;
    case 2:
      switch(dev)
      {
        case 0: tx = dx; ty = 0; return;
        case 1: tx = 0; ty = dy; return;
        case 2: tx = dx; ty = dy; return;
      }
  }
}

void JPS2DNeib::FNeib( int dx, int dy, int norm1, int dev,
                       int& fx, int& fy, int& nx, int& ny)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; break;
        case 1: fx= 0; fy= -1;  break;
      }

      // switch order if different direction
      if(dx == 0)
        fx = fy, fy = 0;

      nx = dx + fx; ny = dy + fy;
      return;
    case 2:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0;
          nx = -dx; ny = dy;
          return;
        case 1:
          fx = 0; fy = -dy;
          nx = dx; ny = -dy;
          return;
      }
  }
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib() {
  int id = 0;
  for(int dz = -1; dz <= 1; ++dz) {
    for(int dy = -1; dy <= 1; ++dy) {
      for(int dx = -1; dx <= 1; ++dx) {
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for(int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,
              ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
        for(int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,
              f1[id][0][dev],f1[id][1][dev], f1[id][2][dev],
              f2[id][0][dev],f2[id][1][dev], f2[id][2][dev]);
        }
        id ++;
      }
    }
  }
}


void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev,
    int& tx, int& ty, int& tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=1; ty=1; tz=0; return;
        case 4: tx=-1; ty=1; tz=0; return;
        case 5: tx=0; ty=-1; tz=0; return;
        case 6: tx=1; ty=-1; tz=0; return;
        case 7: tx=-1; ty=-1; tz=0; return;
        case 8: tx=0; ty=0; tz=1; return;
        case 9: tx=1; ty=0; tz=1; return;
        case 10: tx=-1; ty=0; tz=1; return;
        case 11: tx=0; ty=1; tz=1; return;
        case 12: tx=1; ty=1; tz=1; return;
        case 13: tx=-1; ty=1; tz=1; return;
        case 14: tx=0; ty=-1; tz=1; return;
        case 15: tx=1; ty=-1; tz=1; return;
        case 16: tx=-1; ty=-1; tz=1; return;
        case 17: tx=0; ty=0; tz=-1; return;
        case 18: tx=1; ty=0; tz=-1; return;
        case 19: tx=-1; ty=0; tz=-1; return;
        case 20: tx=0; ty=1; tz=-1; return;
        case 21: tx=1; ty=1; tz=-1; return;
        case 22: tx=-1; ty=1; tz=-1; return;
        case 23: tx=0; ty=-1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}

void JPS3DNeib::FNeib( int dx, int dy, int dz, int norm1, int dev,
                          int& fx, int& fy, int& fz,
                          int& nx, int& ny, int& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // switch order if different direction
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}