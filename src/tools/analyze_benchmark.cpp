/* ----------------------------------------------------------------------------
 * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Kota Kondo, et al.
 * See LICENSE file for the license information
 *
 * C++ port of analyze_dynamic_benchmark.py
 * Provides much faster rosbag-based collision and violation analysis.
 * -------------------------------------------------------------------------- */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// ROS2 bag reading
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rclcpp/serialization.hpp>

// Message types
#include "dynus_interfaces/msg/goal.hpp"
#include "dynus_interfaces/msg/dyn_traj.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fs = std::filesystem;

// ============================================================================
// Data structures
// ============================================================================

struct TrialData {
  int trial_id = -1;
  bool goal_reached = false;
  bool timeout_reached = false;
  bool collision = false;
  double flight_travel_time = 0.0;
  double path_length = 0.0;
  double path_efficiency = 0.0;
  double jerk_rms = 0.0;
  double jerk_integral = 0.0;
  int collision_count = 0;
  double min_distance_to_obstacles = 1e18;
  double collision_free_ratio = 1.0;
  int collision_unique_obstacles = 0;

  // Computation time
  int num_replans = 0;
  double avg_replanning_time = 0.0;
  double max_replanning_time = 0.0;
  double total_replanning_time = 0.0;
  double avg_global_planning_time = 0.0;
  double avg_sfc_corridor_time = 0.0;
  double avg_local_traj_time = 0.0;

  // Violation counts
  int vel_violation_count = 0;
  int vel_violation_total = 0;
  int acc_violation_count = 0;
  int acc_violation_total = 0;
  int jerk_violation_count = 0;
  int jerk_violation_total = 0;

  // SFC violations
  int sfc_violation_count = 0;
  int sfc_violation_total = 0;
};

struct Statistics {
  int total_trials = 0;
  int n_successful = 0;
  double success_rate = 0.0;
  double timeout_rate = 0.0;
  double collision_rate = 0.0;

  // Collision
  int collision_count_total = 0;
  double collision_count_mean = 0.0;
  double collision_free_rate = 0.0;

  // Min distance
  double min_distance_to_obstacles_min = 1e18;
  double min_distance_to_obstacles_max = -1e18;
  double min_distance_to_obstacles_mean = 0.0;
  double min_distance_to_obstacles_std = 0.0;
  bool has_min_distance = false;

  // Computation times (mean of per-trial averages)
  double avg_local_traj_time_mean = 0.0;
  double avg_replanning_time_mean = 0.0;
  double avg_global_planning_time_mean = 0.0;
  double avg_sfc_corridor_time_mean = 0.0;
  double num_replans_mean = 0.0;

  // Performance (on successful trials)
  double flight_travel_time_mean = 0.0;
  double flight_travel_time_std = 0.0;
  double path_length_mean = 0.0;
  double path_length_std = 0.0;
  double path_efficiency_mean = 0.0;
  double jerk_rms_mean = 0.0;
  double jerk_integral_mean = 0.0;

  // Violation rates
  double vel_violation_rate = 0.0;
  double acc_violation_rate = 0.0;
  double jerk_violation_rate = 0.0;
  double sfc_violation_rate = 0.0;
};

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct ObstacleSnapshot {
  double time;
  Vec3 pos;
};

struct ObstacleTrack {
  std::vector<double> times;
  std::vector<Vec3> positions;
  double half_x = 0.4, half_y = 0.4, half_z = 0.4;
};

struct AgentSnapshot {
  double time;
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;
};

// ============================================================================
// CSV parsing
// ============================================================================

static std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    // Trim whitespace
    size_t start = token.find_first_not_of(" \t\r\n");
    size_t end = token.find_last_not_of(" \t\r\n");
    if (start != std::string::npos)
      tokens.push_back(token.substr(start, end - start + 1));
    else
      tokens.push_back("");
  }
  return tokens;
}

static int find_column(const std::vector<std::string>& header, const std::string& name) {
  for (size_t i = 0; i < header.size(); i++) {
    if (header[i] == name) return static_cast<int>(i);
  }
  return -1;
}

static double safe_stod(const std::string& s) {
  if (s.empty() || s == "inf" || s == "nan" || s == "N/A") return 0.0;
  try { return std::stod(s); } catch (...) { return 0.0; }
}

static int safe_stoi(const std::string& s) {
  if (s.empty()) return 0;
  try { return std::stoi(s); } catch (...) { return 0; }
}

static bool safe_stob(const std::string& s) {
  return s == "True" || s == "true" || s == "1";
}

static std::vector<TrialData> load_benchmark_csv(const fs::path& csv_path) {
  std::vector<TrialData> trials;
  std::ifstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "ERROR: Cannot open CSV: " << csv_path << "\n";
    return trials;
  }

  std::string line;
  std::getline(file, line); // header
  auto header = split_csv_line(line);

  // Find column indices
  int col_trial_id = find_column(header, "trial_id");
  int col_goal_reached = find_column(header, "goal_reached");
  int col_timeout = find_column(header, "timeout_reached");
  int col_collision = find_column(header, "collision");
  int col_travel_time = find_column(header, "flight_travel_time");
  int col_path_length = find_column(header, "path_length");
  int col_path_efficiency = find_column(header, "path_efficiency");
  int col_jerk_rms = find_column(header, "jerk_rms");
  int col_jerk_integral = find_column(header, "jerk_integral");
  int col_collision_count = find_column(header, "collision_count");
  int col_min_dist = find_column(header, "min_distance_to_obstacles");
  int col_sfc_viol_count = find_column(header, "sfc_violation_count");
  int col_sfc_viol_total = find_column(header, "sfc_violation_total");
  int col_vel_viol_count = find_column(header, "vel_violation_count");
  int col_vel_viol_total = find_column(header, "vel_violation_total");
  int col_acc_viol_count = find_column(header, "acc_violation_count");
  int col_acc_viol_total = find_column(header, "acc_violation_total");
  int col_jerk_viol_count = find_column(header, "jerk_violation_count");
  int col_jerk_viol_total = find_column(header, "jerk_violation_total");

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto cols = split_csv_line(line);
    TrialData t;
    if (col_trial_id >= 0 && col_trial_id < (int)cols.size()) t.trial_id = safe_stoi(cols[col_trial_id]);
    if (col_goal_reached >= 0 && col_goal_reached < (int)cols.size()) t.goal_reached = safe_stob(cols[col_goal_reached]);
    if (col_timeout >= 0 && col_timeout < (int)cols.size()) t.timeout_reached = safe_stob(cols[col_timeout]);
    if (col_collision >= 0 && col_collision < (int)cols.size()) t.collision = safe_stob(cols[col_collision]);
    if (col_travel_time >= 0 && col_travel_time < (int)cols.size()) t.flight_travel_time = safe_stod(cols[col_travel_time]);
    if (col_path_length >= 0 && col_path_length < (int)cols.size()) t.path_length = safe_stod(cols[col_path_length]);
    if (col_path_efficiency >= 0 && col_path_efficiency < (int)cols.size()) t.path_efficiency = safe_stod(cols[col_path_efficiency]);
    if (col_jerk_rms >= 0 && col_jerk_rms < (int)cols.size()) t.jerk_rms = safe_stod(cols[col_jerk_rms]);
    if (col_jerk_integral >= 0 && col_jerk_integral < (int)cols.size()) t.jerk_integral = safe_stod(cols[col_jerk_integral]);
    if (col_collision_count >= 0 && col_collision_count < (int)cols.size()) t.collision_count = safe_stoi(cols[col_collision_count]);
    if (col_min_dist >= 0 && col_min_dist < (int)cols.size()) t.min_distance_to_obstacles = safe_stod(cols[col_min_dist]);
    if (col_sfc_viol_count >= 0 && col_sfc_viol_count < (int)cols.size()) t.sfc_violation_count = safe_stoi(cols[col_sfc_viol_count]);
    if (col_sfc_viol_total >= 0 && col_sfc_viol_total < (int)cols.size()) t.sfc_violation_total = safe_stoi(cols[col_sfc_viol_total]);
    if (col_vel_viol_count >= 0 && col_vel_viol_count < (int)cols.size()) t.vel_violation_count = safe_stoi(cols[col_vel_viol_count]);
    if (col_vel_viol_total >= 0 && col_vel_viol_total < (int)cols.size()) t.vel_violation_total = safe_stoi(cols[col_vel_viol_total]);
    if (col_acc_viol_count >= 0 && col_acc_viol_count < (int)cols.size()) t.acc_violation_count = safe_stoi(cols[col_acc_viol_count]);
    if (col_acc_viol_total >= 0 && col_acc_viol_total < (int)cols.size()) t.acc_violation_total = safe_stoi(cols[col_acc_viol_total]);
    if (col_jerk_viol_count >= 0 && col_jerk_viol_count < (int)cols.size()) t.jerk_violation_count = safe_stoi(cols[col_jerk_viol_count]);
    if (col_jerk_viol_total >= 0 && col_jerk_viol_total < (int)cols.size()) t.jerk_violation_total = safe_stoi(cols[col_jerk_viol_total]);
    trials.push_back(t);
  }

  return trials;
}

static fs::path find_most_recent_csv(const fs::path& dir) {
  if (!fs::exists(dir) || !fs::is_directory(dir)) return {};
  std::vector<fs::path> csv_files;
  for (auto& entry : fs::directory_iterator(dir)) {
    auto name = entry.path().filename().string();
    if (name.find("benchmark_") == 0 && name.find(".csv") != std::string::npos &&
        name.find("summary") == std::string::npos) {
      csv_files.push_back(entry.path());
    }
  }
  if (csv_files.empty()) return {};

  // Sort by modification time, pick most recent
  std::sort(csv_files.begin(), csv_files.end(), [](const fs::path& a, const fs::path& b) {
    return fs::last_write_time(a) < fs::last_write_time(b);
  });
  return csv_files.back();
}

// ============================================================================
// Computation data loading (num_*.csv)
// ============================================================================

struct ComputationStats {
  int num_replans = 0;
  double avg_replanning_time = 0.0;
  double max_replanning_time = 0.0;
  double total_replanning_time = 0.0;
  double avg_global_planning_time = 0.0;
  double avg_sfc_corridor_time = 0.0;
  double avg_local_traj_time = 0.0;
};

static std::map<int, ComputationStats> load_computation_data(const fs::path& data_dir) {
  std::map<int, ComputationStats> result;
  fs::path csv_dir = data_dir / "csv";
  if (!fs::exists(csv_dir)) return result;

  for (auto& entry : fs::directory_iterator(csv_dir)) {
    auto name = entry.path().filename().string();
    if (name.find("num_") != 0 || name.find(".csv") == std::string::npos) continue;

    // Extract trial id: num_X.csv
    int trial_id = 0;
    try {
      auto stem = entry.path().stem().string();
      trial_id = std::stoi(stem.substr(4));
    } catch (...) { continue; }

    std::ifstream file(entry.path());
    if (!file.is_open()) continue;

    std::string line;
    std::getline(file, line); // header
    auto header = split_csv_line(line);

    int col_result = find_column(header, "Result");
    int col_total_replan = find_column(header, "Total replanning time [ms]");
    int col_global = find_column(header, "Global Planning Time [ms]");
    int col_local = find_column(header, "Local Traj Time [ms]");
    int col_cvx = find_column(header, "CVX Decomposition Time [ms]");
    if (col_cvx < 0) col_cvx = find_column(header, "SFC Corridor Time [ms]");

    int count = 0;
    double sum_replan = 0, max_replan = 0, sum_global = 0, sum_local = 0, sum_cvx = 0;

    while (std::getline(file, line)) {
      if (line.empty()) continue;
      auto cols = split_csv_line(line);
      if (col_result < 0 || col_result >= (int)cols.size()) continue;
      if (safe_stoi(cols[col_result]) != 1) continue; // only successful replans

      count++;
      double r = (col_total_replan >= 0 && col_total_replan < (int)cols.size()) ? safe_stod(cols[col_total_replan]) : 0.0;
      sum_replan += r;
      max_replan = std::max(max_replan, r);
      if (col_global >= 0 && col_global < (int)cols.size()) sum_global += safe_stod(cols[col_global]);
      if (col_local >= 0 && col_local < (int)cols.size()) sum_local += safe_stod(cols[col_local]);
      if (col_cvx >= 0 && col_cvx < (int)cols.size()) sum_cvx += safe_stod(cols[col_cvx]);
    }

    ComputationStats stats;
    stats.num_replans = count;
    if (count > 0) {
      stats.avg_replanning_time = sum_replan / count;
      stats.max_replanning_time = max_replan;
      stats.total_replanning_time = sum_replan;
      stats.avg_global_planning_time = sum_global / count;
      stats.avg_local_traj_time = sum_local / count;
      stats.avg_sfc_corridor_time = sum_cvx / count;
    }
    result[trial_id] = stats;
  }

  std::cout << "  Loaded computation data for " << result.size() << " trial(s)\n";
  return result;
}

// ============================================================================
// Bag reading — single-pass extraction
// ============================================================================

struct BagData {
  std::vector<AgentSnapshot> agent; // from /NX01/goal
  std::map<int, ObstacleTrack> obstacles; // from /trajs_ground_truth or /tf
};

static Vec3 interpolate_position(const std::vector<double>& times,
                                  const std::vector<Vec3>& positions,
                                  double t_query) {
  if (times.empty()) return {0, 0, 0};
  if (times.size() == 1) return positions[0];
  if (t_query <= times.front()) return positions.front();
  if (t_query >= times.back()) return positions.back();

  // Binary search
  auto it = std::lower_bound(times.begin(), times.end(), t_query);
  size_t idx = std::distance(times.begin(), it);
  if (idx >= times.size()) return positions.back();
  if (idx == 0) return positions.front();

  double t0 = times[idx - 1], t1 = times[idx];
  double alpha = (t1 > t0) ? (t_query - t0) / (t1 - t0) : 0.0;
  const auto& p0 = positions[idx - 1];
  const auto& p1 = positions[idx];
  return {
    p0.x + alpha * (p1.x - p0.x),
    p0.y + alpha * (p1.y - p0.y),
    p0.z + alpha * (p1.z - p0.z)
  };
}

static BagData read_bag_single_pass(const fs::path& bag_path,
                                     const std::string& trajs_topic,
                                     bool read_tf) {
  BagData data;

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path.string();
  storage_options.storage_id = "sqlite3";

  try {
    reader.open(storage_options);
  } catch (const std::exception& e) {
    std::cerr << "    Warning: Cannot open bag " << bag_path << ": " << e.what() << "\n";
    return data;
  }

  rclcpp::Serialization<dynus_interfaces::msg::Goal> goal_ser;
  rclcpp::Serialization<dynus_interfaces::msg::DynTraj> dyntraj_ser;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_ser;

  while (reader.has_next()) {
    auto msg = reader.read_next();
    double timestamp = msg->time_stamp / 1e9;

    if (msg->topic_name == "/NX01/goal") {
      dynus_interfaces::msg::Goal goal_msg;
      rclcpp::SerializedMessage serialized(*msg->serialized_data);
      goal_ser.deserialize_message(&serialized, &goal_msg);

      AgentSnapshot snap;
      snap.time = timestamp;
      snap.pos = {goal_msg.p.x, goal_msg.p.y, goal_msg.p.z};
      snap.vel = {goal_msg.v.x, goal_msg.v.y, goal_msg.v.z};
      snap.acc = {goal_msg.a.x, goal_msg.a.y, goal_msg.a.z};
      snap.jerk = {goal_msg.j.x, goal_msg.j.y, goal_msg.j.z};
      data.agent.push_back(snap);
    }
    else if (!trajs_topic.empty() && msg->topic_name == trajs_topic) {
      dynus_interfaces::msg::DynTraj dyntraj_msg;
      rclcpp::SerializedMessage serialized(*msg->serialized_data);
      dyntraj_ser.deserialize_message(&serialized, &dyntraj_msg);

      if (dyntraj_msg.is_agent) continue;

      int obs_id = dyntraj_msg.id;
      auto& track = data.obstacles[obs_id];
      track.times.push_back(timestamp);
      track.positions.push_back({dyntraj_msg.pos.x, dyntraj_msg.pos.y, dyntraj_msg.pos.z});

      if (dyntraj_msg.bbox.size() >= 3) {
        track.half_x = dyntraj_msg.bbox[0] / 2.0;
        track.half_y = dyntraj_msg.bbox[1] / 2.0;
        track.half_z = dyntraj_msg.bbox[2] / 2.0;
      }
    }
    else if (read_tf && msg->topic_name == "/tf") {
      tf2_msgs::msg::TFMessage tf_msg;
      rclcpp::SerializedMessage serialized(*msg->serialized_data);
      tf_ser.deserialize_message(&serialized, &tf_msg);

      for (auto& transform : tf_msg.transforms) {
        std::string frame = transform.child_frame_id;
        if (frame.find("obstacle") != std::string::npos || frame.find("obs_") == 0) {
          // Use hash of frame name as id
          int obs_id = static_cast<int>(std::hash<std::string>{}(frame) & 0x7FFFFFFF);
          auto& track = data.obstacles[obs_id];
          track.times.push_back(timestamp);
          track.positions.push_back({
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
          });
        }
      }
    }
  }

  return data;
}

// ============================================================================
// Collision analysis
// ============================================================================

struct CollisionResult {
  int collision_count = 0;
  double min_distance = 1e18;
  double collision_free_ratio = 1.0;
  int unique_obstacles = 0;
};

static CollisionResult analyze_collisions(const BagData& data) {
  CollisionResult result;
  if (data.agent.empty() || data.obstacles.empty()) {
    result.unique_obstacles = static_cast<int>(data.obstacles.size());
    return result;
  }

  result.unique_obstacles = static_cast<int>(data.obstacles.size());
  int collision_free_segments = 0;

  for (const auto& snap : data.agent) {
    bool segment_collision_free = true;
    double px = snap.pos.x, py = snap.pos.y, pz = snap.pos.z;

    for (const auto& [obs_id, track] : data.obstacles) {
      if (track.times.empty()) continue;

      Vec3 obs_pos = interpolate_position(track.times, track.positions, snap.time);

      // Pre-filter: skip far obstacles
      double dx = px - obs_pos.x, dy = py - obs_pos.y, dz = pz - obs_pos.z;
      double center_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (center_dist > 5.0) continue;

      double hx = track.half_x, hy = track.half_y, hz = track.half_z;

      // Point-to-AABB distance
      double cx = std::clamp(px, obs_pos.x - hx, obs_pos.x + hx);
      double cy = std::clamp(py, obs_pos.y - hy, obs_pos.y + hy);
      double cz = std::clamp(pz, obs_pos.z - hz, obs_pos.z + hz);

      double ddx = px - cx, ddy = py - cy, ddz = pz - cz;
      double distance = std::sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
      result.min_distance = std::min(result.min_distance, distance);

      // Check collision: point inside AABB
      if (obs_pos.x - hx <= px && px <= obs_pos.x + hx &&
          obs_pos.y - hy <= py && py <= obs_pos.y + hy &&
          obs_pos.z - hz <= pz && pz <= obs_pos.z + hz) {
        segment_collision_free = false;
        result.collision_count++;
      }
    }

    if (segment_collision_free) collision_free_segments++;
  }

  result.collision_free_ratio = static_cast<double>(collision_free_segments) / data.agent.size();
  if (result.min_distance > 1e17) result.min_distance = 0.0;
  return result;
}

// ============================================================================
// Static obstacle collision analysis (from CSV)
// ============================================================================

struct StaticObstacle {
  int id;
  double x, y, z, radius, height;
};

static std::vector<StaticObstacle> load_static_obstacles(const fs::path& csv_path) {
  std::vector<StaticObstacle> obstacles;
  std::ifstream file(csv_path);
  if (!file.is_open()) return obstacles;

  std::string line;
  std::getline(file, line); // header
  auto header = split_csv_line(line);
  int col_id = find_column(header, "id");
  int col_x = find_column(header, "x");
  int col_y = find_column(header, "y");
  int col_z = find_column(header, "z");
  int col_radius = find_column(header, "radius");
  int col_height = find_column(header, "height");

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto cols = split_csv_line(line);
    StaticObstacle obs;
    obs.id = (col_id >= 0 && col_id < (int)cols.size()) ? safe_stoi(cols[col_id]) : 0;
    obs.x = (col_x >= 0 && col_x < (int)cols.size()) ? safe_stod(cols[col_x]) : 0;
    obs.y = (col_y >= 0 && col_y < (int)cols.size()) ? safe_stod(cols[col_y]) : 0;
    obs.z = (col_z >= 0 && col_z < (int)cols.size()) ? safe_stod(cols[col_z]) : 0;
    obs.radius = (col_radius >= 0 && col_radius < (int)cols.size()) ? safe_stod(cols[col_radius]) : 0;
    obs.height = (col_height >= 0 && col_height < (int)cols.size()) ? safe_stod(cols[col_height]) : 0;
    obstacles.push_back(obs);
  }
  return obstacles;
}

static CollisionResult analyze_static_collisions(const BagData& data,
                                                   const std::vector<StaticObstacle>& obstacles) {
  CollisionResult result;
  if (data.agent.empty() || obstacles.empty()) return result;

  int collision_free_segments = 0;
  std::set<int> obstacles_hit;

  for (const auto& snap : data.agent) {
    bool segment_collision_free = true;
    double px = snap.pos.x, py = snap.pos.y, pz = snap.pos.z;

    for (const auto& obs : obstacles) {
      double horiz_dist = std::sqrt((px - obs.x) * (px - obs.x) + (py - obs.y) * (py - obs.y));
      double horiz_clearance = horiz_dist - obs.radius;

      double obs_z_min = obs.z - obs.height / 2.0;
      double obs_z_max = obs.z + obs.height / 2.0;
      bool vert_inside = (pz > obs_z_min && pz < obs_z_max);

      double dist = std::max(0.0, horiz_clearance);
      result.min_distance = std::min(result.min_distance, dist);

      if (horiz_clearance < 0 && vert_inside) {
        segment_collision_free = false;
        result.collision_count++;
        obstacles_hit.insert(obs.id);
      }
    }

    if (segment_collision_free) collision_free_segments++;
  }

  result.unique_obstacles = static_cast<int>(obstacles_hit.size());
  result.collision_free_ratio = static_cast<double>(collision_free_segments) / data.agent.size();
  if (result.min_distance > 1e17) result.min_distance = 0.0;
  return result;
}

// ============================================================================
// Violation analysis (from bag data, already extracted)
// ============================================================================

struct ViolationResult {
  int vel_count = 0, vel_total = 0;
  int acc_count = 0, acc_total = 0;
  int jerk_count = 0, jerk_total = 0;
};

static ViolationResult analyze_violations(const BagData& data,
                                           double vel_limit = 5.0,
                                           double acc_limit = 20.0,
                                           double jerk_limit = 100.0,
                                           double tolerance = 1e-3) {
  ViolationResult result;
  for (const auto& snap : data.agent) {
    result.vel_total++;
    result.acc_total++;
    result.jerk_total++;

    double vmax = std::max({std::abs(snap.vel.x), std::abs(snap.vel.y), std::abs(snap.vel.z)});
    if (vmax > vel_limit + tolerance) result.vel_count++;

    double amax = std::max({std::abs(snap.acc.x), std::abs(snap.acc.y), std::abs(snap.acc.z)});
    if (amax > acc_limit + tolerance) result.acc_count++;

    double jmax = std::max({std::abs(snap.jerk.x), std::abs(snap.jerk.y), std::abs(snap.jerk.z)});
    if (jmax > jerk_limit + tolerance) result.jerk_count++;
  }
  return result;
}

// ============================================================================
// Path metrics from bag data
// ============================================================================

struct PathMetrics {
  std::optional<double> travel_time;
  std::optional<double> path_length;
};

static PathMetrics compute_path_metrics(const BagData& data, Vec3 goal_pos, Vec3 start_pos,
                                         double dist_threshold = 0.5, double speed_threshold = 0.1) {
  PathMetrics result;
  if (data.agent.empty()) return result;

  // Detect gap
  auto& first = data.agent.front();
  double gap_dist = std::sqrt(
    (first.pos.x - start_pos.x) * (first.pos.x - start_pos.x) +
    (first.pos.y - start_pos.y) * (first.pos.y - start_pos.y) +
    (first.pos.z - start_pos.z) * (first.pos.z - start_pos.z));
  bool has_gap = gap_dist > 0.5;

  // Find first movement
  size_t move_start_idx = 0;
  for (size_t i = 0; i < data.agent.size(); i++) {
    auto& s = data.agent[i];
    double speed = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y + s.vel.z * s.vel.z);
    if (speed > 0.01) { move_start_idx = i; break; }
  }

  // Find goal arrival
  std::optional<size_t> goal_idx;
  for (size_t i = 0; i < data.agent.size(); i++) {
    auto& s = data.agent[i];
    double dist = std::sqrt(
      (s.pos.x - goal_pos.x) * (s.pos.x - goal_pos.x) +
      (s.pos.y - goal_pos.y) * (s.pos.y - goal_pos.y) +
      (s.pos.z - goal_pos.z) * (s.pos.z - goal_pos.z));
    double speed = std::sqrt(s.vel.x * s.vel.x + s.vel.y * s.vel.y + s.vel.z * s.vel.z);
    if (dist < dist_threshold && speed < speed_threshold) {
      goal_idx = i;
      break;
    }
  }

  // Path length
  size_t end_idx = goal_idx.value_or(data.agent.size() - 1);
  double path_length = gap_dist;
  for (size_t i = 1; i <= end_idx; i++) {
    double dx = data.agent[i].pos.x - data.agent[i-1].pos.x;
    double dy = data.agent[i].pos.y - data.agent[i-1].pos.y;
    double dz = data.agent[i].pos.z - data.agent[i-1].pos.z;
    path_length += std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  result.path_length = path_length;

  // Travel time
  if (!has_gap && goal_idx.has_value()) {
    result.travel_time = data.agent[goal_idx.value()].time - data.agent[move_start_idx].time;
  }

  return result;
}

// ============================================================================
// Statistics computation
// ============================================================================

static double compute_mean(const std::vector<double>& v) {
  if (v.empty()) return 0.0;
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

static double compute_std(const std::vector<double>& v, double mean) {
  if (v.size() < 2) return 0.0;
  double sum = 0.0;
  for (double x : v) sum += (x - mean) * (x - mean);
  return std::sqrt(sum / (v.size() - 1));
}

static Statistics compute_statistics(std::vector<TrialData>& trials) {
  Statistics stats;
  stats.total_trials = static_cast<int>(trials.size());
  if (trials.empty()) return stats;

  // Mark timeout
  for (auto& t : trials) {
    if (t.flight_travel_time > 100.0) {
      t.goal_reached = false;
      t.timeout_reached = true;
    }
  }

  // Success, timeout, collision rates
  int success_count = 0, timeout_count = 0, collision_count_trials = 0;
  for (auto& t : trials) {
    if (t.goal_reached && t.collision_count == 0) success_count++;
    if (t.timeout_reached) timeout_count++;
    if (t.collision) collision_count_trials++;
  }
  stats.success_rate = 100.0 * success_count / trials.size();
  stats.timeout_rate = 100.0 * timeout_count / trials.size();
  stats.collision_rate = 100.0 * collision_count_trials / trials.size();

  // Collision metrics (ALL trials)
  int total_coll = 0;
  int zero_coll = 0;
  for (auto& t : trials) {
    total_coll += t.collision_count;
    if (t.collision_count == 0) zero_coll++;
  }
  stats.collision_count_total = total_coll;
  stats.collision_count_mean = static_cast<double>(total_coll) / trials.size();
  stats.collision_free_rate = 100.0 * zero_coll / trials.size();

  // Min distance (ALL trials)
  std::vector<double> min_dists;
  for (auto& t : trials) {
    if (t.min_distance_to_obstacles < 1e17 && t.min_distance_to_obstacles > 0) {
      min_dists.push_back(t.min_distance_to_obstacles);
    }
  }
  if (!min_dists.empty()) {
    stats.has_min_distance = true;
    stats.min_distance_to_obstacles_min = *std::min_element(min_dists.begin(), min_dists.end());
    stats.min_distance_to_obstacles_max = *std::max_element(min_dists.begin(), min_dists.end());
    stats.min_distance_to_obstacles_mean = compute_mean(min_dists);
    stats.min_distance_to_obstacles_std = compute_std(min_dists, stats.min_distance_to_obstacles_mean);
  }

  // Successful trials
  std::vector<TrialData*> successful;
  for (auto& t : trials) {
    if (t.goal_reached && t.collision_count == 0) successful.push_back(&t);
  }
  stats.n_successful = static_cast<int>(successful.size());
  if (successful.empty()) {
    std::cout << "WARNING: No successful trials found!\n";
    return stats;
  }

  // Computation times
  std::vector<double> v_local, v_replan, v_global, v_sfc, v_nreplans;
  for (auto* t : successful) {
    if (t->avg_local_traj_time > 0) v_local.push_back(t->avg_local_traj_time);
    if (t->avg_replanning_time > 0) v_replan.push_back(t->avg_replanning_time);
    if (t->avg_global_planning_time > 0) v_global.push_back(t->avg_global_planning_time);
    if (t->avg_sfc_corridor_time > 0) v_sfc.push_back(t->avg_sfc_corridor_time);
    if (t->num_replans > 0) v_nreplans.push_back(t->num_replans);
  }
  stats.avg_local_traj_time_mean = compute_mean(v_local);
  stats.avg_replanning_time_mean = compute_mean(v_replan);
  stats.avg_global_planning_time_mean = compute_mean(v_global);
  stats.avg_sfc_corridor_time_mean = compute_mean(v_sfc);
  stats.num_replans_mean = compute_mean(v_nreplans);

  // Performance
  std::vector<double> v_tt, v_pl, v_pe, v_jrms, v_jint;
  for (auto* t : successful) {
    v_tt.push_back(t->flight_travel_time);
    v_pl.push_back(t->path_length);
    v_pe.push_back(t->path_efficiency);
    v_jrms.push_back(t->jerk_rms);
    v_jint.push_back(t->jerk_integral);
  }
  stats.flight_travel_time_mean = compute_mean(v_tt);
  stats.flight_travel_time_std = compute_std(v_tt, stats.flight_travel_time_mean);
  stats.path_length_mean = compute_mean(v_pl);
  stats.path_length_std = compute_std(v_pl, stats.path_length_mean);
  stats.path_efficiency_mean = compute_mean(v_pe);
  stats.jerk_rms_mean = compute_mean(v_jrms);
  stats.jerk_integral_mean = compute_mean(v_jint);

  // Violation rates (among successful)
  long total_vel_v = 0, total_vel_s = 0;
  long total_acc_v = 0, total_acc_s = 0;
  long total_jerk_v = 0, total_jerk_s = 0;
  long total_sfc_v = 0, total_sfc_s = 0;
  for (auto* t : successful) {
    total_vel_v += t->vel_violation_count; total_vel_s += t->vel_violation_total;
    total_acc_v += t->acc_violation_count; total_acc_s += t->acc_violation_total;
    total_jerk_v += t->jerk_violation_count; total_jerk_s += t->jerk_violation_total;
    total_sfc_v += t->sfc_violation_count; total_sfc_s += t->sfc_violation_total;
  }
  stats.vel_violation_rate = (total_vel_s > 0) ? 100.0 * total_vel_v / total_vel_s : 0.0;
  stats.acc_violation_rate = (total_acc_s > 0) ? 100.0 * total_acc_v / total_acc_s : 0.0;
  stats.jerk_violation_rate = (total_jerk_s > 0) ? 100.0 * total_jerk_v / total_jerk_s : 0.0;
  stats.sfc_violation_rate = (total_sfc_s > 0) ? 100.0 * total_sfc_v / total_sfc_s : 0.0;

  return stats;
}

// ============================================================================
// Console output
// ============================================================================

static void print_statistics(const Statistics& s) {
  std::string sep(80, '=');
  std::string dash(80, '-');

  std::cout << "\n" << sep << "\nBENCHMARK ANALYSIS RESULTS\n" << sep << "\n";

  std::cout << "\n" << std::string(30, '-') << " OVERVIEW " << std::string(30, '-') << "\n";
  std::cout << "  Total trials: " << s.total_trials << "\n";
  std::cout << "  Successful trials: " << s.n_successful << "\n";
  std::cout << std::fixed << std::setprecision(1);
  std::cout << "  Success rate: " << s.success_rate << "%\n";
  std::cout << "  Timeout rate: " << s.timeout_rate << "%\n";
  std::cout << "  Collision rate: " << s.collision_rate << "%\n";

  std::cout << "\n" << std::string(25, '-') << " COMPUTATION TIME (ms) " << std::string(25, '-') << "\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  Local Traj Time: " << s.avg_local_traj_time_mean << " ms\n";
  std::cout << "  Replanning Time: " << s.avg_replanning_time_mean << " ms\n";
  std::cout << "  Global Planning Time: " << s.avg_global_planning_time_mean << " ms\n";
  std::cout << "  SFC Corridor Time: " << s.avg_sfc_corridor_time_mean << " ms\n";

  std::cout << "\n" << std::string(25, '-') << " PERFORMANCE METRICS " << std::string(25, '-') << "\n";
  std::cout << "  Travel Time: " << s.flight_travel_time_mean << " +/- " << s.flight_travel_time_std << " s\n";
  std::cout << "  Path Length: " << s.path_length_mean << " +/- " << s.path_length_std << " m\n";
  std::cout << "  Path Efficiency: " << std::setprecision(3) << s.path_efficiency_mean << "\n";
  std::cout << "  Jerk RMS: " << std::setprecision(2) << s.jerk_rms_mean << " m/s^3\n";
  std::cout << "  Jerk Integral: " << s.jerk_integral_mean << "\n";

  std::cout << "\n" << std::string(25, '-') << " CONSTRAINT VIOLATIONS " << std::string(25, '-') << "\n";
  std::cout << std::setprecision(1);
  std::cout << "  VEL Violation Rate: " << s.vel_violation_rate << "%\n";
  std::cout << "  ACC Violation Rate: " << s.acc_violation_rate << "%\n";
  std::cout << "  JERK Violation Rate: " << s.jerk_violation_rate << "%\n";

  std::cout << "\n" << std::string(25, '-') << " COLLISION METRICS " << std::string(25, '-') << "\n";
  std::cout << "  Collision-free rate: " << s.collision_free_rate << "%\n";
  std::cout << "  Total collision events: " << s.collision_count_total << "\n";
  std::cout << std::setprecision(2);
  std::cout << "  Mean collisions per trial: " << s.collision_count_mean << "\n";

  std::cout << "\n" << std::string(25, '-') << " MIN DISTANCE " << std::string(25, '-') << "\n";
  if (s.has_min_distance) {
    std::cout << std::setprecision(3);
    std::cout << "  Min: " << s.min_distance_to_obstacles_min << " m\n";
    std::cout << "  Max: " << s.min_distance_to_obstacles_max << " m\n";
    std::cout << "  Mean: " << s.min_distance_to_obstacles_mean << " m\n";
    std::cout << "  Std: " << s.min_distance_to_obstacles_std << " m\n";
  } else {
    std::cout << "  N/A\n";
  }

  std::cout << "\n" << sep << "\n\n";
}

// ============================================================================
// CSV summary output
// ============================================================================

static void save_statistics_csv(const Statistics& s, const fs::path& output_path) {
  fs::create_directories(output_path.parent_path());
  std::ofstream f(output_path);
  if (!f.is_open()) {
    std::cerr << "ERROR: Cannot write CSV: " << output_path << "\n";
    return;
  }

  f << "total_trials,n_successful,success_rate,timeout_rate,collision_rate,"
    << "collision_count_total,collision_count_mean,collision_free_rate,"
    << "avg_local_traj_time_mean,avg_replanning_time_mean,"
    << "flight_travel_time_mean,flight_travel_time_std,"
    << "path_length_mean,path_length_std,"
    << "jerk_integral_mean,jerk_rms_mean,"
    << "min_distance_to_obstacles_mean,min_distance_to_obstacles_std,"
    << "vel_violation_rate,acc_violation_rate,jerk_violation_rate\n";

  f << std::fixed;
  f << s.total_trials << "," << s.n_successful << ","
    << std::setprecision(1) << s.success_rate << ","
    << s.timeout_rate << "," << s.collision_rate << ","
    << s.collision_count_total << ","
    << std::setprecision(2) << s.collision_count_mean << ","
    << std::setprecision(1) << s.collision_free_rate << ","
    << std::setprecision(2) << s.avg_local_traj_time_mean << ","
    << s.avg_replanning_time_mean << ","
    << s.flight_travel_time_mean << "," << s.flight_travel_time_std << ","
    << s.path_length_mean << "," << s.path_length_std << ","
    << s.jerk_integral_mean << "," << s.jerk_rms_mean << ","
    << std::setprecision(3) << s.min_distance_to_obstacles_mean << ","
    << s.min_distance_to_obstacles_std << ","
    << std::setprecision(1) << s.vel_violation_rate << ","
    << s.acc_violation_rate << "," << s.jerk_violation_rate << "\n";

  std::cout << "Statistics saved to CSV: " << output_path << "\n";
}

// ============================================================================
// LaTeX table generation
// ============================================================================

static std::string generate_dynus_row(const Statistics& s, const std::string& case_name,
                                        const std::string& table_type) {
  std::ostringstream oss;
  oss << std::fixed;

  double success_rate = s.success_rate;
  double travel_time = s.flight_travel_time_mean;
  double path_length = s.path_length_mean;
  double jerk_integral = s.jerk_integral_mean;
  double vel_viol = s.vel_violation_rate;
  double acc_viol = s.acc_violation_rate;
  double jerk_viol = s.jerk_violation_rate;

  if (table_type == "static") {
    double total_opt_time = s.avg_local_traj_time_mean;
    double total_replan_time = s.avg_replanning_time_mean;
    oss << "       & DYNUS & Hard & $L_\\infty$ & "
        << std::setprecision(1) << "{" << success_rate << "} & "
        << "{" << total_opt_time << "} & "
        << "{" << total_replan_time << "} & "
        << "\\best{" << travel_time << "} & "
        << "{" << path_length << "} & "
        << "\\best{" << jerk_integral << "} & "
        << "{" << vel_viol << "} & "
        << "{" << acc_viol << "} & "
        << "{" << jerk_viol << "} \\\\";
  } else if (table_type == "unknown_dynamic") {
    double per_opt_time = s.avg_local_traj_time_mean;
    std::string min_dist_str;
    if (s.has_min_distance) {
      std::ostringstream md;
      md << std::fixed << std::setprecision(2) << s.min_distance_to_obstacles_mean;
      min_dist_str = md.str();
    } else {
      min_dist_str = "N/A";
    }

    oss << "      " << case_name << " & "
        << std::setprecision(1)
        << success_rate << " & " << per_opt_time << " & "
        << travel_time << " & " << path_length << " & "
        << jerk_integral << " & " << min_dist_str << " & "
        << vel_viol << " & " << acc_viol << " & "
        << jerk_viol << " \\\\";
  } else {
    // dynamic
    double per_opt_time = s.avg_local_traj_time_mean;
    std::string min_dist_str;
    if (s.has_min_distance) {
      std::ostringstream md;
      md << std::fixed << std::setprecision(2) << s.min_distance_to_obstacles_mean;
      min_dist_str = md.str();
    } else {
      min_dist_str = "N/A";
    }

    oss << "      & \\multicolumn{2}{c}{DYNUS} & "
        << std::setprecision(1)
        << success_rate << " & " << per_opt_time << " & "
        << travel_time << " & " << path_length << " & "
        << jerk_integral << " & " << min_dist_str << " & "
        << vel_viol << " & " << acc_viol << " & "
        << jerk_viol << " \\\\";
  }

  return oss.str();
}

static std::string generate_new_unknown_dynamic_table(const std::string& case_name,
                                                       const std::string& dynus_row) {
  std::string dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\";
  std::vector<std::string> cases = {"Easy", "Medium", "Hard"};

  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Benchmark results in unknown dynamic environments. "
      << "DYNUS navigates using only pointcloud sensing (no ground truth obstacle trajectories). "
      << "We report success rate, computation time, flight performance, smoothness, safety, "
      << "and constraint violation metrics.}\n"
      << "  \\label{tab:unknown_dynamic_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Safety}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){2-2}\n"
      << "      \\cmidrule(lr){3-3}\n"
      << "      \\cmidrule(lr){4-6}\n"
      << "      \\cmidrule(lr){7-7}\n"
      << "      \\cmidrule(lr){8-10}\n"
      << "      &\n"
      << "      $R_{\\mathrm{succ}}$ [\\%] &\n"
      << "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &\n"
      << "      $T_{\\mathrm{trav}}$ [s] &\n"
      << "      $L_{\\mathrm{path}}$ [m] &\n"
      << "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &\n"
      << "      $d_{\\mathrm{min}}$ [m] &\n"
      << "      $\\rho_{\\mathrm{vel}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{acc}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n";

  for (auto& c : cases) {
    if (c == case_name) {
      oss << dynus_row << "\n";
    } else {
      oss << "      " << c << " & " << dashes << "\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string generate_new_dynamic_table(const std::string& dynus_row) {
  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Dynamic obstacle benchmarking results: DYNUS performance with moving obstacles. "
      << "We report success rate, computation time, flight performance, smoothness, safety, "
      << "and constraint violation metrics.}\n"
      << "  \\label{tab:dynamic_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4em]{\\textbf{Env}}\n"
      << "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4em]{\\textbf{Algorithm}}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Safety}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){4-4}\n"
      << "      \\cmidrule(lr){5-5}\n"
      << "      \\cmidrule(lr){6-8}\n"
      << "      \\cmidrule(lr){9-9}\n"
      << "      \\cmidrule(lr){10-12}\n"
      << "      &&&\n"
      << "      $R_{\\mathrm{succ}}$ [\\%] &\n"
      << "      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &\n"
      << "      $T_{\\mathrm{trav}}$ [s] &\n"
      << "      $L_{\\mathrm{path}}$ [m] &\n"
      << "      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &\n"
      << "      $d_{\\mathrm{min}}$ [m] &\n"
      << "      $\\rho_{\\mathrm{vel}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{acc}}$ [\\%] &\n"
      << "      $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n"
      << dynus_row << "\n"
      << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string generate_new_static_table(const std::string& case_name,
                                               const std::string& /*dynus_row*/,
                                               const std::string& data_values) {
  std::string dashes = "{-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} & {-} \\\\";
  std::vector<std::string> cases = {"Easy", "Medium", "Hard"};

  std::ostringstream oss;
  oss << "\\begin{table*}\n"
      << "  \\caption{Benchmark results against state-of-the-art methods in static environments.}\n"
      << "  \\label{tab:static_benchmark}\n"
      << "  \\centering\n"
      << "  \\renewcommand{\\arraystretch}{1.2}\n"
      << "  \\resizebox{\\textwidth}{!}{\n"
      << "    \\begin{tabular}{c c c c c c c c c c c c c}\n"
      << "      \\toprule\n"
      << "      \\multirow{2}{*}[-0.4ex]{\\textbf{Env}}\n"
      << "      & \\multirow{2}{*}[-0.4ex]{\\textbf{Algorithm}}\n"
      << "      & \\multicolumn{2}{c}{\\multirow{2}{*}[-0.4ex]{\\textbf{Constr.}}}\n"
      << "      & \\multicolumn{1}{c}{\\textbf{Success}}\n"
      << "      & \\multicolumn{2}{c}{\\textbf{Computation Time}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Performance}}\n"
      << "      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}\n"
      << "      \\\\\n"
      << "      \\cmidrule(lr){5-5}\n"
      << "      \\cmidrule(lr){6-7}\n"
      << "      \\cmidrule(lr){8-10}\n"
      << "      \\cmidrule(lr){11-13}\n"
      << "      &&&&\n"
      << "      $R_{\\mathrm{succ}}$ [\\%]\n"
      << "      & $T^{\\mathrm{total}}_{\\mathrm{opt}}$ [ms]\n"
      << "      & $T^{\\mathrm{total}}_{\\mathrm{replan}}$ [ms]\n"
      << "      & $T_{\\mathrm{trav}}$ [s]\n"
      << "      & $L_{\\mathrm{path}}$ [m]\n"
      << "      & $S_{\\mathrm{jerk}}$ [m/s$^{2}$]\n"
      << "      & $\\rho_{\\mathrm{vel}}$ [\\%]\n"
      << "      & $\\rho_{\\mathrm{acc}}$ [\\%]\n"
      << "      & $\\rho_{\\mathrm{jerk}}$ [\\%]\n"
      << "      \\\\\n"
      << "      \\midrule\n";

  for (size_t i = 0; i < cases.size(); i++) {
    if (i > 0) oss << "\n      \\midrule\n\n";

    oss << "      \\multirow{5}{*}{" << cases[i] << "} & EGO-Swarm2 & Soft & $L_\\infty$ & " << dashes << "\n";
    oss << "       & \\multirow{2}{*}{SUPER} & Soft & $L_2$ & " << dashes << "\n";
    oss << "       & & Soft & $L_\\infty$ & " << dashes << "\n";
    oss << "       & FASTER & Hard & $L_\\infty$ & " << dashes << "\n";

    if (cases[i] == case_name) {
      oss << "       & DYNUS & Hard & $L_\\infty$ & " << data_values << "\n";
    } else {
      oss << "       & DYNUS & Hard & $L_\\infty$ & " << dashes << "\n";
    }
  }

  oss << "      \\bottomrule\n"
      << "    \\end{tabular}\n"
      << "  }\n"
      << "  \\vspace{-1.0em}\n"
      << "\\end{table*}";

  return oss.str();
}

static std::string update_existing_table(const fs::path& tex_path,
                                          const std::string& case_name,
                                          const std::string& dynus_row,
                                          const std::string& table_type) {
  std::ifstream f(tex_path);
  if (!f.is_open()) return "";

  std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  f.close();

  std::istringstream iss(content);
  std::string line;
  std::vector<std::string> updated_lines;
  bool row_updated = false;
  std::string current_case;
  std::vector<std::string> valid_cases = {"Easy", "Medium", "Hard"};

  while (std::getline(iss, line)) {
    std::string stripped = line;
    // Trim
    size_t s = stripped.find_first_not_of(" \t");
    if (s != std::string::npos) stripped = stripped.substr(s);

    // Track current case from multirow
    std::regex multirow_re(R"(\\multirow\{[^}]*\}\{[^}]*\}\{(\w+)\})");
    std::sregex_iterator it(line.begin(), line.end(), multirow_re);
    for (; it != std::sregex_iterator(); ++it) {
      std::string match = (*it)[1];
      for (auto& vc : valid_cases) {
        if (match == vc) current_case = match;
      }
    }

    // Case 1: DYNUS row on same line as multirow
    if (line.find(case_name) != std::string::npos && line.find("DYNUS") != std::string::npos &&
        line.find("&") != std::string::npos && line.find("multirow") != std::string::npos) {
      updated_lines.push_back(dynus_row);
      row_updated = true;
    }
    // Case 2: DYNUS row in separate line (static table)
    else if (current_case == case_name && line.find("DYNUS") != std::string::npos &&
             line.find("&") != std::string::npos && line.find("\\\\") != std::string::npos &&
             line.find("multirow") == std::string::npos) {
      updated_lines.push_back(dynus_row);
      row_updated = true;
    }
    // Case 3: unknown_dynamic format
    else if (table_type == "unknown_dynamic" && stripped.find(case_name) == 0 &&
             stripped.find("&") != std::string::npos && stripped.find("\\\\") != std::string::npos) {
      updated_lines.push_back(dynus_row);
      row_updated = true;
    }
    else {
      updated_lines.push_back(line);
    }
  }

  if (!row_updated) {
    std::cout << "  Warning: No matching " << case_name << " + DYNUS row found, appending before \\bottomrule...\n";
    for (int i = static_cast<int>(updated_lines.size()) - 1; i >= 0; i--) {
      if (updated_lines[i].find("\\bottomrule") != std::string::npos) {
        updated_lines.insert(updated_lines.begin() + i, dynus_row);
        break;
      }
    }
  } else {
    std::cout << "  Updated " << case_name << " row\n";
  }

  std::ostringstream result;
  for (size_t i = 0; i < updated_lines.size(); i++) {
    result << updated_lines[i];
    if (i + 1 < updated_lines.size()) result << "\n";
  }
  return result.str();
}

static std::string generate_latex_table(const Statistics& s, const std::string& case_name,
                                         const fs::path& existing_file,
                                         const std::string& table_type) {
  std::string dynus_row = generate_dynus_row(s, case_name, table_type);

  // Try updating existing table
  if (fs::exists(existing_file)) {
    std::cout << "  Found existing table, updating " << case_name << " row...\n";
    std::string updated = update_existing_table(existing_file, case_name, dynus_row, table_type);
    if (!updated.empty()) return updated;
  }

  // Generate new table
  if (table_type == "static") {
    // Extract data_values from dynus_row (everything after the 4th &)
    // For simplicity, just generate with full row
    std::string data_values;
    int amp_count = 0;
    for (size_t i = 0; i < dynus_row.size(); i++) {
      if (dynus_row[i] == '&') {
        amp_count++;
        if (amp_count == 4) {
          data_values = dynus_row.substr(i + 1);
          // Trim leading whitespace
          size_t s = data_values.find_first_not_of(" \t");
          if (s != std::string::npos) data_values = data_values.substr(s);
          break;
        }
      }
    }
    return generate_new_static_table(case_name, dynus_row, data_values);
  } else if (table_type == "unknown_dynamic") {
    return generate_new_unknown_dynamic_table(case_name, dynus_row);
  } else {
    return generate_new_dynamic_table(dynus_row);
  }
}

// ============================================================================
// Main analysis
// ============================================================================

static void analyze_single_case(const fs::path& data_dir, const std::string& output_name,
                                  const fs::path& latex_output, const std::string& table_type,
                                  Vec3 goal_pos) {
  // Extract case name
  std::string dir_name = data_dir.filename().string();
  std::string case_name = "Unknown";
  if (dir_name.find("easy_") == 0) case_name = "Easy";
  else if (dir_name.find("medium_") == 0) case_name = "Medium";
  else if (dir_name.find("hard_") == 0) case_name = "Hard";

  std::string sep(80, '=');
  std::cout << sep << "\n";
  std::cout << "ANALYZING " << case_name << " CASE\n";
  std::cout << sep << "\n";
  std::cout << "\nLoading data from: " << data_dir << "\n\n";

  // Load CSV
  fs::path csv_path = find_most_recent_csv(data_dir);
  if (csv_path.empty()) {
    std::cerr << "ERROR: No benchmark CSV files found in " << data_dir << "\n";
    return;
  }
  std::cout << "Loading: " << csv_path.filename().string() << "\n";
  auto trials = load_benchmark_csv(csv_path);
  std::cout << "Total trials loaded: " << trials.size() << "\n\n";

  // Load computation data
  std::cout << "Loading computation time data...\n";
  auto comp_stats = load_computation_data(data_dir);
  for (auto& t : trials) {
    auto it = comp_stats.find(t.trial_id);
    if (it != comp_stats.end()) {
      t.num_replans = it->second.num_replans;
      t.avg_replanning_time = it->second.avg_replanning_time;
      t.max_replanning_time = it->second.max_replanning_time;
      t.total_replanning_time = it->second.total_replanning_time / 1000.0;
      t.avg_global_planning_time = it->second.avg_global_planning_time;
      t.avg_sfc_corridor_time = it->second.avg_sfc_corridor_time;
      t.avg_local_traj_time = it->second.avg_local_traj_time;
    }
  }
  std::cout << "\n";

  // Process each trial's bag (SINGLE PASS)
  fs::path bags_dir = data_dir / "bags";
  if (fs::exists(bags_dir)) {
    // Determine which topics to read from bags
    std::string trajs_topic;
    bool read_tf = false;
    if (table_type == "unknown_dynamic") {
      trajs_topic = "/trajs_ground_truth";
    } else if (table_type == "dynamic") {
      read_tf = true;
    }
    // static: we don't need /tf or /trajs for collision (uses CSV obstacles)

    std::cout << "Processing rosbags (single-pass)...\n";
    Vec3 start_pos = {0.0, 0.0, 2.0};

    for (auto& trial : trials) {
      fs::path bag_path = bags_dir / ("trial_" + std::to_string(trial.trial_id));
      if (!fs::exists(bag_path)) {
        std::cout << "  Warning: Bag not found for trial " << trial.trial_id << "\n";
        continue;
      }

      std::cout << "  Trial " << trial.trial_id << ": reading bag... " << std::flush;

      // Single-pass read
      BagData bag_data = read_bag_single_pass(bag_path, trajs_topic, read_tf);
      std::cout << bag_data.agent.size() << " goal msgs";
      if (!bag_data.obstacles.empty())
        std::cout << ", " << bag_data.obstacles.size() << " obstacles";
      std::cout << "\n";

      if (bag_data.agent.empty()) continue;

      // 1) Path metrics
      auto path_metrics = compute_path_metrics(bag_data, goal_pos, start_pos);
      if (path_metrics.path_length.has_value()) {
        trial.path_length = path_metrics.path_length.value();
      }

      // 2) Violation analysis
      auto viol = analyze_violations(bag_data);
      trial.vel_violation_count = viol.vel_count;
      trial.vel_violation_total = viol.vel_total;
      trial.acc_violation_count = viol.acc_count;
      trial.acc_violation_total = viol.acc_total;
      trial.jerk_violation_count = viol.jerk_count;
      trial.jerk_violation_total = viol.jerk_total;

      // 3) Collision analysis
      if (table_type == "unknown_dynamic" || table_type == "dynamic") {
        auto coll = analyze_collisions(bag_data);
        trial.collision_count = coll.collision_count;
        trial.min_distance_to_obstacles = coll.min_distance;
        trial.collision_free_ratio = coll.collision_free_ratio;
        trial.collision = (coll.collision_count > 0);
        std::cout << "    Collisions: " << coll.collision_count
                  << ", min_dist: " << std::fixed << std::setprecision(3) << coll.min_distance << "m\n";
      } else if (table_type == "static") {
        // Static collisions need obstacle CSV
        std::map<std::string, std::string> case_csv_map = {
          {"Easy", "easy_forest_obstacle_parameters.csv"},
          {"Medium", "medium_forest_obstacle_parameters.csv"},
          {"Hard", "hard_forest_obstacle_parameters.csv"},
        };
        auto csv_it = case_csv_map.find(case_name);
        if (csv_it != case_csv_map.end()) {
          // Look for obstacle CSV relative to script location
          fs::path script_dir = fs::path(__FILE__).parent_path().parent_path().parent_path();
          fs::path obs_csv = script_dir / "benchmark_data" / "static" / csv_it->second;
          auto static_obs = load_static_obstacles(obs_csv);
          if (!static_obs.empty()) {
            auto coll = analyze_static_collisions(bag_data, static_obs);
            trial.collision_count = coll.collision_count;
            trial.min_distance_to_obstacles = coll.min_distance;
            trial.collision_free_ratio = coll.collision_free_ratio;
            trial.collision_unique_obstacles = coll.unique_obstacles;
            trial.collision = (coll.collision_count > 0);
          }
        }
      }

      std::cout << "    vel=" << trial.vel_violation_count << "/" << trial.vel_violation_total
                << ", acc=" << trial.acc_violation_count << "/" << trial.acc_violation_total
                << ", jerk=" << trial.jerk_violation_count << "/" << trial.jerk_violation_total << "\n";
    }
    std::cout << "\n";
  }

  // Compute statistics
  std::cout << "Computing statistics...\n";
  auto stats = compute_statistics(trials);
  print_statistics(stats);

  // Save CSV
  fs::path csv_output = data_dir / (output_name + ".csv");
  save_statistics_csv(stats, csv_output);

  // Generate LaTeX
  std::cout << "\nUpdating LaTeX table...\n";
  std::string latex = generate_latex_table(stats, case_name, latex_output, table_type);
  fs::create_directories(latex_output.parent_path());
  std::ofstream tex_file(latex_output);
  tex_file << latex;
  tex_file.close();
  std::cout << "LaTeX table updated for " << case_name << " case\n";

  std::cout << "\n" << sep << "\n";
  std::cout << case_name << " CASE ANALYSIS COMPLETE\n";
  std::cout << sep << "\n\n";
}

// ============================================================================
// CLI
// ============================================================================

struct Args {
  std::string data_dir;
  std::string output_name = "benchmark_summary";
  std::string latex_name = "dynamic_benchmark.tex";
  std::string config_name = "default";
  bool all_cases = false;
  std::string table_type = "dynamic";
  Vec3 goal_pos = {105.0, 0.0, 2.0};
};

static void print_usage() {
  std::cout << "Usage: analyze_benchmark [options]\n"
            << "  --data-dir DIR        Path to benchmark data directory (required)\n"
            << "  --output-name NAME    Output filename prefix (default: benchmark_summary)\n"
            << "  --latex-name NAME     LaTeX table filename (default: dynamic_benchmark.tex)\n"
            << "  --config-name NAME    Configuration name (default: default)\n"
            << "  --all-cases           Analyze all cases (easy, medium, hard)\n"
            << "  --table-type TYPE     Table format: dynamic, static, unknown_dynamic (default: dynamic)\n"
            << "  --goal-pos X Y Z      Goal position (default: 105.0 0.0 2.0)\n";
}

static Args parse_args(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--data-dir" && i + 1 < argc) { args.data_dir = argv[++i]; }
    else if (arg == "--output-name" && i + 1 < argc) { args.output_name = argv[++i]; }
    else if (arg == "--latex-name" && i + 1 < argc) { args.latex_name = argv[++i]; }
    else if (arg == "--config-name" && i + 1 < argc) { args.config_name = argv[++i]; }
    else if (arg == "--all-cases") { args.all_cases = true; }
    else if (arg == "--table-type" && i + 1 < argc) { args.table_type = argv[++i]; }
    else if (arg == "--goal-pos" && i + 3 < argc) {
      args.goal_pos.x = std::stod(argv[++i]);
      args.goal_pos.y = std::stod(argv[++i]);
      args.goal_pos.z = std::stod(argv[++i]);
    }
    else if (arg == "--help" || arg == "-h") { print_usage(); exit(0); }
    else {
      std::cerr << "Unknown argument: " << arg << "\n";
      print_usage();
      exit(1);
    }
  }

  if (args.data_dir.empty()) {
    std::cerr << "ERROR: --data-dir is required\n";
    print_usage();
    exit(1);
  }

  // Auto-set latex-name
  if (args.table_type == "unknown_dynamic" && args.latex_name == "dynamic_benchmark.tex") {
    args.latex_name = "unknown_dynamic_sim.tex";
  }

  return args;
}

int main(int argc, char** argv) {
  auto args = parse_args(argc, argv);

  fs::path latex_output = fs::path("/home/kkondo/paper_writing/DYNUS_v3/tables") / args.latex_name;

  if (args.all_cases) {
    fs::path base_dir(args.data_dir);

    if (!fs::exists(base_dir) || !fs::is_directory(base_dir)) {
      std::cerr << "ERROR: Directory does not exist: " << base_dir << "\n";
      return 1;
    }

    // Find case directories
    std::vector<fs::path> case_dirs;
    for (auto& pattern : {"easy_", "medium_", "hard_"}) {
      std::vector<fs::path> matching;
      for (auto& entry : fs::directory_iterator(base_dir)) {
        if (entry.is_directory() && entry.path().filename().string().find(pattern) == 0) {
          matching.push_back(entry.path());
        }
      }
      if (!matching.empty()) {
        std::sort(matching.begin(), matching.end());
        case_dirs.push_back(matching.back()); // Most recent
      }
    }

    if (case_dirs.empty()) {
      std::cerr << "Error: No case directories (easy_*, medium_*, hard_*) found in " << base_dir << "\n";
      return 1;
    }

    std::string sep(80, '=');
    std::cout << sep << "\n";
    std::cout << "DYNUS BENCHMARK ANALYZER (C++) - ALL CASES\n";
    std::cout << sep << "\n\n";
    std::cout << "Found " << case_dirs.size() << " case(s) to analyze:\n";
    for (auto& d : case_dirs) std::cout << "  - " << d.filename().string() << "\n";
    std::cout << "\n";

    for (auto& case_dir : case_dirs) {
      analyze_single_case(case_dir, args.output_name, latex_output, args.table_type, args.goal_pos);
    }

    std::cout << "\n" << sep << "\n";
    std::cout << "ALL CASES ANALYSIS COMPLETE\n";
    std::cout << sep << "\n";
    std::cout << "\nLaTeX table updated: " << latex_output << "\n\n";
  } else {
    fs::path single_dir(args.data_dir);
    if (!fs::exists(single_dir) || !fs::is_directory(single_dir)) {
      std::cerr << "ERROR: Directory does not exist: " << single_dir << "\n";
      return 1;
    }
    analyze_single_case(single_dir, args.output_name, latex_output, args.table_type, args.goal_pos);
  }

  return 0;
}
