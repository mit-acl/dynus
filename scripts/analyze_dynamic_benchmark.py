#!/usr/bin/env python3
"""
DYNUS Dynamic Benchmark Analyzer

Analyzes benchmark data from dynamic obstacle benchmarks and generates:
1. Statistical summary (console output)
2. CSV summary file
3. LaTeX table for paper

Usage:
    # Analyze all data in a directory
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/20260204_092346

    # With custom output names
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/20260204_092346 \
        --output-name my_results \
        --latex-name my_table.tex

    # Analyze multiple configurations
    python3 analyze_dynamic_benchmark.py --data-dir benchmark_data/default/*/benchmark_*.csv
"""

import argparse
import glob
import sys
from pathlib import Path
from typing import List, Dict, Tuple

import numpy as np
import pandas as pd
import math

# ROS2 bag reading
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_ROSBAG = True
except ImportError:
    HAS_ROSBAG = False
    print("Warning: rosbag2_py not available, bag-based collision analysis disabled")


class BoundingBox:
    """Axis-aligned bounding box for collision detection"""
    def __init__(self, min_x, max_x, min_y, max_y, min_z, max_z):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z

    @classmethod
    def from_center_and_half_extents(cls, cx, cy, cz, hx, hy, hz):
        return cls(cx - hx, cx + hx, cy - hy, cy + hy, cz - hz, cz + hz)

    def intersects(self, other):
        """Check if this AABB intersects with another AABB"""
        return not (self.max_x < other.min_x or self.min_x > other.max_x or
                    self.max_y < other.min_y or self.min_y > other.max_y or
                    self.max_z < other.min_z or self.min_z > other.max_z)

    def distance_to(self, other):
        """Calculate minimum distance between two AABBs"""
        if self.intersects(other):
            return 0.0
        dx = max(0.0, max(self.min_x, other.min_x) - min(self.max_x, other.max_x))
        dy = max(0.0, max(self.min_y, other.min_y) - min(self.max_y, other.max_y))
        dz = max(0.0, max(self.min_z, other.min_z) - min(self.max_z, other.max_z))
        return math.sqrt(dx*dx + dy*dy + dz*dz)


def interpolate_position(times: np.ndarray, positions: np.ndarray, t_query: float) -> np.ndarray:
    """Interpolate position at a given time"""
    if len(times) == 0:
        return np.zeros(3)
    if len(times) == 1:
        return positions[0]

    if t_query <= times[0]:
        return positions[0]
    if t_query >= times[-1]:
        return positions[-1]

    # Linear interpolation
    idx = np.searchsorted(times, t_query)
    if idx >= len(times):
        return positions[-1]

    t0, t1 = times[idx-1], times[idx]
    p0, p1 = positions[idx-1], positions[idx]

    alpha = (t_query - t0) / (t1 - t0) if t1 > t0 else 0.0
    return p0 + alpha * (p1 - p0)


def load_benchmark_data(data_pattern: str) -> pd.DataFrame:
    """Load benchmark CSV files matching pattern

    By default, only loads the most recent (largest) CSV file to avoid
    counting trials from partial/interrupted runs.
    """

    # Handle directory input
    data_path = Path(data_pattern)
    if data_path.is_dir():
        # Look for benchmark_default_*.csv files in directory (exclude benchmark_summary.csv)
        pattern = str(data_path / "benchmark_default_*.csv")
    else:
        pattern = data_pattern

    csv_files = glob.glob(pattern)

    if not csv_files:
        print(f"ERROR: No CSV files found matching: {pattern}")
        print(f"       Make sure you're pointing to a directory with benchmark_default_*.csv files")
        sys.exit(1)

    # Filter out any summary files that might match
    csv_files = [f for f in csv_files if 'summary' not in Path(f).name.lower()]

    if not csv_files:
        print(f"ERROR: No valid benchmark CSV files found (only found summary files)")
        sys.exit(1)

    # Sort by modification time and get the most recent file
    csv_files_sorted = sorted(csv_files, key=lambda f: Path(f).stat().st_mtime)
    most_recent_file = csv_files_sorted[-1]

    print(f"Found {len(csv_files)} benchmark CSV file(s)")
    print(f"Loading most recent file (to avoid counting partial runs): {Path(most_recent_file).name}")

    # Load only the most recent file
    df = pd.read_csv(most_recent_file)
    print(f"\nTotal trials loaded: {len(df)}")

    # Check if data looks valid
    if 'path_length' in df.columns:
        valid_data = (df['path_length'] > 0).sum()
        if valid_data == 0:
            print(f"\n⚠️  WARNING: All trials have path_length=0.0")
            print(f"   This suggests no trajectory data was collected.")
            print(f"   The benchmark may have crashed or not published odometry data.\n")

    return df


def load_computation_data(data_dir: Path) -> dict:
    """Load computation time data from num_*.csv files in csv/ subdirectory

    Returns dict mapping trial_id -> computation stats
    """
    csv_dir = data_dir / "csv"
    if not csv_dir.exists():
        print(f"  Warning: No csv/ subdirectory found at {csv_dir}")
        return {}

    # Find all num_*.csv files
    num_files = sorted(csv_dir.glob("num_*.csv"))
    if not num_files:
        print(f"  Warning: No num_*.csv files found in {csv_dir}")
        return {}

    print(f"  Loading computation data from {len(num_files)} num_*.csv file(s)...")

    computation_stats = {}

    for num_file in num_files:
        # Extract trial number from filename (num_0.csv -> trial 0)
        trial_id = int(num_file.stem.split('_')[1])

        try:
            # Read the CSV file
            comp_df = pd.read_csv(num_file)

            # Filter for successful replans (Result=1)
            successful_replans = comp_df[comp_df['Result'] == 1]

            if len(successful_replans) > 0:
                # Compute statistics for this trial
                stats = {
                    'num_replans': len(successful_replans),
                    'avg_replanning_time': successful_replans['Total replanning time [ms]'].mean(),
                    'max_replanning_time': successful_replans['Total replanning time [ms]'].max(),
                    'total_replanning_time': successful_replans['Total replanning time [ms]'].sum(),
                    'avg_global_planning_time': successful_replans['Global Planning Time [ms]'].mean(),
                    'avg_local_traj_time': successful_replans['Local Traj Time [ms]'].mean(),
                }

                # Check if CVX Decomposition Time column exists (might be called "CVX Decomposition Time" or "SFC Corridor Time")
                if 'CVX Decomposition Time [ms]' in comp_df.columns:
                    stats['avg_sfc_corridor_time'] = successful_replans['CVX Decomposition Time [ms]'].mean()
                elif 'SFC Corridor Time [ms]' in comp_df.columns:
                    stats['avg_sfc_corridor_time'] = successful_replans['SFC Corridor Time [ms]'].mean()
                else:
                    stats['avg_sfc_corridor_time'] = 0.0

                computation_stats[trial_id] = stats
            else:
                # No successful replans
                computation_stats[trial_id] = {
                    'num_replans': 0,
                    'avg_replanning_time': 0.0,
                    'max_replanning_time': 0.0,
                    'total_replanning_time': 0.0,
                    'avg_global_planning_time': 0.0,
                    'avg_sfc_corridor_time': 0.0,
                    'avg_local_traj_time': 0.0,
                }

        except Exception as e:
            print(f"    Warning: Failed to load {num_file.name}: {e}")
            continue

    print(f"  Loaded computation data for {len(computation_stats)} trial(s)")
    return computation_stats


def analyze_collision_from_bag(bag_path: Path, drone_bbox: Tuple[float, float, float]) -> Dict:
    """Analyze collisions from rosbag data

    Args:
        bag_path: Path to rosbag directory
        drone_bbox: Drone half-extents (hx, hy, hz)

    Returns:
        Dictionary with collision statistics
    """
    if not HAS_ROSBAG:
        print(f"  Warning: Cannot analyze bag {bag_path}, rosbag2_py not available")
        return {'collision_count': 0, 'min_distance': float('inf'), 'collision_free_ratio': 1.0}

    if not bag_path.exists():
        print(f"  Warning: Bag not found at {bag_path}")
        return {'collision_count': 0, 'min_distance': float('inf'), 'collision_free_ratio': 1.0}

    print(f"  Analyzing collision from bag: {bag_path.name}")

    # Setup bag reader
    storage_options = StorageOptions(uri=str(bag_path), storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Extract data
    agent_trajectory = []  # [(time, x, y, z), ...]
    obstacle_trajectories = {}  # {obs_id: [(time, x, y, z), ...]}

    # Message types
    goal_msg_type = get_message('dynus_interfaces/msg/Goal')
    tf_msg_type = get_message('tf2_msgs/msg/TFMessage')

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        timestamp = t / 1e9  # Convert to seconds

        if topic == '/NX01/goal':
            msg = deserialize_message(data, goal_msg_type)
            agent_trajectory.append((timestamp, msg.p.x, msg.p.y, msg.p.z))

        elif topic == '/tf':
            msg = deserialize_message(data, tf_msg_type)
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                # Look for obstacle frames (typically "obstacle_N" or similar)
                if 'obstacle' in frame_id.lower() or frame_id.startswith('obs_'):
                    trans = transform.transform.translation
                    if frame_id not in obstacle_trajectories:
                        obstacle_trajectories[frame_id] = []
                    obstacle_trajectories[frame_id].append((timestamp, trans.x, trans.y, trans.z))

    del reader

    if len(agent_trajectory) == 0:
        print(f"    Warning: No agent trajectory found in bag")
        return {'collision_count': 0, 'min_distance': float('inf'), 'collision_free_ratio': 1.0}

    # Convert to numpy arrays for efficient processing
    agent_times = np.array([t for t, x, y, z in agent_trajectory])
    agent_positions = np.array([[x, y, z] for t, x, y, z in agent_trajectory])

    obstacle_data = {}
    for obs_id, traj in obstacle_trajectories.items():
        if len(traj) > 0:
            times = np.array([t for t, x, y, z in traj])
            positions = np.array([[x, y, z] for t, x, y, z in traj])
            obstacle_data[obs_id] = {'times': times, 'positions': positions}

    print(f"    Loaded {len(agent_positions)} agent positions, {len(obstacle_data)} obstacles")

    # Collision checking
    drone_hx, drone_hy, drone_hz = drone_bbox
    # NOTE: URDF has collision box at 1.0m but visual mesh scaled to 0.8m
    # The /tf frames come from the visual center, so we use visual size (0.8m)
    # This matches what's published in /tf and what the user sees
    obs_half_extents = (0.4, 0.4, 0.4)  # Visual size: 0.8m cubes

    collisions = 0
    min_distance = float('inf')
    min_dist_info = None  # Track which obstacle/time gave min distance
    collision_free_segments = 0

    for i, (t, px, py, pz) in enumerate(zip(agent_times, agent_positions[:, 0],
                                              agent_positions[:, 1], agent_positions[:, 2])):
        drone_bbox_obj = BoundingBox.from_center_and_half_extents(px, py, pz, drone_hx, drone_hy, drone_hz)

        segment_collision_free = True
        for obs_id, obs_data in obstacle_data.items():
            # Interpolate obstacle position at this time
            obs_pos = interpolate_position(obs_data['times'], obs_data['positions'], t)

            # Pre-filter: only consider obstacles within reasonable proximity (5m center-to-center)
            center_dist = np.sqrt((px - obs_pos[0])**2 + (py - obs_pos[1])**2 + (pz - obs_pos[2])**2)
            if center_dist > 5.0:
                continue  # Skip obstacles too far away

            # Point-to-AABB distance (Euclidean)
            # Find closest point on obstacle box to drone center
            closest_x = np.clip(px, obs_pos[0] - obs_half_extents[0], obs_pos[0] + obs_half_extents[0])
            closest_y = np.clip(py, obs_pos[1] - obs_half_extents[1], obs_pos[1] + obs_half_extents[1])
            closest_z = np.clip(pz, obs_pos[2] - obs_half_extents[2], obs_pos[2] + obs_half_extents[2])

            # Euclidean distance from drone center to closest point on obstacle surface
            distance = np.sqrt((px - closest_x)**2 + (py - closest_y)**2 + (pz - closest_z)**2)

            # Also calculate per-axis distances for debugging
            dist_x = max(0.0, abs(px - obs_pos[0]) - obs_half_extents[0])
            dist_y = max(0.0, abs(py - obs_pos[1]) - obs_half_extents[1])
            dist_z = max(0.0, abs(pz - obs_pos[2]) - obs_half_extents[2])

            if distance < min_distance:
                min_distance = distance
                min_dist_info = {
                    'obstacle': obs_id,
                    'time': t,
                    'agent_pos': (px, py, pz),
                    'obs_pos': tuple(obs_pos),
                    'distance': distance,
                    'closest_point': (closest_x, closest_y, closest_z),
                    'axis_distances': (dist_x, dist_y, dist_z)
                }

            # Check for collision using bounding boxes
            obs_bbox = BoundingBox.from_center_and_half_extents(
                obs_pos[0], obs_pos[1], obs_pos[2],
                obs_half_extents[0], obs_half_extents[1], obs_half_extents[2]
            )
            if drone_bbox_obj.intersects(obs_bbox):
                segment_collision_free = False
                collisions += 1

        if segment_collision_free:
            collision_free_segments += 1

    collision_free_ratio = collision_free_segments / len(agent_positions) if len(agent_positions) > 0 else 1.0

    result = {
        'collision_count': collisions,
        'min_distance': min_distance if min_distance != float('inf') else 0.0,
        'collision_free_ratio': collision_free_ratio,
        'unique_obstacles': len(obstacle_data)
    }

    print(f"    Collisions: {collisions}, Min distance: {result['min_distance']:.3f}m")
    if min_dist_info:
        gap_cm = min_dist_info['distance'] * 100  # Convert to cm
        obs_size = 2 * obs_half_extents[0]  # Total obstacle size
        dist_x, dist_y, dist_z = min_dist_info['axis_distances']
        closest_pt = min_dist_info['closest_point']

        print(f"    Min distance details:")
        print(f"      Obstacle: {min_dist_info['obstacle']}")
        print(f"      Time: {min_dist_info['time']:.2f}s")
        print(f"      Drone center: ({min_dist_info['agent_pos'][0]:.2f}, {min_dist_info['agent_pos'][1]:.2f}, {min_dist_info['agent_pos'][2]:.2f})")
        print(f"      Closest point on obstacle: ({closest_pt[0]:.2f}, {closest_pt[1]:.2f}, {closest_pt[2]:.2f})")
        print(f"      Euclidean distance: {gap_cm:.1f}cm")
        print(f"      Distance per axis: X={dist_x*100:.1f}cm, Y={dist_y*100:.1f}cm, Z={dist_z*100:.1f}cm")
        print(f"      Note: Point-to-AABB distance from drone center to {obs_size*100:.0f}cm obstacle surface.")
    return result


def merge_computation_data(df: pd.DataFrame, computation_stats: dict) -> pd.DataFrame:
    """Merge computation statistics into the main benchmark dataframe"""
    if not computation_stats:
        return df

    # Update each row with computation data
    for idx, row in df.iterrows():
        trial_id = row['trial_id']
        if trial_id in computation_stats:
            stats = computation_stats[trial_id]
            df.at[idx, 'num_replans'] = stats['num_replans']
            df.at[idx, 'avg_replanning_time'] = stats['avg_replanning_time']
            df.at[idx, 'max_replanning_time'] = stats['max_replanning_time']
            df.at[idx, 'total_replanning_time'] = stats['total_replanning_time'] / 1000.0  # Convert ms to s
            df.at[idx, 'avg_global_planning_time'] = stats['avg_global_planning_time']
            df.at[idx, 'avg_sfc_corridor_time'] = stats['avg_sfc_corridor_time']
            df.at[idx, 'avg_local_traj_time'] = stats['avg_local_traj_time']

    return df


def compute_statistics(df: pd.DataFrame) -> dict:
    """Compute comprehensive statistics from benchmark data"""

    stats = {}

    # Total trials
    stats['total_trials'] = len(df)

    # Success metrics - success = goal reached AND collision-free
    stats['success_rate'] = ((df['goal_reached']) & (df['collision_count'] == 0)).mean() * 100  # percentage
    stats['timeout_rate'] = df['timeout_reached'].mean() * 100
    stats['collision_rate'] = df['collision'].mean() * 100

    # Filter successful trials for performance metrics (goal reached AND collision-free)
    successful = df[(df['goal_reached'] == True) & (df['collision_count'] == 0)]
    n_success = len(successful)

    if n_success == 0:
        print("WARNING: No successful trials found!")
        return stats

    stats['n_successful'] = n_success

    # Computation time metrics (convert to ms if needed)
    for metric in ['avg_local_traj_time', 'avg_global_planning_time',
                   'avg_sfc_corridor_time', 'avg_replanning_time']:
        if metric in successful.columns:
            values = successful[metric].dropna()
            if len(values) > 0:
                stats[f'{metric}_min'] = values.min()
                stats[f'{metric}_max'] = values.max()
                stats[f'{metric}_mean'] = values.mean()
                stats[f'{metric}_std'] = values.std()

    # Travel time
    if 'flight_travel_time' in successful.columns:
        values = successful['flight_travel_time'].dropna()
        if len(values) > 0:
            stats['flight_travel_time_min'] = values.min()
            stats['flight_travel_time_max'] = values.max()
            stats['flight_travel_time_mean'] = values.mean()
            stats['flight_travel_time_std'] = values.std()

    # Path length
    if 'path_length' in successful.columns:
        values = successful['path_length'].dropna()
        if len(values) > 0:
            stats['path_length_min'] = values.min()
            stats['path_length_max'] = values.max()
            stats['path_length_mean'] = values.mean()
            stats['path_length_std'] = values.std()

    # Path efficiency
    if 'path_efficiency' in successful.columns:
        values = successful['path_efficiency'].dropna()
        if len(values) > 0:
            stats['path_efficiency_mean'] = values.mean()

    # Jerk smoothness (RMS)
    if 'jerk_rms' in successful.columns:
        values = successful['jerk_rms'].dropna()
        if len(values) > 0:
            stats['jerk_rms_min'] = values.min()
            stats['jerk_rms_max'] = values.max()
            stats['jerk_rms_mean'] = values.mean()
            stats['jerk_rms_std'] = values.std()

    # Jerk integral
    if 'jerk_integral' in successful.columns:
        values = successful['jerk_integral'].dropna()
        if len(values) > 0:
            stats['jerk_integral_mean'] = values.mean()

    # Constraint violations (rates among successful trials)
    for viol_type in ['sfc', 'vel', 'acc', 'jerk']:
        col = f'{viol_type}_violation_count'
        if col in successful.columns:
            # Rate of trials with violations
            viol_rate = (successful[col] > 0).mean() * 100
            stats[f'{viol_type}_violation_rate'] = viol_rate

            # Average count when violations occur
            trials_with_viol = successful[successful[col] > 0]
            if len(trials_with_viol) > 0:
                stats[f'{viol_type}_violation_avg_count'] = trials_with_viol[col].mean()

    # Collision metrics
    if 'collision_count' in df.columns:
        stats['collision_count_total'] = df['collision_count'].sum()
        stats['collision_count_mean'] = df['collision_count'].mean()

        # Collision-free ratio
        stats['collision_free_rate'] = (df['collision_count'] == 0).mean() * 100

        # Among trials with collisions
        trials_with_coll = df[df['collision_count'] > 0]
        if len(trials_with_coll) > 0:
            stats['collision_penetration_max_avg'] = trials_with_coll['collision_penetration_max'].mean()
            stats['collision_unique_obstacles_avg'] = trials_with_coll['collision_unique_obstacles'].mean()

    # Number of replans
    if 'num_replans' in successful.columns:
        values = successful['num_replans'].dropna()
        if len(values) > 0:
            stats['num_replans_mean'] = values.mean()

    # Minimum distance to obstacles
    if 'min_distance_to_obstacles' in df.columns:
        # Use all trials (not just successful) for minimum distance
        values = df['min_distance_to_obstacles'].dropna()
        # Filter out inf values
        values = values[values != float('inf')]
        if len(values) > 0:
            stats['min_distance_to_obstacles_min'] = values.min()
            stats['min_distance_to_obstacles_max'] = values.max()
            stats['min_distance_to_obstacles_mean'] = values.mean()
            stats['min_distance_to_obstacles_std'] = values.std()
        else:
            stats['min_distance_to_obstacles_mean'] = 'N/A'
    else:
        stats['min_distance_to_obstacles_mean'] = 'N/A (not tracked)'

    return stats


def print_statistics(stats: dict):
    """Print statistics in a formatted way"""

    print("\n" + "="*80)
    print("BENCHMARK ANALYSIS RESULTS")
    print("="*80)

    print(f"\n{'OVERVIEW':-^80}")
    print(f"  Total trials: {stats.get('total_trials', 0)}")
    print(f"  Successful trials: {stats.get('n_successful', 0)}")
    print(f"  Success rate: {stats.get('success_rate', 0):.1f}%")
    print(f"  Timeout rate: {stats.get('timeout_rate', 0):.1f}%")
    print(f"  Collision rate: {stats.get('collision_rate', 0):.1f}%")

    print(f"\n{'COMPUTATION TIME (ms)':-^80}")
    for metric in ['avg_local_traj_time', 'avg_global_planning_time',
                   'avg_sfc_corridor_time', 'avg_replanning_time']:
        min_key = f'{metric}_min'
        max_key = f'{metric}_max'
        mean_key = f'{metric}_mean'
        std_key = f'{metric}_std'

        if mean_key in stats:
            label = metric.replace('avg_', '').replace('_', ' ').title()
            print(f"  {label}:")
            print(f"    Min: {stats.get(min_key, 0):.2f} ms")
            print(f"    Max: {stats.get(max_key, 0):.2f} ms")
            print(f"    Mean: {stats.get(mean_key, 0):.2f} ms")
            print(f"    Std: {stats.get(std_key, 0):.2f} ms")

    print(f"\n{'PERFORMANCE METRICS':-^80}")
    if 'flight_travel_time_mean' in stats:
        print(f"  Travel Time:")
        print(f"    Min: {stats.get('flight_travel_time_min', 0):.2f} s")
        print(f"    Max: {stats.get('flight_travel_time_max', 0):.2f} s")
        print(f"    Mean: {stats.get('flight_travel_time_mean', 0):.2f} s")
        print(f"    Std: {stats.get('flight_travel_time_std', 0):.2f} s")

    if 'path_length_mean' in stats:
        print(f"  Path Length:")
        print(f"    Min: {stats.get('path_length_min', 0):.2f} m")
        print(f"    Max: {stats.get('path_length_max', 0):.2f} m")
        print(f"    Mean: {stats.get('path_length_mean', 0):.2f} m")
        print(f"    Std: {stats.get('path_length_std', 0):.2f} m")

    if 'path_efficiency_mean' in stats:
        print(f"  Path Efficiency: {stats['path_efficiency_mean']:.3f}")

    print(f"\n{'SMOOTHNESS METRICS':-^80}")
    if 'jerk_rms_mean' in stats:
        print(f"  Jerk RMS:")
        print(f"    Min: {stats.get('jerk_rms_min', 0):.2f} m/s³")
        print(f"    Max: {stats.get('jerk_rms_max', 0):.2f} m/s³")
        print(f"    Mean: {stats.get('jerk_rms_mean', 0):.2f} m/s³")
        print(f"    Std: {stats.get('jerk_rms_std', 0):.2f} m/s³")

    if 'jerk_integral_mean' in stats:
        print(f"  Jerk Integral: {stats['jerk_integral_mean']:.2f}")

    print(f"\n{'CONSTRAINT VIOLATIONS':-^80}")
    for viol_type in ['sfc', 'vel', 'acc', 'jerk']:
        rate_key = f'{viol_type}_violation_rate'
        if rate_key in stats:
            label = viol_type.upper()
            rate = stats[rate_key]
            print(f"  {label} Violation Rate: {rate:.1f}%")

            avg_count_key = f'{viol_type}_violation_avg_count'
            if avg_count_key in stats:
                print(f"    Avg violations when occurs: {stats[avg_count_key]:.1f}")

    print(f"\n{'COLLISION METRICS':-^80}")
    print(f"  Collision-free rate: {stats.get('collision_free_rate', 0):.1f}%")
    print(f"  Total collision events: {stats.get('collision_count_total', 0):.0f}")
    print(f"  Mean collisions per trial: {stats.get('collision_count_mean', 0):.2f}")
    if 'collision_penetration_max_avg' in stats:
        print(f"  Avg max penetration (when collisions occur): {stats['collision_penetration_max_avg']:.4f} m")
    if 'collision_unique_obstacles_avg' in stats:
        print(f"  Avg unique obstacles hit: {stats['collision_unique_obstacles_avg']:.1f}")

    print(f"\n{'OTHER METRICS':-^80}")
    if 'num_replans_mean' in stats:
        print(f"  Average replans per trial: {stats['num_replans_mean']:.1f}")

    print(f"\n{'MINIMUM DISTANCE TO OBSTACLES':-^80}")
    if 'min_distance_to_obstacles_mean' in stats:
        mean_val = stats['min_distance_to_obstacles_mean']
        if isinstance(mean_val, str):
            print(f"  {mean_val}")
        else:
            print(f"  Min: {stats.get('min_distance_to_obstacles_min', 0):.3f} m")
            print(f"  Max: {stats.get('min_distance_to_obstacles_max', 0):.3f} m")
            print(f"  Mean: {stats.get('min_distance_to_obstacles_mean', 0):.3f} m")
            print(f"  Std: {stats.get('min_distance_to_obstacles_std', 0):.3f} m")
    else:
        print(f"  N/A (not tracked)")

    print("\n" + "="*80 + "\n")


def save_statistics_csv(stats: dict, output_path: Path):
    """Save statistics to CSV file"""

    # Create DataFrame from stats
    df = pd.DataFrame([stats])

    # Save to CSV
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)

    print(f"✓ Statistics saved to CSV: {output_path}")


def generate_latex_table(stats: dict, config_name: str = "default", case_name: str = "Unknown", existing_file: Path = None) -> str:
    """Generate LaTeX table with benchmark results, updating only matching case+DYNUS row if table exists"""

    # Compute DYNUS row data
    success_rate = stats.get('success_rate', 0)
    per_opt_time = stats.get('avg_local_traj_time_mean', 0)
    travel_time = stats.get('flight_travel_time_mean', 0)
    path_length = stats.get('path_length_mean', 0)
    jerk_integral = stats.get('jerk_integral_mean', 0)
    min_distance = stats.get('min_distance_to_obstacles_mean', 0)
    vel_viol = stats.get('vel_violation_rate', 0)
    acc_viol = stats.get('acc_violation_rate', 0)
    jerk_viol = stats.get('jerk_violation_rate', 0)

    # Format min_distance properly
    if isinstance(min_distance, str):
        min_dist_str = min_distance
    else:
        min_dist_str = f"{min_distance:.3f}"

    # Format data values (9 data columns)
    data_values = f"{success_rate:.1f} & {per_opt_time:.1f} & " \
                  f"{travel_time:.1f} & {path_length:.1f} & {jerk_integral:.1f} & {min_dist_str} & " \
                  f"{vel_viol:.1f} & {acc_viol:.1f} & {jerk_viol:.1f} \\\\"

    # Default row format for new tables (simple: case & algorithm & data)
    # For first row in case block: \multirow{4}{*}{Case} & DYNUS & data
    dynus_row = f"      \\multirow{{4}}{{*}}{{{case_name}}} & DYNUS & {data_values}"

    # Try to update existing table
    if existing_file and existing_file.exists():
        print(f"  Found existing table, updating {case_name} + DYNUS row...")
        try:
            content = existing_file.read_text()
            lines = content.split('\n')

            # Find and replace matching case + DYNUS row
            updated_lines = []
            row_updated = False
            for line in lines:
                stripped = line.strip()

                # Skip comments and rules
                if stripped.startswith('%') or stripped.startswith('\\midrule') or stripped.startswith('\\cmidrule'):
                    updated_lines.append(line)
                    continue

                # Match: \multirow{4}{*}{Case} & DYNUS & data...
                # This is the first DYNUS row in a case block
                if (case_name in line and 'DYNUS' in line and '&' in line and 'multirow' in line):
                    updated_lines.append(dynus_row)
                    print(f"  Updated {case_name} + DYNUS row with new data")
                    row_updated = True
                else:
                    updated_lines.append(line)

            if not row_updated:
                print(f"  Warning: No matching {case_name} + DYNUS row found, appending new row...")
                # Find the last data row (before \bottomrule) and insert before it
                for i in range(len(updated_lines) - 1, -1, -1):
                    if '\\bottomrule' in updated_lines[i]:
                        updated_lines.insert(i, dynus_row)
                        break

            return '\n'.join(updated_lines)
        except Exception as e:
            print(f"  Warning: Could not update existing table ({e}), generating new one...")

    # Generate new table if file doesn't exist or update failed
    latex = []
    latex.append("\\begin{table*}")
    latex.append("  \\caption{Dynamic obstacle benchmarking results: DYNUS performance with moving obstacles. "
                 "We report success rate, computation time, flight performance, smoothness, safety, and constraint violation metrics.}")
    latex.append("  \\label{tab:dynamic_benchmark}")
    latex.append("  \\centering")
    latex.append("  \\renewcommand{\\arraystretch}{1.2}")
    latex.append("  \\resizebox{\\textwidth}{!}{")
    latex.append("    \\begin{tabular}{c c c c c c c c c c c}")
    latex.append("      \\toprule")

    # Simple header (Case + Algorithm + data columns)
    latex.append("      \\multicolumn{1}{c}{\\textbf{Case}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Algorithm}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Success}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Comp. Time}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Performance}}")
    latex.append("      & \\multicolumn{1}{c}{\\textbf{Safety}}")
    latex.append("      & \\multicolumn{3}{c}{\\textbf{Constraint Violation}}")
    latex.append("      \\\\")

    # Column rules (11 columns: 1 case + 1 algorithm + 9 data)
    latex.append("      \\cmidrule(lr){3-3}")
    latex.append("      \\cmidrule(lr){4-4}")
    latex.append("      \\cmidrule(lr){5-7}")
    latex.append("      \\cmidrule(lr){8-8}")
    latex.append("      \\cmidrule(lr){9-11}")

    # Column headers
    latex.append("      & &")
    latex.append("      $R_{\\mathrm{succ}}$ [\\%] &")
    latex.append("      $T^{\\mathrm{per}}_{\\mathrm{opt}}$ [ms] &")
    latex.append("      $T_{\\mathrm{trav}}$ [s] &")
    latex.append("      $L_{\\mathrm{path}}$ [m] &")
    latex.append("      $S_{\\mathrm{jerk}}$ [m/s$^{2}$] &")
    latex.append("      $d_{\\mathrm{min}}$ [m] &")
    latex.append("      $\\rho_{\\mathrm{vel}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{acc}}$ [\\%] &")
    latex.append("      $\\rho_{\\mathrm{jerk}}$ [\\%]")
    latex.append("      \\\\")
    latex.append("      \\midrule")

    # Data row
    latex.append(dynus_row)

    latex.append("      \\bottomrule")
    latex.append("    \\end{tabular}")
    latex.append("  }")
    latex.append("  \\vspace{-1.0em}")
    latex.append("\\end{table*}")

    return "\n".join(latex)


def analyze_single_case(data_dir: Path, output_name: str, config_name: str, latex_output: Path):
    """Analyze a single case directory and update LaTeX table"""

    # Extract case name from directory (easy_, medium_, hard_)
    dir_name = data_dir.name
    if dir_name.startswith('easy_'):
        case_name = 'Easy'
    elif dir_name.startswith('medium_'):
        case_name = 'Medium'
    elif dir_name.startswith('hard_'):
        case_name = 'Hard'
    else:
        case_name = 'Unknown'  # Fallback for old directory names

    # Load data
    print("="*80)
    print(f"ANALYZING {case_name.upper()} CASE")
    print("="*80)
    print(f"\nLoading data from: {data_dir}")
    print(f"Case: {case_name}\n")

    df = load_benchmark_data(str(data_dir))

    # Load computation data from num_*.csv files
    if data_dir.is_dir():
        print("\nLoading computation time data...")
        computation_stats = load_computation_data(data_dir)
        if computation_stats:
            df = merge_computation_data(df, computation_stats)
            print("  ✓ Computation data merged successfully\n")
        else:
            print("  No computation data found (num_*.csv files)\n")

    # Analyze collisions from rosbags
    if data_dir.is_dir():
        bags_dir = data_dir / "bags"
        if bags_dir.exists() and HAS_ROSBAG:
            print("\nAnalyzing collisions from rosbags...")
            # Default drone bbox (can be loaded from dynus.yaml if needed)
            drone_bbox = (0.1, 0.1, 0.1)  # half-extents

            for idx, row in df.iterrows():
                trial_id = row['trial_id']
                bag_path = bags_dir / f"trial_{trial_id}"

                if bag_path.exists():
                    collision_result = analyze_collision_from_bag(bag_path, drone_bbox)
                    # Update dataframe with collision results
                    df.at[idx, 'collision_count'] = collision_result['collision_count']
                    df.at[idx, 'min_distance_to_obstacles'] = collision_result['min_distance']
                    df.at[idx, 'collision_free_ratio'] = collision_result['collision_free_ratio']
                    df.at[idx, 'collision'] = collision_result['collision_count'] > 0
                else:
                    print(f"  Warning: Bag not found for trial {trial_id}")

            print("  ✓ Collision analysis complete\n")
        elif bags_dir.exists() and not HAS_ROSBAG:
            print("\n  Warning: Bags found but rosbag2_py not available, skipping collision analysis\n")

    # Compute statistics
    print("Computing statistics...")
    stats = compute_statistics(df)

    # Print results
    print_statistics(stats)

    # Save CSV
    if data_dir.is_dir():
        output_dir = data_dir
    else:
        output_dir = data_dir.parent

    csv_output = output_dir / f"{output_name}.csv"
    save_statistics_csv(stats, csv_output)

    # Generate and save LaTeX table
    print("\nUpdating LaTeX table...")
    latex_code = generate_latex_table(stats, config_name, case_name=case_name, existing_file=latex_output)
    latex_output.parent.mkdir(parents=True, exist_ok=True)
    latex_output.write_text(latex_code)

    print(f"✓ LaTeX table updated for {case_name} case")

    # Summary
    print("\n" + "="*80)
    print(f"{case_name.upper()} CASE ANALYSIS COMPLETE")
    print("="*80)
    print(f"\nGenerated files:")
    print(f"  CSV: {csv_output}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze DYNUS dynamic benchmark data',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        '--data-dir',
        type=str,
        required=True,
        help='Path to benchmark data directory or CSV file pattern'
    )

    parser.add_argument(
        '--output-name',
        type=str,
        default='benchmark_summary',
        help='Output filename prefix (default: benchmark_summary)'
    )

    parser.add_argument(
        '--latex-name',
        type=str,
        default='dynamic_benchmark.tex',
        help='LaTeX table filename (default: dynamic_benchmark.tex)'
    )

    parser.add_argument(
        '--config-name',
        type=str,
        default='default',
        help='Configuration name for table caption (default: default)'
    )

    parser.add_argument(
        '--all-cases',
        action='store_true',
        help='Analyze all cases (easy, medium, hard) from parent directory'
    )

    args = parser.parse_args()

    # Determine which cases to analyze
    if args.all_cases:
        # Find all case directories in parent
        base_dir = Path(args.data_dir)
        if base_dir.is_file():
            base_dir = base_dir.parent

        # Look for easy_*, medium_*, hard_* directories
        case_dirs = []
        for pattern in ['easy_*', 'medium_*', 'hard_*']:
            matching = sorted(base_dir.glob(pattern))
            if matching:
                case_dirs.append(matching[-1])  # Use most recent

        if not case_dirs:
            print(f"Error: No case directories (easy_*, medium_*, hard_*) found in {base_dir}")
            return

        print("="*80)
        print("DYNUS DYNAMIC BENCHMARK ANALYZER - ALL CASES")
        print("="*80)
        print(f"\nFound {len(case_dirs)} case(s) to analyze:")
        for d in case_dirs:
            print(f"  - {d.name}")
        print()

        # Analyze each case
        latex_output = Path("/home/kkondo/paper_writing/DYNUS_v3/tables") / args.latex_name
        for case_dir in case_dirs:
            analyze_single_case(case_dir, args.output_name, args.config_name, latex_output)

        print("\n" + "="*80)
        print("ALL CASES ANALYSIS COMPLETE")
        print("="*80)
        print(f"\nLaTeX table updated: {latex_output}")
        print(f"  Include in paper: \\input{{{latex_output.name}}}\n")

    else:
        # Single case analysis
        latex_output = Path("/home/kkondo/paper_writing/DYNUS_v3/tables") / args.latex_name
        analyze_single_case(Path(args.data_dir), args.output_name, args.config_name, latex_output)



if __name__ == '__main__':
    main()
