#!/usr/bin/env python3
"""
This script checks for collisions between a drone (NX01/base_link) and a set of obstacles.
It reads ROS2 bag files (using rosbag2_py) stored in a folder (with subdirectories named like "num_0", "num_1", ...),
extracts /tf messages, and checks if at any time the Euclidean distance between the drone and any obstacle is
less than the specified drone radius.

Usage:
   python3 dynamic_obstacle_collision_checker.py --num_obstacles 20 --bag_folder /media/XXXXXXX/T7/dynus/tro_paper/yaw_optimization/benchmark2/bags/dynus --drone_radius 0.05
"""

import os
import argparse
import math
import bisect
from tqdm import tqdm

# rosbag2_py imports
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

def check_collision_in_bag(bag_path, num_obstacles, drone_radius, sample_interval=0.01):
    """
    Opens a bag (assumed to be stored with sqlite3 storage) at bag_path,
    reads all /tf messages and stores the transforms per frame.
    
    Then, it discretizes the time from the bag's start to end in increments of
    sample_interval (default: 0.01 sec, i.e. 10ms) and, at each sample time, looks up
    the latest transform for the drone (NX01/base_link) and for each obstacle (child_frame_id "obstacle_i").
    If at any sample the Euclidean distance between the drone and an obstacle is less than drone_radius,
    a collision is reported.
    
    Returns True if a collision is detected, else False.
    """
    # Set up the reader
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Dictionary to store transforms: { frame_id: [(time, (x, y, z)), ...] }
    transforms = {}
    start_time = None
    end_time = None

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/tf":
            continue
        # Convert timestamp to seconds
        t_sec = timestamp / 1e9
        if start_time is None or t_sec < start_time:
            start_time = t_sec
        if end_time is None or t_sec > end_time:
            end_time = t_sec
        
        try:
            msg = deserialize_message(data, TFMessage)
        except Exception as e:
            print(f"Error deserializing message: {e}")
            continue

        for transform in msg.transforms:
            frame = transform.child_frame_id.strip()
            pos = (transform.transform.translation.x,
                   transform.transform.translation.y,
                   transform.transform.translation.z)
            if frame not in transforms:
                transforms[frame] = []
            transforms[frame].append((t_sec, pos))
    
    # No tf data found?
    if start_time is None or end_time is None:
        print("No TF data found in the bag.")
        return False

    # Sort the lists for each frame by time
    for frame in transforms:
        transforms[frame].sort(key=lambda entry: entry[0])
    
    # Helper: get the latest transform for a given frame at or before t_sample
    def get_latest_transform(frame, t_sample):
        if frame not in transforms:
            return None
        times = [entry[0] for entry in transforms[frame]]
        idx = bisect.bisect_right(times, t_sample) - 1
        if idx >= 0:
            return transforms[frame][idx][1]
        return None

    total_steps = int((end_time - start_time) / sample_interval) + 1
    collision_found = False

    for step in tqdm(range(total_steps), desc="Processing time samples"):
        t = start_time + step * sample_interval

        # Lookup the drone's position at time t
        drone_pos = get_latest_transform("NX01/base_link", t)
        if drone_pos is None:
            continue

        # For each obstacle index, get its latest transform
        for i in range(num_obstacles):
            frame = f"obstacle_{i}"
            obs_pos = get_latest_transform(frame, t)
            if obs_pos is None:
                continue

            dx = drone_pos[0] - obs_pos[0]
            dy = drone_pos[1] - obs_pos[1]
            dz = drone_pos[2] - obs_pos[2]

            if abs(dx) < drone_radius + 0.5 and abs(dy) < drone_radius + 0.5 and abs(dz) < drone_radius + 0.5:
                collision_found = True
                print(f"Collision detected at t={t:.3f}s : drone_pos={drone_pos}, obstacle_{i}_pos={obs_pos}") 
                # Early exit if collision detected
                break
        if collision_found:
            break

    return collision_found

def main():
    parser = argparse.ArgumentParser(
        description="Check collision between drone and obstacles from rosbag2 files."
    )
    parser.add_argument('--num_obstacles', type=int, required=True,
                        help="Number of obstacles (expected indices: 0 .. num_obstacles-1)")
    parser.add_argument('--bag_folder', type=str, required=True,
                        help="Folder containing rosbag2 directories (e.g. num_0, num_1, ...)")
    parser.add_argument('--drone_radius', type=float, default=0.1,
                        help="Drone radius in meters (default 0.1)")
    args = parser.parse_args()

    bag_folder = args.bag_folder
    num_obstacles = args.num_obstacles
    drone_radius = args.drone_radius

    # List all subdirectories that start with "num_"
    bag_dirs = sorted([d for d in os.listdir(bag_folder) if d.startswith("num_") and os.path.isdir(os.path.join(bag_folder, d))])

    results = {}
    for bag_dir in bag_dirs:
        bag_path = os.path.join(bag_folder, bag_dir)
        print(f"Processing bag: {bag_path}")
        try:
            collision = check_collision_in_bag(bag_path, num_obstacles, drone_radius)
            results[bag_dir] = "collision" if collision else "no collision"
            print("Collision found!" if collision else "No collision found.")
        except Exception as e:
            print(f"Error processing bag {bag_dir}: {e}")
            results[bag_dir] = "error"

    # Write results to collision_check.txt in the same folder
    output_file = os.path.join(bag_folder, "collision_check.txt")
    with open(output_file, "w") as f:
        for bag_dir, result in results.items():
            f.write(f"{bag_dir}: {result}\n")

    print(f"Collision check results written to {output_file}")

if __name__ == '__main__':
    main()
