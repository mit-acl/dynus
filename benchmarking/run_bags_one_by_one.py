import subprocess

# Define the commands to run
commands = [
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_0 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_1 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_2 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_3 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_4 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_5 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_6 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_7 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_8 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/num_9 --topics /NX01/mid360_PointCloud2 /tf /tf_static -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_0_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_1_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_2_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_3_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_4_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_5_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_6_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_7_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_8_shareable.db3 -r 0.5",
    "ros2 bag play /media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/bags/dgp/processed/num_9_shareable.db3 -r 0.5"
]

# Source your ROS 2 workspace before running the commands
source_command = "source /home/kkondo/code/dynus_ws/install/setup.bash"

# Run each command sequentially
for command in commands:
    print(f"Running: {command}")
    process = subprocess.Popen(
        f"{source_command} && {command}", 
        shell=True, 
        executable="/bin/bash"
    )
    process.wait()  # Wait for the current command to finish
    print(f"Finished: {command}")

print("All commands executed.")
