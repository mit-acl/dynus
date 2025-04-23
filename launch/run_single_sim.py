#!/usr/bin/env python3
# File: launch_sim.py
import subprocess
import sys
import argparse

def run_tmux(cmd):
    """Run a tmux command and exit if an error occurs."""
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError:
        print(f"Error executing: {cmd}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(
        description="Launch multiagent simulation tmux session with a specified environment"
    )
    parser.add_argument(
        "--env",
        type=str,
        default="easy_forest",
        help=(
            "Environment name. Available options include:\n"
            "  high_res_forest\n"
            "  static_uncertainty_test2\n"
            "  static_uncertainty_test3\n"
            "  static_uncertainty_test4\n"
            "  office_faster\n"
            "  office\n"
            "  cave_start\n"
            "  cave_vertical\n"
            "  cave_person\n"
            "  forest3\n"
            "  yaw_benchmark\n"
            "  global_planner\n"
            "  multiagent_performance\n"
            "  path_push\n"
            "  XXXXX_office\n"
            "  ground_robot\n"
            "  multiagent_testing\n"
            "  empty_wo_ground\n"
            "  hospital\n"
            "  easy_forest\n"
            "  medium_forest\n"
            "  hard_forest\n"
            "  quadruped\n"
        ),
    )
    args = parser.parse_args()
    env_name = args.env

    # Mapping of environment names to their corresponding onboard command.
    env_cmds = {
        "high_res_forest": f"ros2 launch dynus onboard_dynus.launch.py x:=-35.0 y:=0.0 z:=1.5 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "static_uncertainty_test2": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=10.0 z:=3.0 yaw:=-90 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "static_uncertainty_test3": f"ros2 launch dynus onboard_dynus.launch.py x:=2.5 y:=-4.0 z:=1.0 yaw:=90 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "static_uncertainty_test4": f"ros2 launch dynus onboard_dynus.launch.py x:=4.0 y:=0.0 z:=3.0 yaw:=90 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "office_faster": f"ros2 launch dynus onboard_dynus.launch.py x:=30.0 y:=10.0 z:=1.5 yaw:=90 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "office": f"ros2 launch dynus onboard_dynus.launch.py x:=8.0 y:=1.0 z:=1.5 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "cave_start": f"ros2 launch dynus onboard_dynus.launch.py x:=8.0 y:=0.0 z:=1.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "cave_vertical": f"ros2 launch dynus onboard_dynus.launch.py x:=120.0 y:=-50.0 z:=-4.0 yaw:=-90 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "cave_person": f"ros2 launch dynus onboard_dynus.launch.py x:=80.0 y:=30.0 z:=1.0 yaw:=90 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "forest3": f"ros2 launch dynus onboard_dynus.launch.py x:=-50.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "yaw_benchmark": f"ros2 launch dynus onboard_dynus.launch.py x:=-30.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "global_planner": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "multiagent_performance": f"ros2 launch dynus onboard_dynus.launch.py x:=-50.0 y:=30.0 z:=2.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "path_push": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "XXXXX_office": f"ros2 launch dynus onboard_dynus.launch.py x:=-5.0 y:=6.0 z:=1.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "ground_robot": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=1.0 yaw:=0 namespace:=NX01 use_ground_robot:=true depth_camera_name:=d435 env:={env_name}",
        "multiagent_testing": f"ros2 launch dynus onboard_dynus.launch.py x:=-5.0 y:=0.0 z:=2.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "empty_wo_ground": f"ros2 launch dynus onboard_dynus.launch.py x:=-35.0 y:=0.0 z:=2.5 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "empty": f"ros2 launch dynus onboard_dynus.launch.py x:=-35.0 y:=0.0 z:=2.5 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
        "hospital": f"ros2 launch dynus onboard_dynus.launch.py x:=5.0 y:=20.0 z:=2.0 yaw:=-90 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "easy_forest": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "medium_forest": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "hard_forest": f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=false depth_camera_name:=d435 env:={env_name}",
        "quadruped" : f"ros2 launch dynus onboard_dynus.launch.py x:=-45.0 y:=0.0 z:=1.0 yaw:=0 namespace:=NX01 use_ground_robot:=false use_quadruped:=true env:={env_name}",
        "dynamic_debug" : f"ros2 launch dynus onboard_dynus.launch.py x:=0.0 y:=0.0 z:=3.0 yaw:=0 namespace:=NX01 use_obstacle_tracker:=true depth_camera_name:=d435 env:={env_name}",
    }

    if env_name not in env_cmds:
        print(f"Error: Environment '{env_name}' not recognized.")
        print("Available environments:")
        for key in env_cmds.keys():
            print("  -", key)
        sys.exit(1)

    env_cmd = env_cmds[env_name]

    # Common setup command for both panes.
    setup_cmd = "source /home/XXXXXXX/code/dynus_ws/install/setup.bash; export ROS_DOMAIN_ID=7;"

    # use_dyn_obs
    if env_name in ["empty_wo_ground", "empty", "dynamic_debug"]:
        use_dyn_obs = "true"
    else:
        use_dyn_obs = "false"

    # environment for dynamic_debug
    if env_name == "dynamic_debug":
        env_name = "empty_wo_ground"

    # environment for ground robot
    if env_name == "ground_robot":
        env_name = "easy_forest"

    # environment for quadruped
    use_quadruped = False
    if env_name == "quadruped":
        use_quadruped = True
        env_name = "easy_forest"
        # this is not really base station but we need this for quadruped sim and we don't need base_ station for quadruped
        base_station_cmd = ("ros2 launch go2_config gazebo_velodyne.launch.py rviz:=false use_sim_time:=false world_init_x:=0.0") 
    else:
        # Base station command (pane 0) now includes the env argument.
        base_station_cmd = (
            f"{setup_cmd} "
            "ros2 launch dynus base_dynus.launch.py "
            f"use_dyn_obs:={use_dyn_obs} "
            "use_gazebo_gui:=false "
            "use_rviz:=true "
            "use_ground_robot:=false "
            "benchmark_name:=benchmark2 "
            f"env:={env_name}"
        )

    # Onboard command (pane 1) with a sleep delay before launching.
    onboard_cmd = f"{setup_cmd} sleep 10; {env_cmd}"

    session_name = "multiagent_sim"
    window_name = "main"

    # Kill any existing tmux session with the same name.
    subprocess.run(f"tmux kill-session -t {session_name}", shell=True, stderr=subprocess.DEVNULL)

    # Create a new detached tmux session.
    run_tmux(f"tmux new-session -d -s {session_name} -n {window_name}")

    # Send the base station command to pane 0.
    run_tmux(f"tmux send-keys -t {session_name}:0.0 '{base_station_cmd}' C-m")

    # Split the window horizontally to create pane 1.
    run_tmux(f"tmux split-window -h -t {session_name}:0")
    run_tmux(f"tmux send-keys -t {session_name}:0.1 '{onboard_cmd}' C-m")

    # If the environment is 'cave_*', create an additional pane for object detection.
    if env_name == "cave_start" or env_name == "cave_person" or env_name == "cave_vertical":
        obj_det_cmd = f"{setup_cmd} conda activate yolo && ros2 launch dynus object_detection.launch.py"
        # Split pane 1 vertically to create a new pane (pane 2).
        run_tmux(f"tmux split-window -v -t {session_name}:0.1")
        run_tmux(f"tmux send-keys -t {session_name}:0.2 '{obj_det_cmd}' C-m")

    # If quadruped, we need to convert velodyne pointcloud timestamp to ros2 time
    if use_quadruped:
        cmd = "ros2 run dynus convert_velodyne_to_ros_time"
        run_tmux(f"tmux split-window -v -t {session_name}:0.1")
        run_tmux(f"tmux send-keys -t {session_name}:0.2 '{cmd}' C-m")

    # Arrange panes in a tiled layout.
    run_tmux(f"tmux select-layout -t {session_name}:0 tiled")

    # Attach to the tmux session.
    run_tmux(f"tmux attach-session -t {session_name}")

if __name__ == "__main__":
    main()
