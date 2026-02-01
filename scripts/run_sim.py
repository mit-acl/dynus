#!/usr/bin/env python3
"""
DYNUS Simulation Launcher

This script provides a unified interface to launch DYNUS simulations in two modes:
1. Multi-agent simulation with fake sensing (fake_sim)
2. Single-agent simulation with Gazebo and ACL mapper (gazebo)

Usage:
    # Multi-agent fake simulation (10 agents in a circle)
    python3 scripts/run_sim.py --mode multiagent --setup-bash install/setup.bash

    # Single-agent Gazebo simulation with default goal
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash

    # Single-agent Gazebo with custom goal
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash --goal 100 50 3

    # Custom number of agents for multiagent mode
    python3 scripts/run_sim.py --mode multiagent --setup-bash install/setup.bash --num-agents 5

    # Custom environment for Gazebo mode
    python3 scripts/run_sim.py --mode gazebo --setup-bash install/setup.bash --env easy_forest
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile
import yaml
from pathlib import Path


def find_setup_bash(args_setup_bash: str = None) -> Path:
    """Find setup.bash path. Requires explicit --setup-bash argument."""
    if not args_setup_bash:
        print("[ERROR] --setup-bash is required. Please specify the path to setup.bash", file=sys.stderr)
        print("  Example: python3 run_sim.py --mode gazebo --setup-bash install/setup.bash", file=sys.stderr)
        sys.exit(1)

    path = Path(args_setup_bash)
    if path.exists():
        return path

    print(f"[ERROR] Specified setup.bash not found: {args_setup_bash}", file=sys.stderr)
    sys.exit(1)


def generate_multiagent_positions(num_agents: int, radius: float = 10.0, z: float = 1.0):
    """Generate agent positions in a circle formation."""
    agents = []
    for i in range(num_agents):
        angle = 2 * math.pi * i / num_agents
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        # Yaw points toward center (opposite of position angle)
        yaw_deg = math.degrees(angle + math.pi)
        # Normalize to [-180, 180]
        if yaw_deg > 180:
            yaw_deg -= 360
        agents.append({
            'namespace': f'NX{i+1:02d}',
            'x': round(x, 3),
            'y': round(y, 3),
            'z': z,
            'yaw': round(yaw_deg, 1)
        })
    return agents


def generate_multiagent_yaml(setup_bash: Path, agents: list, ros_domain_id: int = 7) -> str:
    """Generate YAML for multi-agent fake simulation."""
    panes = []

    # Base station (simulator)
    panes.append({
        'shell_command': [
            'ros2 launch dynus simulator.launch.py'
        ]
    })

    # Agent panes
    for agent in agents:
        panes.append({
            'shell_command': [
                'sleep 10',
                f"ros2 launch dynus onboard_mighty.launch.py namespace:={agent['namespace']} "
                f"x:={agent['x']} y:={agent['y']} z:={agent['z']} yaw:={agent['yaw']}"
            ]
        })

    # Goal monitor
    panes.append({
        'shell_command': [
            'sleep 20',
            'ros2 launch dynus goal_monitor.launch.py'
        ]
    })

    yaml_content = {
        'session_name': 'dynus_sim',
        'windows': [{
            'window_name': 'main',
            'layout': 'tiled',
            'shell_command_before': [
                f'''if [ -z "$SETUP_BASH" ] || [ ! -f "$SETUP_BASH" ]; then
  echo "[ERROR] SETUP_BASH is missing or invalid: $SETUP_BASH" >&2
  exit 1
fi
. "$SETUP_BASH"''',
                f'export ROS_DOMAIN_ID={ros_domain_id}'
            ],
            'panes': panes
        }]
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def generate_gazebo_yaml(setup_bash: Path, goal: tuple,
                         env: str = 'hard_forest',
                         start_pos: tuple = (0, 0, 3.0), start_yaw: float = 0.0,
                         ros_domain_id: int = 7, use_rviz: bool = True,
                         use_gazebo_gui: bool = True, use_dyn_obs: bool = False,
                         use_mapper: bool = True) -> str:
    """Generate YAML for single-agent Gazebo simulation."""
    goal_x, goal_y, goal_z = goal
    start_x, start_y, start_z = start_pos

    panes = [
        # Base station with Gazebo
        {
            'shell_command': [
                f'ros2 launch dynus base_mighty.launch.py use_dyn_obs:={str(use_dyn_obs).lower()} '
                f'use_gazebo_gui:={str(use_gazebo_gui).lower()} use_rviz:={str(use_rviz).lower()} env:={env}'
            ]
        }
    ]

    # ACL mapper (optional)
    if use_mapper:
        panes.append({
            'shell_command': [
                'sleep 5',
                'ros2 launch global_mapper_ros global_mapper_node.launch.py quad:=NX01 depth_pointcloud_topic:=mid360_PointCloud2'
            ]
        })

    # Onboard agent NX01
    panes.append({
        'shell_command': [
            'sleep 5',
            f'ros2 launch dynus onboard_mighty.launch.py namespace:=NX01 x:={start_x} y:={start_y} z:={start_z} yaw:={start_yaw}'
        ]
    })

    # Goal sender
    panes.append({
        'shell_command': [
            'sleep 20',
            f"ros2 launch dynus goal_sender.launch.py list_agents:=\"['NX01']\" list_goals:=\"['[{goal_x}, {goal_y}, {goal_z}]']\""
        ]
    })

    yaml_content = {
        'session_name': 'dynus_sim',
        'windows': [{
            'window_name': 'main',
            'layout': 'tiled',
            'shell_command_before': [
                f'''if [ -z "$SETUP_BASH" ] || [ ! -f "$SETUP_BASH" ]; then
  echo "[ERROR] SETUP_BASH is missing or invalid: $SETUP_BASH" >&2
  exit 1
fi
. "$SETUP_BASH"''',
                f'export ROS_DOMAIN_ID={ros_domain_id}'
            ],
            'panes': panes
        }]
    }

    return yaml.dump(yaml_content, default_flow_style=False, sort_keys=False)


def main():
    parser = argparse.ArgumentParser(
        description='DYNUS Simulation Launcher',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        '--mode', '-m',
        choices=['multiagent', 'gazebo'],
        default='gazebo',
        help='Simulation mode: multiagent (fake sensing) or gazebo (single agent with ACL mapper) [default: gazebo]'
    )

    parser.add_argument(
        '--setup-bash', '-s',
        type=str,
        required=True,
        help='Path to setup.bash (required)'
    )

    parser.add_argument(
        '--goal', '-g',
        type=float,
        nargs=3,
        metavar=('X', 'Y', 'Z'),
        default=[50.0, 0.0, 3.0],
        help='Goal position for gazebo mode (default: 50.0 0.0 3.0)'
    )

    parser.add_argument(
        '--start', '-p',
        type=float,
        nargs=3,
        metavar=('X', 'Y', 'Z'),
        default=[0.0, 0.0, 3.0],
        help='Start position for gazebo mode (default: 0.0 0.0 3.0)'
    )

    parser.add_argument(
        '--start-yaw',
        type=float,
        default=0.0,
        help='Start yaw in radians for gazebo mode (default: 0.0)'
    )

    parser.add_argument(
        '--num-agents', '-n',
        type=int,
        default=10,
        help='Number of agents for multiagent mode (default: 10)'
    )

    parser.add_argument(
        '--radius', '-r',
        type=float,
        default=10.0,
        help='Circle radius for multiagent formation (default: 10.0)'
    )

    parser.add_argument(
        '--env', '-e',
        type=str,
        default='hard_forest',
        help='Gazebo environment (default: hard_forest). Options: easy_forest, medium_forest, hard_forest, empty, etc.'
    )

    parser.add_argument(
        '--ros-domain-id',
        type=int,
        default=7,
        help='ROS_DOMAIN_ID (default: 7)'
    )

    parser.add_argument(
        '--rviz',
        action='store_true',
        default=True,
        help='Enable RViz (default: True)'
    )

    parser.add_argument(
        '--no-rviz',
        action='store_true',
        help='Disable RViz'
    )

    parser.add_argument(
        '--gazebo-gui',
        action='store_true',
        default=False,
        help='Enable Gazebo GUI (default: False)'
    )

    parser.add_argument(
        '--no-gazebo-gui',
        action='store_true',
        help='Disable Gazebo GUI'
    )

    parser.add_argument(
        '--dyn-obs',
        action='store_true',
        help='Enable dynamic obstacles (default: False)'
    )

    parser.add_argument(
        '--no-mapper',
        action='store_true',
        help='Disable ACL mapper in gazebo mode'
    )

    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Print the generated YAML without launching'
    )

    args = parser.parse_args()

    # Find setup.bash path
    setup_bash = find_setup_bash(args.setup_bash)
    print(f"[INFO] Using setup.bash: {setup_bash}")

    # Determine sim_env and generate YAML
    if args.mode == 'multiagent':
        agents = generate_multiagent_positions(args.num_agents, args.radius)
        yaml_content = generate_multiagent_yaml(setup_bash, agents, args.ros_domain_id)
        print(f"[INFO] Mode: Multi-agent simulation with {args.num_agents} agents (fake_sim)")
    else:  # gazebo
        use_rviz = args.rviz and not args.no_rviz
        use_gazebo_gui = args.gazebo_gui and not args.no_gazebo_gui
        use_mapper = not args.no_mapper
        yaml_content = generate_gazebo_yaml(
            setup_bash,
            goal=tuple(args.goal),
            env=args.env,
            start_pos=tuple(args.start),
            start_yaw=args.start_yaw,
            ros_domain_id=args.ros_domain_id,
            use_rviz=use_rviz,
            use_gazebo_gui=use_gazebo_gui,
            use_dyn_obs=args.dyn_obs,
            use_mapper=use_mapper
        )
        print(f"[INFO] Mode: Single-agent Gazebo simulation")
        print(f"[INFO] Environment: {args.env}")
        print(f"[INFO] Start: ({args.start[0]}, {args.start[1]}, {args.start[2]}) yaw={args.start_yaw}")
        print(f"[INFO] Goal: ({args.goal[0]}, {args.goal[1]}, {args.goal[2]})")
        print(f"[INFO] RViz: {use_rviz}, Gazebo GUI: {use_gazebo_gui}, Dynamic Obs: {args.dyn_obs}, Mapper: {use_mapper}")

    if args.dry_run:
        print("\n[DRY RUN] Generated YAML:")
        print("-" * 60)
        print(yaml_content)
        print("-" * 60)
        return

    # Write temporary YAML file and launch with tmuxp
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        f.write(yaml_content)
        temp_yaml_path = f.name

    try:
        print(f"[INFO] Launching simulation with tmuxp...")
        print(f"[INFO] Attach to session: tmux attach -t dynus_sim")
        env = os.environ.copy()
        env['SETUP_BASH'] = str(setup_bash)
        subprocess.run(['tmuxp', 'load', temp_yaml_path], env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Failed to launch simulation: {e}", file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print("[ERROR] tmuxp not found. Install with: pip install tmuxp", file=sys.stderr)
        sys.exit(1)
    finally:
        # Clean up temp file
        os.unlink(temp_yaml_path)


if __name__ == '__main__':
    main()
