from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Prefix for gdb debugging
    # -q: quiet
    # -ex run: immediately run after starting gdb
    # --args: pass remaining args as program args
    gdb_prefix = 'xterm -e gdb -q -ex run --args'

    node = Node(
        package='mighty',
        executable='local_traj_benchmark_node',
        name='local_traj_benchmark_node',
        output='screen',
        # Toggle prefix by setting use_gdb:=true/false
        # prefix=gdb_prefix,
        parameters=[{
            'visualize': True,
            # 'playback_period_sec': 2.0,
            'num_N_list': [4, 5, 6],
            # these values are selected after running them with a wide range and checked FASTER's min and max factors for each num_N
            'factor_initial': [2.2, 1.7, 1.5], # for [N = 4, 5, 6]
            'factor_final': [3.8, 2.2, 1.9],   # for [N = 4, 5, 6]
            # for single-threaded test
            # 'use_single_threaded': True,
            # 'planner_names': ["dynus", "faster"], # no dynus* for single-threaded test
            # for multi-threaded test
            'use_single_threaded': False,
            'planner_names': ["dynus", "dynus_star", "faster"],
        }],
    )

    return LaunchDescription([
        node,
    ])
