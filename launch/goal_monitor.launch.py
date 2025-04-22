from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'goal_tolerance',
            default_value='0.5',
            description='Distance tolerance to consider a goal reached'
        ),
        # Node
        Node(
            package='dynus',  # Replace with your package name
            executable='goal_monitor_node.py',  # Replace with your node executable
            namespace='NX01',
            name='goal_monitor_node',
            output='screen',
            parameters=[
                {'goal_tolerance': LaunchConfiguration('goal_tolerance')},
            ]
        )
    ])
