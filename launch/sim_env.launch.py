from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from math import radians

def generate_launch_description():

    # Declare launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'quadrotor.urdf.xacro']),
        description='Path to the URDF file'
    )

    world_path_arg = DeclareLaunchArgument(
        'world_path', default_value=PathJoinSubstitution([FindPackageShare('dynus'), 'worlds', 'flight_space.world']),
        description='Path to the Gazebo world file'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='20', description='Initial x position of the quadrotor'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='9', description='Initial y position of the quadrotor'
    )

    z_arg = DeclareLaunchArgument(
        'z', default_value='0', description='Initial z position of the quadrotor'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='-90', description='Initial yaw angle of the quadrotor'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Flag to enable or disable RViz'
    )

    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):
        urdf_path = LaunchConfiguration('urdf_path').perform(context)
        world_path = LaunchConfiguration('world_path').perform(context)
        x = LaunchConfiguration('x').perform(context)
        y = LaunchConfiguration('y').perform(context)
        z = LaunchConfiguration('z').perform(context)
        yaw = LaunchConfiguration('yaw').perform(context)
        use_rviz = LaunchConfiguration('use_rviz').perform(context)

        # Robot state publisher node
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
                'use_sim_time': False
            }]
        )

        # Include Gazebo launch file
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ),
            launch_arguments={'world': world_path, 'use_sim_time': 'False', 'gui': 'True', 'enable_gpu': 'True'}.items()
        )

        # convert yaw [deg] to yaw [rad]
        yaw = str(radians(float(yaw)))

        # Spawn entity node for Gazebo
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', 'quadrotor', '-x', x, '-y', y, '-z', z, '-Y', yaw],
            output='screen'
        )

        # Conditional RViz node
        rviz_node = None
        if use_rviz == 'true':
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', PathJoinSubstitution([FindPackageShare('dynus'), 'rviz', 'dynus.rviz'])],
                output='screen'
            )

        # Return launch description
        # nodes_to_start = [robot_state_publisher_node, gazebo_launch, spawn_entity_node, static_transform_publisher_node]
        nodes_to_start = [robot_state_publisher_node, gazebo_launch, spawn_entity_node]

        if rviz_node:
            nodes_to_start.append(rviz_node)

        return nodes_to_start

    # Create launch description
    return LaunchDescription([
        urdf_path_arg,
        world_path_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        use_rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
