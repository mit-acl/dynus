import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from math import radians

def convert_str_to_bool(str):
    return True if (str == 'true' or str == 'True' or str == 1 or str == '1') else False

def generate_launch_description():

    # Declare launch arguments

    # initial position and yaw of the quadrotor
    x_arg = DeclareLaunchArgument('x', default_value='20.0', description='Initial x position of the quadrotor')
    y_arg = DeclareLaunchArgument('y', default_value='9.0', description='Initial y position of the quadrotor')
    z_arg = DeclareLaunchArgument('z', default_value='2.0', description='Initial z position of the quadrotor')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='180', description='Initial yaw angle of the quadrotor')

    # namespace
    namespace_arg = DeclareLaunchArgument('namespace', default_value='NX01', description='Namespace of the nodes')

    # environment
    env_arg = DeclareLaunchArgument('env', default_value='easy_forest', description='Environment name')

    # flag to indicate whether to start the obstacle tracker node
    use_obstacle_tracker_arg = DeclareLaunchArgument('use_obstacle_tracker', default_value='false', description='Flag to indicate whether to start the obstacle tracker node')

    # file name to store data
    data_file_arg = DeclareLaunchArgument('data_file', default_value='/media/kkondo/T7/dynus/tro_paper/global_planner_benchmarking/dgp.csv', description='File name to store data')

    # global planner
    global_planner_arg = DeclareLaunchArgument('global_planner', default_value='dgp', description='Global planner to use')

    # global planner benchmark
    use_benchmark_arg = DeclareLaunchArgument('use_benchmark', default_value='false', description='Flag to indicate whether to use the global planner benchmark')

    # flag to indicate whether to use ground robot
    use_ground_robot_arg = DeclareLaunchArgument('use_ground_robot', default_value='false', description='Flag to indicate whether to use the ground robot')

    # flat to incicate whether to use quadruped
    use_quadruped_arg = DeclareLaunchArgument('use_quadruped', default_value='false', description='Flag to indicate whether to use quadruped')

    # flag to indicte if this is hardware or simulation
    use_hardware_arg = DeclareLaunchArgument('use_hardware', default_value='false', description='Flag to indicate whether to use hardware or simulation')

    # flag to indicate whether to use t265 (odom) or vicon (pose & twist) for localization
    use_onboard_localization_arg = DeclareLaunchArgument('use_onboard_localization', default_value='false', description='Flag to indicate whether to use t265 or vicon for localization')

    # depth camera name
    depth_camera_name_arg = DeclareLaunchArgument('depth_camera_name', default_value='d435', description='Depth camera name')

    # p_n_mode
    p_n_mode_arg = DeclareLaunchArgument('p_n_mode', default_value='dynus', description='Flag to indicate whether to use p_n_mode')

    # num_N
    num_N_arg = DeclareLaunchArgument('num_N', default_value='4', description='Number of discretization')

    # use_yaw
    use_yaw_arg = DeclareLaunchArgument('use_yaw', default_value='true', description='Flag to indicate whether to use yaw')

    # use_hard_constr_for_final_state
    use_hard_constr_for_final_state_arg = DeclareLaunchArgument('use_hard_constr_for_final_state', default_value='false', description='Flag to indicate whether to use hard constraints for the final state')

    # Opaque function to launch nodes
    def launch_setup(context, *args, **kwargs):

        x = LaunchConfiguration('x').perform(context)
        y = LaunchConfiguration('y').perform(context)
        z = LaunchConfiguration('z').perform(context)
        yaw = LaunchConfiguration('yaw').perform(context)
        namespace = LaunchConfiguration('namespace').perform(context)
        env = LaunchConfiguration('env').perform(context)
        use_obstacle_tracker = convert_str_to_bool(LaunchConfiguration('use_obstacle_tracker').perform(context))
        data_file = LaunchConfiguration('data_file').perform(context)
        global_planner = LaunchConfiguration('global_planner').perform(context)
        use_benchmark = convert_str_to_bool(LaunchConfiguration('use_benchmark').perform(context))
        use_ground_robot = convert_str_to_bool(LaunchConfiguration('use_ground_robot').perform(context))
        use_quadruped = convert_str_to_bool(LaunchConfiguration('use_quadruped').perform(context))
        use_hardware = convert_str_to_bool(LaunchConfiguration('use_hardware').perform(context))
        use_onboard_localization = convert_str_to_bool(LaunchConfiguration('use_onboard_localization').perform(context))
        depth_camera_name = LaunchConfiguration('depth_camera_name').perform(context)
        p_n_mode = LaunchConfiguration('p_n_mode').perform(context)
        num_N = LaunchConfiguration('num_N').perform(context)
        use_yaw = convert_str_to_bool(LaunchConfiguration('use_yaw').perform(context))
        use_hard_constr_for_final_state = convert_str_to_bool(LaunchConfiguration('use_hard_constr_for_final_state').perform(context))

        # The path to the urdf file
        if use_quadruped: # quadruped
            urdf_path=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'quadrotor.urdf.xacro']) # this doesn't matter since actual urdf is launched in gazebo_velodyne.launch.py
            parameters_path=os.path.join(get_package_share_directory('dynus'), 'config', 'dynus_sim_quadruped.yaml')
        elif use_ground_robot and use_hardware: # Red Rover hardware
            urdf_path=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'p3at.urdf.xacro'])
            parameters_path=os.path.join(get_package_share_directory('dynus'), 'config', 'dynus_hw_ground_robot.yaml')
        elif use_ground_robot and not use_hardware: # Red Rover simulation
            urdf_path=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'p3at.urdf.xacro'])
            parameters_path=os.path.join(get_package_share_directory('dynus'), 'config', 'dynus_sim_ground_robot.yaml')
        elif use_hardware: # UAV hardware
            urdf_path=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'quadrotor.urdf.xacro']) # this doesn't matter 
            parameters_path=os.path.join(get_package_share_directory('dynus'), 'config', 'dynus_hw_quadrotor.yaml')
        else: # UAV simulation
            urdf_path=PathJoinSubstitution([FindPackageShare('dynus'), 'urdf', 'quadrotor.urdf.xacro'])
            parameters_path=os.path.join(get_package_share_directory('dynus'), 'config', 'dynus.yaml')

        # Use fine tuned parameters for the specific environment
        if env == "high_res_forest":
            parameters_path = os.path.join(get_package_share_directory('dynus'), 'config', 'high_res_forest_param.yaml')
        elif env == "office":
            parameters_path = os.path.join(get_package_share_directory('dynus'), 'config', 'office_param.yaml')
        elif env == "cave_start":
            parameters_path = os.path.join(get_package_share_directory('dynus'), 'config', 'cave_param.yaml')
        elif env == "easy_forest":
            parameters_path = os.path.join(get_package_share_directory('dynus'), 'config', 'easy_forest_param.yaml')
        elif env == "empty":
            parameters_path = os.path.join(get_package_share_directory('dynus'), 'config', 'dynamic_obstacle_empty_param.yaml')

        # Get the dict of parameters from the yaml file
        with open(parameters_path, 'r') as file:
            parameters = yaml.safe_load(file)

        # Extract specific node parameters
        parameters = parameters['dynus_node']['ros__parameters']
    
        # Update parameters for benchmarking
        parameters['file_path'] = data_file
        parameters['use_benchmark'] = bool(use_benchmark)
        if use_benchmark:
            parameters['global_planner'] = global_planner
            parameters['p_n_mode'] = p_n_mode
            parameters['num_N'] = int(num_N)
            parameters['use_yaw'] = bool(use_yaw)
            parameters['use_hard_constr_for_final_state'] = bool(use_hard_constr_for_final_state)

        if use_quadruped:
            lidar_point_could_topic = '/velodyne_points_ros_time'
        else:
            lidar_point_could_topic = 'livox/lidar' if use_hardware else 'mid360_PointCloud2'
            
        # Create a Dynus node
        dynus_node = Node(
                    package='dynus',
                    executable='dynus',
                    name='dynus_node',
                    namespace=namespace,
                    output='screen',
                    emulate_tty=True,
                    parameters=[parameters],
                    remappings=[('lidar_cloud_in', lidar_point_could_topic),
                                ('depth_camera_cloud_in', f'{depth_camera_name}/depth/color/points')],
                    # prefix='xterm -e gdb -q -ex run --args', # gdb debugging
                    # arguments=['--ros-args', '--log-level', 'error']
        )

        # Robot state publisher node
        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_path, ' namespace:=', namespace, ' d435_range_max_depth:=', str(parameters['depth_camera_depth_max'])]), value_type=str),
                'use_sim_time': False,
                'frame_prefix': namespace + '/',
            }],
            arguments=['--ros-args', '--log-level', 'error']
        )

        # Spawn entity node for Gazebo
        # Get the start position and yaw from the parameters
        yaw = str(radians(float(yaw)))
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            namespace=namespace,
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=['-topic', 'robot_description', '-entity', namespace, '-x', x, '-y', y, '-z', z, '-Y', yaw, '--ros-args', '--log-level', 'error'],
        )

        # dynus command node (TODO: i don't think we are using this)
        dynus_command_node = Node(
            package='dynus',
            executable='dynus_commands.py',
            name='dynus_command',
            namespace=namespace,
            # output='screen',
            emulate_tty=True
        )

        # When using ground robot, we don't need to send the exact state to gazebo - the state will be taken care of by wheel controllers
        send_state_to_gazebo = False if use_ground_robot else True
        # Create a fake sim node
        fake_sim_node = Node(
                    package='dynus',
                    executable='fake_sim',
                    name='fake_sim',
                    namespace=namespace,
                    emulate_tty=True,
                    parameters=[{"start_pos": [float(x), float(y), float(z)], 
                                 "start_yaw": float(yaw),
                                 "send_state_to_gazebo": send_state_to_gazebo,
                                 "use_ground_robot": use_ground_robot,
                                 "visual_level": parameters['visual_level']}],
        )
        
        # Create an obstacle tracker node
        obstacle_tracker_node = Node(
            package='dynus',
            executable='obstacle_tracker_node',
            namespace=namespace,
            name='obstacle_tracker_node',
            emulate_tty=True,
            parameters=[parameters],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            output='screen',
            remappings=[('point_cloud', f'{depth_camera_name}/depth/color/points')],
        )

        if not use_quadruped:
            cmd_vel_topic_name = 'cmd_vel_auto' if use_hardware else 'cmd_vel'
        else:
            cmd_vel_topic_name = '/cmd_vel'

        # Convert goal to cmd_vel for ground robot
        goal_to_cmd_vel_node = Node(
            package='dynus',
            executable='convert_goal_to_cmd_vel',
            name='convert_goal_to_cmd_vel',
            namespace=namespace,
            emulate_tty=True,
            output='screen',
            parameters=[parameters,
                        {"x": float(x),
                         "y": float(y),
                         "z": float(z),
                         "yaw": float(yaw),
                         "cmd_vel_topic_name": cmd_vel_topic_name}],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Convert goal to cmd_vel for quadruped
        quadruped_goal_to_cmd_vel_node = Node(
            package='dynus',
            # executable='quadruped_convert_goal_to_cmd_vel',
            executable='convert_goal_to_cmd_vel',
            name='quadruped_convert_goal_to_cmd_vel',
            namespace=namespace,
            emulate_tty=True,
            output='screen',
            parameters=[parameters,
                        {"x": float(x),
                         "y": float(y),
                         "z": float(z),
                         "yaw": float(yaw),
                         "cmd_vel_topic_name": cmd_vel_topic_name}],
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Convert odom (from T265) to state
        odom_to_state_node = Node(
            package='dynus',
            executable='convert_odom_to_state',
            name='convert_odom_to_state',
            namespace=namespace,
            remappings=[
                ('odom', 'dlio/odom_node/odom'),  # Remap incoming Odometry topic
                ('state', 'state')  # Remap outgoing State topic
            ],
            emulate_tty=True,
            output='screen',
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Converyt odom to state (for quadruped)
        quadruped_odom_to_state_node = Node(
            package='dynus',
            executable='quadruped_convert_odom_to_state',
            name='quadruped_convert_odom_to_state',
            namespace=namespace,
            remappings=[
                ('odom', 'dlio/odom_node/odom'),  # Remap incoming Odometry topic
                ('state', 'state')  # Remap outgoing State topic
            ],
            emulate_tty=True,
            output='screen',
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Convert pose and twist (from Vicon) to state
        pose_twist_to_state_node = Node(
            package='dynus',
            executable='convert_vicon_to_state',
            name='convert_vicon_to_state',
            namespace=namespace,
            remappings=[
                ('world', 'world'),  # Remap incoming PoseStamped topic
                ('twist', 'twist'),  # Remap incoming TwistStamped topic
                ('state', 'state')   # Remap outgoing State topic
            ],
            emulate_tty=True,
            output='screen',
            # prefix='xterm -e gdb -ex run --args', # gdb debugging
            # arguments=['--ros-args', '--log-level', 'error']
        )

        # Create a static transform publisher node (this is for quadruped)
        static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            output='screen',
            # The arguments are:
            # x, y, z, qx, qy, qz, qw, parent_frame, child_frame
            # For an identity transform: translation is (0,0,0) and rotation is (0,0,0,1)
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        )

        # Return launch description
        if not use_quadruped:
            if use_hardware and use_onboard_localization:
                nodes_to_start = [dynus_node, odom_to_state_node, dynus_command_node] # use T265 for localization
            elif use_hardware and not use_onboard_localization:
                nodes_to_start = [dynus_node, pose_twist_to_state_node, dynus_command_node] # use Vicon for localization
            else:
                nodes_to_start = [dynus_node, robot_state_publisher_node, spawn_entity_node, dynus_command_node, fake_sim_node] # simulation
        else:
            nodes_to_start = [dynus_node, dynus_command_node, quadruped_odom_to_state_node]

        nodes_to_start.append(obstacle_tracker_node) if use_obstacle_tracker else None
        nodes_to_start.append(goal_to_cmd_vel_node) if use_ground_robot else None
        nodes_to_start.append(static_tf_node) if use_quadruped else None
        nodes_to_start.append(quadruped_goal_to_cmd_vel_node) if use_quadruped else None

        return nodes_to_start

    # Create launch description
    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        namespace_arg,
        env_arg,
        use_obstacle_tracker_arg,
        data_file_arg,
        global_planner_arg,
        use_benchmark_arg,
        use_ground_robot_arg,
        use_quadruped_arg,
        use_hardware_arg,
        use_onboard_localization_arg,
        depth_camera_name_arg,
        p_n_mode_arg,
        num_N_arg,
        use_yaw_arg,
        use_hard_constr_for_final_state_arg,
        OpaqueFunction(function=launch_setup)
    ])