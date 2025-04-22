import os
import yaml
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def get_parameters():
    """Get the parameters from the yaml file

    Returns:
        dict : parameters from the yaml file
    """

    # Get the path to the parameters file
    parameters_path = os.path.join(
        get_package_share_directory('dynus'),
        'config',
        'dynus.yaml'
        )

    # Get the dict of parameters from the yaml file
    with open(parameters_path, 'r') as file:
        parameters = yaml.safe_load(file)

    # Extract specific node parameters
    parameters = parameters['dynus_node']['ros__parameters']

    return parameters

def trefoil(x: float, y: float, z: float, scale_x: float, scale_y: float, scale_z: float, offset: float, slower: float):
    """Generates a trefoil knot trajectory

    Args:
        x (float): obstacle's x position
        y (float): obstacle's y position
        z (float): obstacle's z position
        scale_x (float): scale factor for x
        scale_y (float): scale factor for y
        scale_z (float): scale factor for z
        offset (float): time offset for the trajectory
        slower (float): time scaling factor - higher values make the obstacles move slower

    Returns:
        str : x, y, z strings for the trefoil knot trajectory
    """    

    # Generate the trefoil knot trajectory
    tt = 't/' + str(slower) + '+'
    x_string = str(scale_x / 6.0) + '*(sin(' + tt + str(offset) + ')+2*sin(2*' + tt + str(offset) + '))' + '+' + str(x) 
    y_string = str(scale_y / 5.0) + '*(cos(' + tt + str(offset) + ')-2*cos(2*' + tt + str(offset) + '))' + '+' + str(y) 
    z_string = str(scale_z / 2.0) + '*(-sin(3*'+ tt +str(offset)+'))' + '+' + str(z)        

    return x_string, y_string, z_string

def line(x: float, y: float, z: float, scale_x: float, scale_y: float, scale_z: float, offset: float, slower: float):
    """Generates a line trajectory

    Args:
        x (float): obstacle's x position
        y (float): obstacle's y position
        z (float): obstacle's z position
        scale_x (float): scale factor for x
        scale_y (float): scale factor for y
        scale_z (float): scale factor for z
        offset (float): time offset for the trajectory
        slower (float): time scaling factor - higher values make the obstacles move slower

    Returns:
        str : x, y, z strings for the line trajectory
    """    

    tt = 't/' + str(slower) + '+' + str(offset)

    x_string = str(scale_x) + '*sin(' + tt + ')' + '+' + str(x)
    y_string = str(scale_y) + '*sin(' + tt + ')' + '+' + str(y)
    z_string = str(scale_z) + '*sin(' + tt + ')' + '+' + str(z)

    return x_string, y_string, z_string

def generate_launch_description():
    # Declare the benchmark_name argument
    benchmark_name_arg = DeclareLaunchArgument(
        'benchmark_name', default_value='default',
        description='Name of the benchmark to configure dynamic obstacles.'
    )

    # Get the benchmark_name value
    benchmark_name = LaunchConfiguration('benchmark_name')

    # Path to URDF file
    urdf_path = os.path.join(
        get_package_share_directory('dynus'),
        'urdf',
        'dyn_obstacle1.urdf.xacro'
    )

    # Launch description
    ld = LaunchDescription([benchmark_name_arg])

    # Add obstacles based on benchmark_name
    ld.add_action(OpaqueFunction(function=lambda context: configure_obstacles(context, urdf_path)))

    return ld

def configure_obstacles(context, urdf_path):
    """
    Configures obstacles based on the given benchmark_name.

    Args:
        context (LaunchContext): Context to access launch configuration values.
        urdf_path (str): Path to the URDF file for the obstacles.

    Returns:
        list: List of actions to add to the launch description.
    """
    benchmark_name = context.launch_configurations['benchmark_name']
    ld = LaunchDescription()

    # random obstacles  
    ld = generate_random_obstacle_ld(urdf_path)

    # controlled obstacles
    # ld = generate_controlled_obstacle_ld(urdf_path)

    # controlled obstacles for yaw benchmark
    # ld = generate_controlled_obstacle_for_yaw_benchmark_ld(urdf_path, benchmark_name)

    # controlled obstacles for path push visulaization 
    # ld = generate_controlled_obstacle_for_push_visualiation_ld(urdf_path, benchmark_name)

    return ld.entities

def generate_controlled_obstacle_ld(urdf_path: str):

    # map_range = [[-40.0, 35.0], [-40.0, 35.0], [2.0, 5.0]] # [x_min, x_max], [y_min, y_max], [z_min, z_max]
    map_range = [[-20.0, 20.0], [-5.0, 5.0], [2.0, 5.0]] # [x_min, x_max], [y_min, y_max], [z_min, z_max]
    scale_range = [[8.0, 20.0], [8.0, 20.0], [0.0, 3.0]] # [scale_x_min, scale_x_max], [scale_y_min, scale_y_max], [scale_z_min, scale_z_max]
    offset_range = [0.0, 2.0] # [offset_min, offset_max]
    slower_range = [4.0, 7.0] # [slower_min, slower_max]

    # Parameters
    ld = LaunchDescription()
    
    # # Obstacle 0
    # x = 3.0
    # y = 0.0
    # z = 2.0
    # scale_x = 1.0
    # scale_y = 2.0
    # scale_z = 2.0
    # offset = 0.0
    # slower = 2.0
    # ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, 'obstacle_0', 0)

    # # Obstacle 1
    # x = -1.0
    # y = -1.0
    # z = 3.0
    # scale_x = 16.0
    # scale_y = 0.0
    # scale_z = 1.0
    # offset = 1.0
    # slower = 8.0
    # ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, 'obstacle_1')

    # # Obstacle 2
    # x = 1.0
    # y = 1.0
    # z = 2.5
    # scale_x = 17.0
    # scale_y = 0.0
    # scale_z = 1.0
    # offset = 2.0
    # slower = 6.0
    # ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, 'obstacle_2')

    # obstacle_num = 5

    # for i in range(obstacle_num):
    #     x = -20.0 + i * (40.0 / obstacle_num)
    #     y = 0.0
    #     z = 2.0
    #     scale_x = 3.0
    #     scale_y = 3.0
    #     scale_z = 3.0
    #     offset = round(offset_range[0] + (offset_range[1] - offset_range[0]) * random.random(), 2)
    #     slower =  round(slower_range[0] + (slower_range[1] - slower_range[0]) * random.random(), 2)
    #     namespace = 'obstacle_' + str(i)
    #     ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, namespace, i)

    return ld

def generate_controlled_obstacle_for_push_visualiation_ld(urdf_path: str, benchmark_name: str):
    """
    Generate a list of obstacles arranged in a specific pattern for a yaw benchmark.

    Args:
        urdf_path (str): The path to the URDF file for the obstacles.

    Returns:
        LaunchDescription: The launch description with the configured obstacles.
    """

    ld = LaunchDescription()

    # Determine the position based on the pattern
    x = 4.0
    y = 0.0
    z = 3.0

    # Scales
    scale_x = 0.0
    scale_y = 4.0
    scale_z = 0.0

    # Get random values for the obstacle
    offset = 0.0
    slower = 4.0


    # Create a new obstacle and add it to the LaunchDescription
    ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, 'obstacle_0', 0)

    return ld

def generate_controlled_obstacle_for_yaw_benchmark_ld(urdf_path: str, benchmark_name: str):
    """
    Generate a list of obstacles arranged in a specific pattern for a yaw benchmark.

    Args:
        urdf_path (str): The path to the URDF file for the obstacles.

    Returns:
        LaunchDescription: The launch description with the configured obstacles.
    """

    # Parameters for obstacle generation
    x_limit = 25.0  # Max x range
    y_limit = 5.0   # Max y range
    x_increment = 5.0  # Increment in x direction
    num_obstacles = 10  # Number of obstacles to generate
    # Constant z
    z = 3.0
    # Random z value between 0 to 6
    randomized_z = []
    for i in range(num_obstacles):
        randomized_z.append(round(random.uniform(0.0, 6.0), 2))
    scale_x, scale_y, scale_z = 3.0, 3.0, 3.0  # Constant scale
    offset_range = [0.0, 10.0] # [offset_min, offset_max]
    slower_range = [3.0, 7.0] # [slower_min, slower_max]
    
    # Initialize the LaunchDescription
    ld = LaunchDescription()

    # generate random offset and slower values 
    random.seed(0)   
    
    if benchmark_name == 'benchmark1':

        # Generate obstacles in the specified pattern (benchmark 1)
        for i in range(num_obstacles):
            # Determine the position based on the pattern
            x = -x_limit + i * x_increment
            y = -y_limit if i % 2 == 0 else y_limit

            # Get random values for the obstacle
            offset = round(offset_range[0] + (offset_range[1] - offset_range[0]) * random.random(), 2)
            slower = round(slower_range[0] + (slower_range[1] - slower_range[0]) * random.random(), 2)

            # Create a new obstacle and add it to the LaunchDescription
            ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, f'obstacle_{i}', i)

    elif benchmark_name == 'benchmark2':

        # Generate obstacles in the specified pattern (benchmark 2)
        for i in range(num_obstacles):
            # Determine the position based on the pattern
            x = -x_limit + i * x_increment
            y = -y_limit if i % 4 == 0 else (y_limit if i % 4 == 2 else -y_limit + y_limit)

            # Get random values for the obstacle
            offset = round(offset_range[0] + (offset_range[1] - offset_range[0]) * random.random(), 2)
            slower = round(slower_range[0] + (slower_range[1] - slower_range[0]) * random.random(), 2)
            
            # Create a new obstacle and add it to the LaunchDescription
            ld = create_obstacle_ld(ld, x, y, randomized_z[i], scale_x, scale_y, scale_z, offset, slower, urdf_path, f'obstacle_{i}', i)

    return ld


def generate_random_obstacle_ld(urdf_path: str):

    # # Seed the random number generator
    # random.seed(3)

    # # Parameters
    # num_obstacles = 2
    # map_range = [[-7.0, -3.5], [-2.0, 2.0], [1.5, 2.5]] # [x_min, x_max], [y_min, y_max], [z_min, z_max]
    # scale_range = [[0.5, 0.5], [0.5, 0.5], [0.5, 0.5]] # [scale_x_min, scale_x_max], [scale_y_min, scale_y_max], [scale_z_min, scale_z_max]
    # offset_range = [0.0, 2.0] # [offset_min, offset_max]
    # slower_range = [3.0, 5.0] # [slower_min, slower_max]

    # for five dynamic obstacles
    # Seed the random number generator
    random.seed(0)

    # Parameters
    # urdf_path = os.path.join(get_package_share_directory('dynus'), 'urdf', 'dyn_obstacle1.urdf.xacro')
    num_obstacles = 20
    # map_range = [[-40.0, 35.0], [-40.0, 35.0], [2.0, 5.0]] # [x_min, x_max], [y_min, y_max], [z_min, z_max]
    # map_range = [[-40.0, 30.0], [-30.0, 30.0], [1.0, 4.0]] # [x_min, x_max], [y_min, y_max], [z_min, z_max]
    map_range = [[-20.0, 20.0], [-5.0, 5.0], [2.0, 6.0]] # Global planner Benchmarking
    scale_range = [[2.0, 4.0], [2.0, 4.0], [0.0, 2.0]] # [scale_x_min, scale_x_max], [scale_y_min, scale_y_max], [scale_z_min, scale_z_max]
    offset_range = [0.0, 2.0] # [offset_min, offset_max]
    slower_range = [4.0, 6.0] # [slower_min, slower_max]
    
    # parameters = get_parameters()
    
    # Create multiple nodes randomly
    ld = LaunchDescription()
    for i in range(num_obstacles):

        # Get random values for the obstacle
        # x = round(map_range[0][0] + (map_range[0][1] - map_range[0][0]) * random.random(), 2)
        x = map_range[0][0] + (map_range[0][1] - map_range[0][0]) / num_obstacles * i
        y = round(map_range[1][0] + (map_range[1][1] - map_range[1][0]) * random.random(), 2)
        z = round(map_range[2][0] + (map_range[2][1] - map_range[2][0]) * random.random(), 2)
        scale_x = round(scale_range[0][0] + (scale_range[0][1] - scale_range[0][0]) * random.random(), 2)
        scale_y = round(scale_range[1][0] + (scale_range[1][1] - scale_range[1][0]) * random.random(), 2)
        scale_z = round(scale_range[2][0] + (scale_range[2][1] - scale_range[2][0]) * random.random(), 2)
        offset = round(offset_range[0] + (offset_range[1] - offset_range[0]) * random.random(), 2)
        slower = round(slower_range[0] + (slower_range[1] - slower_range[0]) * random.random(), 2)
        namespace = 'obstacle_' + str(i)

        # Create obstacle launch description
        ld = create_obstacle_ld(ld, x, y, z, scale_x, scale_y, scale_z, offset, slower, urdf_path, namespace, i)

    return ld

def create_obstacle_ld(ld, x: float, y: float, z: float, scale_x: float, scale_y: float, scale_z: float, offset: float, slower: float, urdf_path: str, namespace: str, idx: int):

    # Create the trefoil knot trajectory
    traj_x, traj_y, traj_z = trefoil(x, y, z, scale_x, scale_y, scale_z, offset, slower)

    # Create the line trajectory
    # traj_x, traj_y, traj_z = line(x, y, z, scale_x, scale_y, scale_z, offset, slower)

    # Robot state publisher node
    ld.add_action(
        TimerAction( # This is added to spawn obstacles one by one - otherwise the computation is too heavy and cannot spawn all obstacles
            period=idx*0.5,
            actions=[Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                namespace=namespace,
                parameters=[{
                    'robot_description': ParameterValue(Command(['xacro ', urdf_path, ' traj_x:=', traj_x,
                                                                    ' traj_y:=', traj_y, ' traj_z:=', traj_z,
                                                                    ' namespace:=', namespace]), value_type=str),
                    'use_sim_time': False,
                    'frame_prefix': namespace + '/',
                }])]
        )
    )

    # Spawn entity node for Gazebo
    # Get the start position and yaw from the parameters
    ld.add_action(
        TimerAction(
            period=idx*0.5,
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                output='screen',
                namespace=namespace,
                arguments=['-topic', 'robot_description', '-entity', namespace, '-x', str(x), '-y', str(y), '-z', str(z)],
                )]
        )
    )

    return ld