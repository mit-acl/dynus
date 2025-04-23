#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
#  * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
#  * All Rights Reserved
#  * Authors: XXXXX XXXXX, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch argument for the initial guess planner name
    ld.add_action(DeclareLaunchArgument(
        'global_planner',
        default_value='sjps',  # Default value if none is provided
        description='global_planner name to use'
    ))

    # Declare the obstacle arguments
    ld.add_action(DeclareLaunchArgument(
        'static_obs1_start_x',
        default_value='-5.0',
        description='static obstacle 1 start x'
    ))

    ld.add_action(DeclareLaunchArgument(
        'static_obs2_start_x',
        default_value='-5.0',
        description='static obstacle 2 start x'
    ))

    ld.add_action(DeclareLaunchArgument(
        'dynamic_obs_height',
        default_value='5.0',
        description='dynamic obstacle height'
    ))

    ld.add_action(DeclareLaunchArgument(
        'dynamic_obs_empty_space',
        default_value='0',
        description='where the empty space is in the dynamic obstacle'
    ))

    # Get the path to the dynus_node parameters file
    parameters_file = os.path.join(
        get_package_share_directory('dynus'),
        'config',
        'dynus.yaml'
    )

    # Get the dict of parameters from the yaml file
    with open(parameters_file, 'r') as file:
        parameters = yaml.safe_load(file)

    # Extract specific node parameters
    parameters = parameters['dynus_node']['ros__parameters']

    # Add parameters to be set from launch arguments
    parameters['global_planner'] = LaunchConfiguration('global_planner')
    parameters['static_obs1_start_x'] = LaunchConfiguration('static_obs1_start_x')
    parameters['static_obs2_start_x'] = LaunchConfiguration('static_obs2_start_x')
    parameters['dynamic_obs_height'] = LaunchConfiguration('dynamic_obs_height')
    parameters['dynamic_obs_empty_space'] = LaunchConfiguration('dynamic_obs_empty_space')
    parameters['use_benchmark'] = True
    
    # Create a node with the algorithm parameter
    node1 = Node(
        package='dynus',
        executable='dynus',
        name='dynus_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameters],
    )

    # Add the node to the launch description
    ld.add_action(node1)
    
    return ld
