#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
#  * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
#  * All Rights Reserved
#  * Authors: XXXXX XXXXX, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

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

def generate_launch_description():

    # namespace
    namespace_arg = DeclareLaunchArgument('namespace', default_value='quadrotor', description='Namespace of the nodes')

    def launch_setup(context, *args, **kwargs):

        namespace = LaunchConfiguration('namespace').perform(context)

        node = Node(
            package='dynus',  
            executable='convert_odom_to_state',
            name='convert_odom_to_state',
            namespace=namespace,
            remappings=[
                ('odom', 'odom/sample'),  # Remap incoming Odometry topic
                ('state', 'state')  # Remap outgoing State topic
            ],
            output='screen'
        )

        return [node]
    
    return LaunchDescription([
        namespace_arg,
        OpaqueFunction(function=launch_setup)
    ])
