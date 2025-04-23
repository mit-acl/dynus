#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
#  * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
#  * All Rights Reserved
#  * Authors: XXXXX XXXXX, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='dynus',
            executable='convert_vicon_to_state', 
            name='convert_vicon_to_state',
            remappings=[
                ('world', 'world'),  # Remap incoming PoseStamped topic
                ('twist', 'twist'),  # Remap incoming TwistStamped topic
                ('state', 'state')   # Remap outgoing State topic
            ],
            output='screen'
        )
    ])
