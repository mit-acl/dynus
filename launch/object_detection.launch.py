#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('image_topic', default_value='d435/color/image_raw'),
        DeclareLaunchArgument('yolo_model', default_value='/home/kkondo/code/yolo/yolo11n.pt'),
        DeclareLaunchArgument('output_topic', default_value='yolo/image_yolo'),
        DeclareLaunchArgument('successful_detection_output_topic', default_value='successful_detection'),
        DeclareLaunchArgument('target_object_name', default_value='person'),
        DeclareLaunchArgument('publish_frequency', default_value='10.0'),
        DeclareLaunchArgument('namespace', default_value='NX01'),

        # Node
        Node(
            package='dynus',  # Replace with your package name
            executable='object_detection_node.py',       # Replace with the name of your node's executable
            namespace=LaunchConfiguration('namespace'),
            name='yolo_node',
            output='screen',
            parameters=[
                {'image_topic': LaunchConfiguration('image_topic')},
                {'yolo_model': LaunchConfiguration('yolo_model')},
                {'output_topic': LaunchConfiguration('output_topic')},
                {'successful_detection_output_topic': LaunchConfiguration('successful_detection_output_topic')},
                {'target_object_name': LaunchConfiguration('target_object_name')},
                {'publish_frequency': LaunchConfiguration('publish_frequency')},
            ]
        )
    ])
