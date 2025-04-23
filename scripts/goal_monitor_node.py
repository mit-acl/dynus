#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, XXXXX XXXXX, XXXXXXXXX XXXXXXXX XXXXXXXXXXX
#  * XXXXXXXX XXXXXXXXX XXXXXXXXXXXX
#  * All Rights Reserved
#  * Authors: XXXXX XXXXX, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import State
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Header
import math

class GoalMonitorNode(Node):
    def __init__(self):
        super().__init__('goal_monitor_node')

        # Parameters
        self.declare_parameter('goal_tolerance', 0.5)  # Distance tolerance to consider goal reached
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_points = [[17.0, 4.0, 1.0], [17.0, -3.0, 3.0], [0.0, -3.0, 1.0], [0.0, 4.0, 1.0]]
        self.distance_check_frequency = 1.0  # Frequency to check the distance to the goal
        self.current_goal_index = 0

        # Publishers and Subscribers
        self.state_sub = self.create_subscription(State, 'state', self.state_callback, 10)
        self.term_goal_pub = self.create_publisher(PoseStamped, 'term_goal', 10)

        # Timer to check the distance to the current goal
        self.goal_timer = self.create_timer(self.distance_check_frequency, self.distance_check_callback)

        # Timer to publish the current goal periodically
        self.term_goal_timer = self.create_timer(1.0, self.publish_term_goal)

        # Data to store
        self.current_position = Vector3()

        self.get_logger().info("Goal Monitor Node initialized.")

    def state_callback(self, msg: State):

        """Callback for monitoring the drone's position."""
        self.current_position = msg.pos

    def distance_check_callback(self):

        # Get the current goal point
        goal_x, goal_y, goal_z = self.goal_points[self.current_goal_index]

        # Compute the Euclidean distance to the current goal
        distance = math.sqrt(
            (self.current_position.x - goal_x) ** 2 +
            (self.current_position.y - goal_y) ** 2 +
            (self.current_position.z - goal_z) ** 2
        )

        self.get_logger().info(f"Distance to goal: {distance:.2f}")

        # Check if the drone has reached the current goal
        if distance < self.goal_tolerance:
            self.get_logger().info(f"Goal {self.current_goal_index} reached!")
            self.current_goal_index = (self.current_goal_index + 1) % len(self.goal_points)

    def publish_term_goal(self):
        """Publishes the current goal as a PoseStamped message."""
        goal_x, goal_y, goal_z = self.goal_points[self.current_goal_index]

        # Create PoseStamped message
        term_goal = PoseStamped()
        term_goal.header = Header()
        term_goal.header.stamp = self.get_clock().now().to_msg()
        term_goal.header.frame_id = "world"

        term_goal.pose.position.x = goal_x
        term_goal.pose.position.y = goal_y
        term_goal.pose.position.z = goal_z

        term_goal.pose.orientation.x = 0.0
        term_goal.pose.orientation.y = 0.0
        term_goal.pose.orientation.z = 0.0
        term_goal.pose.orientation.w = 1.0  # Identity quaternion

        # Publish the term goal
        self.term_goal_pub.publish(term_goal)
        self.get_logger().info(f"Published term goal: [{goal_x}, {goal_y}, {goal_z}]")

def main(args=None):
    rclpy.init(args=args)
    node = GoalMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
