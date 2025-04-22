#!/usr/bin/env python3

# /* ----------------------------------------------------------------------------
#  * Copyright 2024, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import rclpy
from rclpy.node import Node
from dynus_interfaces.msg import Mode, Goal, State
from dynus_interfaces.srv import FlightCommand
from geometry_msgs.msg import Pose
import math
import numpy as np

def quat2yaw(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw

class DynusCommand(Node):

    def __init__(self):

        super().__init__('dynus_command')

        # Initialize the mode
        self.mode = Mode()
        self.mode.mode = Mode.OTHER

        # Initialize the pose
        self.pose = Pose()
        
        # Initialize the takeoff goal
        self.takeoff_goal = Goal()

        # Initialize the publishers
        self.pub_goal = self.create_publisher(Goal, 'goal', 10)
        self.pub_mode = self.create_publisher(Mode, 'mode', 10)

        # Initialize the subscribers
        self.sub_state = self.create_subscription(State, 'state', self.state_cb, 10)

        # Intialize the service
        self.srv_flight_command = self.create_service(FlightCommand, 'flight_command', self.flight_command_cb)

        # Initialize the timer for takeoff
        self.timer_take_off = self.create_timer(0.004, self.timer_take_off_cb)
        self.timer_take_off.cancel()  # Stop the timer initially

        # Initialize the variables
        self.initialized = False
        self.is_kill = False

    def state_cb(self, data):

        # Initialize the position
        if not self.initialized:
            self.init_pos = np.array([data.pos.x, data.pos.y, data.pos.z])
            self.initialized = True

        self.pose.position.x = data.pos.x
        self.pose.position.y = data.pos.y
        self.pose.position.z = data.pos.z
        self.pose.orientation = data.quat

    def flight_command_cb(self, request, response):

        # Check if the state is initialized
        if not self.initialized:
            self.get_logger().info("Not initialized yet. Is DRONE_NAME/state being published?")
            response.success = False
            return response

        self.get_logger().info("flight_command_cb is called")

        if request.command == "take_off" and self.mode.mode == Mode.OTHER:
            self.get_logger().info("Starting takeoff")
            self.take_off()
            self.mode.mode = Mode.DYNUS

        if request.command == "kill":
            self.timer_take_off.cancel()
            self.get_logger().info("Killing")
            self.kill()
            self.get_logger().info("Kill done")

        if request.command == "land" and self.mode.mode == Mode.DYNUS:
            self.timer_take_off.cancel()
            self.get_logger().info("Landing")
            self.land()
            self.get_logger().info("Landing done")

        response.success = True

        return response

    def send_whoplans(self):
        self.mode.header.stamp = self.get_clock().now().to_msg()
        self.pub_mode.publish(self.mode)

    def take_off(self):
        self.get_logger().info("In take_off")
        self.is_kill = False
        self.takeoff_goal.p.x = self.pose.position.x
        self.takeoff_goal.p.y = self.pose.position.y
        self.takeoff_goal.p.z = self.pose.position.z
        self.takeoff_goal.yaw = quat2yaw(self.pose.orientation)

        self.takeoff_goal.power = True

        self.timer_take_off = self.create_timer(0.002, self.timer_take_off_cb)

    def timer_take_off_cb(self):
        alt_taken_off = 1.8
        self.takeoff_goal.p.z = min(self.takeoff_goal.p.z + 0.0005, alt_taken_off)
        # self.get_logger().info(f"Taking off... error={self.pose.position.z - alt_taken_off:.2f}")

        self.send_goal(self.takeoff_goal)

        if abs(self.pose.position.z - alt_taken_off) < 0.1:
            self.timer_take_off.cancel()
            self.mode.mode = Mode.DYNUS
            self.send_whoplans()

    def land(self):
        self.is_kill = False
        self.mode.mode = Mode.OTHER
        self.send_whoplans()

        goal = Goal()
        goal.p.x = self.pose.position.x
        goal.p.y = self.pose.position.y
        goal.p.z = self.pose.position.z
        goal.power = True
        goal.yaw = quat2yaw(self.pose.orientation)

        while abs(self.pose.position.z - self.init_pos[2]) > 0.08:
            goal.p.z = max(goal.p.z - 0.0035, self.init_pos[2])
            rclpy.sleep(0.01)
            self.send_goal(goal)

        self.kill()

    def kill(self):
        self.is_kill = True
        self.mode.mode = Mode.OTHER
        self.send_whoplans()

        goal = Goal()
        goal.p.x = self.pose.position.x
        goal.p.y = self.pose.position.y
        goal.p.z = self.pose.position.z
        goal.yaw = quat2yaw(self.pose.orientation)
        goal.power = False

        self.send_goal(goal)

    def send_goal(self, goal):
        goal.header.stamp = self.get_clock().now().to_msg()
        self.pub_goal.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = DynusCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
