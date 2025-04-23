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
import subprocess
import os
import signal
from std_msgs.msg import Empty
from time import sleep
from random import seed, random, uniform
import time

class BenchmarkNode(Node):

    def __init__(self):

        # random seed
        seed(1)
        
        # Global planner algorithm list
        self.algorithms = ['dgp']

        # Benchmarking types
        self.benchmark_types = 'global_planner_benchmarking'

        # Use dynamic obstacles
        self.use_dyn_obs = False

        # Use RViz
        self.use_rviz = False

        super().__init__('benchmark_node')
        self.declare_parameter('iterations', 10)
        self.iterations = self.get_parameter('iterations').value
        self.current_run = 0
        self.start_iter = 0

        # Agents
        self.agent_list = ['NX01']

        # Subscribe to the goal_reached topic
        self.goal_reached_sub = []
        for agent in self.agent_list:
            self.goal_reached_sub.append(self.create_subscription(
                Empty, f'/{agent}/goal_reached', self.goal_reached_callback, 1)) 

        # Flag to check if the goal has been reached
        self.goal_reached = False

        # Goal and start positions range
        start_x = 0.0
        goal_x = 105.0
        # y_range = [-20.0, 20.0]
        y_range = [0.0, 0.0]
        self.goal_z = 3.0
        # Pre-generate start and goal positions for all iterations
        self.start_positions = []
        self.goal_positions = []
        for i in range(self.iterations):
            # start_y = uniform(0, 1) * (y_range[1] - y_range[0]) + y_range[0]
            start_y = 0.0
            # goal_y = uniform(0, 1) * (y_range[1] - y_range[0]) + y_range[0]
            goal_y = y_range[0] + i * (y_range[1] - y_range[0]) / self.iterations
            self.start_positions.append([start_x, start_y])
            self.goal_positions.append([goal_x, goal_y])

    def run_simulation(self):

        # algorithm loop
        for algorithm in self.algorithms:

            # create directory for the algorithm
            csv_folder_path = f"/media/XXXXXXX/T7/dynus/tro_paper/{self.benchmark_types}/csv/{algorithm}"
            bag_folder_path = f"/media/XXXXXXX/T7/dynus/tro_paper/{self.benchmark_types}/bags/{algorithm}"
            log_folder_path = f"/media/XXXXXXX/T7/dynus/tro_paper/{self.benchmark_types}/logs/{algorithm}"
            os.makedirs(csv_folder_path, exist_ok=True)
            os.makedirs(bag_folder_path, exist_ok=True)
            os.makedirs(log_folder_path, exist_ok=True)

            # simulation loop
            for i in range(self.start_iter, self.iterations):
                self.current_run = i
                self.get_logger().info(f'Starting simulation {self.current_run + 1}/{self.iterations}')
                self.start_simulation(algorithm)
                result = self.wait_for_goal()
                self.log_info(result, algorithm, i, log_folder_path)
                self.stop_simulation()
                self.get_logger().info(f'Completed simulation {self.current_run + 1}/{self.iterations}')
                sleep(5)

    def log_info(self, result, algorithm, i, log_folder_path):
        """
        Log the result of the simulation to the given log file.
        """
        # Log the result of the simulation
        with open(f"{log_folder_path}/log.txt", "a") as log_file:
            log_file.write(f"Time: {time.asctime(time.localtime(time.time()))} Simulation {i}/{self.iterations} using {algorithm} algorithm: {result}\n")

    def start_simulation(self, algorithm='dgp'):

        # Get the start and goal positions
        start_x, start_y = self.start_positions[self.current_run]
        goal_x, goal_y = self.goal_positions[self.current_run]

        # Print the algorithm, start, and goal positions
        self.get_logger().info(f'Algorithm: {algorithm}')
        self.get_logger().info(f'Start position: {start_x}, {start_y}')
        self.get_logger().info(f'Goal position: {goal_x}, {goal_y}')

        # Base
        self.sim_process_base = subprocess.Popen(["ros2", "launch", "dynus", "base_dynus.launch.py", f"use_dyn_obs:={self.use_dyn_obs}", "use_gazebo_gui:=false", f"use_rviz:={self.use_rviz}"], preexec_fn=os.setsid)

        sleep(10)
        
        # Onboard
        self.sim_process_onboard = subprocess.Popen(["ros2", "launch", "dynus", "onboard_dynus.launch.py", f"x:={start_x}", f"y:={start_y}", f"z:={self.goal_z}", "yaw:=0", "namespace:=NX01", f"use_obstacle_tracker:={self.use_dyn_obs}", f"data_file:=/media/XXXXXXX/T7/dynus/tro_paper/{self.benchmark_types}/csv/{algorithm}/num_{self.current_run}.csv", f"global_planner:={algorithm}", "use_benchmark:=true", "depth_camera_name:=d435"], preexec_fn=os.setsid)
        
        sleep(5)

        # Bag recording
        self.sim_bag_record = subprocess.Popen(["python3", "/home/XXXXXXX/code/dynus_ws/src/dynus/scripts/bag_record.py", "--bag_number", str(self.current_run), "--bag_path", f"/media/XXXXXXX/T7/dynus/tro_paper/{self.benchmark_types}/bags/{algorithm}", "--agents", "['NX01']"], preexec_fn=os.setsid)

        sleep(20)
        
        # Goal
        # print("Sending goal")
        self.sim_process_goal =  subprocess.Popen(["ros2", "launch", "dynus", "goal_sender.launch.py", "list_agents:=['NX01']", f"list_goals:=['[{goal_x}, {goal_y}]']", f"default_goal_z:={self.goal_z}"], preexec_fn=os.setsid)

    def stop_simulation(self):

        # kill the ros2 processes
        os.killpg(os.getpgid(self.sim_process_base.pid), signal.SIGTERM)
        self.sim_process_base.wait()
        os.killpg(os.getpgid(self.sim_process_onboard.pid), signal.SIGTERM)
        self.sim_process_onboard.wait()
        os.killpg(os.getpgid(self.sim_bag_record.pid), signal.SIGTERM)
        self.sim_bag_record.wait()
        os.killpg(os.getpgid(self.sim_process_goal.pid), signal.SIGTERM)
        self.sim_process_goal.wait()

        sleep(10)

        # Gazebo is not killed by the above commands, so we need to kill it separately
        gazebo_processes = self.get_gazebo_process()
        for process in gazebo_processes:
            # Extract PID (2nd column in `ps aux` output)
            parts = process.split()
            if len(parts) > 1:
                pid = parts[1]
                print(f"Found gazebo process to kill: PID={pid}, Command={' '.join(parts[10:])}")
                self.kill_process(pid)

        sleep(10)

    def wait_for_goal(self):

        # Unregister the subscription before waiting
        for sub in self.goal_reached_sub:
            self.destroy_subscription(sub)

        # Re-subscribe to the topic
        self.goal_reached_sub = [
            self.create_subscription(
                Empty, f'/{agent}/goal_reached', self.goal_reached_callback, 1
            ) for agent in self.agent_list
        ]

        self.goal_reached = False
        # start_time = time.time()
        while not self.goal_reached:
            rclpy.spin_once(self)
            sleep(1)
            # if time.time() - start_time > self.timeout:
            #     self.get_logger().error("Timeout reached while waiting for goal")
            #     return False

        return True            

    def goal_reached_callback(self, msg):
        if msg:
            self.goal_reached = True
    
    def get_gazebo_process(self):
        """Get a list of gazebo-related processes."""
        try:
            # Run `ps aux | grep gazebo`
            result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, text=True)
            processes = result.stdout.splitlines()
            return [line for line in processes if 'gazebo' in line and 'Microsoft' not in line]
        except subprocess.CalledProcessError as e:
            print(f"Error executing ps aux: {e}")
            return []

    def kill_process(self, pid):
        """Kill a process by its PID."""
        try:
            subprocess.run(['kill', '-9', str(pid)], check=True)
            print(f"Successfully killed process with PID: {pid}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to kill process with PID {pid}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkNode()
    node.run_simulation()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()