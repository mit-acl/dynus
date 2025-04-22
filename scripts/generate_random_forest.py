#!/usr/bin/env python3
"""
Instruction: First run "ros2 launch gazebo_ros gazebo.launch.py"
Then run this script by "ros2 run dynus generate_random_forest.py"
Parameters: seed (int), difficulty (easy, medium, hard)
Note: This script generates cylinders until the target area is reached.
It now writes the location, radius, and height of each cylinder to a CSV file.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from visualization_msgs.msg import Marker, MarkerArray
import random
import math
import time
import csv
import os

CSV_PATH = "/home/kkondo/data/easy_forest_obstacle_parameters.csv"  # CSV file to record cylinder parameters

class GazeboCubeSpawner:
    def __init__(self, node: Node):
        self.node = node
        self.client = node.create_client(SpawnEntity, 'spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for spawn_entity service...')

    def spawn_cube(self, name: str, frame_id: str, x: float, y: float, z: float,
                   qx: float, qy: float, qz: float, qw: float,
                   width: float, height: float, depth: float, mass: float):
        # In our case we use a cylinder (do_cube is False)
        self.spawn_primitive(name, False, frame_id, x, y, z, qx, qy, qz, qw,
                             width, height, depth, mass)

    def spawn_primitive(self, name: str, do_cube: bool, frame_id: str,
                        x: float, y: float, z: float,
                        qx: float, qy: float, qz: float, qw: float,
                        width_or_radius: float, height: float, depth: float, mass: float):
        # Build the pose.
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # Build the geometry string.
        if do_cube:
            geometry_str = f"<box><size>{width_or_radius} {height} {depth}</size></box>"
        else:
            geometry_str = f"<cylinder><length>{height}</length><radius>{width_or_radius}</radius></cylinder>"

        mass12 = mass / 12.0
        do_inertia = True

        # Calculate inertia parameters.
        if do_inertia:
            if do_cube:
                xx = mass12 * (height**2 + depth**2)
                yy = mass12 * (width_or_radius**2 + depth**2)
                zz = mass12 * (width_or_radius**2 + height**2)
            else:
                xx = mass12 * (3 * width_or_radius**2 + height**2)
                yy = mass12 * (3 * width_or_radius**2 + height**2)
                zz = 0.5 * mass * width_or_radius**2

        # Build the SDF string.
        sdf = (
            f"<?xml version='1.0'?>"
            f"<sdf version='1.4'>"
            f"<model name='{name}'>"
            f"<static>1</static>"
            f"<link name='link'>"
        )
        if do_inertia:
            sdf += (
                f"<inertial>"
                f"<mass>{mass}</mass>"
                f"<inertia>"
                f"<ixx>{xx}</ixx>"
                f"<ixy>0.0</ixy>"
                f"<ixz>0.0</ixz>"
                f"<iyy>{yy}</iyy>"
                f"<iyz>0.0</iyz>"
                f"<izz>{zz}</izz>"
                f"</inertia>"
                f"</inertial>"
            )
        sdf += (
            f"<collision name='collision'>"
            f"<geometry>{geometry_str}</geometry>"
            f"<surface>"
            f"<contact><collide_without_contact>true</collide_without_contact></contact>"
            f"</surface>"
            f"</collision>"
            f"<visual name='visual'>"
            f"<geometry>{geometry_str}</geometry>"
            f"<material>"
            f"<script>"
            f"<uri>file://media/materials/scripts/gazebo.material</uri>"
            f"<name>Gazebo/Blue</name>"
            f"</script>"
            f"</material>"
            f"</visual>"
            f"<gravity>0</gravity>"
            f"</link>"
            f"</model>"
            f"</sdf>"
        )

        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.robot_namespace = "cube_spawner"
        request.initial_pose = pose
        request.reference_frame = frame_id

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        # You can log spawn results if desired.
        # if future.result() is not None:
        #     self.node.get_logger().info(f"Spawn result for {name}: {future.result().status_message}")
        # else:
        #     self.node.get_logger().error("Failed to call spawn_entity service")

class ForestSpawner(Node):
    def __init__(self):
        super().__init__('create_random_forest')
        self.declare_parameter("seed", 0)
        self.declare_parameter("difficulty", "easy")
        seed = self.get_parameter("seed").value
        difficulty = self.get_parameter("difficulty").value

        # Set density based on difficulty.
        if difficulty == "easy":
            self.density = 0.05
        elif difficulty == "medium":
            self.density = 0.1
        elif difficulty == "hard":
            self.density = 0.2
        else:
            self.get_logger().warn(f"Unknown difficulty '{difficulty}', defaulting to medium.")
            self.density = 0.1

        self.get_logger().info(f"Seed: {seed}, Difficulty: {difficulty}, Density: {self.density}")

        # Publisher for RViz visualization.
        from visualization_msgs.msg import MarkerArray  # Import here so that the script runs in ROS2
        self.marker_pub = self.create_publisher(MarkerArray, "forest", 10)

        # Seed the random generator.
        random.seed(seed)

        # Define map boundaries (for example, x and y).
        self.min_x = 1.0
        self.max_x = 101.0
        self.min_y = -20.0
        self.max_y = 20.0
        # Set world height.
        self.size_z = 5.0

        # Spawn a ground plane that is larger than the map.
        self.spawner = GazeboCubeSpawner(self)
        self.spawn_ground_plane()

        # Tree (cylinder) properties.
        self.kMinHeight = 1.0
        self.kMaxHeight = 5.0
        self.kMinRadius = 0.2
        self.kMaxRadius = 1.0

        usable_area = (self.max_x - self.min_x) * (self.max_y - self.min_y)
        target_area = self.density * usable_area
        self.get_logger().info(f"Usable area: {usable_area}, Target occupied area: {target_area}")

        occupied_area = 0.0
        marker_array = MarkerArray()
        i = 0

        # Prepare CSV file to record forest parameters.
        csv_path = CSV_PATH
        with open(csv_path, "w", newline="") as csvfile:
            import csv
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(["id", "x", "y", "z", "radius", "height"])

            # Generate trees until the accumulated 2D area meets or exceeds the target area.
            while occupied_area < target_area:
                print(f"Occupied area: {round(occupied_area, 2)}, Target area: {round(target_area, 2)}, Percentage: {round(100 * occupied_area / target_area, 2)}%")
                height = random.uniform(self.kMinHeight, self.kMaxHeight)
                radius = random.uniform(self.kMinRadius, self.kMaxRadius)
                tree_area = math.pi * radius * radius
                occupied_area += tree_area

                # Ensure trees are spawned within boundaries considering their radius.
                pos_x = random.uniform(self.min_x + radius, self.max_x - radius)
                pos_y = random.uniform(self.min_y + radius, self.max_y - radius)
                pos_z = height / 2.0

                # Spawn the cylinder (using spawn_cube with do_cube False).
                self.spawner.spawn_cube(str(i), "world", pos_x, pos_y, pos_z,
                                         0.0, 0.0, 0.0, 1.0,
                                         radius, height, 1.0, 2.0)

                # Write the cylinder parameters to CSV.
                csv_writer.writerow([i, pos_x, pos_y, pos_z, radius, height])

                # Create marker for visualization.
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "basic_shapes"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = pos_x
                marker.pose.position.y = pos_y
                marker.pose.position.z = pos_z
                marker.pose.orientation.w = 1.0
                marker.scale.x = 2 * radius
                marker.scale.y = 2 * radius
                marker.scale.z = height
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)

                i += 1

        self.get_logger().info(f"Total occupied area = {occupied_area}, Number of objects: {i}")
        time.sleep(3)  # Allow time for Gazebo and RViz to initialize.
        self.get_logger().info("Publishing marker array for visualization.")
        self.marker_pub.publish(marker_array)

    def delete_model(self, model_name: str):
        from gazebo_msgs.srv import DeleteEntity
        delete_client = self.create_client(DeleteEntity, 'delete_entity')
        while not delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete_entity service...')
        request = DeleteEntity.Request()
        request.name = model_name
        future = delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Deleted model '{model_name}': {future.result().status_message}")
        else:
            self.get_logger().error(f"Failed to delete model '{model_name}'")

    def spawn_ground_plane(self):
        # Implementation unchanged from your original script.
        margin_size = 5.0
        size_x = (self.max_x - self.min_x) + 2 * margin_size
        size_y = (self.max_y - self.min_y) + 2 * margin_size
        center_x = (self.min_x + self.max_x) / 2.0
        center_y = (self.min_y + self.max_y) / 2.0
        pos_z = 0.0

        self.delete_model("ground_plane")

        sdf = f"""<?xml version="1.0" ?>
        <sdf version="1.5">
            <model name='ground_plane_map'>
                <static>1</static>
                <link name='link'>
                    <collision name='collision'>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>{size_x} {size_y}</size>
                        </plane>
                    </geometry>
                    <surface>
                        <contact><collide_without_contact>true</collide_without_contact></contact>
                        <friction>
                            <ode>
                                <mu>100</mu>
                                <mu2>50</mu2>
                            </ode>
                        </friction>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                    </collision>
                    <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>{size_x} {size_y}</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Grey</name>
                        </script>
                    </material>
                    </visual>
                    <self_collide>0</self_collide>
                    <enable_wind>0</enable_wind>
                    <kinematic>0</kinematic>
                </link>
            </model>
        </sdf>
        """
        pose = Pose()
        pose.position.x = center_x
        pose.position.y = center_y
        pose.position.z = pos_z
        pose.orientation.w = 1.0

        from gazebo_msgs.srv import SpawnEntity
        request = SpawnEntity.Request()
        request.name = "ground_plane_map"
        request.xml = sdf
        request.robot_namespace = "ground_plane_map"
        request.initial_pose = pose
        request.reference_frame = "world"

        future = self.spawner.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Ground plane spawn result: {future.result().status_message}")
        else:
            self.get_logger().error("Failed to spawn ground plane")

def main(args=None):
    rclpy.init(args=args)
    forest_spawner = ForestSpawner()
    rclpy.spin(forest_spawner)
    forest_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
