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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('output_topic', '/camera/image_processed')
        self.declare_parameter('successful_detection_output_topic', 'successful_detection')
        self.declare_parameter('target_object_name', 'backpack')
        self.declare_parameter('publish_frequency', 2.0)  # Frequency in Hz

        # ROS Parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.output_topic = self.get_parameter('output_topic').value
        self.successful_detection_output_topic = self.get_parameter('successful_detection_output_topic').value
        self.target_object_name = self.get_parameter('target_object_name').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.confidence_threshold = 0.5
        self.default_color = (0, 255, 0) # defualt color is green
        self.detected_color = (0, 0, 255) # detected color is red

        # Initialize YOLO
        self.model = YOLO(self.yolo_model_path)
        self.bridge = CvBridge()

        # State
        self.target_detected = False

        # ROS subscriptions and publications
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.image_pub = self.create_publisher(Image, self.output_topic, 10)
        self.semantics_pub = self.create_publisher(String, 'detected_objects', 10)
        self.successful_detection_pub = self.create_publisher(String, self.successful_detection_output_topic, 10)
        self.target_name_sub = self.create_subscription(String, 'target_name', self.target_name_callback, 10)

        # Timer for controlled publishing
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_results)

        self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing processed images to: {self.output_topic}")
        self.get_logger().info(f"Publishing successful detections to: {self.successful_detection_output_topic}")
        self.get_logger().info(f"Looking for target object: {self.target_object_name}")
        self.get_logger().info(f"Publish frequency set to: {self.publish_frequency} Hz")
        self.get_logger().info("YOLO Node initialized")

    def image_callback(self, msg):
        try:
            # Skip processing if the target is already detected
            if self.target_detected:
                return

            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run YOLO inference
            results = self.model(cv_image, verbose=False)

            target_detected_flag = False
            self.detected_objects = []
            for result in results:
                for box in result.boxes:

                    # Bounding box coordinates
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Top-left and bottom-right
                    confidence = box.conf[0].item()  # Confidence score
                    class_id = int(box.cls[0].item())  # Class ID
                    label = self.model.names[class_id]  # Class label
                    color = self.default_color

                    # Check if the target object is detected
                    if label.lower() == self.target_object_name.lower() and confidence > self.confidence_threshold:

                        self.get_logger().info(f"Target object '{self.target_object_name}' detected!")

                        # Publish successful detection
                        target_object_str = f"Successfully detected target object: {self.target_object_name}"

                        # Add time stamp to the detected objects string
                        target_object_str = f"{target_object_str} @ {msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9}"

                        # Publish the successful detection
                        self.successful_detection_pub.publish(String(data=target_object_str))

                        # Change the color to red
                        color = self.detected_color

                        target_detected_flag = True

                    # Draw the bounding box and label on the image with color green
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(cv_image, f'{label} {confidence:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.detected_objects.append(label)


            # Store the processed image for periodic publishing
            self.processed_image = cv_image

            # Publish the image one last time if the target has been detected
            self.publish_results() if target_detected_flag else None

            # Set the target detected flag
            self.target_detected = target_detected_flag

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def publish_results(self):
        # Stop publishing if the target has been detected
        if self.target_detected:
            return

        if hasattr(self, 'processed_image'):
            # Publish the processed image
            processed_image_msg = self.bridge.cv2_to_imgmsg(self.processed_image, encoding='bgr8')
            self.image_pub.publish(processed_image_msg)

            # Publish detected objects
            detected_objects_str = ', '.join(self.detected_objects) if self.detected_objects else 'None'

            # Add time stamp to the detected objects string
            detected_objects_str = f"{detected_objects_str} @ {processed_image_msg.header.stamp.sec + processed_image_msg.header.stamp.nanosec / 1e9}"

            # Publish the detected objects
            self.semantics_pub.publish(String(data=detected_objects_str))

    def target_name_callback(self, msg):
        self.target_object_name = msg.data
        self.target_detected = False  # Reset detection state
        self.get_logger().info(f"Updated target object to: {self.target_object_name}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
