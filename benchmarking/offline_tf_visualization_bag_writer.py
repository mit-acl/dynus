#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time as TimeMsg
import argparse

# example: ./offline_tf_visualization_bag_writer.py --input_bag /media/kkondo/T7/dynus/tro_paper/hardware/red_rover/old/test1 --output_bag /media/kkondo/T7/dynus/tro_paper/hardware/red_rover/old/test1_with_tf_viz --frames RR01/base_link --interval 1.0

def create_marker(transform: TransformStamped, marker_id: int, stamp: TimeMsg) -> Marker:
    from geometry_msgs.msg import Point
    from std_msgs.msg import ColorRGBA

    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = stamp
    m.ns = "tf_trail"
    m.id = marker_id
    m.type = Marker.LINE_LIST  # Use LINE_LIST to draw lines.
    m.action = Marker.ADD

    # Set the marker's pose from the transform.
    m.pose.position.x = transform.transform.translation.x
    m.pose.position.y = transform.transform.translation.y
    m.pose.position.z = transform.transform.translation.z
    m.pose.orientation = transform.transform.rotation

    # For LINE_LIST, only scale.x is used as the line width.
    m.scale.x = 0.05  # Line width.

    # Define the length for each axis line.
    axis_length = 0.5

    # In the marker's local frame, define points for each axis.
    # X-axis: from origin to (axis_length, 0, 0)
    # Y-axis: from origin to (0, axis_length, 0)
    # Z-axis: from origin to (0, 0, axis_length)
    origin = Point(x=0.0, y=0.0, z=0.0)
    x_end = Point(x=axis_length, y=0.0, z=0.0)
    y_end = Point(x=0.0, y=axis_length, z=0.0)
    z_end = Point(x=0.0, y=0.0, z=axis_length)

    # For a LINE_LIST marker, every two consecutive points form a line.
    m.points = [origin, x_end,  # X-axis line.
                origin, y_end,  # Y-axis line.
                origin, z_end]  # Z-axis line.

    # Define individual colors for each point so that each axis has its own color.
    red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    green = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
    blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)

    # Assign colors per point (each line gets two points with the same color).
    m.colors = [red, red, green, green, blue, blue]

    # Marker persists forever.
    m.lifetime.sec = 0
    return m


def main():
    parser = argparse.ArgumentParser(
        description="Offline Bag Processor: Adds a RR01/tf_visulaization topic with tf markers based on time differences")
    parser.add_argument('--input_bag', required=True,
                        help='Input bag directory (e.g., my_robot_bag)')
    parser.add_argument('--output_bag', required=True,
                        help='Output bag directory (e.g., my_robot_bag_with_markers)')
    parser.add_argument('--frames', nargs='+', default=['RR01/base_link'],
                        help='TF frames to visualize (only messages from "map" to this frame are used)')
    parser.add_argument('--interval', type=float, default=0.2,
                        help='Minimum time interval (in seconds) between marker creation')
    args = parser.parse_args()

    rclpy.init()

    # Setup reader for the input bag.
    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.input_bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    # Check that at least the /tf topic is present.
    if '/tf' not in type_map:
        print("Missing topic /tf in the bag!")
        return

    messages = []        # List to store all original messages.
    marker_msgs = []     # List to store generated marker messages.
    marker_id = 0
    last_marker_time = None  # In seconds

    # Process all bag messages.
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_type_str = type_map[topic]
        msg_cls = get_message(msg_type_str)
        msg = deserialize_message(data, msg_cls)

        # If this is a tf message, check for our desired transform.
        if topic == '/tf':
            for tf_msg in msg.transforms:
                # Skip messages from certain frames if desired.
                if tf_msg.header.frame_id == "odom":
                    continue

                # Remove a leading slash if present.
                if tf_msg.header.frame_id.startswith('/'):
                    tf_msg.header.frame_id = tf_msg.header.frame_id[1:]
                if tf_msg.child_frame_id.startswith('/'):
                    tf_msg.child_frame_id = tf_msg.child_frame_id[1:]

                # We're interested only in transforms from "map" to our target frame.
                if tf_msg.header.frame_id == "map" and tf_msg.child_frame_id in args.frames:
                    # Convert header stamp to seconds.
                    stamp_sec = tf_msg.header.stamp.sec + tf_msg.header.stamp.nanosec * 1e-9

                    # Check if we should create a marker (if it's the first one or the time difference is enough).
                    if last_marker_time is None or (stamp_sec - last_marker_time) >= args.interval:
                        # Create a TransformStamped from the current tf_msg.
                        tf_stamped = TransformStamped()
                        tf_stamped.header = tf_msg.header
                        tf_stamped.child_frame_id = tf_msg.child_frame_id
                        tf_stamped.transform = tf_msg.transform

                        # Create a TimeMsg for the marker header.
                        stamp_msg = TimeMsg(
                            sec=tf_msg.header.stamp.sec,
                            nanosec=tf_msg.header.stamp.nanosec
                        )
                        marker = create_marker(tf_stamped, marker_id, stamp_msg)
                        marker_msgs.append(("RR01/tf_visulaization", marker, int(stamp_sec * 1e9)))
                        marker_id += 1
                        last_marker_time = stamp_sec
                        print(f"Marker created at time {stamp_sec} for transform map -> {tf_msg.child_frame_id}")

        # Save all original messages.
        messages.append((topic, msg, t))

    print(f"Finished reading bag. Generated {len(marker_msgs)} marker messages.")

    # Setup writer for the new bag.
    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.output_bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    # Create topics for all original topics.
    for t in topics_and_types:
        writer.create_topic(TopicMetadata(
            name=t.name,
            type=t.type,
            serialization_format='cdr'
        ))
    # Create the new topic for tf visualization markers.
    writer.create_topic(TopicMetadata(
        name='RR01/tf_visulaization',
        type='visualization_msgs/msg/Marker',
        serialization_format='cdr'
    ))

    # Merge original messages with marker messages and sort by timestamp.
    all_msgs = messages + marker_msgs
    all_msgs.sort(key=lambda x: x[2])
    for topic, msg, t in all_msgs:
        if topic in type_map:
            msg_type_str = type_map[topic]
        else:
            msg_type_str = 'visualization_msgs/msg/Marker'
        writer.write(topic, serialize_message(msg), t)

    print(f"Done. New bag with tf visualization markers written to {args.output_bag}")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
