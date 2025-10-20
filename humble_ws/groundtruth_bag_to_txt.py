#!/usr/bin/env python3
import argparse
import os
import sys

# ROS 2 Python APIs
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    print("[ERROR] Failed to import ROS 2 Python libraries. Did you source ROS 2 Humble? e.g.:")
    print("        source /opt/ros/humble/setup.bash")
    print(f"Details: {e}")
    sys.exit(1)


def extract_pointstamped(bag_path: str, topic: str, out_file: str) -> int:
    if not os.path.isdir(bag_path):
        print(f"[ERROR] Bag path does not look like a directory: {bag_path}")
        return 2
    print(f"[INFO] Bag path: {bag_path}")
    # Setup reader
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Check topic exists & type
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        print(f"[ERROR] Topic '{topic}' not found in bag. Available topics:")
        for tn in topics:
            print(f"  - {tn}  ({topics[tn]})")
        return 3

    expected_type = 'geometry_msgs/msg/PointStamped'
    if topics[topic] != expected_type:
        print(f"[ERROR] Topic '{topic}' has type '{topics[topic]}', expected '{expected_type}'.")
        return 4

    # Filter to just the topic
    reader.set_filter(StorageFilter(topics=[topic]))

    # Prepare deserializer
    print(f"[INFO] Reading topic '{topic}' of type '{expected_type}' from bag: {bag_path}")
    msg_cls = get_message(expected_type)

    # Write output
    os.makedirs(os.path.dirname(os.path.abspath(out_file)), exist_ok=True)
    count = 0
    with open(out_file, 'w') as f:
        f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
        while reader.has_next():
            topic_name, serialized, t_ns = reader.read_next()

            # t_ns is nanoseconds; convert to float seconds with 9 decimal digits
            t_sec = t_ns / 1e9

            msg = deserialize_message(serialized, msg_cls)
            x = float(msg.point.x)
            y = float(msg.point.y)
            z = float(msg.point.z)

            # Quaternions 
            qx = qy = qz = 0.0
            qw = 1.0

            # Write line
            f.write(f"{t_sec:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.1f} {qy:.1f} {qz:.1f} {qw:.1f}\n")
            count += 1

    print(f"[OK] Wrote {count} rows to: {out_file}")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description="Extract PointStamped to TXT with timestamp, position, and zeroed quaternion."
    )
    ap.add_argument("bag_dir", help="Path to the ROS 2 bag directory (folder containing .db3 and metadata.yaml)")
    ap.add_argument("--topic", default="/boxi/ap20/prism_position",
                    help="PointStamped topic to read (default: /boxi/ap20/prism_position)")
    ap.add_argument("-o", "--out", default=None,
                help="Output .txt path (default: saved next to the bag folder)")
    args = ap.parse_args()

    out_file = args.out
    if out_file is None:
        # Save in same folder as bag_dir
        bag_name = os.path.basename(os.path.normpath(args.bag_dir))
        out_file = os.path.join(args.bag_dir, f"{bag_name}_groundtruth.txt")

    sys.exit(extract_pointstamped(args.bag_dir, args.topic, out_file))


if __name__ == "__main__":
    main()
