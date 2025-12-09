#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extract (t, x, y, z, qx, qy, qz, qw) from a ROS 2 *SQLite* bag without installing custom packages.
- Uses rosbags' rosbag2 (SQLite) reader only.
- Registers minimal IDL for builtin_interfaces/Time, std_msgs/Header,
  geometry_msgs/Point, geometry_msgs/PointStamped, and ap20_driver_ros/PositionDebug.
- Reads position.point.{x,y,z}; orientation is identity (0,0,0,1).
"""

import argparse
import os
import sys
import importlib

# --- rosbags (install with: python3 -m pip install --user rosbags) ---
from rosbags.rosbag2 import Reader as Bag2Reader
from rosbags.typesys import get_types_from_idl, Stores, get_typestore

# ---------- Minimal IDL so we don't need ap20_driver_ros installed ----------

IDL_TIME = """
module builtin_interfaces { module msg {
  struct Time { int32 sec; uint32 nanosec; };
}; };
"""

IDL_HEADER = """
module std_msgs { module msg {
  struct Header { builtin_interfaces::msg::Time stamp; string frame_id; };
}; };
"""

IDL_POINT = """
module geometry_msgs { module msg {
  struct Point { double x; double y; double z; };
}; };
"""

IDL_POINTSTAMPED = """
module geometry_msgs { module msg {
  struct PointStamped {
    std_msgs::msg::Header header;
    geometry_msgs::msg::Point point;
  };
}; };
"""

IDL_POSITIONDEBUG = """
module ap20_driver_ros { module msg {
  struct PositionDebug {
    std_msgs::msg::Header header;
    double x;
    double y;
    double z;
    double norm;
  };
}; };
"""

def register_minimal_types():
    """Register custom types with the rosbags typestore"""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typemap = {}
    for idl in (IDL_TIME, IDL_HEADER, IDL_POINT, IDL_POINTSTAMPED, IDL_POSITIONDEBUG):
        typemap.update(get_types_from_idl(idl))
    typestore.register(typemap)
    return typestore

# ---------- helpers ----------

def is_bag_dir(path: str) -> bool:
    return os.path.isdir(path) and os.path.isfile(os.path.join(path, 'metadata.yaml'))

def deduce_bag_dir(bag_or_parent: str) -> str:
    if is_bag_dir(bag_or_parent):
        return bag_or_parent
    if not os.path.isdir(bag_or_parent):
        return ''
    for name in sorted(os.listdir(bag_or_parent)):
        cand = os.path.join(bag_or_parent, name)
        if is_bag_dir(cand):
            return cand
    return ''

def list_topics(reader: Bag2Reader):
    return {conn.topic: conn.msgtype for conn in reader.connections}

def get_connection(reader: Bag2Reader, topic: str):
    for conn in reader.connections:
        if conn.topic == topic:
            return conn
    return None

# ---------- core ----------

def write_txt_sqlite(bag_root: str, topic: str, out_file: str, typestore) -> int:
    if not os.path.isdir(bag_root):
        print(f'[ERROR] Path does not exist: {bag_root}')
        print('        Hint: is the host path mounted into the container?')
        return 2

    bag_dir = deduce_bag_dir(bag_root)
    if not bag_dir:
        print(f'[ERROR] No rosbag2 folder with metadata.yaml found under: {bag_root}')
        return 2

    print(f'[INFO] Using bag (sqlite): {bag_dir}')

    try:
        reader = Bag2Reader(bag_dir)
        reader.open()
    except Exception as e:
        print(f'[ERROR] Failed to open bag as SQLite: {e}')
        return 2

    topics = list_topics(reader)
    if topic not in topics:
        print(f"[ERROR] Topic '{topic}' not found. Available topics:")
        for tn, tt in topics.items():
            print(f'  - {tn} ({tt})')
        reader.close()
        return 3

    msgtype = topics[topic]
    print(f"[INFO] Reading topic '{topic}' of type '{msgtype}'")

    conn = get_connection(reader, topic)
    if conn is None:
        print(f'[ERROR] Internal: no connection found for {topic}')
        reader.close()
        return 3

    os.makedirs(os.path.dirname(os.path.abspath(out_file)), exist_ok=True)

    count = 0
    skipped = 0
    first_error = None
    with open(out_file, 'w') as f:
        f.write('# timestamp(s) tx ty tz qx qy qz qw\n')

        for _conn, timestamp, rawdata in reader.messages(connections=[conn]):
            t_sec = timestamp / 1e9
            msg = None
            try:
                # Use the reader's deserialize method with typestore
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                
                # Debug: print message structure on first successful deserialization
                if count == 0:
                    print(f'[DEBUG] Successfully deserialized message')
                    print(f'[DEBUG] Message structure: {msg}')
                    print(f'[DEBUG] Message type: {type(msg)}')
                    print(f'[DEBUG] Message attributes: {[a for a in dir(msg) if not a.startswith("_")]}')
                
                x = float(msg.x)
                y = float(msg.y)
                z = float(msg.z)
                
                f.write(f'{t_sec:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1\n')
                count += 1
            except Exception as e:
                if first_error is None:
                    first_error = str(e)
                    print(f'[DEBUG] First error: {e}')
                    print(f'[DEBUG] Error type: {type(e).__name__}')
                    print(f'[DEBUG] Message type: {conn.msgtype}')
                    print(f'[DEBUG] Raw data size: {len(rawdata)} bytes')
                    if msg is not None:
                        try:
                            print(f'[DEBUG] Message object: {msg}')
                            print(f'[DEBUG] Message attributes: {[a for a in dir(msg) if not a.startswith("_")]}')
                        except Exception as e2:
                            print(f'[DEBUG] Could not inspect message: {e2}')
                    else:
                        print(f'[DEBUG] Deserialization failed, msg is None')
                skipped += 1
                continue

    reader.close()

    if count == 0:
        print('[WARN] Wrote 0 rows. Could not extract position.point.{x,y,z}.')
    else:
        if skipped:
            print(f'[INFO] Skipped {skipped} messages without position.point.x/y/z.')
        print(f'[OK] Wrote {count} rows to: {out_file}')
    return 0

# ---------- CLI ----------

def main():
    ap = argparse.ArgumentParser(
        description='Extract x,y,z (+identity quat) from a ROS 2 SQLite bag topic to TXT, without custom packages.'
    )
    ap.add_argument('bag_dir', help='Bag directory OR parent containing the bag folder (with metadata.yaml).')
    ap.add_argument('--topic', required=True, help='Topic to read (e.g., /gt_box/ap20/position_debug).')
    ap.add_argument('-o', '--out', default=None, help='Output .txt path.')
    args = ap.parse_args()

    # Register minimal IDL so rosbags can decode the custom type
    typestore = register_minimal_types()

    out_file = args.out
    if out_file is None:
        bag_name = os.path.basename(os.path.normpath(args.bag_dir))
        out_file = os.path.join(args.bag_dir, f'{bag_name}_groundtruth.txt')

    sys.exit(write_txt_sqlite(args.bag_dir, args.topic, out_file, typestore))

if __name__ == '__main__':
    main()
