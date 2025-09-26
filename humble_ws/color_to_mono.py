#!/usr/bin/env python3
import os, importlib
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

# ---- CONFIG: set your input bag folder here ----
INPUT_BAG  = "/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted/construction/merged_stim320_zed2_60s_uncompressed"
OUTPUT_BAG = INPUT_BAG + "_mono8"

# Image topics to convert (keep names, only content becomes mono8)
LEFT_IMG  = "/boxi/zed2i/left/image_raw/uncompressed"
RIGHT_IMG = "/boxi/zed2i/right/image_raw/uncompressed"

bridge = CvBridge()

def to_mono8(img_msg):
    enc = img_msg.encoding.lower()
    cv = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    if cv.ndim == 2:
        mono = cv
    elif enc == 'bgr8':
        mono = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
    elif enc == 'rgb8':
        mono = cv2.cvtColor(cv, cv2.COLOR_RGB2GRAY)
    elif enc == 'bgra8':
        mono = cv2.cvtColor(cv, cv2.COLOR_BGRA2GRAY)
    elif enc == 'rgba8':
        mono = cv2.cvtColor(cv, cv2.COLOR_RGBA2GRAY)
    elif enc == 'mono16':
        mono = cv2.convertScaleAbs(cv, alpha=1.0/256.0)
    elif enc.startswith('bayer_'):
        code = {
            'bayer_bggr8': cv2.COLOR_BayerBG2GRAY,
            'bayer_gbrg8': cv2.COLOR_BayerGB2GRAY,
            'bayer_grbg8': cv2.COLOR_BayerGR2GRAY,
            'bayer_rggb8': cv2.COLOR_BayerRG2GRAY
        }.get(enc)
        if code is None:
            raise RuntimeError(f'Unsupported Bayer pattern: {enc}')
        mono = cv2.cvtColor(cv, code)
    else:
        mono = cv2.cvtColor(cv, cv2.COLOR_BGR2GRAY)
    out = bridge.cv2_to_imgmsg(mono, encoding='mono8')
    out.header = img_msg.header  # preserve original header stamp & frame_id
    return out

def import_msg_class(ros_type: str):
    # ros_type like 'sensor_msgs/msg/Image'
    pkg, _, rest = ros_type.partition('/')
    mod = importlib.import_module(pkg + '.msg')
    cls_name = rest.split('/')[-1]
    return getattr(mod, cls_name)

def main():
    if not os.path.isdir(INPUT_BAG):
        raise SystemExit(f"Input bag not found: {INPUT_BAG}")
    if os.path.exists(OUTPUT_BAG):
        raise SystemExit(f"Output bag already exists: {OUTPUT_BAG}")

    # Open reader (sqlite3 assumed; change if your bag uses another storage)
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=INPUT_BAG, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    msg_cls_cache = {}

    # Open writer
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=OUTPUT_BAG, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    # Create same topics in the output
    for t in topics:
        writer.create_topic(TopicMetadata(
            name=t.name,
            type=t.type,
            serialization_format='cdr'
        ))

    # Stream through and write with original timestamps
    while reader.has_next():
        topic, data, t = reader.read_next()
        ros_type = type_map[topic]
        if ros_type not in msg_cls_cache:
            msg_cls_cache[ros_type] = import_msg_class(ros_type)
        MsgType = msg_cls_cache[ros_type]
        msg = deserialize_message(data, MsgType)

        if topic in (LEFT_IMG, RIGHT_IMG) and ros_type == 'sensor_msgs/msg/Image':
            try:
                msg = to_mono8(msg)
            except Exception as e:
                print(f"[WARN] convert failed on {topic} @ {t}: {e} (keeping original)")

        writer.write(topic, serialize_message(msg), t)

    print(f"Done. New bag: {OUTPUT_BAG}")

if __name__ == "__main__":
    main()
