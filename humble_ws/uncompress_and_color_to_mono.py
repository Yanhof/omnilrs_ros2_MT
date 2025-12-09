#!/usr/bin/env python3
import os, importlib
import cv2
import numpy as np
from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

# Default bag path - can be overridden by command line arguments
DEFAULT_INPUT_BAG = "/home/yhofmann/git/grand_tour_dataset/clean_construction_data_processing/ros2_bags/intermediary_bags/hdr_80s_imu"

# These will be initialized properly in the main function
INPUT_BAG = None
OUTPUT_BAG = None

#detection of missing header timetamps
EPOCH_2000_NS = 946_684_800_000_000_000   # 2000-01-01
EPOCH_2100_NS = 4_102_444_800_000_000_000 # 2100-01-01

def pick_timestamp_ns(msg, bag_t_ns: int, topic: str) -> int:
    """
    Use header time if present and plausible (between 2000 and 2100),
    otherwise fall back to bag record time.
    """
    try:
        hdr = getattr(msg, "header", None)
        if hdr is not None and hasattr(hdr, "stamp"):
            ts = int(hdr.stamp.sec) * 1_000_000_000 + int(hdr.stamp.nanosec)
            if EPOCH_2000_NS <= ts <= EPOCH_2100_NS:
                return ts
            else:
                print(f"[TIME WARNING] Header stamp for topic '{topic}' is {ts} ns "
                      f"({hdr.stamp.sec}.{hdr.stamp.nanosec}) — outside plausible range. Using bag time instead.")
                return int(bag_t_ns)
    except Exception as e:
        print(f"[TIME WARNING] Could not read header stamp for topic '{topic}': {e}. Using bag time.")
    return int(bag_t_ns)


# Automatic topic detection is enabled - no need to specify topics manually

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
    try:
        mod = importlib.import_module(pkg + '.msg')
        cls_name = rest.split('/')[-1]
        return getattr(mod, cls_name)
    except (ImportError, AttributeError) as e:
        # Return None if the package or message type doesn't exist
        return None


def uncompress_image(compressed_img_msg):
    """
    Takes a sensor_msgs/CompressedImage message and returns an uncompressed sensor_msgs/Image message.
    """
    
    # Decompress the image
    compressed_data = np.frombuffer(compressed_img_msg.data, np.uint8)
    cv_image = cv2.imdecode(compressed_data, cv2.IMREAD_UNCHANGED)
    
    if cv_image is None:
        raise RuntimeError(f"Failed to decompress image with format: {compressed_img_msg.format}")
    
    # Create an uncompressed Image message
    img_msg = Image()
    img_msg.header = compressed_img_msg.header
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    
    if len(cv_image.shape) == 2:  # Grayscale image
        img_msg.encoding = 'mono8'
        img_msg.step = cv_image.shape[1]
    else:  # Color image
        img_msg.encoding = 'bgr8'  # OpenCV decodes to BGR by default
        img_msg.step = cv_image.shape[1] * 3  # 3 bytes per pixel for BGR
    
   
    img_msg.data = cv_image.tobytes()
    return img_msg

def derive_output_path(input_bag: str, suffix: str = "_uncompressed_mono8_mcap") -> str:
    # Normalize to remove trailing slashes
    norm = os.path.normpath(input_bag)
    parent = os.path.dirname(norm)          # parent of the input folder
    base   = os.path.basename(norm)         # name of the input folder
    return os.path.join(parent, f"{base}{suffix}")

def main(input_bag, output_bag):
    if not os.path.isdir(input_bag):
        raise SystemExit(f"Input bag not found: {input_bag}")
    if os.path.exists(output_bag):
        raise SystemExit(f"Output bag already exists: {output_bag}")
        
    print(f"Processing bag: {input_bag}")
    print(f"Output will be saved to: {output_bag}")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=input_bag, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    msg_cls_cache = {}

    # Automatically find all compressed image topics
    compressed_topics = []
    for topic in topics:
        if topic.type == 'sensor_msgs/msg/CompressedImage':
            compressed_topics.append(topic.name)
    
    print(f"Found {len(compressed_topics)} compressed image topics:")
    for topic in compressed_topics:
        print(f"  {topic}")
        
    # Define new topic mappings for uncompressed images
    topic_remapping = {}
    for topic in compressed_topics:
        if "/compressed" in topic:
            new_topic = topic.replace("/compressed", "/uncompressed")
        else:
            new_topic = topic + "/uncompressed"
        topic_remapping[topic] = new_topic
        
    # Print remapping for clarity
    print("Topic remappings:")
    for old, new in topic_remapping.items():
        print(f"  {old} -> {new}")

    # Open writer
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=output_bag, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    # Create same topics in the output
    for t in topics:
        # For image topics, create them as Image instead of CompressedImage
        if t.name in topic_remapping:
            writer.create_topic(TopicMetadata(
                name=topic_remapping[t.name],
                type='sensor_msgs/msg/Image',  # Always Image for uncompressed
                serialization_format='cdr'
            ))
            continue  # Skip creating the compressed topic
        
        # Check if we can import this message type before creating the topic
        msg_cls = import_msg_class(t.type)
        if msg_cls is None:
            # Skip topics we can't deserialize
            continue
            
        # Create the original topic too (for non-image messages)
        writer.create_topic(TopicMetadata(
            name=t.name,
            type=t.type,
            serialization_format='cdr'
        ))
    metadata = reader.get_metadata()
    total_messages = metadata.message_count

    # Keep track of topics without headers to avoid repeated warnings
    skipped_topics = set()
    # Keep track of topics with missing message definitions
    missing_msg_types = set()

    # Stream through and write with original timestamps
    with tqdm(total=total_messages, desc="Processing bag messages") as pbar:

        while reader.has_next():
            topic, data, t = reader.read_next()
            ros_type = type_map[topic]
            
            # Try to import message class if not already cached
            if ros_type not in msg_cls_cache:
                msg_cls = import_msg_class(ros_type)
                if msg_cls is None:
                    # Mark this type as missing and skip all messages of this type
                    if ros_type not in missing_msg_types:
                        print(f"[SKIP] Cannot import message type '{ros_type}' for topic '{topic}', skipping all messages of this type")
                        missing_msg_types.add(ros_type)
                    pbar.update(1)
                    continue
                msg_cls_cache[ros_type] = msg_cls
            
            # Skip if this message type couldn't be imported
            if ros_type in missing_msg_types:
                pbar.update(1)
                continue
                
            MsgType = msg_cls_cache[ros_type]
            msg = deserialize_message(data, MsgType)

            if topic in topic_remapping:  # This is a compressed image topic we want to convert
                if ros_type == 'sensor_msgs/msg/CompressedImage':
                    try:
                        # First uncompress the image
                        uncompressed_msg = uncompress_image(msg)
                        # Then convert to mono8 using existing function
                        if uncompressed_msg.encoding != 'mono8':
                            uncompressed_msg = to_mono8(uncompressed_msg)
                   # Write uncompressed to new topic using header timestamp
                        timestamp_ns = pick_timestamp_ns(uncompressed_msg, t, topic_remapping[topic])
                        writer.write(topic_remapping[topic], serialize_message(uncompressed_msg), timestamp_ns)
                    except Exception as e:
                        print(f"[WARN] uncompress/convert failed on {topic} @ {t}: {e} (keeping original)")
                        continue  # Skip this message if conversion fails
                # Skip writing the compressed image to output bag
                continue  # Skip to next message

            # Check if message has header for timestamp
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                # Use header timestamp for messages that have it
                ts_ns = pick_timestamp_ns(msg, t, topic)
                writer.write(topic, serialize_message(msg), ts_ns)
            else:
                # Skip messages without headers and print warning (only once per topic)
                if topic not in skipped_topics:
                    print(f"[SKIP] Topic '{topic}' (type: {ros_type}) has no header timestamp, skipping all messages from this topic")
                    skipped_topics.add(topic)
                continue
            pbar.update(1)
    
    # Print summary of skipped topics
    if skipped_topics:
        print(f"\nSkipped {len(skipped_topics)} topic(s) without header timestamps:")
        for topic in sorted(skipped_topics):
            topic_type = type_map.get(topic, "unknown")
            print(f"  - {topic} ({topic_type})")
    
    # Print summary of missing message types
    if missing_msg_types:
        print(f"\nSkipped {len(missing_msg_types)} message type(s) due to missing package definitions:")
        for msg_type in sorted(missing_msg_types):
            print(f"  - {msg_type}")

    print(f"Done. New bag: {output_bag}")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Uncompress and convert image topics to mono8')
    parser.add_argument('--input', '-i', help='Input bag directory path')
    parser.add_argument('--output', '-o', help='Output bag directory path')
    
    args = parser.parse_args()
    
    # Set input bag
    input_bag = args.input if args.input else DEFAULT_INPUT_BAG
    
    # Set output bag
    if args.output:
        output_bag = args.output
    else:
        output_bag = derive_output_path(input_bag)

    print(f"[paths] parent: {os.path.dirname(os.path.normpath(input_bag))}")
    print(f"[paths] input_basename: {os.path.basename(os.path.normpath(input_bag))}")
    print(f"[paths] output_bag: {output_bag}")
    
    main(input_bag, output_bag)
