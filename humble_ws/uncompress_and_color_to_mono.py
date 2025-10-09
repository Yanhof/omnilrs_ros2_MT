#!/usr/bin/env python3
import os, importlib
import cv2
import numpy as np
from tqdm import tqdm
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata

# ---- CONFIG: set your input bag folder here ----

INPUT_BAG  = "/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted/construction/zed2i/zed2i_all_imu_60s"
#INPUT_BAG  = "/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted/construction/HDR/full_hdr_all_imus"

#INPUT_BAG  = "/home/yhofmann/git/grand_tour_dataset/examples_ros1/ros2_bags_converted/construction/alphasense_bags/alphasense_all_imus_60s"
OUTPUT_BAG = INPUT_BAG + "_uncompressed_mono8_sqlite"
# Image topics to convert (keep names, only content becomes mono8)

#LEFT_IMG  = "/boxi/hdr/front/image_raw/compressed"
#RIGHT_IMG = "/boxi/hdr/rear/image_raw/compressed"
LEFT_IMG  = "/boxi/zed2i/left/image_raw/compressed"
RIGHT_IMG = "/boxi/zed2i/right/image_raw/compressed"

#LEFT_IMG  = "/boxi/alphasense/front_left/image_raw/compressed"
#RIGHT_IMG = "/boxi/alphasense/front_right/image_raw/compressed"

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

def main():
    if not os.path.isdir(INPUT_BAG):
        raise SystemExit(f"Input bag not found: {INPUT_BAG}")
    if os.path.exists(OUTPUT_BAG):
        raise SystemExit(f"Output bag already exists: {OUTPUT_BAG}")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=INPUT_BAG, storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    msg_cls_cache = {}

        # Define new topic mappings for uncompressed images
    topic_remapping = {}
    for topic in (LEFT_IMG, RIGHT_IMG):
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
        StorageOptions(uri=OUTPUT_BAG, storage_id='mcap'),
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
        # Create the original topic too (for non-image messages)
        writer.create_topic(TopicMetadata(
            name=t.name,
            type=t.type,
            serialization_format='cdr'
        ))
    metadata = reader.get_metadata()
    total_messages = metadata.message_count


    # Stream through and write with original timestamps
    with tqdm(total=total_messages, desc="Processing bag messages") as pbar:

        while reader.has_next():
            topic, data, t = reader.read_next()
            ros_type = type_map[topic]
            if ros_type not in msg_cls_cache:
                msg_cls_cache[ros_type] = import_msg_class(ros_type)
            MsgType = msg_cls_cache[ros_type]
            msg = deserialize_message(data, MsgType)

            if topic in (LEFT_IMG, RIGHT_IMG):
                if ros_type == 'sensor_msgs/msg/CompressedImage':
                    try:
                        original_msg = msg

                        # First uncompress the image
                        uncompressed_msg = uncompress_image(msg)
                        # Then convert to mono8 using existing function
                        if uncompressed_msg.encoding != 'mono8':
                            uncompressed_msg = to_mono8(uncompressed_msg)
                    except Exception as e:
                        print(f"[WARN] uncompress/convert failed on {topic} @ {t}: {e} (keeping original)")
                    # Write uncompressed to new topic
                    writer.write(topic_remapping[topic], 
                                serialize_message(uncompressed_msg), 
                                int(uncompressed_msg.header.stamp.sec * 1e9 + uncompressed_msg.header.stamp.nanosec))
                        
                    # Write original compressed to original topic
                    writer.write(topic, 
                                serialize_message(original_msg), 
                                int(original_msg.header.stamp.sec * 1e9 + original_msg.header.stamp.nanosec))
            writer.write(topic, serialize_message(msg), int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec))
            pbar.update(1)

    print(f"Done. New bag: {OUTPUT_BAG}")

if __name__ == "__main__":
    main()
