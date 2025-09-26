#!/usr/bin/env python3
import rclpy, os, time, cv2, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self, topic, out_dir, limit=None, duration=None, bgr=False, inactivity_timeout=5.0):
        super().__init__('image_saver')
        self.topic = topic
        self.out_dir = out_dir
        self.limit = limit
        self.duration = duration
        self.start_time = time.time()
        self.inactivity_timeout = inactivity_timeout
        self.last_msg_time = time.time()
        self.bridge = CvBridge()
        self.count = 0
        self.bgr = bgr
        os.makedirs(self.out_dir, exist_ok=True)

        # --- auto-detect type for this topic ---
        names_types = dict(self.get_topic_names_and_types())
        if self.topic not in names_types:
            self.get_logger().warn(f"Topic '{self.topic}' not found on the graph yet. "
                                   f"Start playback first, or re-run in a few seconds.")
        types = names_types.get(self.topic, [])
        self.get_logger().info(f"Topic '{self.topic}' types: {types}")

        if 'sensor_msgs/msg/Image' in types and 'sensor_msgs/msg/CompressedImage' in types:
            self.get_logger().warn("Both types seen for this name. "
                                   "Only one subscription is allowed per topic name.")
            # Prefer raw Image if both appear.
            chosen = 'Image'
        elif 'sensor_msgs/msg/Image' in types:
            chosen = 'Image'
        elif 'sensor_msgs/msg/CompressedImage' in types:
            chosen = 'CompressedImage'
        else:
            # Fallback: try Image; if you really have CompressedImage, use the '/compressed' suffix.
            chosen = 'Image'

        if chosen == 'Image':
            self.sub = self.create_subscription(Image, self.topic, self.cb_image, qos_profile_sensor_data)
            self.get_logger().info("Subscribing as sensor_msgs/Image")
        else:
            self.sub = self.create_subscription(CompressedImage, self.topic, self.cb_compressed, qos_profile_sensor_data)
            self.get_logger().info("Subscribing as sensor_msgs/CompressedImage")

        self.create_timer(0.5, self.watchdog)
        self.get_logger().info(f"Saving to '{self.out_dir}'")

    def cb_image(self, msg: Image):
        self.last_msg_time = time.time()
        try:
            enc = 'bgr8' if self.bgr else 'passthrough'
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
            self._save(frame)
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")

    def cb_compressed(self, msg: CompressedImage):
        self.last_msg_time = time.time()
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if frame is None:
                raise RuntimeError("cv2.imdecode returned None")
            self._save(frame)
        except Exception as e:
            self.get_logger().error(f"CompressedImage decode error: {e}")

    def _save(self, frame):
        fname = os.path.join(self.out_dir, f"frame_{self.count:06d}.png")
        if not cv2.imwrite(fname, frame):
            self.get_logger().error(f"cv2.imwrite failed for {fname}")
            return
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f"Saved {self.count} images...")

        if self.limit and self.count >= self.limit:
            self.get_logger().info(f"Reached limit {self.limit}, shutting down.")
            rclpy.shutdown()

    def watchdog(self):
        if self.duration and (time.time() - self.start_time) > self.duration:
            self.get_logger().info("Time limit reached, shutting down.")
            rclpy.shutdown()
        if (time.time() - self.last_msg_time) > self.inactivity_timeout and self.count == 0:
            self.get_logger().warn("No messages received. Check topic name / type / QoS. Shutting down.")
            rclpy.shutdown()

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--limit', type=int, default=None)
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--bgr', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = ImageSaver(args.topic, args.out, args.limit, args.duration, args.bgr)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
