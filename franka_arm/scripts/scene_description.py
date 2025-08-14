#!/usr/bin/env python3
import argparse
import sys
import time
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

SAVE_DIR = "captured_images"

class OneShotImageSaver(Node):
    def __init__(self, topic, timeout):
        super().__init__('one_shot_image_saver')
        self.bridge = CvBridge()
        self.received = False
        self.subscription = self.create_subscription(Image, topic, self.callback, 10)
        self.timeout = timeout

    def callback(self, msg: Image):
        if self.received:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            os.makedirs(SAVE_DIR, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"scene_{timestamp}.jpg"
            out_path = os.path.join(SAVE_DIR, filename)
            ok, buf = cv2.imencode(".jpg", cv_img)
            if not ok:
                self.get_logger().error("cv2.imencode failed")
                return
            with open(out_path, "wb") as f:
                f.write(buf.tobytes())
            self.received = True
            self.get_logger().info(f"Saved frame to {out_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to convert/save image: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/rgb", help="ROS 2 image topic")
    parser.add_argument("--timeout", type=float, default=5.0, help="Timeout seconds")
    args = parser.parse_args()

    rclpy.init()
    node = OneShotImageSaver(args.topic, args.timeout)

    start = time.time()
    try:
        while rclpy.ok() and not node.received and (time.time() - start) < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.received:
        sys.stderr.write(f"Timeout: no image received from '{args.topic}' within {args.timeout}s\n")
        sys.exit(1)

    sys.exit(0)

if __name__ == "__main__":
    main()

