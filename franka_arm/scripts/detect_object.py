import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math

from tf2_ros import TransformListener, Buffer
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from std_msgs.msg import String  # NEW


class YoloDepthDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_detector_node')
        self.get_logger().info('Initializing YOLO Depth Detector Node...')

        self.model_name = 'yolo12m.pt'
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_depth_image = None

        # Load YOLO model
        try:
            self.model = YOLO(self.model_name)
            self.get_logger().info(f'Successfully loaded YOLO model: {self.model_name}')
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            return

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10)

        self.depth_subscription = self.create_subscription(
            Image, '/depth', self.depth_callback, 10)

        self.image_subscription = self.create_subscription(
            Image, '/rgb', self.image_callback, 10)

        self.get_logger().info('Node initialization complete. Waiting for data...')

        # --- Publishers --- (NEW)
        self.coord_label_base_pub = self.create_publisher(String, 'coord_label_base', 10)  # NEW
        self.orientation_label_pub = self.create_publisher(String, 'orientation_label', 10)  # NEW

    def euler_from_quaternion(self, quaternion):
        """
        Converts a quaternion (from geometry_msgs.msg.Quaternion) to Euler angles (roll, pitch, yaw).
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received.')

    def depth_callback(self, msg):
        self.latest_depth_image = msg

    def image_callback(self, msg):
        if self.camera_info is None or self.latest_depth_image is None:
            self.get_logger().warn('Waiting for camera info and depth image...')
            return

        try:
            cv_image_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image_depth = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert images: {e}')
            return

        results = self.model(cv_image_rgb, conf=0.1, iou=0.0)

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u = (x1 + x2) // 2
                v = (y1 + y2) // 2

                if 0 <= v < cv_image_depth.shape[0] and 0 <= u < cv_image_depth.shape[1]:
                    depth = cv_image_depth[v, u]

                    if self.latest_depth_image.encoding == '16UC1':
                        depth_meters = float(depth) / 1000.0
                    else:
                        depth_meters = float(depth)

                    if depth_meters > 0:
                        z_cam = depth_meters
                        x_cam = (u - cx) * z_cam / fx
                        y_cam = (v - cy) * z_cam / fy

                        class_name = self.model.names[int(box.cls[0])]
                        self.get_logger().info(
                            f"Detected '{class_name}': X={x_cam:.2f}m, Y={y_cam:.2f}m, Z={z_cam:.2f}m"
                        )

                        point_cam = PointStamped()
                        point_cam.header.stamp = msg.header.stamp
                        point_cam.header.frame_id = 'Camera_OmniVision_OV9782_Color'

                        point_cam.point.x = x_cam
                        point_cam.point.y = y_cam
                        point_cam.point.z = z_cam + 0.03  # end effector offset 3 cm

                        try:
                            transform_stamp = tf2_ros.Time()
                            tf = self.tf_buffer.lookup_transform(
                                'base', point_cam.header.frame_id,
                                transform_stamp, rclpy.duration.Duration(seconds=0.5)
                            )
                            point_base = do_transform_point(point_cam, tf)

                            q = tf.transform.rotation
                            roll, pitch, yaw = self.euler_from_quaternion(q)
                            roll_deg = math.degrees(roll)
                            pitch_deg = math.degrees(pitch)
                            yaw_deg = math.degrees(yaw)
                            orientation_label = f"RPY:({roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}) deg"

                            self.get_logger().info(
                                f"Transformed to base frame: X={point_base.point.x:.2f}, Y={point_base.point.y:.2f}, Z={point_base.point.z:.2f}"
                            )
                        except Exception as e:
                            self.get_logger().warn(f'TF transform failed: {e}')
                            point_base = PointStamped()
                            point_base.point.x = float('nan')
                            point_base.point.y = float('nan')
                            point_base.point.z = float('nan')
                            orientation_label = "Orientation TF Failed"

                        conf = float(box.conf[0])
                        label = f'{class_name} {conf:.2f}'
                        coord_label_cam = f'Cam:({x_cam:.1f}, {y_cam:.1f}, {z_cam:.1f})m'

                        if not np.isnan(point_base.point.x):
                            coord_label_base = f'Base:({point_base.point.x:.2f}, {point_base.point.y:.2f}, {point_base.point.z:.2f})m'
                        else:
                            coord_label_base = 'Base Coords TF Failed'

                        # --- Publish labels (NEW) ---
                        self.coord_label_base_pub.publish(String(data=coord_label_base))      # NEW
                        self.orientation_label_pub.publish(String(data=orientation_label))    # NEW

                        # --- MODIFICATION: Drawing visualizations ---
                        cv2.rectangle(cv_image_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(cv_image_rgb, (u, v), 5, (0, 0, 255), -1)
                        cv2.circle(cv_image_rgb, (x1, v), 5, (255, 0, 0), -1)
                        cv2.circle(cv_image_rgb, (x2, v), 5, (255, 0, 0), -1)

                        # Line 1: Class label (Green, normal size)
                        cv2.putText(cv_image_rgb, label, (x1, y1 - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        # Line 2: Camera Coords (Red, smaller size)
                        cv2.putText(cv_image_rgb, coord_label_cam, (x1, y1 - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        # Line 3: Base Coords (Blue, smaller size)
                        cv2.putText(cv_image_rgb, coord_label_base, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        # Line 4: Orientation (Blue, smaller size)
                        cv2.putText(cv_image_rgb, orientation_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        # --- MODIFICATION END ---

        cv2.imshow("YOLO Live 3D Detections", cv_image_rgb)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

