import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
from tf2_ros import TransformListener, Buffer
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from std_msgs.msg import String  # NEW

class HandDepthDetectorNode(Node):
    """
    A ROS2 node that detects hands using MediaPipe, calculates the 3D position
    of the palm center, and transforms it to the robot's base frame.
    This version is pre-configured with settings from the user's YOLO script.
    """
    def __init__(self):
        super().__init__('hand_depth_detector_node')

        # --- Parameters set to match the user's YOLO script environment ---
        self.declare_parameter('rgb_topic', '/rgb')
        self.declare_parameter('depth_topic', '/depth')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('camera_frame', 'Camera_OmniVision_OV9782_Color')

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value

        # --- Class Attributes ---
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_depth_image = None

        # --- MediaPipe Hands Initialization ---
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # --- TF2 Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        self.image_sub = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10)

        # --- Publisher for Annotated Image ---
        self.annotated_image_pub = self.create_publisher(Image, 'mediapipe/hands_3d_image', 10)

        # --- Publisher for Base text (NEW) ---
        self.base_text_pub = self.create_publisher(String, '/hand_base_position', 10)  # NEW

        self.get_logger().info(f"Hand Depth Detector Node initialized. Using settings:\n"
                               f"\tRGB Topic: {rgb_topic}\n"
                               f"\tDepth Topic: {depth_topic}\n"
                               f"\tCamera Info Topic: {camera_info_topic}\n"
                               f"\tTarget Frame: {self.base_frame}\n"
                               f"\tCamera Frame: {self.camera_frame}")

    def camera_info_callback(self, msg):
        """Stores camera intrinsic parameters."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Camera info received.')

    def depth_callback(self, msg):
        """Stores the latest depth image."""
        self.latest_depth_image = msg

    def image_callback(self, msg):
        """
        Main callback to process RGB images, detect hands, and calculate 3D coordinates.
        """
        if self.camera_info is None or self.latest_depth_image is None:
            return

        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb.flags.writeable = False
        results = self.hands.process(frame_rgb)
        frame_rgb.flags.writeable = True

        h, w, _ = frame_bgr.shape
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame_bgr, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                ids = [0, 5, 9, 13]
                palm_x = int(sum([hand_landmarks.landmark[i].x for i in ids]) / len(ids) * w)
                palm_y = int(sum([hand_landmarks.landmark[i].y for i in ids]) / len(ids) * h)

                if 0 <= palm_y < h and 0 <= palm_x < w:
                    depth = depth_image[palm_y, palm_x]
                    
                    if self.latest_depth_image.encoding == '16UC1':
                        depth_meters = float(depth) / 1000.0
                    elif self.latest_depth_image.encoding == '32FC1':
                        depth_meters = float(depth)
                    else:
                        depth_meters = 0.0

                    if np.isnan(depth_meters) or depth_meters <= 0.1:
                        continue

                    x_cam = (palm_x - cx) * depth_meters / fx
                    y_cam = (palm_y - cy) * depth_meters / fy
                    z_cam = depth_meters

                    point_camera = PointStamped()
                    point_camera.header.stamp = self.get_clock().now().to_msg()
                    point_camera.header.frame_id = self.camera_frame
                    point_camera.point.x, point_camera.point.y, point_camera.point.z = x_cam, y_cam, z_cam

                    point_base = None
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.base_frame,
                            self.camera_frame,
                            rclpy.time.Time(),
                            rclpy.duration.Duration(seconds=0.5))
                        point_base = do_transform_point(point_camera, transform)
                    except Exception as e:
                        self.get_logger().warn(f'Could not transform point: {e}')
                    
                    cv2.circle(frame_bgr, (palm_x, palm_y), 7, (0, 255, 0), -1)
                    
                    text_pos_y = palm_y - 15
                    cam_text = f'Cam:({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f})m'
                    cv2.putText(frame_bgr, cam_text, (palm_x + 15, text_pos_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    if point_base:
                        pb_x, pb_y, pb_z = point_base.point.x, point_base.point.y, point_base.point.z
                        base_text = f'Base:({pb_x:.2f}, {pb_y:.2f}, {pb_z:.2f})m'
                        cv2.putText(frame_bgr, base_text, (palm_x + 15, text_pos_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        self.base_text_pub.publish(String(data=base_text))  # NEW

        ros_image_out = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        ros_image_out.header = msg.header
        self.annotated_image_pub.publish(ros_image_out)

        cv2.imshow("MediaPipe Live 3D Detections", frame_bgr)
        cv2.waitKey(1)

    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Closing MediaPipe hands model...')
        self.hands.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HandDepthDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

