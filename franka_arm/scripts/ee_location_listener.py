import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import math

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import String  # already added in prior step


class EndEffectorListener(Node):
    """
    A ROS 2 node to listen for the transform of the end-effector.
    Publishes the position string every cycle and prints only when it changes.
    """
    def __init__(self):
        super().__init__('end_effector_listener_node')

        # --- Parameters ---
        self.target_frame_ = 'base'
        self.source_frame_ = 'fr3v2_hand'
        
        # Publish rate parameter (Hz) for 'realtime' updates
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 30.0).value  # NEW

        # --- State Tracking ---
        self.last_position_ = None
        self.position_tolerance_ = 1e-3  # 1 mm

        # --- TF2 Listener Setup ---
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        # --- Publisher ---
        self.position_pub = self.create_publisher(String, '/end_effector_position', 10)

        # --- Timer ---
        period = 1.0 / max(self.publish_rate_hz, 1e-3)  # avoid div-by-zero
        self.timer_ = self.create_timer(period, self.lookup_transform)
        self.get_logger().info(
            f"Listener started @ {self.publish_rate_hz:.1f} Hz. Looking for transform from "
            f"'{self.source_frame_}' to '{self.target_frame_}'."
        )

    def lookup_transform(self):
        """
        Looks up the transform and publishes every cycle; logs only on significant change.
        """
        try:
            t = self.tf_buffer_.lookup_transform(
                self.target_frame_,
                self.source_frame_,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )

            current_pos = t.transform.translation

            # Always publish (realtime)
            pos_str = f'[x: {current_pos.x:.3f}, y: {current_pos.y:.3f}, z: {current_pos.z:.3f}]'
            self.position_pub.publish(String(data=pos_str))

            # Only log when position changes beyond tolerance
            should_log = True
            if self.last_position_:
                dist_sq = (current_pos.x - self.last_position_.x)**2 + \
                          (current_pos.y - self.last_position_.y)**2 + \
                          (current_pos.z - self.last_position_.z)**2
                if dist_sq < self.position_tolerance_**2:
                    should_log = False

            if should_log:
                self.get_logger().info(
                    f'End-effector position in "{self.target_frame_}" frame: {pos_str}'
                )
                self.last_position_ = current_pos

        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {self.source_frame_} to {self.target_frame_}: {ex}',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

