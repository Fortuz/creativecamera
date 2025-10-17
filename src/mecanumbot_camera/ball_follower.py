#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')

        # --- Subscriptions & Publishers ---
        self.sub_det = self.create_subscription(
            Detection2DArray,
            '/detections/ball',
            self.det_callback,
            10
        )
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Parameters ---
        self.image_width = 640.0     # pixels (adjust if your camera is wider)
        self.center_x = self.image_width / 2.0
        self.target_width = 120.0    # px → desired apparent ball size when “close enough”
        self.Kp_rot = 0.002          # proportional gain for rotation
        self.Kp_fwd = 0.005          # proportional gain for forward motion
        self.max_ang = 1.0           # rad/s cap for rotation
        self.max_lin = 0.4           # m/s cap for forward speed
        self.search_speed = 0.4      # angular speed during search
        self.threshold_px = 40       # pixel deadband for centering
        self.lost_timeout = 1.0      # seconds until “ball lost”

        # --- Internal State ---
        self.ball_x = None
        self.ball_w = None
        self.last_seen = self.get_clock().now()

        # Run control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

    # -----------------------------

    def det_callback(self, msg: Detection2DArray):
        """Receive the ball detection and store its x and width."""
        if len(msg.detections) == 0:
            return

        # Select the largest detected blob (by area)
        biggest = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        try:
            cx = biggest.bbox.center.x
        except AttributeError:
            cx = biggest.bbox.center.position.x  # fallback
        w = biggest.bbox.size_x

        self.ball_x = float(cx)
        self.ball_w = float(w)
        self.last_seen = self.get_clock().now()

    # -----------------------------

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now()
        time_since_seen = (now - self.last_seen).nanoseconds * 1e-9

        if self.ball_x is not None and time_since_seen < self.lost_timeout:
            # --- Ball visible: track & approach ---
            error_x = self.ball_x - self.center_x
            error_w = self.target_width - (self.ball_w or self.target_width)

            # Rotation (horizontal centering)
            if abs(error_x) < self.threshold_px:
                twist.angular.z = 0.0
            else:
                twist.angular.z = -self.Kp_rot * error_x
                twist.angular.z = max(-self.max_ang, min(self.max_ang, twist.angular.z))

            # Forward motion (approach target size)
            twist.linear.x = self.Kp_fwd * error_w
            twist.linear.x = max(0.0, min(self.max_lin, twist.linear.x))  # no reverse

            self.get_logger().info(
                f"Tracking ball: x={self.ball_x:.1f}, w={self.ball_w:.1f}, "
                f"ang={twist.angular.z:.3f}, lin={twist.linear.x:.3f}"
            )

        else:
            # --- Ball lost: search in place ---
            twist.angular.z = self.search_speed
            twist.linear.x = 0.0
            self.get_logger().info("Searching for ball...")

        self.pub_cmd.publish(twist)

# -----------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
