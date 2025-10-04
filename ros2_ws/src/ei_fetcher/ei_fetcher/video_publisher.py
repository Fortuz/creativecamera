#!/usr/bin/env python3
import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.declare_parameter('video_path', '/ws/ros2_ws/media/sample.mp4')
        self.video_path = self.get_parameter('video_path').value
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video {self.video_path}")
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        fps = fps if fps and fps > 0 else 30.0
        self.timer = self.create_timer(1.0/float(fps), self.tick)

    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().info("End of video—looping.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(VideoPublisher())
    rclpy.shutdown()
