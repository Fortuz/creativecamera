#!/usr/bin/env python3
import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

class BallTrackerRGB(Node):
    def __init__(self):
        super().__init__('ball_tracker_rgb')
        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('camera_info_topic', '/camera/camera_info'),
            ('hsv_lower', [25, 140, 140]),
            ('hsv_upper', [45, 255, 255]),
            ('min_area_px', 80),
            ('det_ball_topic', '/detections/ball'),
            ('debug_image_topic', '/camera/image_debug'),
            ('mask_image_topic', '/camera/image_mask'),
        ])
        vals = [p.value for p in self.get_parameters([
            'image_topic',
            'camera_info_topic',
            'hsv_lower',
            'hsv_upper',
            'min_area_px',
            'det_ball_topic',
            'debug_image_topic',
            'mask_image_topic',
        ])]
        (self.image_topic, info_topic, hsv_l, hsv_u, self.min_area, det_topic, dbg_topic, mask_topic) = vals

        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_det = self.create_publisher(Detection2DArray, det_topic, 10)
        self.pub_dbg = self.create_publisher(Image, dbg_topic, 10)
        self.pub_mask = self.create_publisher(Image, mask_topic, 10)
        self.get_logger().info(f'Ball tracker publishing debug={dbg_topic} mask={mask_topic}')

        self.hsv_lower = np.array(hsv_l, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_u, dtype=np.uint8)

    def on_info(self, msg: CameraInfo):
        pass  # kept for potential future use

    def on_image(self, img_msg: Image):
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.medianBlur(mask, 5)

        # publish binary mask so you can tune HSV in Foxglove
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = img_msg.header
        self.pub_mask.publish(mask_msg)

        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dets = Detection2DArray()
        dets.header = img_msg.header

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x,y,w,h = cv2.boundingRect(c)
            cx, cy = x + w/2, y + h/2

            d = Detection2D()
            d.header = img_msg.header
            d.bbox.center.position.x = float(cx)
            d.bbox.center.position.y = float(cy)
            d.bbox.center.theta = 0.0
            d.bbox.size_x = float(w)
            d.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "ball"
            hyp.hypothesis.score = 1.0
            d.results.append(hyp)
            dets.detections.append(d)

            cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.circle(img, (int(cx),int(cy)), 3, (0,0,255), -1)

        self.pub_det.publish(dets)
        dbg_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        dbg_msg.header = img_msg.header
        self.pub_dbg.publish(dbg_msg)

def main():
    rclpy.init()
    rclpy.spin(BallTrackerRGB())
    rclpy.shutdown()

