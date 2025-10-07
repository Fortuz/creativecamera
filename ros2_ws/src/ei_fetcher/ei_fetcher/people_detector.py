#!/usr/bin/env python3
import os
import rclpy, numpy as np, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, Point2, Color, TextAnnotation


try:
    import onnxruntime as ort
except Exception:
    ort = None

def parse_yolo_outputs(outputs, num_classes=80, person_cls=0):
    """
    Accepts tensors shaped like:
      - YOLOv8: [N, 4+num_classes] = cx,cy,w,h + class scores
      - YOLOv5: [N, 5+num_classes] = cx,cy,w,h,objectness + class scores
    Returns:
      boxes_cxcywh (float32, Nx4), scores (float32, N)
    """
    if outputs.ndim == 3:
        # common: (1, N, D) or (1, D, N)
        if outputs.shape[0] == 1:
            outputs = outputs[0]
        else:
            # pick the axis with D >= 6 as the last dim
            if outputs.shape[2] >= 6:
                outputs = np.transpose(outputs, (0, 2, 1))[0]
            else:
                outputs = outputs[0]
    if outputs.ndim == 2 and outputs.shape[0] < outputs.shape[1] and outputs.shape[0] in (6, 84, 85):
        outputs = outputs.T

    if outputs.ndim != 2 or outputs.shape[1] < 6:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32)

    D = outputs.shape[1]
    if D == 84:  # v8 exported without objness: 4 + 80
        boxes = outputs[:, :4]
        class_scores = outputs[:, 4:4 + num_classes]
        scores = class_scores[:, person_cls]
    elif D == 85:  # v5: 4 + 1 obj + 80
        boxes = outputs[:, :4]
        obj = outputs[:, 4]
        class_scores = outputs[:, 5:5 + num_classes]
        scores = class_scores[:, person_cls] * obj
    else:
        boxes = outputs[:, :4]
        class_scores = outputs[:, -num_classes:]
        scores = class_scores[:, person_cls]
    return boxes.astype(np.float32), scores.astype(np.float32)

class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')
        # in __init__(...)
        self.camera_frame_id = 'camera'   # default if CameraInfo not received yet
        self.last_info = None

        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('camera_info_topic', '/camera/camera_info'),
            ('det_topic', '/detections/person'),
            ('model_path', ''),
            ('input_size', [640, 640]),
            ('conf_threshold', 0.30),
            ('iou_threshold', 0.45),
            ('infer_every_n', 1),          # <- test with 1 so it publishes every frame
            ('person_class_ids', [0]),
        ])
        vals = [p.value for p in self.get_parameters([
            'image_topic', 'camera_info_topic', 'det_topic', 'model_path', 'input_size',
            'conf_threshold', 'iou_threshold', 'infer_every_n', 'person_class_ids'
        ])]
        (self.image_topic, info_topic, self.det_topic, self.model_path, self.input_size,
         self.conf_thr, self.iou_thr, self.infer_every_n, self.person_ids) = vals

        self.ann_topic = self.declare_parameter('ann_topic', '/camera/annotations').get_parameter_value().string_value
        self.pub_ann = self.create_publisher(ImageAnnotations, self.ann_topic, 10)

        # cv_bridge
        self.bridge = CvBridge()

        # QoS: use sensor_data so we actually get frames
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        # Always create publisher up front (Foxglove sees the type and we can publish empty arrays)
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)

        # Model/session
        self.frame_idx = 0
        self.session = None
        self.input_name = None

        if ort is None:
            self.get_logger().warn("onnxruntime not installed; detector will publish empty arrays.")
        elif not self.model_path:
            self.get_logger().warn("No model_path provided; detector will publish empty arrays.")
        elif not os.path.exists(self.model_path):
            self.get_logger().error(f"Model not found: {self.model_path}; publishing empty arrays.")
        else:
            try:
                so = ort.SessionOptions()
                so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                so.intra_op_num_threads = 4
                so.inter_op_num_threads = 1
                self.session = ort.InferenceSession(
                    self.model_path,
                    sess_options=so,
                    providers=['CPUExecutionProvider']
                )
                self.input_name = self.session.get_inputs()[0].name
                self.get_logger().info(
                    f"Loaded ONNX model: {self.model_path} "
                    f"providers={self.session.get_providers()} input={self.input_name}"
                )
            except Exception as e:
                self.get_logger().error(f"ONNX load failed: {e}; publishing empty arrays.")
                self.session = None

    # on_info(...)
    def on_info(self, msg: CameraInfo):
        self.last_info = msg
        if msg.header.frame_id:
            self.camera_frame_id = msg.header.frame_id
            

    def preprocess(self, img):
        """
        Letterbox to (W,H) = input_size, pad at bottom/right only.
        This keeps pad offset = (0,0) so back-projection is only scale by r.
        """
        h, w = img.shape[:2]
        inp_w, inp_h = int(self.input_size[0]), int(self.input_size[1])
        r = min(inp_w / w, inp_h / h)
        nw, nh = int(w * r), int(h * r)
        resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((inp_h, inp_w, 3), dtype=np.uint8)
        canvas[:nh, :nw] = resized
        blob = canvas[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0  # BGR->RGB, NCHW
        return blob, r

    def nms(self, boxes_xywh, scores, iou_thr):
        # boxes for cv2.dnn.NMSBoxes must be ints [x, y, w, h]
        b = [list(map(int, bb)) for bb in boxes_xywh]
        s = list(map(float, scores))
        idxs = cv2.dnn.NMSBoxes(b, s, score_threshold=float(self.conf_thr), nms_threshold=float(iou_thr))
        if len(idxs) == 0:
            return []
        return [int(i) for i in np.array(idxs).flatten()]

    def on_image(self, img_msg: Image):
        self.frame_idx += 1

        stamp = img_msg.header.stamp
        if getattr(stamp, 'sec', 0) == 0 and getattr(stamp, 'nanosec', 0) == 0:
            stamp = self.get_clock().now().to_msg()

        frame_id = img_msg.header.frame_id or self.camera_frame_id

        # Prepare an output message (we'll publish it even if empty)
        dets = Detection2DArray()
        dets.header = img_msg.header

        # If no session or skipping this frame -> publish empty array so tools see traffic
        if self.session is None or (self.frame_idx % max(1, int(self.infer_every_n)) != 0):


            anns = ImageAnnotations()
            # For each kept detection, add a LINE_LOOP rectangle + label text
            for d in dets.detections:
                x = d.bbox.center.position.x - d.bbox.size_x * 0.5
                y = d.bbox.center.position.y - d.bbox.size_y * 0.5
                w = d.bbox.size_x
                h = d.bbox.size_y

                pa = PointsAnnotation()
                pa.timestamp = dets.header.stamp
                pa.type = PointsAnnotation.LINE_LOOP
                pa.points = [
                    Point2(x=float(x),     y=float(y)),
                    Point2(x=float(x+w),   y=float(y)),
                    Point2(x=float(x+w),   y=float(y+h)),
                    Point2(x=float(x),     y=float(y+h)),
                ]
                pa.outline_color = Color(r=0, g=255, b=0, a=255)
                pa.thickness = 2.0
                anns.points.append(pa)

                if d.results:
                    txt = TextAnnotation()
                    txt.timestamp = dets.header.stamp
                    txt.position = Point2(x=float(x), y=float(max(y-6, 0)))
                    txt.text = f"{d.results[0].hypothesis.class_id} {d.results[0].hypothesis.score:.2f}"
                    anns.texts.append(txt)

            self.pub_ann.publish(anns)
            self.pub_det.publish(dets)
            return

        # Convert
        try:
            img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            self.pub_det.publish(dets)
            return

        # Preprocess
        blob, r = self.preprocess(img)
        inp = blob[None, ...]  # [1,3,H,W]

        # Inference
        try:
            outs = self.session.run(None, {self.input_name: inp})
        except Exception as e:
            self.get_logger().error(f"ORT inference failed: {e}")
            self.pub_det.publish(dets)
            return

        pred = outs[0]
        # Normalize output layout and extract person scores
        boxes_cxcywh, person_scores = parse_yolo_outputs(
            pred,
            num_classes=80,
            person_cls=int(self.person_ids[0]) if len(self.person_ids) else 0
        )

        # Build candidate boxes in XYWH on original image scale (no offsets because we letterboxed to top-left)
        boxes_xywh, scores = [], []
        for (cx, cy, w, h), sc in zip(boxes_cxcywh, person_scores):
            if float(sc) < float(self.conf_thr):
                continue
            x = (cx - w / 2.0) / r
            y = (cy - h / 2.0) / r
            bw = w / r
            bh = h / r
            # reject boxes that fall outside image
            if bw <= 1 or bh <= 1:  # trivial filter
                continue
            boxes_xywh.append([x, y, bw, bh])
            scores.append(float(sc))

        # NMS and publish
        if boxes_xywh:
            keep = self.nms(boxes_xywh, scores, self.iou_thr)
            for i in keep:
                x, y, bw, bh = boxes_xywh[i]
                cx, cy = x + bw / 2.0, y + bh / 2.0
                d = Detection2D()
                #d.header = img_msg.header
                d.header.stamp = stamp
                d.header.frame_id = frame_id
                d.bbox.center.position.x = float(cx)
                d.bbox.center.position.y = float(cy)
                d.bbox.center.theta = 0.0
                d.bbox.size_x = float(bw)
                d.bbox.size_y = float(bh)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "person"
                hyp.hypothesis.score = float(scores[i])
                d.results.append(hyp)
                dets.detections.append(d)

        self.pub_det.publish(dets)

def main():
    rclpy.init()
    rclpy.spin(PeopleDetector())
    rclpy.shutdown()
