#!/usr/bin/env python3
import rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
try:
    import onnxruntime as ort
except Exception:
    ort = None

def parse_yolo_outputs(outputs, num_classes=80, person_cls=0):
    """
    Accepts:
      - YOLOv8: [N, 84] = cx,cy,w,h + 80 class scores
      - YOLOv5: [N, 85] = cx,cy,w,h,objectness + 80 class scores
    Returns:
      boxes_cxcywh (float32, Nx4), scores (float32, N)
    """
    if outputs.ndim == 3:
        outputs = outputs[0]
    if outputs.ndim != 2 or outputs.shape[1] < 6:
        return np.zeros((0,4), np.float32), np.zeros((0,), np.float32)

    D = outputs.shape[1]
    if D == 84:  # v8
        boxes = outputs[:, :4]
        class_scores = outputs[:, 4:4+num_classes]
        scores = class_scores[:, person_cls]
    elif D == 85:  # v5
        boxes = outputs[:, :4]
        obj = outputs[:, 4]
        class_scores = outputs[:, 5:5+num_classes]
        scores = class_scores[:, person_cls] * obj
    else:
        # Try to infer: assume last num_classes are class probs
        boxes = outputs[:, :4]
        class_scores = outputs[:, -num_classes:]
        scores = class_scores[:, person_cls]
    return boxes.astype(np.float32), scores.astype(np.float32)

class PeopleDetector(Node):
    def __init__(self):
        super().__init__('people_detector')
        self.declare_parameters('', [
            ('image_topic', '/camera/image_raw'),
            ('camera_info_topic', '/camera/camera_info'),
            ('det_topic', '/detections/person'),
            ('model_path', ''),
            ('input_size', [640, 640]),
            ('conf_threshold', 0.3),
            ('iou_threshold', 0.45),
            ('infer_every_n', 2),
            ('person_class_ids', [0]),
        ])
        vals = [p.value for p in self.get_parameters([
            'image_topic','camera_info_topic','det_topic','model_path','input_size',
            'conf_threshold','iou_threshold','infer_every_n','person_class_ids'
        ])]
        (self.image_topic, info_topic, self.det_topic, self.model_path, self.input_size,
         self.conf_thr, self.iou_thr, self.infer_every_n, self.person_ids) = vals

        self.bridge = CvBridge()
        self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub_det = self.create_publisher(Detection2DArray, self.det_topic, 10)

        self.frame_idx = 0
        self.session = None
        self.input_name = None
        if ort is None:
            self.get_logger().warn("onnxruntime not installed; staying idle.")
        elif self.model_path:
            try:
                self.session = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
                self.input_name = self.session.get_inputs()[0].name
                self.get_logger().info(f"Loaded ONNX model: {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"ONNX load failed: {e}")

    def on_info(self, msg: CameraInfo):
        pass

    def preprocess(self, img):
        h, w = img.shape[:2]
        inp_w, inp_h = self.input_size
        r = min(inp_w / w, inp_h / h)
        nw, nh = int(w * r), int(h * r)
        resized = cv2.resize(img, (nw, nh))
        canvas = np.zeros((inp_h, inp_w, 3), dtype=np.uint8)
        canvas[:nh, :nw] = resized
        blob = canvas[:, :, ::-1].transpose(2,0,1).astype(np.float32) / 255.0
        return blob, r

    def nms(self, boxes, scores, iou_thr):
        idxs = cv2.dnn.NMSBoxes(
            bboxes=[list(map(int, b)) for b in boxes],
            scores=list(map(float, scores)),
            score_threshold=0.0,
            nms_threshold=float(iou_thr)
        )
        if len(idxs)==0:
            return []
        return [int(i) for i in np.array(idxs).flatten()]

    def on_image(self, img_msg: Image):
        self.frame_idx += 1
        if self.session is None or (self.frame_idx % max(1,int(self.infer_every_n)) != 0):
            return
        img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        blob, r = self.preprocess(img)
        inp = blob[None, ...]
        outputs = self.session.run(None, {self.input_name: inp})[0]
        if outputs.ndim == 3:
            outputs = outputs[0]

        boxes, scores = [], []
        dets = Detection2DArray()
        dets.header = img_msg.header

        for row in outputs:
            if row.shape[0] >= 6:
                x, y, w, h = row[0], row[1], row[2], row[3]
                conf = row[4]
                if row.shape[0] > 6:
                    cls = int(np.argmax(row[5:]))
                    cls_conf = row[5 + cls]
                    score = float(conf * cls_conf)
                else:
                    cls = int(row[5])
                    score = float(conf)
                if score < self.conf_thr or cls not in self.person_ids:
                    continue
                bx = (x - w/2) / r
                by = (y - h/2) / r
                bw = w / r
                bh = h / r
                boxes.append([bx, by, bw, bh])
                scores.append(score)

        if boxes:
            keep = self.nms(boxes, scores, self.iou_thr)
            for i in keep:
                bx, by, bw, bh = boxes[i]
                cx, cy = bx + bw/2, by + bh/2
                d = Detection2D()
                d.header = img_msg.header
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
