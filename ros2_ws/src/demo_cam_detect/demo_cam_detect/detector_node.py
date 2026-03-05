#!/usr/bin/env python3
import time
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from .yolov8_onnx import letterbox, postprocess_yolov8
from .metrics import EmaMeters


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        # ---- params ----
        self.declare_parameter('model', '')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('image_topic', '/image')

        # viz output
        self.declare_parameter('publish_viz', True)
        self.declare_parameter('viz_topic', '/detections_image')

        model = str(self.get_parameter('model').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.image_topic = str(self.get_parameter('image_topic').value)

        self.publish_viz = bool(self.get_parameter('publish_viz').value)
        self.viz_topic = str(self.get_parameter('viz_topic').value)

        if not model:
            raise RuntimeError('param "model" is required')

        # ---- ORT session ----
        ort.set_default_logger_severity(3)
        self.sess = ort.InferenceSession(model, providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name

        # ---- QoS (low latency) ----
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()

        # ---- ROS I/O ----
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, qos)
        self.pub_det = self.create_publisher(Detection2DArray, '/detections', qos)
        self.pub_metrics = self.create_publisher(Float32MultiArray, '/detector/metrics', 10)
        self.pub_viz = self.create_publisher(Image, self.viz_topic, qos)

        # ---- meters ----
        self.meters = EmaMeters(alpha=0.1)
        self.last_log_t = time.perf_counter()
        self.frames_1s = 0

        self.get_logger().info(
            f'detector_node started. sub={self.image_topic} pub=/detections '
            f'viz={self.viz_topic if self.publish_viz else "disabled"}'
        )

    def cb_image(self, msg: Image):
        # 1) ROS Image -> cv2 BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        t0 = time.perf_counter()

        # 2) preprocess (match Phase A)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img, r, (dw, dh) = letterbox(img, (self.imgsz, self.imgsz))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))[None, ...]
        t1 = time.perf_counter()

        # 3) infer
        out = self.sess.run(None, {self.input_name: img})
        t2 = time.perf_counter()

        # 4) postprocess (returns list[(xyxy, score, cls_id)])
        dets = postprocess_yolov8(out[0], conf_thres=self.conf, iou_thres=self.iou)
        t3 = time.perf_counter()

        # timings
        pre_ms = (t1 - t0) * 1000.0
        inf_ms = (t2 - t1) * 1000.0
        post_ms = (t3 - t2) * 1000.0
        total_ms = (t3 - t0) * 1000.0
        self.meters.update(pre_ms, inf_ms, post_ms, total_ms)

        # 5) publish detections
        det_msg = Detection2DArray()
        det_msg.header = msg.header
        if not det_msg.header.frame_id:
            det_msg.header.frame_id = self.frame_id

        H, W = frame.shape[:2]

        # optional viz frame (only create if enabled)
        vis = frame.copy() if self.publish_viz else None

        for (xyxy, score, cls_id) in dets:
            x1, y1, x2, y2 = xyxy

            # map back from letterbox coords to original image coords
            x1 = (x1 - dw) / r
            y1 = (y1 - dh) / r
            x2 = (x2 - dw) / r
            y2 = (y2 - dh) / r

            x1 = float(max(0, min(x1, W - 1)))
            x2 = float(max(0, min(x2, W - 1)))
            y1 = float(max(0, min(y1, H - 1)))
            y2 = float(max(0, min(y2, H - 1)))

            # Detection2D message
            det = Detection2D()
            det.header = det_msg.header

            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = (x2 - x1)
            bbox.size_y = (y2 - y1)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(cls_id))
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)

            det_msg.detections.append(det)

            # draw viz
            if vis is not None:
                ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(vis, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                cv2.putText(
                    vis, f"{cls_id}:{score:.2f}",
                    (ix1, max(0, iy1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

        self.pub_det.publish(det_msg)

        # 6) publish viz image
        if vis is not None:
            try:
                viz_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                viz_msg.header = msg.header
                self.pub_viz.publish(viz_msg)
            except Exception as e:
                self.get_logger().warn(f'viz publish error: {e}')

        # 7) log + publish metrics once per ~1s
        self.frames_1s += 1
        now = time.perf_counter()
        if (now - self.last_log_t) >= 1.0:
            fps = self.frames_1s / (now - self.last_log_t)
            self.frames_1s = 0
            self.last_log_t = now

            ema = self.meters.ema()
            self.get_logger().info(
                f"fps={fps:.1f} pre={ema['pre']:.1f}ms inf={ema['infer']:.1f}ms "
                f"post={ema['post']:.1f}ms total={ema['total']:.1f}ms det={len(dets)}"
            )

            m = Float32MultiArray()
            m.data = [float(fps), ema["pre"], ema["infer"], ema["post"], ema["total"], float(len(dets))]
            self.pub_metrics.publish(m)


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
