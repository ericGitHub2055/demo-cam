#!/usr/bin/env python3
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image


class MjpegBridgeNode(Node):
    def __init__(self):
        super().__init__('mjpeg_bridge_node')

        self.declare_parameter('image_topic', '/detections_image')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 80)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_jpeg = None

        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, qos)

        self.httpd = HTTPServer(("0.0.0.0", self.port), self._make_handler())
        t = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        t.start()

        self.get_logger().info(
            f"MJPEG: http://<PI_IP>:{self.port}/stream.mjpg  (topic={self.image_topic})"
        )

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path not in ("/", "/stream.mjpg"):
                    self.send_response(404)
                    self.end_headers()
                    return

                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        with node.lock:
                            buf = node.latest_jpeg
                        if buf is None:
                            time.sleep(0.01)
                            continue

                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(buf)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except Exception:
                    return

            def log_message(self, format, *args):
                return

        return Handler

    def cb_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
        if not ok:
            return

        with self.lock:
            self.latest_jpeg = jpg.tobytes()


def main():
    rclpy.init()
    node = MjpegBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
