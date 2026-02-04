#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

class ImageQoSRelay(Node):
    def __init__(self):
        super().__init__("image_qos_relay")

        self.declare_parameter("in_topic", "/camera/image_raw")
        self.declare_parameter("out_topic", "/camera/image_raw_be")
        self.declare_parameter("depth", 5)

        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        depth = int(self.get_parameter("depth").get_parameter_value().integer_value)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

        self._pub = self.create_publisher(Image, out_topic, pub_qos)
        self._sub = self.create_subscription(Image, in_topic, self._cb, sub_qos)

        self.get_logger().info(
            f"Relaying {in_topic} -> {out_topic} (pub BEST_EFFORT, depth={depth})"
        )

    def _cb(self, msg: Image):
        self._pub.publish(msg)

def main():
    rclpy.init()
    node = ImageQoSRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
