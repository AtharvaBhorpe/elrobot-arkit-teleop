"""Minimal camera publisher: /dev/videoN -> sensor_msgs/Image (bgr8).

One process per camera, so a wedged device can never stall the other camera
or anything else. No cv_bridge - the Image message is filled by hand.

    pixi run python -m elrobot.nodes.cam_node --device /dev/video0 --topic /wrist_cam/image
"""

import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CamNode(Node):
    def __init__(self, args):
        super().__init__(args.name)
        # MJPG must be requested AT OPEN, not after: OpenCV starts the stream
        # during open with the default uncompressed YUYV (~147 Mbps at
        # 640x480@30), and two of those exhaust a USB2 controller's
        # isochronous budget - the second camera dies with
        # VIDIOC_STREAMON: No space left on device.
        self.cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2, [
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"),
            cv2.CAP_PROP_FRAME_WIDTH, args.width,
            cv2.CAP_PROP_FRAME_HEIGHT, args.height,
            cv2.CAP_PROP_FPS, int(args.fps),
        ])
        if not self.cap.isOpened():
            raise SystemExit(f"cannot open {args.device}")
        self.pub = self.create_publisher(Image, args.topic, 1)
        self.frame_id = args.topic.strip("/").split("/")[0]
        self.create_timer(1.0 / args.fps, self._tick)
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))
        self.get_logger().info(
            f"{args.device} -> {args.topic}  negotiated {fourcc} "
            f"{int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}"
            f"@{self.cap.get(cv2.CAP_PROP_FPS):.0f}")

    def _tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("frame grab failed")
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = "bgr8"
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", default="/dev/video0")
    p.add_argument("--topic", default="/wrist_cam/image")
    p.add_argument("--name", default="cam_node")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = CamNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
