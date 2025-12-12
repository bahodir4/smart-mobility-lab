#!/usr/bin/env python3
import argparse
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class QRFollower(Node):
    def __init__(self, image_topic: str, cmd_vel_topic: str, target_text: str = ""):
        super().__init__("tb3_qr_follower")
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
        self.target_text = target_text.strip()
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)

        # Simple controller parameters (tune if needed)
        self.linear_speed = 0.08
        self.angular_kp = 0.0035
        self.center_tol = 30  # px

        self.get_logger().info(
            f"QR follower started. image={image_topic} cmd_vel={cmd_vel_topic} target={self.target_text or '*any*'}"
        )

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        data, points, _ = self.detector.detectAndDecode(frame)

        twist = Twist()
        if points is None or not data:
            # Stop if nothing seen
            self.cmd_pub.publish(twist)
            return

        if self.target_text and data.strip() != self.target_text:
            self.cmd_pub.publish(twist)
            return

        # points: 4 corners -> compute center x
        pts = points[0]
        cx = float(pts[:, 0].mean())
        err = (w / 2.0) - cx

        # If centered, move forward; otherwise rotate to center it.
        if abs(err) > self.center_tol:
            twist.angular.z = float(self.angular_kp * err)
            twist.linear.x = 0.0
        else:
            twist.linear.x = float(self.linear_speed)
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="Camera image topic (bgr8), e.g. /camera/image_raw")
    ap.add_argument("--cmd-vel", required=True, help="cmd_vel topic, e.g. /cmd_vel")
    ap.add_argument("--target", default="", help="If set, follow only QR payload that equals this string.")
    args = ap.parse_args()

    rclpy.init()
    node = QRFollower(args.image, args.cmd_vel, args.target)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
