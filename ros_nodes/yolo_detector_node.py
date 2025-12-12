#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge

from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self, image_topic: str, model_path: str):
        super().__init__("tb3_yolo_detector")
        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.pub_det = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_img = self.create_publisher(Image, "/yolo/annotated", 10)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.get_logger().info(f"YOLO node started. image_topic={image_topic} model={model_path}")

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame, verbose=False)[0]

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        for b in results.boxes:
            cls_id = int(b.cls[0].item())
            conf = float(b.conf[0].item())
            x1, y1, x2, y2 = [float(v.item()) for v in b.xyxy[0]]

            det = Detection2D()
            det.header = msg.header

            bb = BoundingBox2D()
            bb.center.position.x = (x1 + x2) / 2.0
            bb.center.position.y = (y1 + y2) / 2.0
            bb.size_x = (x2 - x1)
            bb.size_y = (y2 - y1)
            det.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            det_arr.detections.append(det)

        self.pub_det.publish(det_arr)

        annotated = results.plot()
        out = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out.header = msg.header
        self.pub_img.publish(out)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="ROS2 image topic, e.g. /camera/image_raw")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLOv8 model path")
    args = parser.parse_args()

    rclpy.init()
    node = YoloNode(args.image, args.model)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
