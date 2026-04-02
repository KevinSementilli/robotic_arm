#!/usr/bin/env python3

from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

class YoloDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("robo_arm_vision_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("annotated_topic", "/camera/color/image_annotated")
        self.declare_parameter("detections_topic", "/detections_2d")

        self.model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        color_topic = str(self.get_parameter("color_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed. Install with: pip install ultralytics"
            ) from exc

        self.model = YOLO(self.model_path)
        self.class_names = self.model.names if isinstance(self.model.names, dict) else {}
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(Image, color_topic, self.image_callback, 10)
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)

        self.get_logger().info(f"YOLO model: {self.model_path}")
        self.get_logger().info(f"Input topic: {color_topic}")
        self.get_logger().info(f"Annotated topic: {annotated_topic}")
        self.get_logger().info(f"Detections topic: {detections_topic}")

    def image_callback(self, msg: Image) -> None:
        if (
            self.annotated_pub.get_subscription_count() == 0
            and self.detections_pub.get_subscription_count() == 0
        ):
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        try:
            results = self.model.predict(
                source=frame,
                imgsz=self.input_size,
                conf=self.confidence_threshold,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"YOLO inference failed: {exc}")
            return

        if not results:
            return

        result = results[0]
        boxes = result.boxes
        if boxes is None:
            return

        xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes.xyxy, "cpu") else np.asarray(boxes.xyxy)
        confs = boxes.conf.cpu().numpy() if hasattr(boxes.conf, "cpu") else np.asarray(boxes.conf)
        clss = boxes.cls.cpu().numpy() if hasattr(boxes.cls, "cpu") else np.asarray(boxes.cls)

        if self.detections_pub.get_subscription_count() > 0:
            detection_msg = self._build_detection_array(msg, xyxy, confs, clss)
            self.detections_pub.publish(detection_msg)

        if self.annotated_pub.get_subscription_count() > 0:
            annotated = result.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)

    def _build_detection_array(
        self,
        src_msg: Image,
        xyxy: np.ndarray,
        confs: np.ndarray,
        clss: np.ndarray,
    ) -> Detection2DArray:
        output = Detection2DArray()
        output.header = src_msg.header

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            score = float(confs[i])
            class_id = int(clss[i])
            label = self.class_names.get(class_id, str(class_id))

            det = Detection2D()
            det.header = src_msg.header

            bbox = BoundingBox2D()
            bbox.center.x = (x1 + x2) / 2.0
            bbox.center.y = (y1 + y2) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = max(x2 - x1, 1.0)
            bbox.size_y = max(y2 - y1, 1.0)
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis(class_id=label, score=score)
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)

            output.detections.append(det)

        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
