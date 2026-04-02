#!/usr/bin/env python3
"""
YOLO 3D Detection Node with depth fusion and TF2 transformation.

Subscribes to synchronized color, depth, and camera_info topics.
Publishes Detection3DArray with object poses in the target frame (default: world).
"""

from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, Vector3
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)


class YoloDetection3DNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detection_3d_node")

        # Declare parameters
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("input_size", 640)
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("annotated_topic", "/camera/color/image_annotated")
        self.declare_parameter("detections_topic", "/detections_3d")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("min_depth", 0.1)
        self.declare_parameter("max_depth", 20.0)
        self.declare_parameter("default_object_depth", 0.05)
        self.declare_parameter("sync_slop", 0.1)

        # Get parameters
        self.model_path = str(self.get_parameter("model_path").value)
        self.device = str(self.get_parameter("device").value)
        self.input_size = int(self.get_parameter("input_size").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.default_object_depth = float(self.get_parameter("default_object_depth").value)
        sync_slop = float(self.get_parameter("sync_slop").value)

        # Load YOLO model
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Ultralytics is not installed. Install with: pip install ultralytics"
            ) from exc

        self.model = YOLO(self.model_path)
        self.class_names = self.model.names if isinstance(self.model.names, dict) else {}
        self.bridge = CvBridge()

        # Camera intrinsics (will be populated from CameraInfo)
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None
        self.cx: Optional[float] = None
        self.cy: Optional[float] = None
        self.camera_frame: Optional[str] = None

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Synchronized subscribers using message_filters
        self.color_sub = message_filters.Subscriber(self, Image, color_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=sync_slop,
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Publishers
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detections_pub = self.create_publisher(Detection3DArray, detections_topic, 10)

        # Object tracking counter for unique IDs
        self.object_counters: dict[str, int] = {}

        self.get_logger().info(f"YOLO 3D Detection Node initialized")
        self.get_logger().info(f"  Model: {self.model_path}")
        self.get_logger().info(f"  Color topic: {color_topic}")
        self.get_logger().info(f"  Depth topic: {depth_topic}")
        self.get_logger().info(f"  Target frame: {self.target_frame}")

    def synchronized_callback(
        self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo
    ) -> None:
        """Process synchronized color, depth, and camera info messages."""
        # Skip if no subscribers
        if (
            self.annotated_pub.get_subscription_count() == 0
            and self.detections_pub.get_subscription_count() == 0
        ):
            return

        # Update camera intrinsics
        self._update_intrinsics(info_msg)

        if self.fx is None:
            self.get_logger().warn("Camera intrinsics not yet available")
            return

        # Convert images
        try:
            color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        # Convert depth to meters (RealSense depth is in mm as uint16)
        if depth_frame.dtype == np.uint16:
            depth_m = depth_frame.astype(np.float32) * 0.001
        else:
            depth_m = depth_frame.astype(np.float32)

        # Run YOLO inference
        try:
            results = self.model.predict(
                source=color_frame,
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

        # Build and publish Detection3DArray
        if self.detections_pub.get_subscription_count() > 0:
            detection_msg = self._build_detection_3d_array(
                color_msg.header, depth_m, xyxy, confs, clss
            )
            if detection_msg.detections:
                self.detections_pub.publish(detection_msg)

        # Publish annotated image
        if self.annotated_pub.get_subscription_count() > 0:
            annotated = result.plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = color_msg.header
            self.annotated_pub.publish(annotated_msg)

    def _update_intrinsics(self, info_msg: CameraInfo) -> None:
        """Extract camera intrinsics from CameraInfo message."""
        if len(info_msg.k) >= 9:
            self.fx = info_msg.k[0]
            self.fy = info_msg.k[4]
            self.cx = info_msg.k[2]
            self.cy = info_msg.k[5]
            self.camera_frame = info_msg.header.frame_id

    def _estimate_depth_roi(
        self, depth_m: np.ndarray, x1: float, y1: float, x2: float, y2: float
    ) -> Optional[float]:
        """
        Estimate depth using the center 1/3 ROI of the bounding box.
        Returns median of valid depth values, or None if no valid depths.
        """
        h, w = depth_m.shape
        ix1 = max(0, min(w - 1, int(x1)))
        iy1 = max(0, min(h - 1, int(y1)))
        ix2 = max(0, min(w, int(x2)))
        iy2 = max(0, min(h, int(y2)))

        if ix2 <= ix1 or iy2 <= iy1:
            return None

        # Extract center 1/3 ROI to avoid edge artifacts
        bw = ix2 - ix1
        bh = iy2 - iy1
        cx1 = ix1 + bw // 3
        cy1 = iy1 + bh // 3
        cx2 = ix2 - bw // 3
        cy2 = iy2 - bh // 3

        # Fallback to full bbox if center ROI is too small
        if cx2 <= cx1 or cy2 <= cy1:
            cx1, cy1, cx2, cy2 = ix1, iy1, ix2, iy2

        roi = depth_m[cy1:cy2, cx1:cx2]
        valid = roi[(roi > self.min_depth) & (roi < self.max_depth)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def _compute_3d_position(
        self, u: float, v: float, z: float
    ) -> tuple[float, float, float]:
        """Convert pixel coordinates and depth to 3D position in camera frame."""
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def _estimate_object_size(
        self, bbox_w: float, bbox_h: float, z: float
    ) -> tuple[float, float, float]:
        """Estimate object dimensions from bounding box size and depth."""
        width = bbox_w * z / self.fx
        height = bbox_h * z / self.fy
        depth = self.default_object_depth  # Assume fixed depth for now
        return width, height, depth

    def _transform_pose_to_target_frame(
        self, pose: Pose, source_frame: str, stamp: Time
    ) -> Optional[Pose]:
        """Transform pose from source frame to target frame using TF2."""
        if source_frame == self.target_frame:
            return pose

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose

        try:
            # Use Time(0) to get the latest available transform
            # This avoids extrapolation errors when transforms are slightly delayed
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            transformed = do_transform_pose_stamped(pose_stamped, transform)
            return transformed.pose
        except TransformException as exc:
            self.get_logger().warn(
                f"TF transform failed ({source_frame} -> {self.target_frame}): {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _generate_object_id(self, class_name: str) -> str:
        """Generate unique object ID for tracking."""
        if class_name not in self.object_counters:
            self.object_counters[class_name] = 0
        idx = self.object_counters[class_name]
        self.object_counters[class_name] = (idx + 1) % 100
        return f"yolo_{class_name}_{idx}"

    def _build_detection_3d_array(
        self,
        header,
        depth_m: np.ndarray,
        xyxy: np.ndarray,
        confs: np.ndarray,
        clss: np.ndarray,
    ) -> Detection3DArray:
        """Build Detection3DArray from YOLO detections and depth data."""
        output = Detection3DArray()
        output.header.stamp = header.stamp
        output.header.frame_id = self.target_frame

        stamp = Time.from_msg(header.stamp)

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            score = float(confs[i])
            class_id = int(clss[i])
            class_name = self.class_names.get(class_id, str(class_id))

            # Estimate depth from ROI
            z = self._estimate_depth_roi(depth_m, x1, y1, x2, y2)
            if z is None:
                self.get_logger().debug(f"No valid depth for {class_name}")
                continue

            # Compute 3D position in camera frame
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0
            cam_x, cam_y, cam_z = self._compute_3d_position(u, v, z)

            # Create pose in camera frame
            camera_pose = Pose()
            camera_pose.position.x = cam_x
            camera_pose.position.y = cam_y
            camera_pose.position.z = cam_z
            camera_pose.orientation.w = 1.0

            # Transform to target frame
            world_pose = self._transform_pose_to_target_frame(
                camera_pose, self.camera_frame, stamp
            )
            if world_pose is None:
                continue

            # Estimate object size
            bbox_w = x2 - x1
            bbox_h = y2 - y1
            width, height, depth = self._estimate_object_size(bbox_w, bbox_h, z)

            # Build Detection3D message
            det = Detection3D()
            det.header.stamp = header.stamp
            det.header.frame_id = self.target_frame

            # Bounding box
            det.bbox = BoundingBox3D()
            det.bbox.center = world_pose
            det.bbox.size = Vector3(x=width, y=height, z=depth)

            # Object hypothesis with pose
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis = ObjectHypothesis(class_id=class_name, score=score)
            hyp.pose = PoseWithCovariance()
            hyp.pose.pose = world_pose
            det.results.append(hyp)

            # Generate tracking ID
            det.id = self._generate_object_id(class_name)

            output.detections.append(det)

        return output


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetection3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
