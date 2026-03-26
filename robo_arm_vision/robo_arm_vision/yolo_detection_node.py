#!/usr/bin/env python3

import asyncio
import base64
from dataclasses import dataclass
import json
import os
import threading
import time
from typing import List, Optional, Tuple

from aiohttp import web
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from hailo_platform import (
    ConfigureParams,
    FormatType,
    HEF,
    HailoStreamInterface,
    InferVStreams,
    InputVStreamParams,
    OutputVStreamParams,
    VDevice,
)

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

INPUT_SIZE = 640


@dataclass
class DetectionResult:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int
    label: str
    distance_m: Optional[float]


def preprocess(frame: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
    h, w = frame.shape[:2]
    scale = INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(frame, (new_w, new_h))
    padded = np.full((INPUT_SIZE, INPUT_SIZE, 3), 114, dtype=np.uint8)
    pad_top = (INPUT_SIZE - new_h) // 2
    pad_left = (INPUT_SIZE - new_w) // 2
    padded[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = resized
    return padded, scale, pad_left, pad_top


def postprocess(
    outputs: dict,
    scale: float,
    pad_left: int,
    pad_top: int,
    orig_shape: Tuple[int, int, int],
    confidence_threshold: float,
) -> List[List[float]]:
    detections: List[List[float]] = []
    oh, ow = orig_shape[:2]

    for _, tensor in outputs.items():
        if isinstance(tensor, list):
            if not tensor:
                continue
            classes_data = tensor[0] if isinstance(tensor[0], list) else tensor
        else:
            arr = np.asarray(tensor)
            if arr.ndim == 4:
                arr = arr[0]
            classes_data = arr

        for class_id, class_dets in enumerate(classes_data):
            dets = np.asarray(class_dets)
            if dets.size == 0:
                continue
            dets = dets.reshape(-1, 5)

            for y_min, x_min, y_max, x_max, score in dets:
                score = float(score)
                if score < confidence_threshold:
                    continue
                x1 = max(0.0, (float(x_min) * INPUT_SIZE - pad_left) / scale)
                y1 = max(0.0, (float(y_min) * INPUT_SIZE - pad_top) / scale)
                x2 = min(float(ow), (float(x_max) * INPUT_SIZE - pad_left) / scale)
                y2 = min(float(oh), (float(y_max) * INPUT_SIZE - pad_top) / scale)
                if x2 > x1 and y2 > y1:
                    detections.append([x1, y1, x2, y2, score, class_id])

    return detections


def depth_to_heatmap(depth_image: np.ndarray) -> np.ndarray:
    depth_8u = cv2.convertScaleAbs(depth_image, alpha=0.03)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)


def estimate_distance_m(
    depth_image_m: np.ndarray,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
) -> Optional[float]:
    h, w = depth_image_m.shape
    ix1 = max(0, min(w - 1, int(x1)))
    iy1 = max(0, min(h - 1, int(y1)))
    ix2 = max(0, min(w, int(x2)))
    iy2 = max(0, min(h, int(y2)))

    if ix2 <= ix1 or iy2 <= iy1:
        return None

    bw = ix2 - ix1
    bh = iy2 - iy1
    cx1 = ix1 + bw // 3
    cy1 = iy1 + bh // 3
    cx2 = ix2 - bw // 3
    cy2 = iy2 - bh // 3

    if cx2 <= cx1 or cy2 <= cy1:
        cx1, cy1, cx2, cy2 = ix1, iy1, ix2, iy2

    roi = depth_image_m[cy1:cy2, cx1:cx2]
    valid = roi[(roi > 0.1) & (roi < 20.0)]
    if valid.size == 0:
        return None

    return float(np.median(valid))


def annotate(
    frame: np.ndarray,
    detections: list,
    depth_image_m: np.ndarray,
) -> Tuple[np.ndarray, List[DetectionResult], List[dict]]:
    detection_results: List[DetectionResult] = []
    enriched_payload: List[dict] = []

    for x1, y1, x2, y2, score, class_id in detections:
        distance_m = estimate_distance_m(depth_image_m, x1, y1, x2, y2)
        label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else str(class_id)

        label_text = f"{label} {score:.2f}"
        if distance_m is not None:
            label_text += f" {distance_m:.2f}m"

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(
            frame,
            label_text,
            (int(x1), max(20, int(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
        )

        detection_results.append(
            DetectionResult(
                x1=float(x1),
                y1=float(y1),
                x2=float(x2),
                y2=float(y2),
                score=float(score),
                class_id=int(class_id),
                label=label,
                distance_m=distance_m,
            )
        )

        enriched_payload.append(
            {
                "label": label,
                "score": round(float(score), 3),
                "distance_m": None if distance_m is None else round(distance_m, 3),
                "bbox": [int(x1), int(y1), int(x2), int(y2)],
            }
        )

    return frame, detection_results, enriched_payload


class HostedStreamer:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.clients = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.latest_payload = ""
        self._runner: Optional[web.AppRunner] = None

    async def ws_handler(self, request):
        ws = web.WebSocketResponse(max_msg_size=8 * 1024 * 1024)
        await ws.prepare(request)
        self.clients.add(ws)
        try:
            if self.latest_payload:
                await ws.send_str(self.latest_payload)
            async for _ in ws:
                pass
        finally:
            self.clients.discard(ws)
        return ws

    async def index_handler(self, _request):
        html = """
<!doctype html>
<html>
<head>
  <meta charset='utf-8' />
  <meta name='viewport' content='width=device-width, initial-scale=1' />
  <title>Detection + Depth Fusion</title>
  <style>
    body { margin: 0; background: #0b0f13; color: #e7edf3; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .bar { padding: 10px 14px; background: #131a22; border-bottom: 1px solid #263241; }
    .wrap { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; padding: 10px; height: calc(100vh - 48px); }
    .panel { display: grid; place-items: center; background: #10161d; border: 1px solid #263241; border-radius: 10px; }
    .title { align-self: start; justify-self: start; padding: 6px 10px; color: #a9bbcc; font-size: 12px; }
    img { max-width: 46vw; max-height: 82vh; border: 2px solid #263241; border-radius: 8px; }
  </style>
</head>
<body>
  <div class='bar'>
    <span id='status'>connecting...</span>
    <span id='stats' style='margin-left: 16px;'></span>
  </div>
  <div class='wrap'>
    <div class='panel'>
      <div class='title'>Color + Detections + Distance</div>
      <img id='color' />
    </div>
    <div class='panel'>
      <div class='title'>Depth Heatmap</div>
      <img id='depth' />
    </div>
  </div>

  <script>
    const statusEl = document.getElementById('status');
    const statsEl = document.getElementById('stats');
    const colorEl = document.getElementById('color');
    const depthEl = document.getElementById('depth');

    const wsProto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${wsProto}://${location.host}/ws`);

    ws.onopen = () => statusEl.textContent = 'connected';
    ws.onclose = () => statusEl.textContent = 'disconnected';
    ws.onerror = () => statusEl.textContent = 'error';

    ws.onmessage = (ev) => {
      const m = JSON.parse(ev.data);
      if (m.type === 'frame') {
        colorEl.src = m.color_image;
        depthEl.src = m.depth_image;
        statsEl.textContent = `frame=${m.frame_count} detections=${m.detection_count}`;
      }
    };
  </script>
</body>
</html>
        """
        return web.Response(text=html, content_type="text/html")

    async def broadcast(self, payload: str):
        if not self.clients:
            return
        dead = []
        for ws in self.clients:
            try:
                await ws.send_str(payload)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    def start(self) -> None:
        def _run() -> None:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            async def _main() -> None:
                app = web.Application()
                app.router.add_get("/", self.index_handler)
                app.router.add_get("/ws", self.ws_handler)
                self._runner = web.AppRunner(app)
                await self._runner.setup()
                site = web.TCPSite(self._runner, self.host, self.port)
                await site.start()
                await asyncio.Future()

            self.loop.run_until_complete(_main())

        self.thread = threading.Thread(target=_run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.loop is None:
            return

        async def _shutdown() -> None:
            if self._runner is not None:
                await self._runner.cleanup()

        fut = asyncio.run_coroutine_threadsafe(_shutdown(), self.loop)
        try:
            fut.result(timeout=2.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)

    def send_frame(self, color_jpeg: bytes, depth_jpeg: bytes, detections: list, frame_count: int) -> None:
        if not self.loop:
            return

        payload = json.dumps(
            {
                "type": "frame",
                "frame_count": frame_count,
                "detection_count": len(detections),
                "detections": detections,
                "color_image": "data:image/jpeg;base64," + base64.b64encode(color_jpeg).decode("ascii"),
                "depth_image": "data:image/jpeg;base64," + base64.b64encode(depth_jpeg).decode("ascii"),
            }
        )
        self.latest_payload = payload
        asyncio.run_coroutine_threadsafe(self.broadcast(payload), self.loop)


class RealsenseYoloNode(Node):
    def __init__(self) -> None:
        super().__init__("robo_arm_vision_node")

        self.declare_parameter("hef", "yolov8n.hef")
        self.declare_parameter("port", 5000)
        self.declare_parameter("stream_fps", 12)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("depth_unit_scale", 0.001)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("annotated_topic", "/camera/color/image_annotated")
        self.declare_parameter("detections_topic", "/detections_2d")
        self.declare_parameter("enriched_topic", "/detections_enriched")

        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        os.environ["HAILO_ENV_FILE"] = os.path.join(project_root, ".env")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.hef_path = str(self.get_parameter("hef").value)
        self.stream_fps = int(self.get_parameter("stream_fps").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.depth_unit_scale = float(self.get_parameter("depth_unit_scale").value)

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        annotated_topic = str(self.get_parameter("annotated_topic").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        enriched_topic = str(self.get_parameter("enriched_topic").value)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_sent = 0.0
        self.min_interval = 1.0 / max(self.stream_fps, 1)
        self._infer_lock = threading.Lock()

        hef = HEF(self.hef_path)
        self.vdevice = VDevice()
        cfg = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.vdevice.configure(hef, cfg)[0]
        self.network_group_params = self.network_group.create_params()
        self.in_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
        self.out_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
        self.input_name = hef.get_input_vstream_infos()[0].name
        self.network_group_activation = self.network_group.activate(self.network_group_params)
        self.network_group_activation.__enter__()
        self.infer_pipe = InferVStreams(self.network_group, self.in_params, self.out_params)
        self.infer_pipe.__enter__()

        self.color_sub = Subscriber(self, Image, color_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.sync = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.08)
        self.sync.registerCallback(self.synced_callback)

        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detections_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self.enriched_pub = self.create_publisher(String, enriched_topic, 10)

        self.streamer = HostedStreamer(self.host, self.port)
        self.streamer.start()

        self.get_logger().info(f"Web page: http://{self.host}:{self.port}")
        self.get_logger().info(f"WebSocket: ws://{self.host}:{self.port}/ws")

        self.get_logger().info(f"Subscribing color: {color_topic}")
        self.get_logger().info(f"Subscribing depth: {depth_topic}")
        self.get_logger().info(f"Publishing annotated image: {annotated_topic}")
        self.get_logger().info(f"Publishing detections: {detections_topic}")
        self.get_logger().info(f"Publishing enriched JSON: {enriched_topic}")

    def synced_callback(self, color_msg: Image, depth_msg: Image) -> None:
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        if depth_raw.ndim != 2:
            self.get_logger().warning("Depth image is not single-channel; skipping frame")
            return

        if np.issubdtype(depth_raw.dtype, np.integer):
            depth_m = depth_raw.astype(np.float32) * self.depth_unit_scale
        else:
            depth_m = depth_raw.astype(np.float32)

        inp, scale, pad_left, pad_top = preprocess(color)
        try:
            with self._infer_lock:
                out = self.infer_pipe.infer({self.input_name: np.expand_dims(inp, axis=0)})
        except Exception as exc:
            self.get_logger().error(f"Hailo inference failed: {exc}")
            return

        detections = postprocess(
            out,
            scale,
            pad_left,
            pad_top,
            color.shape,
            self.confidence_threshold,
        )

        annotated, detection_results, enriched_payload = annotate(color.copy(), detections, depth_m)
        self.frame_count += 1

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = color_msg.header
        self.annotated_pub.publish(annotated_msg)

        detections_msg = self.build_detection_array_msg(color_msg, detection_results)
        self.detections_pub.publish(detections_msg)

        enriched_msg = String()
        enriched_msg.data = json.dumps(
            {
                "frame_count": self.frame_count,
                "detection_count": len(enriched_payload),
                "detections": enriched_payload,
            }
        )
        self.enriched_pub.publish(enriched_msg)

        heatmap = depth_to_heatmap(depth_raw)
        now = time.time()
        if now - self.last_sent >= self.min_interval:
            ok_color, jpeg_color = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
            ok_depth, jpeg_depth = cv2.imencode(".jpg", heatmap, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok_color and ok_depth:
                self.streamer.send_frame(
                    jpeg_color.tobytes(),
                    jpeg_depth.tobytes(),
                    enriched_payload,
                    self.frame_count,
                )
                self.last_sent = now

    def build_detection_array_msg(
        self,
        color_msg: Image,
        detection_results: List[DetectionResult],
    ) -> Detection2DArray:
        output = Detection2DArray()
        output.header = color_msg.header

        for det in detection_results:
            item = Detection2D()
            item.header = color_msg.header

            center_x = (det.x1 + det.x2) / 2.0
            center_y = (det.y1 + det.y2) / 2.0
            size_x = max(det.x2 - det.x1, 1.0)
            size_y = max(det.y2 - det.y1, 1.0)

            bbox = BoundingBox2D()
            bbox.center.x = float(center_x)
            bbox.center.y = float(center_y)
            bbox.center.theta = 0.0
            bbox.size_x = float(size_x)
            bbox.size_y = float(size_y)
            item.bbox = bbox

            result = ObjectHypothesisWithPose()
            result.hypothesis = ObjectHypothesis(class_id=det.label, score=float(det.score))
            result.pose.pose.position.z = float(det.distance_m) if det.distance_m is not None else float("nan")
            result.pose.pose.orientation.w = 1.0
            item.results.append(result)

            output.detections.append(item)

        return output

    def destroy_node(self) -> bool:
        try:
            self.infer_pipe.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self.network_group_activation.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self.vdevice.release()
        except Exception:
            pass
        self.streamer.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseYoloNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()