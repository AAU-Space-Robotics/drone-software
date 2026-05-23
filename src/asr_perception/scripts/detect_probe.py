#!/usr/bin/env python3
"""
detect_probe — runs YOLO on colour frames and publishes 2D detections.

Subscribes: /thyra/out/cam/synced/color   (sensor_msgs/Image)
Publishes:  /probe_detector/detections    (asr_comms/ProbeDetections)

On first run the .pt model is exported to a TensorRT .engine file (~2-5 min).
Subsequent runs load the .engine directly for full GPU performance.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from asr_comms.msg import ProbeDetections
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO


class ProbeDetector(Node):
    def __init__(self):
        super().__init__('probe_detector')

        self.declare_parameter('confidence_threshold', 0.75)
        self.declare_parameter('model', 'YOLO11m.pt')

        conf_thresh = self.get_parameter('confidence_threshold').value
        model_name  = self.get_parameter('model').value

        self._conf_thresh = float(conf_thresh)
        self._bridge      = CvBridge()

        pkg_share  = get_package_share_directory('asr_perception')
        model_path = os.path.join(pkg_share, 'models', model_name)
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'YOLO model not found: {model_path}')

        import torch
        if not torch.cuda.is_available():
            raise RuntimeError(
                'CUDA not available — cannot run probe detector without GPU. '
                'Install the Jetson-specific PyTorch wheel from developer.nvidia.com.')
        self._device = 0

        self._model = self._load_model(model_path)
        self.get_logger().info(f'confidence threshold: {self._conf_thresh:.2f}')

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self._sub = self.create_subscription(
            Image, '/thyra/out/cam/synced/color',
            self._on_image, qos_profile=qos)

        self._pub = self.create_publisher(ProbeDetections, '/probe_detector/detections', 10)

    def _load_model(self, pt_path: str) -> YOLO:
        engine_path = pt_path.replace('.pt', '.engine')
        if os.path.exists(engine_path):
            self.get_logger().info(f'Loading TensorRT engine: {engine_path}')
            return YOLO(engine_path)
        self.get_logger().info(
            f'No TensorRT engine found — exporting from {pt_path} (this takes a few minutes)...')
        model = YOLO(pt_path)
        model.export(format='engine', device=0, half=True, imgsz=640)
        self.get_logger().info(f'Engine saved to {engine_path}')
        return YOLO(engine_path)

    def _on_image(self, msg: Image):
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        results = self._model.predict(
            source=img, imgsz=640, conf=self._conf_thresh, device=self._device, verbose=False)

        out = ProbeDetections()
        out.header = msg.header

        if results[0].masks is not None:
            for box, mask in zip(results[0].boxes, results[0].masks):
                mask_data = mask.data[0].cpu().numpy()
                mask_data = cv2.resize(mask_data, (img.shape[1], img.shape[0]))
                ys, xs = np.where(mask_data > 0.5)
                if xs.size == 0:
                    continue
                out.centroid_x.append(float(np.mean(xs)))
                out.centroid_y.append(float(np.mean(ys)))
                out.confidence.append(float(box.conf.cpu()))
                out.num_detections += 1

        if out.num_detections > 0:
            self._pub.publish(out)
            self.get_logger().debug(f'{out.num_detections} probe(s) detected')


def main(args=None):
    rclpy.init(args=args)
    node = ProbeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
