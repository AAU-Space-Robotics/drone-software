#!/usr/bin/env python3
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from interfaces.msg import ProbeSegmentation, ProbeLocations
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from typing import Tuple, List
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import struct

# Intrinsic parameters for RealSense D435 (depth stream, e.g., 640x480)
CAMERA_INTRINSIC_MATRIX = np.array([
    [462.0,   0.0, 320.0],  # fx, 0, cx
    [0.0,   462.0, 240.0],  # 0, fy, cy
    [0.0,     0.0,   1.0]
], dtype=np.float64)

CAMERA_DISTORTION_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
CAMERA_EXTRINSIC_ROTATION = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64).flatten('F')
CAMERA_EXTRINSIC_ROTATION_MATRIX = CAMERA_EXTRINSIC_ROTATION.reshape((3, 3), order='F')
CAMERA_EXTRINSIC_TRANSLATION = np.array([0.0, 0.0, 0.0], dtype=np.float64)

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.previous_time = 0
        self.fps = 1
        self.confidence_threshold = 0.75
        self.rgb_image = None
        self.depth_image = None
        
        self.rgb_sub = Subscriber(self, CompressedImage, '/thyra/out/color_image/compressed', qos_profile=qos)
        self.depth_sub = Subscriber(self, CompressedImage, '/thyra/out/depth_image/compressed', qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_handler)
        
        self.image_publisher = self.create_publisher(Image, '/probe_detector/segmented_image', 10)
        self.probe_publisher = self.create_publisher(ProbeLocations, '/probe_detector/probe_locations', 10)
                   
        self.bridge = CvBridge()
        
        package_name = 'probe_perception'
        try:
            package_share_dir = get_package_share_directory(package_name)
            self.model_path = os.path.join(package_share_dir, 'models', 'YOLO11m.pt')
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"Model file not found at: {self.model_path}")
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Loaded YOLO model from {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize model: {e}")
            raise

    def image_handler(self, rgb_msg, depth_msg):
        try:
            rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9

            if self.previous_time + (1 / self.fps) > rgb_time:
                return
            self.previous_time = rgb_time

            self.get_logger().info(f"Synchronized RGB Time: {rgb_time}, Depth Time: {depth_time}")

            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            np_arr_depth = np.frombuffer(depth_msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr_depth, cv2.IMREAD_UNCHANGED)

            probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)
            
            if probe_masks:
                probe_locations = self.compute_probe_locations(probe_masks, depth_image, rgb_image, probe_confidences)
                
                if probe_locations:
                    sorted_locations = sorted(probe_locations, key=lambda x: x["confidence"], reverse=True)
                    probe_confidences = [float(np.float32(loc["confidence"])) for loc in sorted_locations]
                    probe_list = [float(np.float32(coord)) for loc in sorted_locations for coord in (loc["x"], loc["y"], loc["z"])]
                    probe_centroid_x = [float(np.float32(loc["centroid_x"])) for loc in sorted_locations]
                    probe_centroid_y = [float(np.float32(loc["centroid_y"])) for loc in sorted_locations]
       
                    probe_msg = ProbeLocations()
                    probe_msg.header = rgb_msg.header
                    probe_msg.classification_confidence = probe_confidences
                    probe_msg.num_probes = len(sorted_locations)
                    probe_msg.probes = probe_list
                    probe_msg.centroid_x = probe_centroid_x
                    probe_msg.centroid_y = probe_centroid_y

                    self.probe_publisher.publish(probe_msg)
                    self.get_logger().info(f"Published {probe_msg.num_probes} probe locations")
        
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to process image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_handler: {e}")

    def compute_probe_locations(self, probe_masks: List[np.ndarray], depth_image: np.ndarray, rgb_image: np.ndarray, probe_confidences: List[float]) -> List[dict]:
        probe_locations = []
        self.get_logger().info(f"RGB Image Size: {rgb_image.shape}")
        self.get_logger().info(f"Depth Image Size: {depth_image.shape}")
               
        for i, mask in enumerate(probe_masks):
            try:
                mask = cv2.resize(mask, (depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_LINEAR)
                binary_mask = (mask > 0.5).astype(np.uint8)
                y, x = np.where(binary_mask == 1)
                if x.size > 0 and y.size > 0:
                    centroid_x = int(np.mean(x))
                    centroid_y = int(np.mean(y))
                    point_cloud_segment = self.depth_to_pointcloud(depth_image, (centroid_x, centroid_y), 5)
                    valid_points = point_cloud_segment[~np.isnan(point_cloud_segment).any(axis=1) & ~np.isinf(point_cloud_segment).any(axis=1)]
                    position = np.nanmedian(valid_points, axis=0)
                    if np.isnan(position).any() or np.isinf(position).any():
                        continue
                else:
                    continue  
                           
                probe_location = {
                    "centroid_x": centroid_x,
                    "centroid_y": centroid_y,
                    "confidence": probe_confidences[i],
                    "x": position[0],
                    "y": position[1],
                    "z": position[2]
                }
                probe_locations.append(probe_location)
                  
            except Exception as e:
                self.get_logger().error(f"Error processing mask {i}: {e}")
                continue

        resized_rgb = cv2.resize(rgb_image, (depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_LINEAR)
        for loc in probe_locations:
            centroid_x = loc["centroid_x"]
            centroid_y = loc["centroid_y"]
            position = [loc["x"], loc["y"], loc["z"]]
            confidence = loc["confidence"]
            cv2.circle(resized_rgb, (centroid_x, centroid_y), 3, (0, 0, 255), -1)
            cv2.putText(resized_rgb, f"conf={confidence:.2f}, x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}", 
                        (centroid_x + 5, centroid_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            print(f"Probe {i}: Centroid ({centroid_x}, {centroid_y}), Position ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), Confidence {confidence:.2f}")
    
        try:
            segmented_msg = self.bridge.cv2_to_imgmsg(resized_rgb, "bgr8")
            self.image_publisher.publish(segmented_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to publish segmented image: {e}")

        return probe_locations

    def infer_yolo(self, rgb_image: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray], np.ndarray]:
        probe_boxes = []
        probe_masks = []
        probe_confidences = []

        try:
            results = self.model.predict(
                source=rgb_image,
                imgsz=640,
                conf=self.confidence_threshold,
                device=0 
            )
            
            if results[0].boxes is not None and results[0].masks is not None:
                for i, (box, mask) in enumerate(zip(results[0].boxes, results[0].masks)):
                    try:
                        box_xyxy = box.xyxy[0].cpu().numpy()
                        mask_data = mask.data[0].cpu().numpy()
                        if rgb_image is not None:
                            mask_data = cv2.resize(mask_data, (rgb_image.shape[1], rgb_image.shape[0]))
                        probe_boxes.append(box_xyxy)
                        probe_masks.append(mask_data)
                        probe_confidences.append(float(box.conf.cpu()))
                    except Exception as e:
                        self.get_logger().error(f"Error processing detection {i}: {e}")
                        continue

            probe_boxes = np.array(probe_boxes, dtype=np.float32) if probe_boxes else np.array([], dtype=np.float32).reshape(0, 4)
            probe_confidences = np.array(probe_confidences, dtype=np.float32) if probe_confidences else np.array([], dtype=np.float32)

        except Exception as e:
            self.get_logger().error(f"Error in YOLO inference: {e}")

        return probe_boxes, probe_masks, probe_confidences

    def depth_to_pointcloud(self, depth_image: np.ndarray, position: tuple, kernel_size: int) -> np.ndarray:
        h, w = depth_image.shape
        u, v = position
        if kernel_size < 1:
            raise ValueError("Kernel size must be positive")
        half_kernel = kernel_size // 2
        u_min = max(0, u - half_kernel)
        u_max = min(w, u + half_kernel + 1)
        v_min = max(0, v - half_kernel)
        v_max = min(h, v + half_kernel + 1)
        depth_patch = depth_image[v_min:v_max, u_min:u_max]
        u_patch, v_patch = np.meshgrid(np.arange(u_min, u_max), np.arange(v_min, v_max))
        fx, fy = CAMERA_INTRINSIC_MATRIX[0, 0], CAMERA_INTRINSIC_MATRIX[1, 1]
        cx, cy = CAMERA_INTRINSIC_MATRIX[0, 2], CAMERA_INTRINSIC_MATRIX[1, 2]
        depth = depth_patch.astype(np.float64) / 1000.0
        x = (u_patch - cx) * depth / fx
        y = (v_patch - cy) * depth / fy
        z = depth
        point_cloud = np.stack((x, y, z), axis=-1).astype(np.float32)
        points = point_cloud.reshape(-1, 3)
        points = (CAMERA_EXTRINSIC_ROTATION_MATRIX @ points.T).T + CAMERA_EXTRINSIC_TRANSLATION
        valid = (depth > 0).reshape(-1)
        point_cloud = points[valid]
        return point_cloud
    
def main(args=None):
    rclpy.init(args=args)
    try:    
        node = SegmentationNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()