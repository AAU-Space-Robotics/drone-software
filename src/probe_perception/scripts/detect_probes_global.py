#!/usr/bin/env python3
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from interfaces.msg import ProbeLocations, ProbeGlobalLocations
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from typing import Tuple, List
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Intrinsic parameters for RealSense D435
CAMERA_INTRINSIC_MATRIX = np.array([
    [425.8813171386719, 0.0, 430.5101623535156],  # fx, 0, cx
    [0.0, 425.8813171386719, 238.53343200683594],  # 0, fy, cy
    [0.0, 0.0, 1.0]
], dtype=np.float64)

CAMERA_DISTORTION_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
CAMERA_EXTRINSIC_ROTATION = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
], dtype=np.float64).flatten('F')
CAMERA_EXTRINSIC_ROTATION_MATRIX = CAMERA_EXTRINSIC_ROTATION.reshape((3, 3), order='F')

CAMERA_EXTRINSIC_TRANSLATION = np.array([0.0, 0.0, 0.0], dtype=np.float64)

# Camera to drone transform (4x4 homogeneous matrix)
CAMERA_TO_DRONE_TRANSFORM = np.array([
    [0.0, -0.70710678, 0.70710678, 0.137751],
    [1.0, 0.0, 0.0, -0.018467],
    [0.0, 0.70710678, 0.70710678, 0.12126],
    [0.0, 0.0, 0.0, 1.0]
], dtype=np.float64)

# Probe class for tracking and merging unique probes
class Probe:
    def __init__(self, x, y, z, confidence):
        self.x_sum = x
        self.y_sum = y
        self.z_sum = z
        self.confidence_sum = confidence
        self.count = 1

    def update(self, x, y, z, confidence):
        self.x_sum += x
        self.y_sum += y
        self.z_sum += z
        self.confidence_sum += confidence
        self.count += 1

    def distance_to(self, x, y, z):
        avg_x, avg_y, avg_z = self.get_average_position()
        dx = avg_x - x
        dy = avg_y - y
        return np.sqrt(dx * dx + dy * dy)  # 2D distance (ignore z for merging)

    def get_average_position(self):
        return (self.x_sum / self.count, self.y_sum / self.count, self.z_sum / self.count)

    def get_average_confidence(self):
        return self.confidence_sum / self.count

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
        self.fps = 5
        self.confidence_threshold = 0.75
        self.merge_threshold = 0.6  # Distance threshold for merging probes (in meters)
        self.tracked_probes = []  # List of tracked Probe objects

        # Subscribers
        self.rgb_sub = Subscriber(self, CompressedImage, '/thyra/out/color_image/compressed', qos_profile=qos)
        self.depth_sub = Subscriber(self, CompressedImage, '/thyra/out/depth_image/compressed', qos_profile=qos)
        self.pose_sub = Subscriber(self, PoseStamped, '/thyra/out/pose/synced_with_RGBD', qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.pose_sub], queue_size=2, slop=0.05)
        self.ts.registerCallback(self.image_handler)

        # Publishers
        self.image_publisher = self.create_publisher(Image, '/probe_detector/segmented_image', 10)
        self.probe_publisher = self.create_publisher(ProbeLocations, '/probe_detector/probe_locations', 10)
        self.global_probe_publisher = self.create_publisher(ProbeGlobalLocations, '/probe_detector/global_probe_locations', 10)

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

    def transform_to_global(self, lx, ly, lz, matched_pose):
        # Extract pose components
        pos = matched_pose.pose.position
        ori = matched_pose.pose.orientation

        # Rotation matrix from quaternion (standard convention)
        R = self.quaternion_to_matrix([ori.x, ori.y, ori.z, ori.w])

        # Local point in drone frame
        p_local = np.array([lx, ly, lz])

        # Transform to global: p_global = R @ p_local + pos_vector
        pos_vector = np.array([pos.x, pos.y, pos.z])
        p_global = np.dot(R, p_local) + pos_vector

        return p_global[0], p_global[1], p_global[2]

    def quaternion_to_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
        ])

    def image_handler(self, rgb_msg, depth_msg, pose_msg):
        try:
            rgb_time = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            depth_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9
            pose_time = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

            if self.previous_time + (1 / self.fps) > rgb_time:
                return
            self.previous_time = rgb_time

            self.get_logger().info(f"Synchronized RGB Time: {rgb_time}, Depth Time: {depth_time}, Pose Time: {pose_time}")

            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            np_arr_depth = np.frombuffer(depth_msg.data, np.uint8)
            depth_image = cv2.imdecode(np_arr_depth, cv2.IMREAD_UNCHANGED)

            probe_boxes, probe_masks, probe_confidences = self.infer_yolo(rgb_image)

            if probe_masks:
                probe_locations = self.compute_probe_locations(probe_masks, depth_image, rgb_image, probe_confidences)

                if probe_locations:
                    # Publish local probe locations
                    sorted_local_locations = sorted(probe_locations, key=lambda x: x["confidence"], reverse=True)
                    local_confidences = [float(np.float32(loc["confidence"])) for loc in sorted_local_locations]
                    local_probe_list = [float(np.float32(coord)) for loc in sorted_local_locations for coord in (loc["x"], loc["y"], loc["z"])]
                    local_centroid_x = [float(np.float32(loc["centroid_x"])) for loc in sorted_local_locations]
                    local_centroid_y = [float(np.float32(loc["centroid_y"])) for loc in sorted_local_locations]

                    local_msg = ProbeLocations()
                    local_msg.header = rgb_msg.header
                    local_msg.classification_confidence = local_confidences
                    local_msg.num_probes = len(sorted_local_locations)
                    local_msg.probes = local_probe_list
                    local_msg.centroid_x = local_centroid_x
                    local_msg.centroid_y = local_centroid_y

                    self.probe_publisher.publish(local_msg)
                    self.get_logger().info(f"Published {local_msg.num_probes} local probe locations")

                    # Process and publish global probe locations
                    self.process_probes(probe_locations, pose_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to process image: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in image_handler: {e}")

    def process_probes(self, probe_locations, matched_pose):
        for loc in probe_locations:
            lx, ly, lz = loc["x"], loc["y"], loc["z"]
            confidence = loc["confidence"]
            distance = np.sqrt(lx * lx + ly * ly + lz * lz)

            if distance > 2.5:
                self.get_logger().warn(f"Probe too far away: {distance}")
                continue

            # Transform local (drone frame) to global
            gx, gy, gz = self.transform_to_global(lx, ly, lz, matched_pose)
            gz = 0.0  # Set z to 0 for 2D merging/visualization consistency

            # Merge or add to tracked probes
            matched_existing = False

            for tracked in self.tracked_probes:
                if tracked.distance_to(gx, gy, gz) < self.merge_threshold:
                    tracked.update(gx, gy, gz, confidence)
                    matched_existing = True
                    break

            if not matched_existing:
                self.tracked_probes.append(Probe(gx, gy, gz, confidence))

        # Collect and publish global probes with at least 2 observations
        valid_probes = [p for p in self.tracked_probes if p.count >= 2]
        if valid_probes:
            sorted_probes = sorted(valid_probes, key=lambda p: p.get_average_confidence(), reverse=True)

            global_x = [float(p.get_average_position()[0]) for p in sorted_probes]
            global_y = [float(p.get_average_position()[1]) for p in sorted_probes]
            global_z = [float(p.get_average_position()[2]) for p in sorted_probes]
            global_confidences = [float(p.get_average_confidence()) for p in sorted_probes]
            global_contributions = [p.count for p in sorted_probes]
            probe_count = len(sorted_probes)

            global_msg = ProbeGlobalLocations()
            global_msg.stamp = self.get_clock().now().to_msg()
            global_msg.x = global_x
            global_msg.y = global_y
            global_msg.z = global_z
            global_msg.confidence = global_confidences
            global_msg.contribution = global_contributions
            global_msg.probe_count = probe_count

            self.global_probe_publisher.publish(global_msg)
            self.get_logger().info(f"Published {global_msg.probe_count} global probe locations")

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

                # Transform from camera to local drone frame
                transformed_position = self.transform_point(position[0], position[1], position[2])

                probe_location = {
                    "centroid_x": centroid_x,
                    "centroid_y": centroid_y,
                    "confidence": probe_confidences[i],
                    "x": transformed_position[0],
                    "y": transformed_position[1],
                    "z": transformed_position[2]
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
            self.get_logger().info(f"Probe {i}: Centroid ({centroid_x}, {centroid_y}), Local Position ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), Confidence {confidence:.2f}")

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

    def transform_point(self, x, y, z):
        point = np.array([x, y, z, 1.0])
        transformed = CAMERA_TO_DRONE_TRANSFORM @ point
        return transformed[:3]

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