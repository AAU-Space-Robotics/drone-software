#!/usr/bin/env python3
"""
camera_relay — compresses and republishes RealSense frames at a fixed rate.

Subscribes to raw color + depth + pose data, synchronises them, and publishes
CompressedImage (JPEG/PNG) + a PoseStamped at the configured fps.

Parameters (set statically in thyra_params.yaml / launch file):
  fps           float  Target publish rate. 0 = publish every synced frame.
  jpeg_quality  int    JPEG quality 0–100 for the colour image.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')

        self.declare_parameter('fps',          5.0)
        self.declare_parameter('jpeg_quality', 50)

        fps          = self.get_parameter('fps').value
        jpeg_quality = self.get_parameter('jpeg_quality').value

        self._bridge        = CvBridge()
        self._jpeg_quality  = int(jpeg_quality)
        self._latest        = {}   # keyed by field name, holds latest synced set
        self._origin_offset = (0.0, 0.0, 0.0)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Synchronised subscribers
        color_sub    = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw',       qos_profile=qos)
        depth_sub    = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw',  qos_profile=qos)
        position_sub = message_filters.Subscriber(self, VehicleLocalPosition, '/fmu/out/vehicle_local_position', qos_profile=qos)
        attitude_sub = message_filters.Subscriber(self, VehicleAttitude,      '/fmu/out/vehicle_attitude',       qos_profile=qos)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, position_sub, attitude_sub],
            queue_size=10, slop=0.1, allow_headerless=True,
        )
        self._sync.registerCallback(self._on_synced)

        self.create_subscription(PoseStamped, '/thyra/out/origin_offset', self._on_origin_offset, 10)

        # Publishers
        self._color_pub = self.create_publisher(CompressedImage, '/thyra/out/color_image/compressed', qos)
        self._depth_pub = self.create_publisher(CompressedImage, '/thyra/out/depth_image/compressed', qos)
        self._pose_pub  = self.create_publisher(PoseStamped,     '/thyra/out/pose/synced_with_RGBD',  qos)

        interval = 1.0 / fps if fps > 0.0 else 0.0
        if interval > 0.0:
            self.create_timer(interval, self._publish)
        else:
            # Publish directly in the sync callback when fps=0
            self._publish_on_sync = True

        self._publish_on_sync = (fps <= 0.0)
        self._last_stamp = None

        self.get_logger().info(
            f'Camera relay ready — {fps:.1f} Hz, JPEG quality {self._jpeg_quality}'
        )

    def _on_origin_offset(self, msg: PoseStamped):
        p = msg.pose.position
        self._origin_offset = (p.x, p.y, p.z)

    def _on_synced(self, color_msg, depth_msg, position_msg, attitude_msg):
        stamp = color_msg.header.stamp
        if self._last_stamp is not None:
            if (stamp.sec, stamp.nanosec) <= (self._last_stamp.sec, self._last_stamp.nanosec):
                return
        self._latest = {
            'color':    color_msg,
            'depth':    depth_msg,
            'position': position_msg,
            'attitude': attitude_msg,
        }
        if self._publish_on_sync:
            self._publish()

    def _publish(self):
        if not self._latest:
            return
        color_msg    = self._latest.get('color')
        depth_msg    = self._latest.get('depth')
        position_msg = self._latest.get('position')
        attitude_msg = self._latest.get('attitude')
        if not all([color_msg, depth_msg, position_msg, attitude_msg]):
            return

        stamp = color_msg.header.stamp
        if self._last_stamp is not None:
            if (stamp.sec, stamp.nanosec) <= (self._last_stamp.sec, self._last_stamp.nanosec):
                return
        self._last_stamp = stamp
        self._latest = {}

        try:
            # Colour → JPEG
            color_cv  = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            color_bgr = cv2.cvtColor(color_cv, cv2.COLOR_RGB2BGR)
            ok, color_buf = cv2.imencode('.jpg', color_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            if not ok:
                self.get_logger().error('JPEG encoding failed')
                return

            # Depth → PNG
            depth_cv = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            ok, depth_buf = cv2.imencode('.png', depth_cv)
            if not ok:
                self.get_logger().error('PNG encoding failed')
                return

            color_out        = CompressedImage()
            color_out.header = color_msg.header
            color_out.format = 'jpeg'
            color_out.data   = color_buf.tobytes()

            depth_out        = CompressedImage()
            depth_out.header = depth_msg.header
            depth_out.format = 'png'
            depth_out.data   = depth_buf.tobytes()

            ox, oy, oz = self._origin_offset
            pose_out = PoseStamped()
            pose_out.header = color_msg.header
            pose_out.pose.position.x    = float(position_msg.x - ox)
            pose_out.pose.position.y    = float(position_msg.y - oy)
            pose_out.pose.position.z    = float(position_msg.z - oz)
            pose_out.pose.orientation.x = float(attitude_msg.q[1])
            pose_out.pose.orientation.y = float(attitude_msg.q[2])
            pose_out.pose.orientation.z = float(attitude_msg.q[3])
            pose_out.pose.orientation.w = float(attitude_msg.q[0])

            self._color_pub.publish(color_out)
            self._depth_pub.publish(depth_out)
            self._pose_pub.publish(pose_out)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'Relay error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
