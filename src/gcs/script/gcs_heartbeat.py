#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import GcsHeartbeat
import time

class GcsHeartbeatPublisher(Node):
    def __init__(self):
        super().__init__('gcs_heartbeat_publisher')
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.publisher_ = self.create_publisher(GcsHeartbeat, 'thyra/in/gcs_heartbeat', qos)
        self.timer = self.create_timer(0.5, self.publish_heartbeat)  # 2 Hz = 0.5 seconds

    def publish_heartbeat(self):
        msg = GcsHeartbeat()
        msg.timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        msg.gcs_nominal = 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published GCS heartbeat: timestamp={msg.timestamp:.2f}, gcs_nominal={msg.gcs_nominal}')

def main(args=None):
    rclpy.init(args=args)
    node = GcsHeartbeatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()