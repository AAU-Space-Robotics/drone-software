#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.msg import ProbeGlobalLocations  # Ensure this matches your package
from builtin_interfaces.msg import Time

class DummyProbeGlobalPublisher(Node):
    def __init__(self):
        super().__init__('dummy_probe_global_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(
            ProbeGlobalLocations,
            '/thyra/out/probe_locations_global',
            10
        )

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.probe_count = 1

        self.get_logger().info('Dummy ProbeGlobalLocations Publisher Started')

    def timer_callback(self):
        msg = ProbeGlobalLocations()

        # Update probe count every 50 iterations
        if self.counter % 50 == 0 and self.counter > 0:
            self.probe_count += 1
        msg.probe_count = self.probe_count

        # Generate dummy positions and confidence
        msg.x = []
        msg.y = []
        msg.z = []
        msg.confidence = []
        msg.contribution = []

        for i in range(self.probe_count):
            x = i + 0.1 * self.counter
            y = i * 2.0 + 0.2 * self.counter
            z = 1.0 + 0.05 * self.counter
            confidence_value = (self.counter % 100) / 100.0
            contribution_value = (self.counter % 10) + 1  # arbitrary example

            msg.x.append(x)
            msg.y.append(y)
            msg.z.append(z)
            msg.confidence.append(confidence_value)
            msg.contribution.append(contribution_value)

        # Timestamp
        msg.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = DummyProbeGlobalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
