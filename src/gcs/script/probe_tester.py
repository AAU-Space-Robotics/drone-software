#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from interfaces.msg import ProbeLocations  # Replace with your actual package name

class DummyProbePublisher(Node):
    def __init__(self):
        super().__init__('dummy_probe_publisher')

        self.publisher_ = self.create_publisher(ProbeLocations, '/thyra/out/probe_locations_global', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.counter = 0
        self.num_probes = 1

        self.get_logger().info('Modified Dummy ProbeLocations Publisher Started')

    def timer_callback(self):
        msg = ProbeLocations()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Occasionally increase num_probes
        if self.counter % 50 == 0 and self.counter > 0:
            self.num_probes += 1
        msg.num_probes = self.num_probes

        # Single changing confidence value between 0 and 1
        confidence_value = (self.counter % 100) / 100.0
        msg.classification_confidence = [confidence_value]

        # Dummy probe coordinates for num_probes
        msg.probes = []
        msg.centroid_x = []
        msg.centroid_y = []

        for i in range(self.num_probes):
            x = i + 0.1 * self.counter
            y = i * 2.0 + 0.2 * self.counter
            z = 1.0 + 0.05 * self.counter

            msg.probes.extend([x, y, z])
            msg.centroid_x.append(x + 0.1)
            msg.centroid_y.append(y + 0.1)

        self.publisher_.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = DummyProbePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()