#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from asr_comms.msg import TelemetryPosition, TelemetryAttitude, TelemetryBattery, TelemetryGPS, TelemetryStatus
import time

class DummyDronePublisher(Node):
    def __init__(self):
        super().__init__('dummy_drone_publisher')

        self.position_pub = self.create_publisher(TelemetryPosition, '/drone/out/telemetry/position', 10)
        self.attitude_pub = self.create_publisher(TelemetryAttitude, '/drone/out/telemetry/attitude', 10)
        self.battery_pub  = self.create_publisher(TelemetryBattery,  '/drone/out/telemetry/battery',  10)
        self.gps_pub      = self.create_publisher(TelemetryGPS,      '/drone/out/telemetry/gps',      10)
        self.status_pub   = self.create_publisher(TelemetryStatus,   '/drone/out/telemetry/status',   10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.counter = 0.0
        self.get_logger().info('Dummy Telemetry Publisher Started')

    def timer_callback(self):
        self.counter += 0.1
        now = self.get_clock().now().nanoseconds / 1e9

        pos = TelemetryPosition()
        pos.timestamp = now
        pos.position = [self.counter, self.counter + 1.0, self.counter + 2.0]
        pos.velocity = [0.5 * self.counter, 0.3 * self.counter, 0.1 * self.counter]
        self.position_pub.publish(pos)

        att = TelemetryAttitude()
        att.timestamp = now
        att.orientation = [0.1 * self.counter, 0.2 * self.counter, 0.3 * self.counter]
        self.attitude_pub.publish(att)

        bat = TelemetryBattery()
        bat.timestamp = now
        bat.voltage = 12.0 - 0.01 * self.counter
        bat.percentage = max(0.0, 100.0 - self.counter)
        self.battery_pub.publish(bat)

        gps = TelemetryGPS()
        gps.timestamp = now
        gps.latitude = 57.0 + self.counter * 0.0001
        gps.longitude = 10.0 + self.counter * 0.0001
        gps.satellites_used = 12
        self.gps_pub.publish(gps)

        status = TelemetryStatus()
        status.timestamp = now
        status.arming_state = 2
        status.flight_mode = 1
        self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = DummyDronePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
