#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from asr_comms.msg import TelemetryStatus
import serial


class DroneLEDNode(Node):
    def __init__(self):
        super().__init__('drone_led_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=1)
        self._led_mode = 0

        self.create_subscription(
            TelemetryStatus,
            'thyra/out/telemetry/status',
            self._state_callback,
            10,
        )

    def _state_callback(self, msg):
        new_mode = msg.led_mode
        if new_mode == self._led_mode:
            return
        try:
            self.ser.write(f'{new_mode}\n'.encode())
            self._led_mode = new_mode
        except Exception as e:
            self.get_logger().error(f'Failed to write to serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DroneLEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
