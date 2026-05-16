#!/usr/bin/env python3
"""
UAV-side traffic imitator.

Publishes telemetry topics at the same rates as the real autopilot so that
comms_uav has data to forward over the radio without needing PX4 or the full
autopilot stack.

Rates (driven by a single 10 Hz tick, matching main.cpp drone_state_timer):
  position  — 10 Hz
  attitude  — 10 Hz
  gps       —  2 Hz  (every 5th tick)
  battery   —  1 Hz  (every 10th tick)
  status    —  1 Hz  (every 10th tick)

Run alongside comms_uav (serial or UDP):
  python3 uav_imitator.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from asr_comms.msg import (
    TelemetryPosition,
    TelemetryAttitude,
    TelemetryGPS,
    TelemetryBattery,
    TelemetryStatus,
)

# Approximate MAVLink2 wire bytes per message type (12-byte header + trimmed payload + 2-byte CRC).
# Status uses V2_EXTENSION trimmed to actual StatusPod size (43 bytes) thanks to MAVLink2 zero-trim.
_WIRE_BYTES = {
    'position': 12 + 28 + 2,   # LOCAL_POSITION_NED
    'attitude': 12 + 28 + 2,   # ATTITUDE
    'gps':      12 + 30 + 2,   # GPS_RAW_INT
    'battery':  12 + 36 + 2,   # BATTERY_STATUS
    'status':   12 + 43 + 2,   # V2_EXTENSION (StatusPod, MAVLink2 trimmed)
    # sent by comms_uav itself, not this node — shown for completeness
    'heartbeat':12 +  9 + 2,
    'rx_kbps':  12 + 18 + 2,
}


class UavImitator(Node):
    def __init__(self):
        super().__init__('uav_imitator', namespace='asr/thyra')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._pos_pub = self.create_publisher(TelemetryPosition, 'out/telemetry/position', qos)
        self._att_pub = self.create_publisher(TelemetryAttitude, 'out/telemetry/attitude', qos)
        self._gps_pub = self.create_publisher(TelemetryGPS,      'out/telemetry/gps',      qos)
        self._bat_pub = self.create_publisher(TelemetryBattery,  'out/telemetry/battery',  qos)
        self._sta_pub = self.create_publisher(TelemetryStatus,   'out/telemetry/status',   qos)

        self._tick     = 0
        self._t0       = time.monotonic()
        self._bytes    = {k: 0 for k in _WIRE_BYTES}
        self._report_t = time.monotonic()

        self.create_timer(0.1, self._tick_cb)
        self.create_timer(5.0, self._report_cb)

        self.get_logger().info('UAV imitator started — publishing telemetry at autopilot rates')

    def _tick_cb(self):
        tick = self._tick
        self._tick += 1
        t = time.monotonic() - self._t0

        # 10 Hz — position and attitude
        pos = TelemetryPosition()
        pos.timestamp       = t
        pos.position        = [math.sin(t * 0.1), math.cos(t * 0.1), -1.5]
        pos.velocity        = [0.1, 0.0, 0.0]
        pos.target_position = [0.0, 0.0, -1.5]
        self._pos_pub.publish(pos)
        self._bytes['position'] += _WIRE_BYTES['position']

        att = TelemetryAttitude()
        att.timestamp    = t
        att.orientation  = [0.02, 0.01, math.sin(t * 0.05)]
        att.acceleration = [0.0, 0.0, 0.0]
        self._att_pub.publish(att)
        self._bytes['attitude'] += _WIRE_BYTES['attitude']

        # 2 Hz — GPS
        if tick % 5 == 0:
            gps = TelemetryGPS()
            gps.timestamp       = t
            gps.latitude        = 55.6761 + math.sin(t * 0.01) * 0.0001
            gps.longitude       = 12.5683 + math.cos(t * 0.01) * 0.0001
            gps.satellites_used = 14
            self._gps_pub.publish(gps)
            self._bytes['gps'] += _WIRE_BYTES['gps']

        # 1 Hz — battery and status
        if tick % 10 == 0:
            bat = TelemetryBattery()
            bat.timestamp       = t
            bat.voltage         = max(14.0, 16.8 - t * 0.001)
            bat.current         = 8.5
            bat.percentage      = max(0.0, 1.0 - t * 0.0001)
            bat.discharged_mah  = t * 2.36
            bat.average_current = 8.5
            self._bat_pub.publish(bat)
            self._bytes['battery'] += _WIRE_BYTES['battery']

            sta = TelemetryStatus()
            sta.timestamp       = t
            sta.arming_state    = 2     # ARMED
            sta.trajectory_mode = 1
            sta.estop           = 0
            sta.flight_mode     = 3
            sta.led_mode        = 0
            sta.flight_time     = t
            sta.probes_found    = 0
            sta.actuator_speeds = [0.45, 0.45, 0.45, 0.45]
            self._sta_pub.publish(sta)
            self._bytes['status'] += _WIRE_BYTES['status']

    def _report_cb(self):
        dt = time.monotonic() - self._report_t
        self._report_t = time.monotonic()

        # Add fixed overhead from comms_uav itself (not published by this node)
        self._bytes['heartbeat'] += int(_WIRE_BYTES['heartbeat'] * 5)  # 1 Hz over 5 s
        self._bytes['rx_kbps']   += int(_WIRE_BYTES['rx_kbps']   * 5)

        total = sum(self._bytes.values())
        kbps  = total * 8 / dt / 1000.0

        lines = ['\n--- UAV → GCS estimated MAVLink2 wire load ---']
        for name, b in self._bytes.items():
            suffix = ' (comms_uav)' if name in ('heartbeat', 'rx_kbps') else ''
            lines.append(f'  {name:<12} {b/dt:5.0f} B/s  {b*8/dt/1000:.2f} kbps{suffix}')
        lines.append(f'  {"TOTAL":<12} {total/dt:5.0f} B/s  {kbps:.2f} kbps')
        lines.append(f'  (SiK usable UAV→GCS capacity: ~4–5 kbps)')
        self.get_logger().info('\n'.join(lines))

        self._bytes = {k: 0 for k in _WIRE_BYTES}


def main():
    rclpy.init()
    node = UavImitator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
