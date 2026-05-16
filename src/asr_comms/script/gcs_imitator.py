#!/usr/bin/env python3
"""
GCS-side traffic imitator.

Publishes RTCM corrections and a GCS heartbeat at realistic rates so that
comms_gcs has data to forward over the radio, then prints live link_stats
so you can observe the radio's actual response.

RTCM messages simulated (MSM4, as configured in rtcm_reader.py):
  1005  base station position   ~25 bytes   1 Hz
  1074  GPS MSM4               ~150 bytes   1 Hz
  1084  GLONASS MSM4           ~120 bytes   1 Hz
  1094  Galileo MSM4           ~110 bytes   1 Hz
  1230  GLONASS code biases     ~76 bytes   1 Hz
  GCS heartbeat (comms_gcs itself sends this, shown for completeness)

Run alongside comms_gcs (serial or UDP):
  python3 gcs_imitator.py
"""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8MultiArray
from asr_comms.msg import GcsHeartbeat, LinkStats

# Realistic RTCM3 frame sizes for MSM4 with ~8 satellites per constellation.
# Format: (message_id, approximate_total_bytes_including_preamble_and_crc)
_RTCM_FRAMES = [
    (1005,  25),   # base station antenna reference point
    (1074, 150),   # GPS MSM4
    (1084, 120),   # GLONASS MSM4
    (1094, 110),   # Galileo MSM4
    (1230,  76),   # GLONASS code-phase biases
]

# MAVLink2 GPS_RTCM_DATA wire overhead per fragment (header + len + flags + data + CRC)
_MAVLINK_RTCM_OVERHEAD = 12 + 2 + 2   # header(10) + msgid(3 encoded) + len(1) + flags(1) + CRC(2) ≈ 16
# GCS heartbeat sent by comms_gcs itself
_HEARTBEAT_WIRE = 12 + 9 + 2


class GcsImitator(Node):
    def __init__(self):
        super().__init__('gcs_imitator', namespace='asr/gcs')

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._rtcm_pub = self.create_publisher(UInt8MultiArray, '/rtcm', 10)
        self._hb_pub   = self.create_publisher(GcsHeartbeat,   'in/gcs_heartbeat', qos_be)

        self._link_sub = self.create_subscription(
            LinkStats, 'link_stats', self._on_link_stats, qos_be)

        self._bytes    = 0   # RTCM bytes enqueued this period
        self._report_t = time.monotonic()
        self._last_stats: LinkStats | None = None

        self.create_timer(1.0, self._rtcm_cb)
        self.create_timer(1.0, self._hb_cb)
        self.create_timer(5.0, self._report_cb)

        self.get_logger().info('GCS imitator started — publishing RTCM + heartbeat')
        self._print_header()

    def _rtcm_cb(self):
        for msg_id, size in _RTCM_FRAMES:
            msg = UInt8MultiArray()
            # Real bytes don't matter — comms_gcs forwards them raw.
            # Use a recognisable pattern so Wireshark / serial sniffers can spot them.
            msg.data = bytes([msg_id & 0xFF, (msg_id >> 8) & 0xFF] + [0xAA] * (size - 2))
            self._rtcm_pub.publish(msg)

            # GPS_RTCM_DATA fragments: each carries up to 180 bytes of RTCM data.
            # With MAVLink2 zero-trimming the payload is trimmed to the actual data length,
            # so wire size = header(10) + flags(1) + len(1) + actual_chunk + CRC(2).
            offset = 0
            while offset < size:
                chunk = min(size - offset, 180)
                self._bytes += 10 + 1 + 1 + chunk + 2  # MAVLink2 trimmed wire size
                offset += chunk

    def _hb_cb(self):
        hb = GcsHeartbeat()
        hb.timestamp   = self.get_clock().now().nanoseconds / 1e9
        hb.gcs_nominal = 1
        self._hb_pub.publish(hb)

    def _on_link_stats(self, msg: LinkStats):
        self._last_stats = msg

    def _report_cb(self):
        dt = time.monotonic() - self._report_t
        self._report_t = time.monotonic()

        # Add fixed comms_gcs heartbeat overhead
        hb_bytes = _HEARTBEAT_WIRE * 5   # 1 Hz over 5 s
        rtcm_kbps = self._bytes * 8 / dt / 1000.0
        total_kbps = (self._bytes + hb_bytes) * 8 / dt / 1000.0

        s = self._last_stats

        lines = ['\n=== GCS → UAV link snapshot ===']
        lines.append(f'  RTCM wire load : {rtcm_kbps:.2f} kbps')
        lines.append(f'  Total GCS→UAV  : {total_kbps:.2f} kbps  (incl. comms_gcs heartbeat)')
        lines.append(f'  SiK capacity   : ~4–5 kbps per direction')

        if s is not None:
            quality_map = {0: 'NO RADIO', 1: 'CRITICAL', 2: 'POOR', 3: 'FAIR', 4: 'GOOD', 5: 'EXCELLENT'}
            lines.append('')
            lines.append('--- GCS radio (RADIO_STATUS from local radio) ---')
            lines.append(f'  connected      : {s.connected}')
            lines.append(f'  tx_kbps (GCS)  : {s.tx_kbps:.2f} kbps  ← GCS→UAV measured at serial')
            lines.append(f'  rx_kbps (GCS)  : {s.rx_kbps:.2f} kbps  ← UAV→GCS measured at serial')
            lines.append(f'  peer_rx_kbps   : {s.peer_rx_kbps:.2f} kbps  ← UAV reports receiving this')
            if s.radio_ok:
                gcs_snr = int(s.rssi) - int(s.noise)
                lines.append(f'  txbuf          : {s.txbuf}%  ← 0% = GCS radio TX buffer full')
                lines.append(f'  rssi/noise/snr : {s.rssi}/{s.noise}/{gcs_snr}')
                lines.append(f'  remrssi/remnoise:{s.remrssi}/{s.remnoise}')
                lines.append(f'  rx errors/fixed: {s.rxerrors}/{s.fixed}')
                lines.append(f'  signal quality : {quality_map.get(s.signal_quality, "?")}')
            else:
                lines.append('  (no RADIO_STATUS — is the GCS radio connected?)')

            lines.append('')
            lines.append('--- UAV radio (forwarded from comms_uav) ---')
            if s.uav_radio_ok:
                uav_snr = int(s.uav_rssi) - int(s.uav_noise)
                lines.append(f'  uav_txbuf      : {s.uav_txbuf}%  ← 0% = UAV radio TX buffer full / saturated')
                lines.append(f'  uav rssi/noise/snr: {s.uav_rssi}/{s.uav_noise}/{uav_snr}')
                lines.append(f'  uav rx errors  : {s.uav_rxerrors}')
            else:
                lines.append('  (not yet received — is comms_uav running with a radio?)')
        else:
            lines.append('')
            lines.append('  (no link_stats yet — is comms_gcs running?)')

        self.get_logger().info('\n'.join(lines))
        self._bytes = 0

    def _print_header(self):
        self.get_logger().info(
            '\nRTCM frames simulated (MSM4):\n' +
            '\n'.join(f'  {mid}: {sz} bytes at 1 Hz' for mid, sz in _RTCM_FRAMES) +
            f'\n  Total raw RTCM : {sum(s for _, s in _RTCM_FRAMES)} bytes/s  '
            f'({sum(s for _, s in _RTCM_FRAMES) * 8 / 1000:.2f} kbps)'
        )


def main():
    rclpy.init()
    node = GcsImitator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
