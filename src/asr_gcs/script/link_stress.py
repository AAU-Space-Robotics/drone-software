#!/usr/bin/env python3
"""
Stress-tests the GCS→UAV radio link by spamming ManualControlInput messages.
Publishes on in/manual_input, which comms_gcs forwards as MAVLink MANUAL_CONTROL
frames over the serial radio.

Usage:
  python3 link_stress.py              # run at max rate
  python3 link_stress.py --hz 500     # cap at 500 Hz

Watch tx_kbps rise in the link_stats topic. The serial ceiling at 115200 baud
is ~11.25 kB/s; txbuf will drop from 100 when the radio starts buffering.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from asr_comms.msg import ManualControlInput, LinkStats


REPORT_INTERVAL = 2.0  # seconds between console prints


class LinkStress(Node):
    def __init__(self, hz: float):
        super().__init__("link_stress")
        self._pub = self.create_publisher(ManualControlInput, "in/manual_input", 10)
        self._stats_sub = self.create_subscription(
            LinkStats, "link_stats", self._on_stats, 10
        )

        self._seq = 0
        self._sent = 0
        self._last_report = time.monotonic()
        self._last_stats: LinkStats | None = None

        period = (1.0 / hz) if hz > 0 else 0.0
        if period > 0:
            self.create_timer(period, self._publish)
        else:
            # As fast as the executor allows — spin is called externally
            self.create_timer(0.0001, self._publish)

        self.get_logger().info(
            f"Stress test started — target {'%.0f Hz' % hz if hz > 0 else 'max rate'}"
        )

    def _publish(self):
        t = time.monotonic()
        msg = ManualControlInput()
        # Cycle through sinusoidal values so the payload isn't all zeros
        msg.roll         =  math.sin(t * 1.0)
        msg.pitch        =  math.sin(t * 1.3)
        msg.yaw_velocity =  math.sin(t * 0.7)
        msg.thrust       = (math.sin(t * 0.5) + 1.0) / 2.0
        msg.arm          = 0
        msg.estop        = 0
        msg.selfdestruct = 0
        self._pub.publish(msg)
        self._seq += 1
        self._sent += 1

        if t - self._last_report >= REPORT_INTERVAL:
            rate = self._sent / (t - self._last_report)
            self._sent = 0
            self._last_report = t

            s = self._last_stats
            if s:
                print(
                    f"  pub {rate:6.0f} Hz | "
                    f"serial_tx {s.tx_kbps:.2f} kB/s  peer_rx {s.peer_rx_kbps:.2f} kB/s  rx {s.rx_kbps:.2f} kB/s | "
                    f"radio_ok={s.radio_ok}  rssi={s.rssi}  noise={s.noise}  "
                    f"SNR={s.rssi - s.noise}  txbuf={s.txbuf}%  "
                    f"rxerr={s.rxerrors}  fixed={s.fixed}"
                )
            else:
                print(f"  pub {rate:6.0f} Hz | waiting for link_stats…")

    def _on_stats(self, msg: LinkStats):
        self._last_stats = msg


def main():
    parser = argparse.ArgumentParser(description="GCS→UAV link stress tester")
    parser.add_argument("--hz", type=float, default=0,
                        help="publish rate in Hz (0 = max rate, default: max)")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = LinkStress(args.hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
