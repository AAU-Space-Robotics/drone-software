#!/usr/bin/env python3
"""
RTCM Reader node — runs on the GCS machine.

Configures a u-blox receiver as an RTK base station (survey-in), then
streams raw RTCM3 correction messages to /rtcm for comms_gcs to forward.

Parameters (set via asr_gcs_params.yaml):
  port                 Serial device  (default: /dev/ttyACM0)
  baudrate             Baud rate      (default: 115200)
  configure_receiver   Run u-blox config + survey-in on startup (default: true)
  survey_min_duration  Survey-in minimum duration in seconds    (default: 120)
  survey_accuracy_mm   Survey-in accuracy limit in 0.1 mm units (default: 500000)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from serial import Serial
from pyrtcm import RTCMReader
from pyubx2 import UBXMessage, UBXReader, SET


NMEA_IDS = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x08]

RTCM_MSGS = [
    (0xF5, 0x05),  # 1005 — base station position
    (0xF5, 0x4A),  # 1074 — GPS MSM4
    (0xF5, 0x54),  # 1084 — GLONASS MSM4
    (0xF5, 0x5E),  # 1094 — Galileo MSM4
    (0xF5, 0x7B),  # 1230 — GLONASS code-phase biases
]


class RtcmReaderNode(Node):
    def __init__(self):
        super().__init__('rtcm_reader')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('configure_receiver', True)
        self.declare_parameter('survey_min_duration', 120)
        self.declare_parameter('survey_accuracy_mm', 500000)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self._configure = self.get_parameter('configure_receiver').value
        self._survey_min_dur = self.get_parameter('survey_min_duration').value
        self._survey_acc_limit = self.get_parameter('survey_accuracy_mm').value

        self._rtcm_pub = self.create_publisher(UInt8MultiArray, 'rtcm', 10)

        self.get_logger().info(f'Opening serial port {port} @ {baudrate} baud')
        self._serial = Serial(port, baudrate, timeout=1)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Main thread: configure → survey-in → stream
    # ------------------------------------------------------------------

    def _run(self):
        try:
            if self._configure:
                self._configure_receiver()
                self._wait_for_survey()
            self._stream_rtcm()
        except Exception as e:
            self.get_logger().error(f'rtcm_reader thread crashed: {e}')

    # ------------------------------------------------------------------
    # Step 1 — Configure u-blox receiver
    # ------------------------------------------------------------------

    def _configure_receiver(self):
        self.get_logger().info('Configuring u-blox receiver...')

        # Disable NMEA output so only UBX/RTCM come through
        for mid in NMEA_IDS:
            msg = UBXMessage('CFG', 'CFG-MSG', SET,
                             msgClass=0xF0, msgID=mid, rateUSB=0)
            self._serial.write(msg.serialize())
            time.sleep(0.05)

        # Enable RTCM3 output messages
        for cls, mid in RTCM_MSGS:
            msg = UBXMessage('CFG', 'CFG-MSG', SET,
                             msgClass=cls, msgID=mid, rateUSB=1)
            self._serial.write(msg.serialize())
            time.sleep(0.05)

        # Enable NAV-SVIN output via CFG-MSG (works across all u-blox generations)
        msg = UBXMessage('CFG', 'CFG-MSG', SET, msgClass=0x01, msgID=0x3B, rateUSB=1)
        self._serial.write(msg.serialize())
        time.sleep(0.05)

        # Reset time mode first so the new acc_limit applies to a fresh survey-in
        reset = UBXMessage.config_set(layers=1, transaction=0, cfgData=[('CFG_TMODE_MODE', 0)])
        self._serial.write(reset.serialize())
        time.sleep(0.1)

        # Start survey-in via CFG-VALSET (modern u-blox API)
        # CFG_TMODE_SVIN_ACC_LIMIT is in 0.1 mm units; parameter is in mm
        acc_limit = int(round(self._survey_acc_limit * 10))
        cfg_data = [
            ('CFG_TMODE_MODE', 1),
            ('CFG_TMODE_SVIN_ACC_LIMIT', acc_limit),
            ('CFG_TMODE_SVIN_MIN_DUR', self._survey_min_dur),
        ]
        ubx = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg_data)
        self._serial.write(ubx.serialize())

        self.get_logger().info(
            f'Survey-in started — min {self._survey_min_dur}s, '
            f'acc limit {self._survey_acc_limit / 1000:.1f} m'
        )

    # ------------------------------------------------------------------
    # Step 2 — Block until survey-in reports valid
    # ------------------------------------------------------------------

    def _wait_for_survey(self):
        self.get_logger().info('Waiting for survey-in to complete...')
        reader = UBXReader(self._serial)
        consecutive_errors = 0
        last_log = (None, None, None)  # (active, acc_rounded, dur_rounded)

        while rclpy.ok():
            try:
                _, msg = reader.read()
                consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 50 == 0:
                    self.get_logger().warn(
                        f'UBX read error (x{consecutive_errors}): {e}'
                    )
                continue

            if msg is None:
                continue

            if msg.identity == 'NAV-SVIN':
                acc_m = msg.meanAcc / 10000.0
                acc_limit_m = self._survey_acc_limit / 1000.0

                # Log only when something meaningful changes (active state, 0.1m acc step, 10s dur step)
                log_key = (msg.active, round(acc_m, 1), msg.dur // 10)
                if log_key != last_log:
                    self.get_logger().info(
                        f'Survey: active={msg.active}, valid={msg.valid}, '
                        f'acc={acc_m:.2f} m, dur={msg.dur} s'
                    )
                    last_log = log_key

                threshold_met = acc_m <= acc_limit_m and msg.dur >= self._survey_min_dur
                if msg.valid or threshold_met:
                    reason = 'receiver valid' if msg.valid else f'threshold met ({acc_m:.2f} m <= {acc_limit_m:.1f} m, {msg.dur} s)'
                    self.get_logger().info(f'Survey-in complete ({reason}) — streaming RTCM')
                    return

    # ------------------------------------------------------------------
    # Step 3 — Stream RTCM messages and publish
    # ------------------------------------------------------------------

    def _stream_rtcm(self):
        # quitonerror=0 makes RTCMReader silently skip non-RTCM frames (e.g. UBX)
        reader = RTCMReader(self._serial, quitonerror=0)
        consecutive_errors = 0

        for raw, _ in reader:
            if not rclpy.ok():
                break
            try:
                if raw:
                    out = UInt8MultiArray()
                    out.data = list(raw)
                    self._rtcm_pub.publish(out)
                    consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors == 1 or consecutive_errors % 50 == 0:
                    self.get_logger().error(
                        f'RTCM publish error (x{consecutive_errors}): {e} — corrections may have stopped'
                    )


def main(args=None):
    rclpy.init(args=args)
    node = RtcmReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
