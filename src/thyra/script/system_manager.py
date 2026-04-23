#!/usr/bin/env python3

import shutil
import subprocess

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces.action import SystemControl


class SystemManagerNode(Node):
    def __init__(self):
        super().__init__('system_manager')
        self._sudo_path = shutil.which('sudo') or '/usr/bin/sudo'
        self._systemctl_path = shutil.which('systemctl') or '/bin/systemctl'
        self._reboot_path = shutil.which('reboot') or '/usr/sbin/reboot'
        self._action_server = ActionServer(
            self,
            SystemControl,
            'system_control',
            execute_callback=self.execute_callback,
        )
        self.get_logger().info('System manager action server ready.')

    def execute_callback(self, goal_handle):
        command = goal_handle.request.command_type.strip().upper()
        result = SystemControl.Result()
        feedback = SystemControl.Feedback()

        feedback.status = f'Processing {command}'
        goal_handle.publish_feedback(feedback)

        if command == 'RESTART_NODES':
            subprocess.Popen(
                [self._sudo_path, self._systemctl_path, 'restart', 'thyra'],
                start_new_session=True,
            )
            result.success = True
            result.message = 'Requested thyra service restart.'
            goal_handle.succeed()
            return result

        if command == 'REBOOT_PI':
            subprocess.Popen(
                [self._sudo_path, self._reboot_path],
                start_new_session=True,
            )
            result.success = True
            result.message = 'Requested Raspberry Pi reboot.'
            goal_handle.succeed()
            return result

        result.success = False
        result.message = f'Unsupported command_type: {command}'
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SystemManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()