#!/usr/bin/env python3

import shutil
import subprocess

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from asr_comms.action import SystemControl


class SystemManagerNode(Node):
    def __init__(self):
        super().__init__('system_manager')
        self._sudo_path = shutil.which('sudo') or '/usr/bin/sudo'
        self._systemctl_path = shutil.which('systemctl') or '/bin/systemctl'
        self._reboot_path = shutil.which('reboot') or '/usr/sbin/reboot'
        self._ros2_path = shutil.which('ros2') or '/opt/ros/humble/bin/ros2'

        self.declare_parameter('system.autostart_flight_stack', True)
        self.declare_parameter('system.flight_stack_launch', 'uav/thyra_jetson')

        self._action_server = ActionServer(
            self,
            SystemControl,
            'system_control',
            execute_callback=self.execute_callback,
        )
        self.get_logger().info('System manager action server ready.')

        autostart = self.get_parameter('system.autostart_flight_stack').get_parameter_value().bool_value
        if autostart:
            launch_file = self.get_parameter('system.flight_stack_launch').get_parameter_value().string_value
            self.get_logger().info(f'autostart_flight_stack=true — launching {launch_file}.')
            self._start_flight_stack()
        else:
            self.get_logger().info('autostart_flight_stack=false — send START_FLIGHT_STACK action to start manually.')

    def _start_flight_stack(self):
        launch_file = self.get_parameter('system.flight_stack_launch').get_parameter_value().string_value
        subprocess.Popen(
            [self._ros2_path, 'launch', 'thyra', f'{launch_file}.launch.py'],
        )

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
            result.message = 'Requested system reboot.'
            goal_handle.succeed()
            return result

        if command == 'START_FLIGHT_STACK':
            self._start_flight_stack()
            result.success = True
            result.message = 'Flight stack launch requested.'
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
