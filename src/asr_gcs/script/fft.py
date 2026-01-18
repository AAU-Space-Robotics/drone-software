#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
import numpy as np
from scipy.fft import fft
import matplotlib.pyplot as plt
import time
import os

class FFTPositionNode(Node):
    def __init__(self):
        super().__init__('fft_position_node')
        
        # Parameters
        self.declare_parameter('capture_duration', 30.0)  # Duration to capture data in seconds
        self.capture_duration = self.get_parameter('capture_duration').value
        self.sample_rate = 100  # Hz, adjust based on expected message frequency
        
        # Data storage
        self.positions = []
        self.velocities = []  # Still collected for potential future use
        self.accelerations = []  # Still collected for potential future use
        self.timestamps = []
        self.start_time = None
        
        # QoS profile for subscription
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Subscriber
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos
        )
        
        self.get_logger().info('FFT Position Node initialized')

    def local_position_callback(self, msg):
        current_time = time.time()
        
        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info('Starting data capture')
        
        # Store data
        self.positions.append([msg.x, msg.y, msg.z])
        self.velocities.append([msg.vx, msg.vy, msg.vz])
        self.accelerations.append([msg.ax, msg.ay, msg.az])
        self.timestamps.append(current_time)
        
        # Log to confirm message reception
        self.get_logger().debug(f'Received message: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
        
        # Check if capture duration is reached
        if current_time - self.start_time >= self.capture_duration:
            self.get_logger().info(f'Capture duration reached, performing FFT on {len(self.positions)} samples')
            self.perform_fft()
            # Note: Shutdown happens in perform_fft after processing

    def perform_fft(self):
        if len(self.positions) < 2:
            self.get_logger().warn('Not enough data points for FFT and plotting')
            self.shutdown_node()
            return
        
        # Convert position data to numpy array
        positions = np.array(self.positions)
        
        # Create a single figure for position FFT
        plt.figure(figsize=(10, 6))
        self.get_logger().info('FFT results for Position:')
        
        for i, axis_label in enumerate(['X', 'Y', 'Z']):
            # Compute FFT for position
            fft_result = fft(positions[:, i])
            freqs = np.fft.fftfreq(len(fft_result), d=1.0/self.sample_rate)
            
            # Get positive frequencies only
            positive_mask = freqs > 0
            freqs = freqs[positive_mask]
            fft_magnitudes = np.abs(fft_result)[positive_mask]
            
            if len(fft_magnitudes) == 0:
                self.get_logger().warn(f'No positive frequencies for Position {axis_label}-axis')
                continue
            
            # Find dominant frequency
            dominant_freq = freqs[np.argmax(fft_magnitudes)]
            self.get_logger().info(
                f'Position {axis_label}-axis: '
                f'Dominant frequency = {dominant_freq:.2f} Hz, '
                f'Magnitude = {np.max(fft_magnitudes):.2f}'
            )
            
            # Plot FFT
            plt.plot(freqs, fft_magnitudes, label=f'{axis_label}-axis')
        
        # Configure and save plot
        plt.title(f'FFT of Position (Capture Duration: {self.capture_duration}s)')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Magnitude')
        plt.legend()
        plt.grid(True)
        plot_filename = f'fft_position_{int(time.time())}.png'
        plt.savefig(plot_filename)
        plt.close()
        self.get_logger().info(f'Saved FFT plot to {os.path.abspath(plot_filename)}')
        
        # Shut down the node after completing the FFT and plot
        self.shutdown_node()

    def shutdown_node(self):
        self.get_logger().info('Shutting down FFT Position Node')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FFTPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        if not rclpy.ok():
            node.get_logger().info('Node already shut down')
        else:
            node.shutdown_node()

if __name__ == '__main__':
    main()