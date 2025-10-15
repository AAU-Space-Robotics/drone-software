#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition #subscription for px4 local vehicle 
from interfaces.msg import DroneState
import time
import math
import numpy as np
from threading import Thread
import matplotlib.pyplot as plt
from dataclasses import dataclass
import pandas as pd
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import os
import warnings
warnings.filterwarnings(
    "ignore",
    message="Pandas requires version",
    module="pandas.core.arrays.masked"
)

class DroneSamplerNode(Node):
    def __init__(self):
        super().__init__('thyra_sampler')
        # ROS2 subscription setup
        qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # Local position is the global position of the drone, related to a home position
        self.velocity_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.velocity_callback,
            qos
        )
        self.state_subscription = self.create_subscription(
            DroneState,
            "thyra/out/drone_state",
            self.state_callback,
            10
        )

        # Declaration of class variables
        self.velocity = np.empty((3, 0), dtype=np.float32)  # 3xN matrix for velocity measurements
        self.timestamps = np.empty((1, 0), dtype=np.float32)  # 1xN matrix for timestamps
        self.goal_handle = None
        self.flight_time = 0.0

    def velocity_callback(self, msg):
        new_measurement = np.array([[msg.vx], [msg.vy], [msg.vz]], dtype=np.float32)
        measurement_time = msg.timestamp
        self.velocity = np.hstack((self.velocity, new_measurement))
        self.timestamps = np.hstack((self.timestamps, np.array([[measurement_time]], dtype=np.float32)))

    def state_callback(self, msg):
        self.flight_time = msg.flight_time
    # ---- GETTERS ----

    def get_velocity_history(self):
        """
        Get the full array of velocity measurements. Returns a 3xN numpy array.
        """
        return self.velocity

    def get_latest_velocity(self):
        """
        Get the latest velocity measurement. Returns a 1D numpy array of size 3.
        """
        if self.velocity.shape[1] == 0:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)
        return self.velocity[:, -1]
    
    def get_sample_size(self):
        return self.velocity.shape[1]
    
    def get_sample_time_difference_history(self):
        """
        Get the time differences between consecutive samples. Returns a 1D numpy array of size N (same as number of samples).
        """
        if self.timestamps.shape[1] == 0:
            return np.array([], dtype=np.float32)
        dt = np.diff(self.timestamps, axis=1).flatten()
        dt = np.insert(dt, 0, 0.0)
        return dt
    def get_flight_time(self):


        return self.flight_time

def start_ros(node):
        try:
            rclpy.spin(node)

        except KeyboardInterrupt:
            node.get_logger().info('Node interrupted by user')
        finally:

            node.shutdown_node()
            rclpy.shutdown()

def calculate_moving_average_metrics(samples, window_length):
    """
    Function takes an array of samples, and a window size. It returns metrics of a signal, like mean, std, max, min based over a moving window. 
    Args:
        samples: 2D numpy array of shape (num_signals, num_samples)
        window_length: length of the moving window
    Returns:
        sample_metrics: ND numpy array of shape (num_signals, num_samples - window_length + 1, 4) containing metrics [mean, std, max, min] for each signal eg. velocity axis
    """
    num_signals, sample_amount = samples.shape
    moving_average_length = sample_amount - window_length + 1
    sample_metrics = np.zeros((num_signals, moving_average_length, 4), dtype=np.float32)  # [mean, std, max, min]

    # Calculate moving window metrics for each signal
    for signal_idx in range(num_signals):
        for i in range(moving_average_length):
            i_end = i + window_length
            window = samples[signal_idx, i:i_end]
            sample_metrics[signal_idx, i, 0] = np.mean(window)
            sample_metrics[signal_idx, i, 1] = np.std(window)
            sample_metrics[signal_idx, i, 2] = np.max(window)
            sample_metrics[signal_idx, i, 3] = np.min(window)

    return sample_metrics
def plot_signals(velocity_metrics):
    # Plot all axes for each metric in subplots
    time_axis = np.arange(velocity_metrics.shape[1])
    signal_labels = ['vx', 'vy', 'vz']
    metric_labels = ['Mean', 'Std Dev', 'Max', 'Min']
    colors = ['blue', 'orange', 'green']

    plt.figure(figsize=(12, 8))
    for metric_idx in range(4):
        plt.subplot(4, 1, metric_idx + 1)
        for signal_idx in range(3):
            plt.plot(
                time_axis,
                velocity_metrics[signal_idx, :, metric_idx],
                label=f'{signal_labels[signal_idx]}',
                color=colors[signal_idx]
            )
        plt.ylabel(f'{metric_labels[metric_idx]} Velocity (m/s)')
        plt.legend()
    plt.xlabel('Samples')
    plt.tight_layout()
    plt.show()

def plot_signal_axis(velocity_metrics, axis_idx):
    """
    Plots four subplots for metrics (Mean, Std Dev, Max, Min) of a specified axis (0: vx, 1: vy, 2: vz).
    Args:
        velocity_metrics: numpy array of shape (num_signals, num_windows, 4)
        axis_idx: int, index of axis to plot (0, 1, or 2)
    """
    time_axis = np.arange(velocity_metrics.shape[1])
    metric_labels = ['Mean', 'Std Dev', 'Max', 'Min']
    axis_labels = ['vx', 'vy', 'vz']
    plt.figure(figsize=(12, 8))
    for metric_idx in range(4):
        plt.subplot(4, 1, metric_idx + 1)
        plt.plot(
            time_axis,
            velocity_metrics[axis_idx, :, metric_idx],
            color='blue'
        )
        plt.ylabel(f'{metric_labels[metric_idx]} Velocity (m/s)')
        plt.title(f'{metric_labels[metric_idx]} for {axis_labels[axis_idx]} axis')
    plt.xlabel('Samples')
    plt.tight_layout()
    plt.show()

def save_to_csv(velocity_samples,velocity_metrics, velocity_idx):
    """
    Save velocity samples to a CSV file.
    Args:
        velocity_samples: 2D numpy array of shape (3, N)
        filename: str, name of the output CSV file
    """
    min_len = min(velocity_samples.shape[1], velocity_metrics.shape[1])
    velocity_samples = velocity_samples[velocity_idx, :min_len]
    mean, std, max_v, min_v = velocity_metrics[velocity_idx, :min_len].T


    combined_data = np.column_stack((velocity_samples, mean, std, max_v, min_v))

    df = pd.DataFrame({
        'velocity': combined_data[:, 0],
        'mean_velocity': combined_data[:, 1],
        'std_velocity': combined_data[:, 2],
        'max_velocity': combined_data[:, 3],
        'min_velocity': combined_data[:, 4],
    })
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "velocity_sample.csv")
    df.to_csv(filename, index=False)
    #print(f"✅ Velocity samples saved to {filename}")
    print("Saving CSV to:", filename)
    print("Current working directory:", os.getcwd())

    
    
def main(arg=None):
    rclpy.init()
    node = DroneSamplerNode() 
    Thread(target=start_ros, args=(node,), daemon=True).start()
    start_time = time.time()

    # Define runtime parameters
    sample_size = 1000
    window_size = 50
    flight_time = DroneSamplerNode.get_flight_time(node)
    axis = 2 #to decide if x (0) y (1) or z (2) axis looking at
    
    while flight_time == 0:
        time.sleep(0.1)
        flight_time = DroneSamplerNode.get_flight_time(node)
    print("flight started")
     
    start_index = DroneSamplerNode.get_sample_size(node)
    print(f"starting at sample size: {start_index}")
    while DroneSamplerNode.get_sample_size(node) < start_index + sample_size:
        time.sleep(0.1)
        current_samples = DroneSamplerNode.get_sample_size(node)
        print(f"Samples collected: {DroneSamplerNode.get_sample_size(node) - start_index}", end='\r')
        


    velocity_samples = DroneSamplerNode.get_velocity_history(node)

    sliced_velocity_samples = velocity_samples[:,start_index : start_index + sample_size]
    print(sliced_velocity_samples)

    velocity_metrics = calculate_moving_average_metrics(sliced_velocity_samples[:, :sample_size - 1], window_size) 
    
    plot_signal_axis(velocity_metrics, axis_idx=2)  # Plot for vx axis
    save_to_csv(velocity_samples, velocity_metrics,2) # Save velocity samples to CSV

if __name__ == '__main__':
    main()