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
        self.position = np.empty((3, 0), dtype=np.float32)  # 3xN matrix for position measurements
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
        if len(msg.position) >= 3:
            new_measurement = np.array([[msg.position[0]],
                                         [msg.position[1]],
                                         [msg.position[2]]],
                                        dtype=np.float32)
            self.position = np.hstack((self.position, new_measurement))
        
            # If your message has a timestamp field:
            measurement_time = getattr(msg, "timestamp", time.time())
            self.timestamps = np.hstack((self.timestamps, np.array([[measurement_time]], dtype=np.float32)))
        if len(msg.velocity) >= 3:
            new_measurement = np.array([[msg.velocity[0]],
                                         [msg.velocity[1]],
                                         [msg.velocity[2]]],
                                        dtype=np.float32)
            self.drone_state_velocity = np.hstack((self.velocity, new_measurement))


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
    def get_position_history(self):
        """
        Get the full array of position measurements. Returns a 3xN numpy array.
        """
        return self.position
    def get_latest_position(self):
        """
        Get the latest position measurement. Returns a 1D numpy array of size 3.
        """
        if self.position.shape[1] == 0:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)
        return self.position[:, -1]
    
    def get_drone_state_velocity_history(self):
        return self.drone_state_velocity
    def get_latest_drone_state_velocity(self):
        if self.drone_state_velocity.shape[1] == 0:
            return np.array([0.0, 0.0, 0.0], dtype=np.float32)
        return self.drone_state_velocity[:, -1]

    def get_sample_size(self):
        return self.velocity.shape[1], self.position.shape[1], self.drone_state_velocity.shape[1]

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
    signal_labels = ['x', 'y', 'z']
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
        plt.ylabel(f'{metric_labels[metric_idx]} Position (m)')
        plt.legend()
    plt.xlabel('Samples')
    plt.tight_layout()
    plt.show()

def plot_signal_axis(velocity_metrics, axis_idx):
    """
    Plots four subplots for metrics (Mean, Std Dev, Max, Min) of a specified axis (0: vx, 1: vy, 2: vz).
    Args:
        velocity_metrics or position_metrics: numpy array of shape (num_signals, num_windows, 4)
        axis_idx: int, index of axis to plot (0, 1, or 2)
    """
    time_axis = np.arange(velocity_metrics.shape[1])
    metric_labels = ['Mean', 'Std Dev', 'Max', 'Min']
    axis_labels = ['x', 'y', 'z']
    plt.figure(figsize=(12, 8))
    for metric_idx in range(4):
        plt.subplot(4, 1, metric_idx + 1)
        plt.plot(
            time_axis,
            velocity_metrics[axis_idx, :, metric_idx],
            color='blue'
        )
        plt.ylabel(f'{metric_labels[metric_idx]} velocity (m/s)')
        plt.title(f'{metric_labels[metric_idx]} for {axis_labels[axis_idx]} axis')
    plt.xlabel('Samples')
    plt.tight_layout()
    plt.show()

def save_to_csv(velocity_samples,velocity_metrics, velocity_idx, working_axis):
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
        f'velocity{working_axis}': combined_data[:, 0],
        'mean_velocity': combined_data[:, 1],
        'std_velocity': combined_data[:, 2],
        'max_velocity': combined_data[:, 3],
        'min_velocity': combined_data[:, 4],
    })
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = os.path.join(script_dir, "sample.csv")
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
    axis = 0 #to decide if x (0) y (1) or z (2) axis looking at
    
    while flight_time == 0:
        time.sleep(0.1)
        flight_time = DroneSamplerNode.get_flight_time(node)
    print("flight started")
     
    start_index_v, start_index_p, start_index_ds_v = DroneSamplerNode.get_sample_size(node)
    print(f"starting at sample size: {start_index_p}")
    while DroneSamplerNode.get_sample_size(node)[0] < start_index_v + sample_size:
        time.sleep(0.1)
        current_samples = DroneSamplerNode.get_sample_size(node)
        print(f"Samples collected: {DroneSamplerNode.get_sample_size(node)[0] - start_index_v}", end='\r')
        


    position_samples = DroneSamplerNode.get_position_history(node)
    drone_state_velocity_samples = DroneSamplerNode.get_drone_state_velocity_history(node)
    #sliced_drone_state_velocity_samples = DroneSamplerNode.get_drone_state_velocity_history(node)[:,start_index_ds_v : start_index_ds_v + sample_size]
    #sliced_position_samples = DroneSamplerNode.get_position_history(node)[:,start_index_p : start_index_p + sample_size]
    
    velocity_metrics = calculate_moving_average_metrics(drone_state_velocity_samples[:, :sample_size - 1], window_size)
    plot_signal_axis(velocity_metrics, axis_idx=axis)  # Plot for some axis
    save_to_csv(drone_state_velocity_samples, velocity_metrics, axis, 'v') # Save velocity samples to CSV
    #position_metrics = calculate_moving_average_metrics(sliced_position_samples[:, :sample_size - 1], window_size)
    #drone_state_velocity_metrics = calculate_moving_average_metrics(sliced_drone_state_velocity_samples[:, :sample_size - 1], window_size)
    #plot_signal_axis(position_metrics, axis_idx=axis)  # Plot for some axis
    #save_to_csv(sliced_position_samples, position_metrics, axis, 'y') # Save position samples to CSV

if __name__ == '__main__':
    main()