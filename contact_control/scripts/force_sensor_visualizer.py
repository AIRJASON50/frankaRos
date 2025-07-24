#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Force Sensor Data Visualizer Node
Subscribes to ROS topic /force_sensor/wrench and displays force sensor data in real-time.
This version displays all 6-axis data (Forces and Torques) in separate subplots.
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from geometry_msgs.msg import WrenchStamped
from collections import deque
import threading
import signal
import sys
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Float64
import time
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

class ForceSensorVisualizer:
    def __init__(self):
        # ROS initialization
        rospy.init_node('force_sensor_visualizer', anonymous=True)
        
        # Data buffers - store last N seconds of data as (timestamp, value) tuples
        self.time_window_seconds = 30 
        self.data_buffers = {
            'Fx': deque(), 'Fy': deque(), 'Fz': deque(),
            'Mx': deque(), 'My': deque(), 'Mz': deque()
        }
        self.value_labels = {}
        
        # Zero offset storage
        self.zero_offsets = {'Fx': 0, 'Fy': 0, 'Fz': 0, 'Mx': 0, 'My': 0, 'Mz': 0}
        self.is_zeroing = True
        self.zero_data = {key: [] for key in self.data_buffers.keys()}
        
        # Data lock
        self.data_lock = threading.Lock()
        
        # Current data
        self.current_data = {'Fx': 0, 'Fy': 0, 'Fz': 0, 'Mx': 0, 'My': 0, 'Mz': 0}
        
        # Data frequency tracking
        self.current_data_frequency = 0.0
        self.last_data_times = deque(maxlen=100)
        
        # Create GUI
        self.setup_gui()
        
        # Subscribe to force sensor topic
        self.force_sub = rospy.Subscriber('/force_sensor/wrench', WrenchStamped, self.force_callback)
        
        rospy.loginfo("Force Sensor Visualizer Node Started")
        rospy.loginfo("Force sensor zeroing in progress, please ensure no external forces...")
        
    def setup_gui(self):
        """Set up GUI elements"""
        self.root = tk.Tk()
        self.root.title("6-Axis Force Sensor Visualizer")
        self.root.geometry("1400x900")
        
        # Configure root grid
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Create main frame structure
        left_frame = ttk.Frame(self.root)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        right_frame = ttk.Frame(self.root)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_columnconfigure(0, weight=1)
        
        # === Left Panel: Status and Controls ===
        status_frame = ttk.LabelFrame(left_frame, text="Status Information", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create labels for all 6 channels and store them
        data_labels_frame = ttk.Frame(status_frame)
        data_labels_frame.pack(fill=tk.X)
        
        ttk.Label(data_labels_frame, text="Forces:", font=("Arial", 11, "bold")).grid(row=0, column=0, columnspan=2, sticky='w', pady=(0,5))
        self.value_labels['Fx'] = ttk.Label(data_labels_frame, text="Fx: 0.000 N", font=("Arial", 10))
        self.value_labels['Fx'].grid(row=1, column=0, sticky='w', padx=5)
        self.value_labels['Fy'] = ttk.Label(data_labels_frame, text="Fy: 0.000 N", font=("Arial", 10))
        self.value_labels['Fy'].grid(row=2, column=0, sticky='w', padx=5)
        self.value_labels['Fz'] = ttk.Label(data_labels_frame, text="Fz: 0.000 N", font=("Arial", 10))
        self.value_labels['Fz'].grid(row=3, column=0, sticky='w', padx=5)

        ttk.Label(data_labels_frame, text="Torques:", font=("Arial", 11, "bold")).grid(row=0, column=1, columnspan=2, sticky='w', padx=(20,0), pady=(0,5))
        self.value_labels['Mx'] = ttk.Label(data_labels_frame, text="Mx: 0.000 Nm", font=("Arial", 10))
        self.value_labels['Mx'].grid(row=1, column=1, sticky='w', padx=(20,0))
        self.value_labels['My'] = ttk.Label(data_labels_frame, text="My: 0.000 Nm", font=("Arial", 10))
        self.value_labels['My'].grid(row=2, column=1, sticky='w', padx=(20,0))
        self.value_labels['Mz'] = ttk.Label(data_labels_frame, text="Mz: 0.000 Nm", font=("Arial", 10))
        self.value_labels['Mz'].grid(row=3, column=1, sticky='w', padx=(20,0))

        separator = ttk.Separator(status_frame, orient='horizontal')
        separator.pack(fill=tk.X, pady=15)
        
        # Status & Frequency Labels
        self.status_label = ttk.Label(status_frame, text="Status: Initializing...", font=("Arial", 10, "italic"), foreground="orange")
        self.status_label.pack(anchor=tk.W, pady=(0,5))
        
        self.frequency_label = ttk.Label(status_frame, text="Update Rate: 0.0 Hz", font=("Arial", 9))
        self.frequency_label.pack(anchor=tk.W)
        
        # Controls frame
        controls_frame = ttk.LabelFrame(left_frame, text="Controls", padding=10)
        controls_frame.pack(fill=tk.X, pady=(20, 0))
        
        self.zero_button = ttk.Button(controls_frame, text="Reset Zero Point", command=self.start_zeroing)
        self.zero_button.pack(fill=tk.X, pady=5)
        
        self.clear_button = ttk.Button(controls_frame, text="Clear Data", command=self.clear_plot)
        self.clear_button.pack(fill=tk.X, pady=5)
        
        # === Right Panel: Plot ===
        plot_frame = ttk.Frame(right_frame)
        plot_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        
        self.fig, (self.ax_force, self.ax_torque) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        self.fig.patch.set_facecolor('white')
        
        self.ax_force.set_title('Forces', fontsize=12, fontweight='bold')
        self.ax_force.set_ylabel('Force (N)', fontsize=10)
        self.ax_force.grid(True, alpha=0.4)
        
        self.ax_torque.set_title('Torques', fontsize=12, fontweight='bold')
        self.ax_torque.set_xlabel('Time (s)', fontsize=10)
        self.ax_torque.set_ylabel('Torque (Nm)', fontsize=10)
        self.ax_torque.grid(True, alpha=0.4)
        
        self.fig.tight_layout(pad=3.0)
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
    def force_callback(self, msg):
        current_time = rospy.get_time()
        self.last_data_times.append(current_time)
        
        raw_data = {
            'Fx': msg.wrench.force.x, 'Fy': msg.wrench.force.y, 'Fz': msg.wrench.force.z,
            'Mx': msg.wrench.torque.x, 'My': msg.wrench.torque.y, 'Mz': msg.wrench.torque.z
        }
        
        with self.data_lock:
            if self.is_zeroing:
                for key, value in raw_data.items():
                    self.zero_data[key].append(value)
                
                if len(self.zero_data['Fx']) >= 100:
                    for key in self.zero_offsets.keys():
                        self.zero_offsets[key] = np.mean(self.zero_data[key])
                    rospy.loginfo(f"Zeroing complete. Offsets: {self.zero_offsets}")
                    self.is_zeroing = False
            else:
                for key, value in raw_data.items():
                    adjusted_value = value - self.zero_offsets[key]
                    self.current_data[key] = adjusted_value
                    self.data_buffers[key].append((current_time, adjusted_value))
                
                cutoff_time = current_time - self.time_window_seconds
                for key in self.data_buffers:
                    while self.data_buffers[key] and self.data_buffers[key][0][0] < cutoff_time:
                        self.data_buffers[key].popleft()

    def update_gui_and_plot(self):
        if rospy.is_shutdown():
            return

        with self.data_lock:
            # Update text labels
            for key, label in self.value_labels.items():
                unit = "N" if key.startswith('F') else "Nm"
                label.config(text=f"{key}: {self.current_data[key]:.3f} {unit}")

            # Update status
            if self.is_zeroing:
                progress = len(self.zero_data['Fx']) if 'Fx' in self.zero_data else 0
                self.status_label.config(text=f"Status: Zeroing... ({progress}/100)", foreground="orange")
            else:
                self.status_label.config(text="Status: Running", foreground="green")
            
            # Update frequency
            if len(self.last_data_times) > 1:
                time_span = self.last_data_times[-1] - self.last_data_times[0]
                if time_span > 0:
                    self.current_data_frequency = (len(self.last_data_times) -1) / time_span
            self.frequency_label.config(text=f"Update Rate: {self.current_data_frequency:.1f} Hz")
            
            # Update plots
            self.ax_force.clear()
            self.ax_torque.clear()
            self.ax_force.grid(True, alpha=0.4)
            self.ax_torque.grid(True, alpha=0.4)
            self.ax_force.set_title('Forces', fontsize=12, fontweight='bold')
            self.ax_force.set_ylabel('Force (N)', fontsize=10)
            self.ax_torque.set_title('Torques', fontsize=12, fontweight='bold')
            self.ax_torque.set_xlabel('Time (s)', fontsize=10)
            self.ax_torque.set_ylabel('Torque (Nm)', fontsize=10)

            force_colors = {'Fx': 'r', 'Fy': 'g', 'Fz': 'b'}
            torque_colors = {'Mx': 'r', 'My': 'g', 'Mz': 'b'}
            
            current_time = rospy.get_time()
            
            for key, data in self.data_buffers.items():
                if data:
                    times, values = zip(*data)
                    relative_times = [t - current_time for t in times]
                    if key in force_colors:
                        self.ax_force.plot(relative_times, values, color=force_colors[key], label=key)
                    elif key in torque_colors:
                        self.ax_torque.plot(relative_times, values, color=torque_colors[key], label=key)
            
            self.ax_force.legend(loc='upper left')
            self.ax_torque.legend(loc='upper left')
            self.ax_force.set_xlim(-self.time_window_seconds, 0)

        self.canvas.draw()
        self.root.after(50, self.update_gui_and_plot) # ~20 Hz GUI refresh

    def start_zeroing(self):
        with self.data_lock:
            self.is_zeroing = True
            self.zero_data = {key: [] for key in self.data_buffers.keys()}
        rospy.loginfo("Re-zeroing process started.")
        
    def clear_plot(self):
        with self.data_lock:
            for buffer in self.data_buffers.values():
                buffer.clear()
        rospy.loginfo("Plot data cleared.")
        
    def on_closing(self):
        rospy.loginfo("Visualizer window closed, shutting down.")
        self.root.quit()
        self.root.destroy()
        rospy.signal_shutdown("GUI closed")

    def run(self):
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.after(100, self.update_gui_and_plot)
            self.root.mainloop()
        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr(f"Visualizer node error: {e}")

def main():
    try:
        visualizer = ForceSensorVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 