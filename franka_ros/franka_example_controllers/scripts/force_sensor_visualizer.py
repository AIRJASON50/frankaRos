#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Force Sensor Data Visualizer Node
Subscribes to ROS topic /force_sensor/wrench and displays force sensor data in real-time
Based on forcesenseRead.py design but adapted for ROS environment
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

class ForceSensorVisualizer:
    def __init__(self):
        # ROS initialization
        rospy.init_node('force_sensor_visualizer', anonymous=True)
        
        # Data buffers - store last 200 data points for each channel
        self.max_points = 200
        self.data_buffers = {
            'Fx': deque(maxlen=self.max_points),
            'Fy': deque(maxlen=self.max_points), 
            'Fz': deque(maxlen=self.max_points),
            'Mx': deque(maxlen=self.max_points),
            'My': deque(maxlen=self.max_points),
            'Mz': deque(maxlen=self.max_points)
        }
        
        # Zero offset storage
        self.zero_offsets = {'Fx': 0, 'Fy': 0, 'Fz': 0, 'Mx': 0, 'My': 0, 'Mz': 0}
        self.is_zeroing = True
        self.zero_data = {key: [] for key in self.data_buffers.keys()}
        
        # Data lock
        self.data_lock = threading.Lock()
        
        # Current data
        self.current_data = {'Fx': 0, 'Fy': 0, 'Fz': 0, 'Mx': 0, 'My': 0, 'Mz': 0}
        
        # Data frequency tracking - 本地计算，资源占用minimal
        self.current_data_frequency = 0.0
        self.last_data_times = deque(maxlen=20)  # 只保留最近20个时间戳用于频率计算
        self.frequency_update_counter = 0        # 计数器，每10次回调计算一次频率
        
        # Create GUI
        self.setup_gui()
        
        # Subscribe to force sensor topic
        self.force_sub = rospy.Subscriber('/force_sensor/wrench', WrenchStamped, self.force_callback)
        
        # Setup signal handling
        signal.signal(signal.SIGINT, self.signal_handler)
        
        rospy.loginfo("Force Sensor Visualizer Node Started")
        rospy.loginfo("Force sensor zeroing in progress, please ensure no external forces...")
        
    def setup_gui(self):
        """Setup GUI interface"""
        self.root = tk.Tk()
        self.root.title("Real-time Force Sensor Data Visualizer")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Status information frame
        status_frame = ttk.LabelFrame(main_frame, text="Status Information", padding="10")
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_label = ttk.Label(status_frame, text="Initializing...", font=('Arial', 10))
        self.status_label.pack()
        
        # Data frequency display
        self.frequency_label = ttk.Label(status_frame, text="Data Frequency: 0.0 Hz", 
                                        font=('Arial', 10, 'bold'), foreground='blue')
        self.frequency_label.pack(pady=(5, 0))
        
        # GUI update frequency display
        self.gui_frequency_label = ttk.Label(status_frame, text="GUI Update: 100.0 Hz", 
                                           font=('Arial', 9), foreground='green')
        self.gui_frequency_label.pack(pady=(2, 0))
        
        # Drift correction status
        self.drift_status_label = ttk.Label(status_frame, text="Drift Correction: Active", 
                                          font=('Arial', 9), foreground='purple')
        self.drift_status_label.pack(pady=(2, 0))
        
        # Real-time values display frame
        values_frame = ttk.LabelFrame(main_frame, text="Real-time Values (After Zeroing)", padding=10)
        values_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create value display grid
        self.value_labels = {}
        for i, (key, unit) in enumerate([('Fx', 'N'), ('Fy', 'N'), ('Fz', 'N'), 
                                        ('Mx', 'N·m'), ('My', 'N·m'), ('Mz', 'N·m')]):
            row = i // 3
            col = i % 3
            
            label_frame = ttk.Frame(values_frame)
            label_frame.grid(row=row, column=col, padx=10, pady=5, sticky="w")
            
            name_label = ttk.Label(label_frame, text=f"{key}:", font=("Arial", 10, "bold"))
            name_label.pack(side=tk.LEFT)
            
            value_label = ttk.Label(label_frame, text="0.000", font=("Arial", 10), foreground="blue")
            value_label.pack(side=tk.LEFT, padx=(5, 0))
            
            unit_label = ttk.Label(label_frame, text=unit, font=("Arial", 10))
            unit_label.pack(side=tk.LEFT, padx=(5, 0))
            
            self.value_labels[key] = value_label
        
        # Control buttons frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.zero_button = ttk.Button(button_frame, text="Re-Zero", command=self.start_zeroing)
        self.zero_button.pack(side=tk.LEFT, padx=(0, 10))
        
        clear_button = ttk.Button(button_frame, text="Clear Plot", command=self.clear_plot)
        clear_button.pack(side=tk.LEFT)
        
        # Plot frame
        plot_frame = ttk.LabelFrame(main_frame, text="Real-time Waveforms", padding=10)
        plot_frame.pack(fill=tk.BOTH, expand=True)
        
        # Create matplotlib plots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.suptitle("6D Force Sensor Real-time Data", fontsize=14, fontweight='bold')
        
        # Setup subplots
        self.ax1.set_title("Forces (Fx, Fy, Fz)")
        self.ax1.set_ylabel("Force (N)")
        self.ax1.grid(True, alpha=0.3)
        self.ax1.legend(['Fx', 'Fy', 'Fz'], loc='upper right')
        
        self.ax2.set_title("Torques (Mx, My, Mz)")
        self.ax2.set_xlabel("Time (Sample Points)")
        self.ax2.set_ylabel("Torque (N·m)")
        self.ax2.grid(True, alpha=0.3)
        self.ax2.legend(['Mx', 'My', 'Mz'], loc='upper right')
        
        # Initialize lines
        self.lines = {}
        colors = ['red', 'green', 'blue']
        for i, key in enumerate(['Fx', 'Fy', 'Fz']):
            line, = self.ax1.plot([], [], color=colors[i], label=key, linewidth=2)
            self.lines[key] = line
            
        for i, key in enumerate(['Mx', 'My', 'Mz']):
            line, = self.ax2.plot([], [], color=colors[i], label=key, linewidth=2)
            self.lines[key] = line
        
        # Embed matplotlib in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def force_callback(self, msg):
        """Force sensor data callback function"""
        current_time = time.time()
        
        # 频率计算 - 轻量级实现，每10次回调计算一次
        self.frequency_update_counter += 1
        if self.frequency_update_counter >= 10:  # 每10次数据回调计算一次频率
            self.last_data_times.append(current_time)
            if len(self.last_data_times) >= 2:
                # 计算平均频率
                time_span = self.last_data_times[-1] - self.last_data_times[0]
                if time_span > 0:
                    self.current_data_frequency = (len(self.last_data_times) - 1) / time_span
            self.frequency_update_counter = 0  # 重置计数器
        
        # Extract force and torque data
        raw_data = {
            'Fx': msg.wrench.force.x,
            'Fy': msg.wrench.force.y, 
            'Fz': msg.wrench.force.z,
            'Mx': msg.wrench.torque.x,
            'My': msg.wrench.torque.y,
            'Mz': msg.wrench.torque.z
        }
        
        with self.data_lock:
            if self.is_zeroing:
                # Zeroing phase: collect data
                for key, value in raw_data.items():
                    self.zero_data[key].append(value)
                
                # Check if enough data collected for zeroing
                if len(self.zero_data['Fx']) >= 100:
                    # Calculate zero offsets
                    for key in self.zero_offsets.keys():
                        self.zero_offsets[key] = np.mean(self.zero_data[key])
                    
                    rospy.loginfo("Force sensor zeroing completed!")
                    rospy.loginfo(f"Zero offsets: Fx={self.zero_offsets['Fx']:.3f}, Fy={self.zero_offsets['Fy']:.3f}, Fz={self.zero_offsets['Fz']:.3f}")
                    rospy.loginfo(f"             Mx={self.zero_offsets['Mx']:.6f}, My={self.zero_offsets['My']:.6f}, Mz={self.zero_offsets['Mz']:.6f}")
                    
                    self.is_zeroing = False
                    # Update status in main thread
                    self.root.after(0, lambda: self.status_label.config(text="Zeroing Complete - Data Acquisition", foreground="green"))
            else:
                # Normal data acquisition: apply zero offset compensation
                for key, value in raw_data.items():
                    adjusted_value = value - self.zero_offsets[key]
                    
                    # Data validity check
                    if -50 <= adjusted_value <= 50:  # Reasonable range check
                        self.current_data[key] = adjusted_value
                        self.data_buffers[key].append(adjusted_value)
                    else:
                        # Use last valid data
                        self.data_buffers[key].append(self.current_data[key])
    
    def start_zeroing(self):
        """Start re-zeroing process"""
        with self.data_lock:
            self.is_zeroing = True
            self.zero_data = {key: [] for key in self.data_buffers.keys()}
            self.zero_offsets = {key: 0 for key in self.zero_offsets.keys()}
            
        self.status_label.config(text="Re-zeroing in progress...", foreground="orange")
        rospy.loginfo("Starting re-zeroing process...")
        
    def clear_plot(self):
        """Clear plot data"""
        with self.data_lock:
            for buffer in self.data_buffers.values():
                buffer.clear()
        rospy.loginfo("Plot data cleared")
        
    def update_plot(self):
        """Update plot display"""
        if rospy.is_shutdown():
            return
            
        with self.data_lock:
            # Update real-time value display
            for key, label in self.value_labels.items():
                value = self.current_data[key]
                if abs(value) < 0.001:
                    label.config(text="0.000")
                else:
                    label.config(text=f"{value:.3f}")
            
            # Update plots
            for key, line in self.lines.items():
                y_data = list(self.data_buffers[key])
                x_data = list(range(len(y_data)))
                line.set_data(x_data, y_data)
            
            # Adjust axis ranges
            if len(self.data_buffers['Fx']) > 0:
                # Force plot
                all_forces = []
                for key in ['Fx', 'Fy', 'Fz']:
                    all_forces.extend(list(self.data_buffers[key]))
                if all_forces:
                    force_range = max(abs(min(all_forces)), abs(max(all_forces)), 1.0)
                    self.ax1.set_xlim(0, self.max_points)
                    self.ax1.set_ylim(-force_range*1.1, force_range*1.1)
                
                # Torque plot  
                all_torques = []
                for key in ['Mx', 'My', 'Mz']:
                    all_torques.extend(list(self.data_buffers[key]))
                if all_torques:
                    torque_range = max(abs(min(all_torques)), abs(max(all_torques)), 0.01)
                    self.ax2.set_xlim(0, self.max_points)
                    self.ax2.set_ylim(-torque_range*1.1, torque_range*1.1)
        
        # Redraw plots
        try:
            self.canvas.draw()
        except:
            pass
        
        # Schedule next update - 100Hz update frequency (10ms interval)
        self.root.after(10, self.update_plot)
        
    def signal_handler(self, sig, frame):
        """Signal handler"""
        rospy.loginfo("Interrupt signal detected, shutting down safely...")
        self.on_closing()
        
    def on_closing(self):
        """Window close handler"""
        rospy.loginfo("Force sensor visualizer node shutting down...")
        if not rospy.is_shutdown():
            rospy.signal_shutdown("GUI closed")
        self.root.quit()
        self.root.destroy()
        
    def run(self):
        """Run main loop"""
        # Start plot updates
        self.update_plot()
        
        # Start GUI main loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_closing()

    def update_gui(self):
        """Update GUI with latest data - runs at 100Hz for real-time response"""
        try:
            with self.data_lock:
                # Update status
                if self.is_zeroing:
                    progress = len(self.zero_data['Fx']) if 'Fx' in self.zero_data else 0
                    self.status_label.config(text=f"Zeroing... ({progress}/100 samples)")
                else:
                    self.status_label.config(text="Zeroing Complete - Data Acquisition")
                
                # Update frequency displays - 使用本地计算的频率
                self.frequency_label.config(text=f"Data Frequency: {self.current_data_frequency:.1f} Hz")
                
                # Calculate GUI update frequency
                current_time = time.time()
                if hasattr(self, 'last_gui_update_time'):
                    gui_interval = current_time - self.last_gui_update_time
                    if gui_interval > 0:
                        gui_freq = 1.0 / gui_interval
                        self.gui_frequency_label.config(text=f"GUI Update: {gui_freq:.1f} Hz")
                self.last_gui_update_time = current_time
                
                # Update real-time values
                for key, value in self.current_data.items():
                    if key in self.value_labels:
                        self.value_labels[key].config(text=f"{value:7.3f}")
                
        except Exception as e:
            rospy.logwarn(f"GUI update error: {e}")
        
        # Schedule next update - 10ms for 100Hz
        self.root.after(10, self.update_gui)

def main():
    try:
        visualizer = ForceSensorVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Force sensor visualizer node error: {e}")

if __name__ == '__main__':
    main() 