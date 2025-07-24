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
from tkinter import messagebox
from std_msgs.msg import Float64, String
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
        
        # DS controller status
        self.control_phase = "CALIBRATION"
        self.energy_level = 0.0
        self.controller_status = "Initializing..."
        
        # Create GUI
        self.setup_gui()
        
        # Subscribe to force sensor topic
        self.force_sub = rospy.Subscriber('/force_sensor/wrench', WrenchStamped, self.force_callback)
        
        # Subscribe to DS controller topics
        rospy.Subscriber('/unified_ds/control_phase', String, self.phase_callback)
        rospy.Subscriber('/unified_ds/energy_tank', Float64, self.energy_callback)
        rospy.Subscriber('/unified_ds/status', String, self.status_callback)
        
        # Command publisher
        self.command_pub = rospy.Publisher('/unified_ds/user_command', String, queue_size=1)
        
        rospy.loginfo("Force Sensor Visualizer Node Started")
        rospy.loginfo("Force sensor zeroing in progress, please ensure no external forces...")
        
    def setup_gui(self):
        """Set up GUI elements"""
        self.root = tk.Tk()
        self.root.title("DS Controller Force Sensor Visualizer")
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
        
        # Status information
        status_info_frame = ttk.Frame(status_frame)
        status_info_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(status_info_frame, text="System Status:", font=("Arial", 11, "bold")).grid(row=0, column=0, sticky='w', pady=(10,5))
        self.status_label = ttk.Label(status_info_frame, text="Status: Initializing...", font=("Arial", 10))
        self.status_label.grid(row=1, column=0, sticky='w')
        
        self.frequency_label = ttk.Label(status_info_frame, text="Update Rate: 0.0 Hz", font=("Arial", 10))
        self.frequency_label.grid(row=2, column=0, sticky='w')
        
        self.phase_label = ttk.Label(status_info_frame, text="Control Phase: CALIBRATION", font=("Arial", 10))
        self.phase_label.grid(row=3, column=0, sticky='w')
        
        self.energy_label = ttk.Label(status_info_frame, text="Energy Tank: 0.00 J", font=("Arial", 10))
        self.energy_label.grid(row=4, column=0, sticky='w')
        
        # Control buttons
        control_frame = ttk.LabelFrame(left_frame, text="Controls", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 使用更大、更醒目的启动按钮
        self.start_button = ttk.Button(control_frame, text="Start Motion", command=self.send_start_command)
        self.start_button.pack(fill=tk.X, pady=5)
        
        # 添加样式
        style = ttk.Style()
        style.configure("Green.TButton", background="green", foreground="white", font=('Arial', 12, 'bold'))
        style.configure("Red.TButton", background="red", foreground="white", font=('Arial', 12, 'bold'))
        
        # 更新启动按钮样式
        self.start_button.configure(style="Green.TButton")
        
        stop_button = ttk.Button(control_frame, text="Stop Motion", command=self.send_stop_command, style="Red.TButton")
        stop_button.pack(fill=tk.X, pady=5)
        
        zero_button = ttk.Button(control_frame, text="Re-Zero Sensor", command=self.start_zeroing)
        zero_button.pack(fill=tk.X, pady=5)
        
        clear_button = ttk.Button(control_frame, text="Clear Plot", command=self.clear_plot)
        clear_button.pack(fill=tk.X, pady=5)
        
        # === Right Panel: Force/Torque Plots ===
        plot_frame = ttk.Frame(right_frame)
        plot_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        
        # Create figure and axes
        self.fig = plt.Figure(figsize=(10, 8), dpi=100)
        self.ax_force = self.fig.add_subplot(211)  # Forces plot
        self.ax_torque = self.fig.add_subplot(212)  # Torques plot
        
        # Configure canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        # Add toolbar
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
    
    def phase_callback(self, msg):
        self.control_phase = msg.data
    
    def energy_callback(self, msg):
        self.energy_level = msg.data
    
    def status_callback(self, msg):
        self.controller_status = msg.data
        if msg.data == "CALIBRATION_COMPLETE":
            rospy.loginfo("Calibration complete. Send 'start' command to begin motion.")
            # 突出显示启动按钮
            self.start_button.configure(text="START MOTION NOW", style="Green.TButton")
            # 闪烁按钮以吸引注意
            self.flash_button(5)  # 闪烁5次
    
    def flash_button(self, times=3):
        """让按钮闪烁以吸引注意"""
        if times <= 0:
            return
            
        current_style = self.start_button.cget("style")
        if current_style == "Green.TButton":
            self.start_button.configure(style="TButton")
        else:
            self.start_button.configure(style="Green.TButton")
            
        self.root.after(500, lambda: self.flash_button(times - 1))

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
                if self.controller_status == "CALIBRATION_COMPLETE":
                    self.status_label.config(text="Status: Ready - Type 'start' to begin", foreground="blue")
                elif self.controller_status == "MOTION_STARTED":
                    self.status_label.config(text="Status: Motion in progress", foreground="green")
                elif self.controller_status == "MOTION_STOPPED":
                    self.status_label.config(text="Status: Motion stopped", foreground="red")
                else:
                    self.status_label.config(text=f"Status: {self.controller_status}", foreground="black")
            
            # Update phase and energy
            self.phase_label.config(text=f"Control Phase: {self.control_phase}")
            self.energy_label.config(text=f"Energy Tank: {self.energy_level:.2f} J")
            
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
            self.ax_torque.set_xlim(-self.time_window_seconds, 0)

        self.canvas.draw()
        self.root.after(50, self.update_gui_and_plot) # ~20 Hz GUI refresh

    def send_start_command(self):
        msg = String()
        msg.data = "start"
        self.command_pub.publish(msg)
        rospy.loginfo("Sent 'start' command")
        
        # 更新按钮状态
        self.start_button.configure(text="MOTION STARTED", state="disabled")
        
        # 显示一个启动确认消息
        messagebox_thread = threading.Thread(target=self._show_start_message)
        messagebox_thread.daemon = True
        messagebox_thread.start()
    
    def _show_start_message(self):
        # 在单独的线程中显示消息框，避免阻塞主线程
        import tkinter.messagebox as messagebox
        messagebox.showinfo("Motion Started", "Robot motion has been initiated.\nMonitoring force data...")
        
    def send_stop_command(self):
        msg = String()
        msg.data = "stop"
        self.command_pub.publish(msg)
        rospy.loginfo("Sent 'stop' command")
        
        # 重置启动按钮
        self.start_button.configure(text="Start Motion", state="normal", style="Green.TButton")

    def start_zeroing(self):
        with self.data_lock:
            self.is_zeroing = True
            self.zero_data = {key: [] for key in self.data_buffers.keys()}
        rospy.loginfo("Re-zeroing process started.")
        
    def clear_plot(self):
        with self.data_lock:
            for buffer in self.data_buffers.values():
                buffer.clear()
        
    def on_closing(self):
        rospy.loginfo("Shutting down Force Sensor Visualizer")
        self.root.quit()
        self.root.destroy()
        rospy.signal_shutdown("User closed window")
        
    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.update_gui_and_plot()
        self.root.mainloop()

def main():
    try:
        visualizer = ForceSensorVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in Force Sensor Visualizer: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 