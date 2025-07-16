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
from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk

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
        """Set up GUI elements"""
        self.root.title("Force Sensor Data Visualizer")
        self.root.geometry("1200x800")
        
        # Configure root grid
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Create main frame structure
        left_frame = ttk.Frame(self.root)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        
        right_frame = ttk.Frame(self.root)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        right_frame.grid_rowconfigure(0, weight=1)
        right_frame.grid_columnconfigure(0, weight=1)
        
        # === Left Panel: Status and Controls ===
        # Status frame
        status_frame = ttk.LabelFrame(left_frame, text="Status Information", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Force readings
        self.force_x_label = ttk.Label(status_frame, text="Force X: 0.000 N", font=("Arial", 10))
        self.force_x_label.pack(anchor=tk.W)
        
        self.force_y_label = ttk.Label(status_frame, text="Force Y: 0.000 N", font=("Arial", 10))
        self.force_y_label.pack(anchor=tk.W)
        
        self.force_z_label = ttk.Label(status_frame, text="Force Z: 0.000 N", font=("Arial", 10, "bold"))
        self.force_z_label.pack(anchor=tk.W)
        
        self.force_magnitude_label = ttk.Label(status_frame, text="Force Magnitude: 0.000 N", font=("Arial", 10))
        self.force_magnitude_label.pack(anchor=tk.W)
        
        # Separator
        separator = ttk.Separator(status_frame, orient='horizontal')
        separator.pack(fill=tk.X, pady=10)
        
        # Data statistics
        self.data_count_label = ttk.Label(status_frame, text="Data Points: 0", font=("Arial", 9))
        self.data_count_label.pack(anchor=tk.W)
        
        self.connection_label = ttk.Label(status_frame, text="Status: Disconnected", 
                                         font=("Arial", 9), foreground="red")
        self.connection_label.pack(anchor=tk.W)
        
        self.update_rate_label = ttk.Label(status_frame, text="Update Rate: 0.0 Hz", font=("Arial", 9))
        self.update_rate_label.pack(anchor=tk.W)
        
        # Force statistics 
        self.force_stats_label = ttk.Label(status_frame, text="Force Z Stats:\nMax: 0.000 N\nMin: 0.000 N\nAvg: 0.000 N", 
                                          font=("Arial", 9), justify=tk.LEFT)
        self.force_stats_label.pack(anchor=tk.W, pady=(10, 0))
        
        # Controls frame
        controls_frame = ttk.LabelFrame(left_frame, text="Controls", padding=10)
        controls_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Display options
        display_frame = ttk.LabelFrame(controls_frame, text="Display Options", padding=5)
        display_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.show_all_forces = tk.BooleanVar(value=False)
        ttk.Checkbutton(display_frame, text="Show All Force Components", 
                       variable=self.show_all_forces).pack(anchor=tk.W)
        
        self.show_magnitude = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Force Magnitude", 
                       variable=self.show_magnitude).pack(anchor=tk.W)
        
        self.show_raw_force_z = tk.BooleanVar(value=False)
        ttk.Checkbutton(display_frame, text="Show Raw Force Z", 
                       variable=self.show_raw_force_z).pack(anchor=tk.W)
        
        # Time window control
        time_frame = ttk.Frame(display_frame)
        time_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(time_frame, text="Time Window:").pack(side=tk.LEFT)
        self.time_window_var = tk.StringVar(value="30")
        time_spinbox = ttk.Spinbox(time_frame, from_=5, to=300, width=8, textvariable=self.time_window_var)
        time_spinbox.pack(side=tk.LEFT, padx=(5, 0))
        ttk.Label(time_frame, text="seconds").pack(side=tk.LEFT, padx=(5, 0))
        
        # Add control buttons
        button_frame = ttk.Frame(controls_frame)
        button_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Zero sensor button
        self.zero_button = ttk.Button(button_frame, text="Reset Zero Point", 
                                     command=self.start_zeroing, width=12)
        self.zero_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # Clear data button
        self.clear_button = ttk.Button(button_frame, text="Clear Data", 
                                      command=self.clear_plot, width=12)
        self.clear_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # ✅ Save image button - English text
        self.save_button = ttk.Button(button_frame, text="Save Image", 
                                     command=self.save_plot_image, width=12)
        self.save_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # Add second row of buttons
        button_frame2 = ttk.Frame(controls_frame)
        button_frame2.pack(fill=tk.X, pady=(5, 0))
        
        # ✅ Export Force Z button - English text
        self.export_button = ttk.Button(button_frame2, text="Export Force Z", 
                                       command=self.export_force_z_data, width=12)
        self.export_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # === Right Panel: Plot ===
        plot_frame = ttk.Frame(right_frame)
        plot_frame.grid(row=0, column=0, sticky="nsew")
        plot_frame.grid_rowconfigure(0, weight=1)
        plot_frame.grid_columnconfigure(0, weight=1)
        
        # Create matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.patch.set_facecolor('white')
        
        # ✅ English plot labels and title
        self.ax.set_xlabel('Time (seconds)', fontsize=12)
        self.ax.set_ylabel('Force (N)', fontsize=12)
        self.ax.set_title('Force Sensor Data - Real-time Plot', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_axisbelow(True)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        # Add toolbar
        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame)
        self.toolbar.update()
        
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
        
    def save_plot_image(self):
        """保存当前绘图为图像文件"""
        try:
            import tkinter.filedialog as filedialog
            import os
            from datetime import datetime
            
            # 生成默认文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"force_sensor_plot_{timestamp}.png"
            
            # 弹出保存对话框
            filename = filedialog.asksaveasfilename(
                defaultextension=".png",
                initialvalue=default_filename,
                filetypes=[
                    ("PNG files", "*.png"),
                    ("PDF files", "*.pdf"),
                    ("SVG files", "*.svg"),
                    ("All files", "*.*")
                ],
                title="保存力传感器绘图"
            )
            
            if filename:
                # 保存图像
                self.fig.savefig(filename, dpi=300, bbox_inches='tight', 
                               facecolor='white', edgecolor='none')
                rospy.loginfo(f"图像已保存到: {filename}")
                
                # 更新状态显示
                self.status_label.config(text=f"图像已保存: {os.path.basename(filename)}", 
                                       foreground="blue")
                # 3秒后恢复状态显示
                self.root.after(3000, lambda: self.status_label.config(
                    text="数据采集中...", foreground="green"))
            
        except Exception as e:
            rospy.logerr(f"保存图像时出错: {e}")
            self.status_label.config(text=f"保存失败: {str(e)}", foreground="red")
    
    def export_force_z_data(self):
        """导出Force_z数据到CSV文件"""
        try:
            import tkinter.filedialog as filedialog
            import csv
            import os
            from datetime import datetime
            
            with self.data_lock:
                fz_data = list(self.data_buffers['Fz'])
            
            if not fz_data:
                rospy.logwarn("没有Force_z数据可以导出")
                self.status_label.config(text="没有数据可导出", foreground="orange")
                return
            
            # 生成默认文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"force_z_data_{timestamp}.csv"
            
            # 弹出保存对话框
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                initialvalue=default_filename,
                filetypes=[
                    ("CSV files", "*.csv"),
                    ("Text files", "*.txt"),
                    ("All files", "*.*")
                ],
                title="导出Force_z数据"
            )
            
            if filename:
                # 写入CSV文件
                with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # 写入头部信息
                    writer.writerow(['# Force_z数据导出'])
                    writer.writerow(['# 导出时间:', timestamp])
                    writer.writerow(['# 数据点数:', len(fz_data)])
                    writer.writerow(['# 列定义: 样本序号, Force_z值(N)'])
                    writer.writerow([])  # 空行
                    
                    # 写入CSV头
                    writer.writerow(['sample_index', 'force_z_N'])
                    
                    # 写入数据
                    for i, value in enumerate(fz_data):
                        writer.writerow([i, f"{value:.6f}"])
                
                rospy.loginfo(f"Force_z数据已导出到: {filename}")
                rospy.loginfo(f"导出了 {len(fz_data)} 个数据点")
                
                # 更新状态显示
                self.status_label.config(text=f"数据已导出: {os.path.basename(filename)}", 
                                       foreground="blue")
                # 3秒后恢复状态显示
                self.root.after(3000, lambda: self.status_label.config(
                    text="数据采集中...", foreground="green"))
                    
        except Exception as e:
            rospy.logerr(f"导出Force_z数据时出错: {e}")
            self.status_label.config(text=f"导出失败: {str(e)}", foreground="red")
        
    def update_plot(self):
        """Update the matplotlib plot with latest data"""
        with self.data_lock:
            if not self.data_buffers:
                return
                
            # Get time window
            try:
                time_window = float(self.time_window_var.get())
            except:
                time_window = 30.0
                
            # Get current time and calculate cutoff
            current_time = time.time()
            cutoff_time = current_time - time_window
            
            # Clear previous plot
            self.ax.clear()
            
            # ✅ English plot labels and title
            self.ax.set_xlabel('Time (seconds)', fontsize=12)
            self.ax.set_ylabel('Force (N)', fontsize=12)
            self.ax.set_title('Force Sensor Data - Real-time Plot', fontsize=14, fontweight='bold')
            self.ax.grid(True, alpha=0.3)
            self.ax.set_axisbelow(True)
            
            plot_lines = []
            
            # Plot Force Z (always shown)
            force_z_data = self.data_buffers.get('Fz', [])
            if force_z_data:
                # Filter data by time window
                filtered_data = [(t, val) for t, val in force_z_data if t >= cutoff_time]
                if filtered_data:
                    times, values = zip(*filtered_data)
                    # Convert to relative time
                    rel_times = [t - current_time for t in times]
                    line, = self.ax.plot(rel_times, values, 'b-', linewidth=2, alpha=0.8)
                    plot_lines.append((line, 'Force Z (Filtered)'))  # ✅ English legend
            
            # Plot all force components if selected
            if self.show_all_forces.get():
                for component, color in [('Fx', 'red'), ('Fy', 'green')]:
                    data = self.data_buffers.get(component, [])
                    if data:
                        filtered_data = [(t, val) for t, val in data if t >= cutoff_time]
                        if filtered_data:
                            times, values = zip(*filtered_data)
                            rel_times = [t - current_time for t in times]
                            line, = self.ax.plot(rel_times, values, color=color, linewidth=1.5, alpha=0.7)
                            # ✅ English legend names
                            legend_name = component.replace('_', ' ')  # Force_X -> Force X
                            plot_lines.append((line, legend_name))
            
            # Plot force magnitude if selected
            if self.show_magnitude.get():
                magnitude_data = self.data_buffers.get('Force_Magnitude', [])
                if magnitude_data:
                    filtered_data = [(t, val) for t, val in magnitude_data if t >= cutoff_time]
                    if filtered_data:
                        times, values = zip(*filtered_data)
                        rel_times = [t - current_time for t in times]
                        line, = self.ax.plot(rel_times, values, 'purple', linewidth=2, alpha=0.6, linestyle='--')
                        plot_lines.append((line, 'Force Magnitude'))  # ✅ English legend
            
            # Plot raw Force Z if selected
            if self.show_raw_force_z.get():
                raw_data = self.data_buffers.get('Raw_Force_Z', [])
                if raw_data:
                    filtered_data = [(t, val) for t, val in raw_data if t >= cutoff_time]
                    if filtered_data:
                        times, values = zip(*filtered_data)
                        rel_times = [t - current_time for t in times]
                        line, = self.ax.plot(rel_times, values, 'orange', linewidth=1, alpha=0.5, linestyle=':')
                        plot_lines.append((line, 'Force Z (Raw)'))  # ✅ English legend
            
            # Add legend if there are lines to show
            if plot_lines:
                lines, labels = zip(*plot_lines)
                self.ax.legend(lines, labels, loc='upper right', fontsize=10)
            
            # Set axis limits
            self.ax.set_xlim(-time_window, 0)
            
            # Auto-scale y-axis with some padding
            try:
                self.ax.relim()
                self.ax.autoscale_view(scalex=False, scaley=True)
                y_min, y_max = self.ax.get_ylim()
                y_range = y_max - y_min
                if y_range > 0:
                    self.ax.set_ylim(y_min - 0.1 * y_range, y_max + 0.1 * y_range)
            except:
                pass
            
            # Refresh canvas
            self.canvas.draw_idle()
        
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