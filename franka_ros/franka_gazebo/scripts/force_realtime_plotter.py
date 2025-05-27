#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float64
import numpy as np
from collections import deque

class ForceRealTimePlotter:
    def __init__(self):
        rospy.init_node('force_plotter', anonymous=True)
        
        # 数据存储
        self.times = deque(maxlen=300)  # 保存最近5分钟数据
        self.force_z_data = deque(maxlen=300)
        self.theoretical_force_data = deque(maxlen=300)
        
        # 初始化图形
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        self.ax.set_xlabel('Time (s)', fontsize=12)
        self.ax.set_ylabel('Force (N)', fontsize=12)
        self.ax.set_title('Real-time Force Comparison', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        
        # 绘制线条
        self.line1, = self.ax.plot([], [], 'b-', label='Actual Force Z', linewidth=2)
        self.line2, = self.ax.plot([], [], 'r--', label='Theoretical Force', linewidth=2)
        self.ax.legend(loc='upper right')
        
        # ROS订阅器
        rospy.Subscriber('/force_plot/force_z', Float64, self.force_z_callback)
        rospy.Subscriber('/force_plot/theoretical_force', Float64, self.theoretical_force_callback)
        
        # 记录开始时间
        self.start_time = rospy.Time.now().to_sec()
        
        # 启动动画
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=200, blit=False)
        
        plt.tight_layout()
        plt.show()
    
    def force_z_callback(self, msg):
        current_time = rospy.Time.now().to_sec() - self.start_time
        self.times.append(current_time)
        self.force_z_data.append(msg.data)
        
        # 如果理论力数据还没有，填充0
        if len(self.theoretical_force_data) < len(self.force_z_data):
            self.theoretical_force_data.append(0.0)
    
    def theoretical_force_callback(self, msg):
        # 如果时间数据还没有，不添加
        if len(self.times) > 0:
            # 更新最后一个理论力值或添加新值
            if len(self.theoretical_force_data) >= len(self.times):
                self.theoretical_force_data[-1] = msg.data
            else:
                self.theoretical_force_data.append(msg.data)
    
    def update_plot(self, frame):
        if len(self.times) > 5:
            times_array = list(self.times)
            force_z_array = list(self.force_z_data)
            theoretical_array = list(self.theoretical_force_data)
            
            # 更新线条数据
            self.line1.set_data(times_array, force_z_array)
            self.line2.set_data(times_array, theoretical_array)
            
            # 动态调整坐标轴
            if times_array:
                self.ax.set_xlim(max(0, times_array[-1] - 60), times_array[-1] + 5)
                
                # Y轴范围
                all_forces = force_z_array + theoretical_array
                if all_forces:
                    y_min = min(all_forces) - 1
                    y_max = max(all_forces) + 1
                    self.ax.set_ylim(y_min, y_max)
        
        return self.line1, self.line2

if __name__ == '__main__':
    try:
        plotter = ForceRealTimePlotter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print('Force plotter stopped by user')
