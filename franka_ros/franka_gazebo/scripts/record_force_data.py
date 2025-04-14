#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
六维力传感数据记录脚本

作者: AIRJASON50
日期: 2024-04-14

此脚本用于:
1. 订阅探头的六维力传感数据
2. 记录数据到CSV文件
3. 实时显示力传感数据
4. 提供数据可视化功能
"""

import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
import time
import os
from datetime import datetime

class ForceDataRecorder:
    def __init__(self):
        rospy.init_node('force_data_recorder', anonymous=True)
        
        # 获取机器人ID参数
        self.arm_id = rospy.get_param('~arm_id', 'fr3')
        
        # 创建订阅者，用于接收力传感数据
        self.force_sub = rospy.Subscriber(
            f'/{self.arm_id}_probe_tcp_wrench', 
            WrenchStamped, 
            self.force_callback
        )
        
        # 创建发布者，用于发布状态信息
        self.status_pub = rospy.Publisher('force_recorder_status', String, queue_size=10)
        
        # 初始化数据存储
        self.force_data = []
        self.time_data = []
        self.start_time = None
        
        # 获取保存路径参数
        self.save_path = rospy.get_param('~save_path', os.path.expanduser('~/force_data'))
        
        # 确保保存目录存在
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        
        # 设置记录频率
        self.rate = rospy.Rate(100)  # 100Hz
        
        # 设置记录状态
        self.is_recording = False
        self.record_start_time = None
        
        # 创建实时绘图窗口
        plt.ion()
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('实时六维力传感数据')
        
        # 初始化绘图数据
        self.line_force_x, = self.ax[0].plot([], [], 'r-', label='Fx')
        self.line_force_y, = self.ax[0].plot([], [], 'g-', label='Fy')
        self.line_force_z, = self.ax[0].plot([], [], 'b-', label='Fz')
        self.line_torque_x, = self.ax[1].plot([], [], 'r-', label='Tx')
        self.line_torque_y, = self.ax[1].plot([], [], 'g-', label='Ty')
        self.line_torque_z, = self.ax[1].plot([], [], 'b-', label='Tz')
        
        # 设置图表属性
        self.ax[0].set_ylabel('力 (N)')
        self.ax[0].set_title('力')
        self.ax[0].grid(True)
        self.ax[0].legend()
        
        self.ax[1].set_xlabel('时间 (s)')
        self.ax[1].set_ylabel('力矩 (Nm)')
        self.ax[1].set_title('力矩')
        self.ax[1].grid(True)
        self.ax[1].legend()
        
        # 设置图表范围
        self.ax[0].set_ylim([-20, 20])
        self.ax[1].set_ylim([-2, 2])
        
        rospy.loginfo("六维力传感数据记录脚本已启动")
        rospy.loginfo(f"数据将保存到: {self.save_path}")
    
    def force_callback(self, msg):
        """处理力传感数据"""
        # 获取当前时间
        current_time = time.time()
        
        # 如果是第一次接收数据，初始化开始时间
        if self.start_time is None:
            self.start_time = current_time
        
        # 计算相对时间（秒）
        relative_time = current_time - self.start_time
        
        # 提取力传感数据
        force_x = msg.wrench.force.x
        force_y = msg.wrench.force.y
        force_z = msg.wrench.force.z
        torque_x = msg.wrench.torque.x
        torque_y = msg.wrench.torque.y
        torque_z = msg.wrench.torque.z
        
        # 如果正在记录，保存数据
        if self.is_recording:
            self.force_data.append([
                relative_time,
                force_x, force_y, force_z,
                torque_x, torque_y, torque_z
            ])
            self.time_data.append(relative_time)
            
            # 更新图表
            self.update_plot()
        
        # 每10Hz发布一次状态信息
        if int(relative_time * 10) % 10 == 0:
            status = f"力: Fx={force_x:.2f}N, Fy={force_y:.2f}N, Fz={force_z:.2f}N, " \
                     f"力矩: Tx={torque_x:.2f}Nm, Ty={torque_y:.2f}Nm, Tz={torque_z:.2f}Nm"
            self.status_pub.publish(status)
    
    def update_plot(self):
        """更新实时图表"""
        if not self.force_data:
            return
        
        # 提取数据
        times = [data[0] for data in self.force_data]
        forces_x = [data[1] for data in self.force_data]
        forces_y = [data[2] for data in self.force_data]
        forces_z = [data[3] for data in self.force_data]
        torques_x = [data[4] for data in self.force_data]
        torques_y = [data[5] for data in self.force_data]
        torques_z = [data[6] for data in self.force_data]
        
        # 更新图表数据
        self.line_force_x.set_data(times, forces_x)
        self.line_force_y.set_data(times, forces_y)
        self.line_force_z.set_data(times, forces_z)
        self.line_torque_x.set_data(times, torques_x)
        self.line_torque_y.set_data(times, torques_y)
        self.line_torque_z.set_data(times, torques_z)
        
        # 更新X轴范围
        if times:
            self.ax[0].set_xlim([max(0, times[-1] - 10), times[-1] + 1])
            self.ax[1].set_xlim([max(0, times[-1] - 10), times[-1] + 1])
        
        # 刷新图表
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def start_recording(self):
        """开始记录数据"""
        if not self.is_recording:
            self.is_recording = True
            self.record_start_time = time.time()
            self.force_data = []
            self.time_data = []
            rospy.loginfo("开始记录六维力传感数据")
    
    def stop_recording(self):
        """停止记录数据并保存"""
        if self.is_recording:
            self.is_recording = False
            record_duration = time.time() - self.record_start_time
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"force_data_{timestamp}.csv"
            filepath = os.path.join(self.save_path, filename)
            
            # 保存数据到CSV文件
            if self.force_data:
                df = pd.DataFrame(
                    self.force_data,
                    columns=['time', 'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
                )
                df.to_csv(filepath, index=False)
                rospy.loginfo(f"数据已保存到: {filepath}")
                rospy.loginfo(f"记录时长: {record_duration:.2f}秒, 数据点数: {len(self.force_data)}")
            else:
                rospy.logwarn("没有数据可保存")
    
    def run(self):
        """主循环"""
        rospy.loginfo("按 'r' 开始记录, 按 's' 停止记录, 按 'q' 退出")
        
        while not rospy.is_shutdown():
            # 检查键盘输入
            if os.path.exists('/dev/tty'):
                with open('/dev/tty', 'r') as f:
                    if f.isatty():
                        key = f.read(1)
                        if key == 'r':
                            self.start_recording()
                        elif key == 's':
                            self.stop_recording()
                        elif key == 'q':
                            rospy.signal_shutdown('用户退出')
            
            self.rate.sleep()
        
        # 关闭图表
        plt.ioff()
        plt.close()

if __name__ == '__main__':
    try:
        recorder = ForceDataRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass 