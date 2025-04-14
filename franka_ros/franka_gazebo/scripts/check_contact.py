#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
探头位置检测与软体块接触分析脚本

作者: AIRJASON50
日期: 2024-04-14

此脚本用于:
1. 获取探头TCP点的位置
2. 获取软体块的位置
3. 计算两者之间的距离
4. 提供调整建议，使探头能够接触软体块
"""

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import String
import time

class ContactChecker:
    def __init__(self):
        rospy.init_node('contact_checker', anonymous=True)
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 获取软体块位置参数
        self.soft_block_x = rospy.get_param('~soft_block_x', 0.4)
        self.soft_block_y = rospy.get_param('~soft_block_y', 0.0)
        self.soft_block_z = rospy.get_param('~soft_block_z', 0.505)
        
        # 获取机器人ID
        self.arm_id = rospy.get_param('~arm_id', 'fr3')
        
        # 创建发布器，用于发布调整建议
        self.advice_pub = rospy.Publisher('contact_advice', String, queue_size=10)
        
        # 设置检查频率
        self.rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("探头位置检测与软体块接触分析脚本已启动")
        rospy.loginfo(f"软体块位置: x={self.soft_block_x}, y={self.soft_block_y}, z={self.soft_block_z}")
    
    def get_probe_tcp_position(self):
        """获取探头TCP点的位置"""
        try:
            # 等待TF树构建完成
            rospy.sleep(1.0)
            
            # 获取探头TCP点相对于世界坐标系的位置
            transform = self.tf_buffer.lookup_transform(
                'world', 
                f'{self.arm_id}_probe_tcp', 
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            return transform.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"无法获取探头TCP点位置: {e}")
            return None
    
    def calculate_distance(self, probe_pos):
        """计算探头TCP点与软体块之间的距离"""
        if probe_pos is None:
            return None
        
        # 创建软体块位置点
        soft_block_pos = Point()
        soft_block_pos.x = self.soft_block_x
        soft_block_pos.y = self.soft_block_y
        soft_block_pos.z = self.soft_block_z
        
        # 计算欧氏距离
        distance = np.sqrt(
            (probe_pos.x - soft_block_pos.x) ** 2 +
            (probe_pos.y - soft_block_pos.y) ** 2 +
            (probe_pos.z - soft_block_pos.z) ** 2
        )
        
        return distance, soft_block_pos
    
    def generate_advice(self, probe_pos, distance, soft_block_pos):
        """生成调整建议"""
        if probe_pos is None or distance is None:
            return "无法获取位置信息，请检查TF树是否正确构建"
        
        advice = []
        
        # 分析X轴方向
        if abs(probe_pos.x - soft_block_pos.x) > 0.01:
            if probe_pos.x < soft_block_pos.x:
                advice.append(f"X轴: 需要向前移动 {abs(probe_pos.x - soft_block_pos.x):.3f}米")
            else:
                advice.append(f"X轴: 需要向后移动 {abs(probe_pos.x - soft_block_pos.x):.3f}米")
        
        # 分析Y轴方向
        if abs(probe_pos.y - soft_block_pos.y) > 0.01:
            if probe_pos.y < soft_block_pos.y:
                advice.append(f"Y轴: 需要向左移动 {abs(probe_pos.y - soft_block_pos.y):.3f}米")
            else:
                advice.append(f"Y轴: 需要向右移动 {abs(probe_pos.y - soft_block_pos.y):.3f}米")
        
        # 分析Z轴方向
        if abs(probe_pos.z - soft_block_pos.z) > 0.01:
            if probe_pos.z < soft_block_pos.z:
                advice.append(f"Z轴: 需要向上移动 {abs(probe_pos.z - soft_block_pos.z):.3f}米")
            else:
                advice.append(f"Z轴: 需要向下移动 {abs(probe_pos.z - soft_block_pos.z):.3f}米")
        
        if not advice:
            return f"探头与软体块已接触，距离: {distance:.3f}米"
        
        return "调整建议:\n" + "\n".join(advice) + f"\n当前距离: {distance:.3f}米"
    
    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            # 获取探头TCP点位置
            probe_pos = self.get_probe_tcp_position()
            
            if probe_pos:
                rospy.loginfo(f"探头TCP点位置: x={probe_pos.x:.3f}, y={probe_pos.y:.3f}, z={probe_pos.z:.3f}")
                
                # 计算距离
                distance, soft_block_pos = self.calculate_distance(probe_pos)
                
                if distance is not None:
                    rospy.loginfo(f"探头与软体块距离: {distance:.3f}米")
                    
                    # 生成调整建议
                    advice = self.generate_advice(probe_pos, distance, soft_block_pos)
                    rospy.loginfo(advice)
                    
                    # 发布建议
                    self.advice_pub.publish(advice)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        checker = ContactChecker()
        checker.run()
    except rospy.ROSInterruptException:
        pass 