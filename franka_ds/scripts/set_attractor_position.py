#!/usr/bin/env python3
"""
Force Guided Controller - 位置设置脚本
用于读取当前机械臂位置并设置为目标位置

功能：
1. 读取当前机械臂末端位置
2. 更新unified_ds_params.yaml配置文件中的target_position和contact_surface_height
3. 通过ROS话题通知当前运行的控制器

使用方法：
1. 手动将机械臂移动到期望位置
2. 运行此脚本读取并保存当前位置
"""

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import yaml
import os
import re

class PositionSetter:
    def __init__(self):
        rospy.init_node('position_setter', anonymous=True)
        
        # 配置文件路径
        self.config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'unified_ds_params.yaml'
        )
        
        # 发布器：设置新的吸引子位置
        self.attractor_pub = rospy.Publisher('/force_guided/set_attractor', PoseStamped, queue_size=1)
        
        # 当前位置
        self.current_pose = None
        self.pose_received = False
        
        # 订阅当前位姿
        self.pose_sub = rospy.Subscriber('/force_guided/current_pose', PoseStamped, self.pose_callback)
        
        rospy.loginfo("位置设置器已启动，等待当前位姿数据...")
    
    def pose_callback(self, msg):
        """接收当前位姿回调"""
        self.current_pose = msg
        self.pose_received = True
    
    def wait_for_pose(self, timeout=10.0):
        """等待接收到位姿数据"""
        rospy.loginfo("等待当前位姿数据...")
        rate = rospy.Rate(10)  # 10Hz
        start_time = rospy.Time.now()
        
        while not self.pose_received and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("超时：未能接收到位姿数据")
                return False
            rate.sleep()
        
        return True
    
    def get_current_position(self):
        """获取当前位置"""
        if not self.wait_for_pose():
            return None
        
        if self.current_pose is None:
            rospy.logerr("未接收到位姿数据")
            return None
        
        pos = self.current_pose.pose.position
        return [pos.x, pos.y, pos.z]
    
    def update_config_file(self, position):
        """更新unified_ds_params.yaml配置文件 - 保持原始格式和注释"""
        try:
            # 读取原始文件内容（保持注释和格式）
            with open(self.config_path, 'r', encoding='utf-8') as file:
                content = file.read()
            

            
            # 更新 target_position 部分（保持注释和格式）
            # 找到 target_position 开始位置
            target_start = content.find('target_position:')
            if target_start == -1:
                rospy.logerr("❌ 在配置文件中找不到 target_position 字段")
                return False
            
            # 找到下一个主要配置段的开始（通过查找下一个不以空格或-开头的行）
            lines = content[target_start:].split('\n')
            target_end_line = 0
            
            for i, line in enumerate(lines[1:], 1):  # 跳过 target_position: 这一行
                if line.strip() and not line.startswith(' ') and not line.startswith('-') and not line.startswith('#'):
                    target_end_line = i
                    break
            
            if target_end_line == 0:
                # 如果没找到下一个段，说明这是文件末尾
                target_end_line = len(lines)
            
            # 构建新的 target_position 段
            new_target_section = f"""target_position:                  # 目标吸引子位置 [m]
- {position[0]:.16f}             # X坐标
- {position[1]:.16f}            # Y坐标  
- {position[2]:.16f}             # Z坐标（与contact_surface_height应保持一致）"""
            
            # 替换原有的 target_position 段
            before_target = content[:target_start]
            after_target = '\n'.join(lines[target_end_line:]) if target_end_line < len(lines) else ''
            
            # 确保段之间有适当的空行
            if after_target and not after_target.startswith('\n'):
                after_target = '\n' + after_target
            
            new_content = before_target + new_target_section + after_target
            
            # 写回文件
            with open(self.config_path, 'w', encoding='utf-8') as file:
                file.write(new_content)
            
            rospy.loginfo(f"✅ 成功更新配置文件:")
            rospy.loginfo(f"   target_position: [{position[0]:.6f}, {position[1]:.6f}, {position[2]:.6f}]")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ 更新配置文件失败: {e}")
            return False
    
    def set_attractor_position(self, position):
        """设置吸引子位置（同时更新配置文件和ROS话题）"""
        # 1. 更新配置文件
        config_success = self.update_config_file(position)
        
        # 2. 通过ROS话题通知当前控制器
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.w = 1.0  # 单位四元数
        
        # 发布设置命令
        self.attractor_pub.publish(msg)
        rospy.loginfo(f"✅ 已通过ROS话题设置吸引子位置: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        
        if config_success:
            rospy.loginfo("🎯 位置设置完成！配置文件和运行时参数均已更新")
        else:
            rospy.logwarn("⚠️  ROS话题设置成功，但配置文件更新失败")
        
        return config_success
    
    def run_interactive(self):
        """交互式运行"""
        print("\n=== 力引导控制器位置设置工具 ===")
        print("功能：读取当前位置并更新到unified_ds_params.yaml配置文件")
        print("请手动将机械臂移动到期望位置，然后按回车键保存位置")
        print("输入 'q' 退出程序")
        
        while not rospy.is_shutdown():
            try:
                user_input = input("\n按回车键读取当前位置 (或输入 'q' 退出): ").strip()
                
                if user_input.lower() == 'q':
                    print("退出位置设置工具")
                    break
                
                # 读取当前位置
                current_pos = self.get_current_position()
                if current_pos is None:
                    print("❌ 无法获取当前位置，请确保控制器正在运行")
                    continue
                
                print(f"📍 当前位置: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")
                print(f"📐 接触面高度: {current_pos[2]:.3f}")
                
                # 确认是否设置
                confirm = input("是否将此位置设置为目标位置并更新配置文件？(y/N): ").strip().lower()
                if confirm == 'y' or confirm == 'yes':
                    success = self.set_attractor_position(current_pos)
                    if success:
                        print("✅ 位置已成功保存到配置文件！")
                        print("   其他程序现在可以从unified_ds_params.yaml读取此位置")
                    else:
                        print("❌ 保存失败，请检查文件权限")
                else:
                    print("❌ 位置未保存")
                    
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                break
            except Exception as e:
                print(f"❌ 错误: {e}")

def main():
    try:
        setter = PositionSetter()
        
        if len(sys.argv) > 1 and sys.argv[1] == '--auto':
            # 自动模式：读取当前位置并立即设置
            rospy.loginfo("🤖 自动模式：读取当前位置并更新配置文件...")
            position = setter.get_current_position()
            if position:
                success = setter.set_attractor_position(position)
                if success:
                    rospy.loginfo("🎉 自动设置完成！位置已保存到unified_ds_params.yaml")
                else:
                    rospy.logerr("❌ 自动设置失败：无法更新配置文件")
            else:
                rospy.logerr("❌ 自动设置失败：无法获取当前位置")
        else:
            # 交互模式
            setter.run_interactive()
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 