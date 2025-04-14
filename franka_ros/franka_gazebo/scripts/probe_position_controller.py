#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf.transformations as tft
import numpy as np

class ProbeController:
    def __init__(self):
        rospy.init_node('probe_controller', anonymous=True)
        
        # 获取参数
        self.robot_model = rospy.get_param('~robot_model', 'fr3')
        self.end_effector_link = rospy.get_param('~end_effector_link', self.robot_model + '_link8')
        self.probe_model_name = rospy.get_param('~probe_model_name', 'probe_model')
        
        # 设置TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 设置服务客户端
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 设置更新率
        self.rate = rospy.Rate(50)  # 50Hz
        
        # 设置偏移
        self.offset = {
            'x': -0.043,
            'y': -0.051,
            'z': 0.05,  # 0.107 - 0.057 (link8到末端的距离加上模型的偏移)
            'roll': 0,
            'pitch': 0,
            'yaw': 0
        }
        
        rospy.loginfo("Probe controller initialized. Following link: %s", self.end_effector_link)
    
    def update_probe_position(self):
        try:
            # 获取机器人末端执行器的变换
            (trans, rot) = self.tf_listener.lookupTransform('/world', self.end_effector_link, rospy.Time(0))
            
            # 创建模型状态消息
            model_state = ModelState()
            model_state.model_name = self.probe_model_name
            model_state.reference_frame = 'world'
            
            # 设置位置
            model_state.pose.position.x = trans[0]
            model_state.pose.position.y = trans[1]
            model_state.pose.position.z = trans[2]
            
            # 设置方向
            model_state.pose.orientation.x = rot[0]
            model_state.pose.orientation.y = rot[1]
            model_state.pose.orientation.z = rot[2]
            model_state.pose.orientation.w = rot[3]
            
            # 调用服务更新探头位置
            self.set_model_state(model_state)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF Error: %s", str(e))
    
    def run(self):
        rospy.loginfo("Starting probe controller...")
        while not rospy.is_shutdown():
            self.update_probe_position()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ProbeController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 