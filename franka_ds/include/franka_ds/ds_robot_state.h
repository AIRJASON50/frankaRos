// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <franka/robot_state.h>
#include <Eigen/Dense>

namespace franka_ds {

/**
 * @brief 统一的机器人状态管理器
 * 
 * 整合所有状态变量，消除重复代码：
 * - 机器人位置、速度、姿态
 * - 力传感器数据和滤波
 * - 接触检测和表面估计
 * - 初始化状态管理
 */
class DSRobotState {
public:
  DSRobotState();
  ~DSRobotState() = default;
  
  // 更新机器人状态
  void updateRobotPose(const franka::RobotState& robot_state);
  void updateForceReading(const geometry_msgs::WrenchStamped& msg);
  void updateState(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                   const Eigen::Vector3d& velocity, const Eigen::Vector3d& angular_velocity);
  
  // 获取状态信息
  bool isInitialized() const { return is_initialized_; }
  bool isInContact() const { return is_in_contact_; }
  
  // 获取位置、速度和方向
  Eigen::Vector3d getPosition() const { return position_; }
  Eigen::Vector3d getCurrentPosition() const { return position_; }  // 别名方法
  Eigen::Vector3d getVelocity() const { return velocity_; }
  Eigen::Quaterniond getOrientation() const { return orientation_; }
  Eigen::Vector3d getAngularVelocity() const { return angular_velocity_; }
  
  // 获取力传感器数据
  Eigen::Vector3d getContactForce() const { return contact_force_; }
  double getNormalForce() const { return normal_force_; }
  Eigen::Vector3d getSurfaceNormal() const { return surface_normal_; }
  
  // 设置参数
  void setContactForceThreshold(double threshold) { contact_force_threshold_ = threshold; }
  void setForceBiasSamples(int samples) { force_bias_samples_ = samples; }
  
  // 重置力传感器校准
  void resetForceBias() {
    is_calibrating_ = true;
    force_bias_buffer_.clear();
  }
  
private:
  // ========== 状态标志 ==========
  bool is_initialized_;                     // 是否已初始化
  
  // ========== 位置和方向 ==========
  Eigen::Vector3d position_;                // 末端位置
  Eigen::Vector3d velocity_;                // 末端速度
  Eigen::Quaterniond orientation_;          // 末端方向（四元数）
  Eigen::Vector3d angular_velocity_;        // 末端角速度
  
  // ========== 力传感器数据 ==========
  Eigen::Vector3d contact_force_;                // 接触力向量
  double normal_force_;                          // 法向力大小
  
  // 力传感器相关
  bool is_calibrating_;                     // 是否正在校准力传感器
  std::vector<Eigen::Matrix<double, 6, 1>> force_bias_buffer_;  // 力传感器零偏计算缓冲区
  int force_bias_samples_;                  // 力传感器校准样本数
  Eigen::Matrix<double, 6, 1> wrench_bias_;             // 力传感器零偏
  Eigen::Matrix<double, 6, 1> filtered_wrench_;         // 处理后的力扭矩数据
  
  // ========== 接触状态 ==========
  bool is_in_contact_;                      // 是否处于接触状态
  double contact_force_threshold_;          // 接触力阈值
  Eigen::Vector3d surface_normal_;          // 接触面法向量
  
  // ========== 私有方法 ==========
  void updateContactDetection();            // 更新接触检测
  void estimateSurfaceNormal();             // 估计表面法向量
};

}  // namespace franka_ds 