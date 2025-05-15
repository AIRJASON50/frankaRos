// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <random>
#include <ros/node_handle.h>
#include <Eigen/Dense>

namespace franka_example_controllers {

// 力轨迹类型枚举
enum ForceProfileType {
  FORCE_CONSTANT = 0, // 恒定力
  FORCE_SINE = 1,     // 正弦变化力
  FORCE_STEP = 2      // 阶跃力
};

class ForceGenerator {
 public:
  /**
   * @brief 默认构造函数
   */
  ForceGenerator();

  /**
   * @brief 初始化力轨迹生成器
   * 
   * @param node_handle ROS节点句柄，用于读取参数
   * @return true 如果初始化成功
   * @return false 如果初始化失败
   */
  bool init(ros::NodeHandle& node_handle);

  /**
   * @brief 生成给定时间的力轨迹
   * 
   * @param time 当前时间（秒）
   * @return double 生成的力值（牛顿）
   */
  double generateForceProfile(double time);

  /**
   * @brief 生成力噪声向量
   * 
   * @return Eigen::Vector3d 三维噪声向量
   */
  Eigen::Vector3d generateForceNoise();

  /**
   * @brief 设置基准力
   * 
   * @param base_force 基准力（牛顿）
   */
  void setBaseForce(double base_force);

  /**
   * @brief 获取当前力轨迹类型
   * 
   * @return ForceProfileType 力轨迹类型
   */
  ForceProfileType getForceProfileType() const;

  /**
   * @brief 获取力轨迹类型名称
   * 
   * @return std::string 力轨迹类型名称
   */
  std::string getForceProfileTypeStr() const;

  /**
   * @brief 启用或禁用力噪声
   * 
   * @param enable 是否启用噪声
   */
  void enableForceNoise(bool enable);

 private:
  // 力轨迹参数
  ForceProfileType force_profile_type_;  // 力轨迹类型
  std::string force_profile_type_str_;  // 力轨迹类型名称
  double base_force_;  // 基准力（牛顿）
  double force_sine_amplitude_;  // 正弦力振幅（牛顿）
  double force_sine_frequency_;  // 正弦力频率（赫兹）
  double force_step_time_;  // 阶跃力时间点（秒）
  double force_step_value_;  // 阶跃力目标值（牛顿）

  // 力噪声参数
  bool force_noise_enable_;  // 是否启用力噪声
  double force_noise_min_;  // 力噪声最小值
  double force_noise_max_;  // 力噪声最大值
  
  // 随机数生成器
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> noise_distribution_;
};

}  // namespace franka_example_controllers 