// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <franka_example_controllers/soft_contact_model.h>

namespace franka_example_controllers {

/**
 * @brief 简化的数据记录类 - 仅负责数据记录功能
 */
class LogGenerator {
public:
  /**
   * @brief 构造函数
   */
  LogGenerator() : log_initialized_(false), 
                  current_depth_(0.0),
                  log_counter_(0) {}

  /**
   * @brief 析构函数
   */
  ~LogGenerator();

  /**
   * @brief 初始化日志文件
   * @param contact_params 接触参数
   * @param target_force 目标力
   */
  void initLogFile(const ContactParams& contact_params, double target_force);
  
  /**
   * @brief 关闭日志文件
   * @param end_time 结束时间
   */
  void closeLogFile(const ros::Time& end_time);
  
  /**
   * @brief 记录数据到日志文件
   * @param time 时间
   * @param position 位置
   * @param force 力
   * @param phase 控制阶段
   * @param soft_block_z 软体块表面高度（未使用，保持接口兼容）
   * @param theoretical_force 理论力（未使用，保持接口兼容）
   * @param probe_length 探头长度（未使用，保持接口兼容）
   * @param contact_reference_z 接触参考Z坐标（未使用，保持接口兼容）
   * @param raw_force_z 未经校准的原始Z轴力值
   * @param energy_level 当前能量罐水平 (J)
   * @param energy_scale_factor 当前能量缩放因子 [0.0, 1.0]
   */
  void logData(const ros::Time& time, 
               const Eigen::Vector3d& position, 
               const Eigen::Vector3d& force,
               int phase,
               double soft_block_z,
               double theoretical_force = 0.0,
               double probe_length = 0.0,
               double contact_reference_z = 0.0,
               double raw_force_z = 0.0,
               double energy_level = 0.0,
               double energy_scale_factor = 1.0);
              
  /**
   * @brief 设置当前深度（保持接口兼容）
   * @param depth 深度值
   */
  void setCurrentDepth(double depth);
              
private:
  // 获取格式化时间字符串
  std::string getFormattedTime();
  
  // 日志文件相关变量
  std::ofstream log_file_;
  std::string log_file_path_;
  bool log_initialized_;
  ros::Time start_time_;
  ros::Time last_log_time_;
  int log_counter_;
  double current_depth_;
};

}  // namespace franka_example_controllers 