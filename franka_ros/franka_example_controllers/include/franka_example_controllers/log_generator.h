// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <fstream>
#include <string>
#include <deque>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_example_controllers/soft_contact_model.h>
#include <franka_example_controllers/force_data_visualizer.h>

namespace franka_example_controllers {

/**
 * @brief 实验数据记录类
 * 
 * 负责日志文件的创建、数据写入、管理和力曲线绘制
 */
class LogGenerator {
public:
  /**
   * @brief 构造函数
   */
  LogGenerator() : log_initialized_(false), 
                  current_depth_(0.0),
                  log_counter_(0),
                  soft_block_surface_z_(0.0),
                  is_in_contact_(false),
                  total_updates_(0),
                  contact_detected_count_(0) {}

  /**
   * @brief 析构函数
   */
  ~LogGenerator();

  //==========================================================================
  // 核心日志功能
  //==========================================================================
  
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
   * @brief 记录数据
   * @param time 时间
   * @param position 位置
   * @param force 力
   * @param phase 控制阶段
   * @param soft_block_z 软体块表面高度
   * @param theoretical_force 理论力
   * @param probe_length 探头长度，用于计算探头末端位置
   * @param contact_reference_z 接触参考Z坐标（首次接触时探头末端Z坐标）
   * @param raw_force_z 未经校准的原始Z轴力值
   */
  void logData(const ros::Time& time, 
               const Eigen::Vector3d& position, 
               const Eigen::Vector3d& force,
               int phase,
               double soft_block_z,
              double theoretical_force = 0.0,
              double probe_length = 0.0,
              double contact_reference_z = 0.0,
              double raw_force_z = 0.0);
              
  /**
   * @brief 设置当前深度（仅用于日志记录）
   * @param depth 深度值
   */
  void setCurrentDepth(double depth);
  
  /**
   * @brief 获取是否记录了接触状态
   * @return 记录的接触状态
   */
  bool getRecordedContact() const { return is_in_contact_; }
              
private:
  // 获取格式化时间字符串
  std::string getFormattedTime();
  
  // 日志文件相关变量
  std::ofstream log_file_;
  std::string log_file_path_;
  bool log_initialized_;
  ros::Time start_time_;
  int log_counter_;
  double current_depth_;

  // 接触状态（仅用于日志）
  bool is_in_contact_;
  double soft_block_surface_z_;

  // 统计数据
  int total_updates_;
  int contact_detected_count_;

  // 添加最后记录时间
  ros::Time last_log_time_;
  
  // 力曲线可视化器
  std::unique_ptr<ForceDataVisualizer> force_visualizer_;
};

}  // namespace franka_example_controllers 