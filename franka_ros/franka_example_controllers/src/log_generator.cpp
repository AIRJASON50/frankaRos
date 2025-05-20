/**
 * @file log_generator.cpp
 * @brief 实验数据记录工具的实现
 *
 * @架构：
 * - LogGenerator类：处理日志文件创建、数据记录和基本处理
 *
 * @数据流：
 * 输入：实验数据（时间、位置、力、控制阶段等） -> 
 * 处理：数据格式化、批量处理 -> 
 * 输出：CSV格式的实验日志文件
 *
 * @功能分类：
 * 1. 核心日志功能：负责文件创建、数据写入和日志管理
 */
// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/log_generator.h>

#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <deque>
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace franka_example_controllers {

//==============================================================================
// 核心日志功能：时间格式化和文件管理
//==============================================================================

/**
 * @brief 获取格式化的当前时间字符串用于日志文件命名
 * @return 格式为YYYYMMDD_HHMMSS的时间字符串
 */
std::string LogGenerator::getFormattedTime() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  char buffer[80];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&time_t_now));
  return std::string(buffer);
}

/**
 * @brief 初始化日志文件，创建CSV格式的数据记录文件
 * 
 * 日志文件包含实验时间、控制阶段、机器人位置、接触力、深度等数据
 */
void LogGenerator::initLogFile(const ContactParams& contact_params, double target_force) {
  if (log_initialized_) {
    return;
  }
  
  try {
    // 记录开始时间
    start_time_ = ros::Time::now();
    
    // 获取日志文件路径，使用格式化时间作为文件名
    std::string package_path = ros::package::getPath("franka_gazebo");
    std::string time_str = getFormattedTime();
    log_file_path_ = package_path + "/logs/force_data_" + time_str + ".csv";
    
    // 创建日志目录（如果不存在）
    std::string log_dir = package_path + "/logs";
    std::string cmd = "mkdir -p " + log_dir;
    if (system(cmd.c_str()) != 0) {
      ROS_ERROR_STREAM("Failed to create log directory: " << log_dir);
      return;
    }
    
    // 打开日志文件
    log_file_.open(log_file_path_);
    
    if (log_file_.is_open()) {
      // 写入CSV头和实验参数
      log_file_ << "# Experiment start time: " << std::fixed << std::setprecision(6) 
                << start_time_.toSec() << " seconds" << std::endl;
      log_file_ << "# Local time: " << time_str << std::endl;
      log_file_ << "# Control phase description: 0=APPROACH, 1=CONTACT, 2=TRAJECTORY" << std::endl;
      log_file_ << "# Soft block physical properties: Young's modulus=" << contact_params.young_modulus 
                << "Pa, Poisson ratio=" << contact_params.poisson_ratio << std::endl;
      log_file_ << "# Target force: " << target_force << "N" << std::endl;
      log_file_ << "# Contact model parameters: Radius=" << contact_params.contact_radius 
                << "m, Depth threshold=" << contact_params.depth_threshold << "m" << std::endl;
      log_file_ << "# Logging frequency: 0.2 seconds" << std::endl;
      log_file_ << "# ---------------------------------------" << std::endl;
      
      // 写入CSV头字段 - 简化合并的字段
      log_file_ << "time,phase,pos_x,pos_y,pos_z,force_x,force_y,force_z,force_magnitude,"
                << "depth,target_force,theoretical_force,contact_state" << std::endl;
      
      log_initialized_ = true;
      log_counter_ = 0;
      last_log_time_ = start_time_;
      ROS_INFO_STREAM("Log file created: " << log_file_path_);
    } else {
      ROS_ERROR_STREAM("Cannot open log file: " << log_file_path_);
    }
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Error initializing log file: " << ex.what());
  }
}

/**
 * @brief 关闭日志文件，记录实验结束时间和摘要信息
 * @param end_time 实验结束时间
 */
void LogGenerator::closeLogFile(const ros::Time& end_time) {
  if (log_initialized_ && log_file_.is_open()) {
    try {
      // 计算实验总持续时间
      double duration = (end_time - start_time_).toSec();
      
      // 写入结束时间和摘要信息
      log_file_ << "# ---------------------------------------" << std::endl;
      log_file_ << "# Experiment end time: " << std::fixed << std::setprecision(6) 
                << end_time.toSec() << " seconds" << std::endl;
      log_file_ << "# Experiment total duration: " << duration << " seconds" << std::endl;
      log_file_ << "# Number of data points recorded: " << log_counter_ << std::endl;
      log_file_ << "# Contact detection rate: " << std::fixed << std::setprecision(2) 
                << (total_updates_ > 0 ? (contact_detected_count_ * 100.0 / total_updates_) : 0.0)
                << "% (" << contact_detected_count_ << "/" << total_updates_ << ")";
      
      // 关闭文件
      log_file_.close();
      ROS_INFO_STREAM("Log file closed: " << log_file_path_);
      ROS_INFO_STREAM("Experiment duration: " << duration << " seconds");
      ROS_INFO_STREAM("Recorded data points: " << log_counter_);
      ROS_INFO_STREAM("Contact detection rate: " << std::fixed << std::setprecision(2) 
                     << (total_updates_ > 0 ? (contact_detected_count_ * 100.0 / total_updates_) : 0.0)
                     << "% (" << contact_detected_count_ << "/" << total_updates_ << ")");
    } catch (const std::exception& ex) {
      ROS_ERROR_STREAM("Error closing log file: " << ex.what());
    }
  }
}

/**
 * @brief 记录数据到日志文件
 * 
 * @param time 当前时间
 * @param position 机器人末端执行器位置
 * @param force 接触力向量
 * @param phase 控制阶段(0=APPROACH, 1=CONTACT, 2=TRAJECTORY)
 * @param soft_block_z 软体块表面z坐标
 * @param target_force 目标力
 * @param theoretical_force 理论力（可选）
 * @param contact_state 接触状态(1=接触, 0=未接触)
 * 
 * 记录的数据包括时间、阶段、位置、力、力大小、深度、目标力和理论力
 * 为优化性能，数据每10次迭代写入一次，文件每100条记录刷新一次
 */
void LogGenerator::logData(const ros::Time& time, 
                          const Eigen::Vector3d& position, 
                          const Eigen::Vector3d& force, 
                          int phase,
                          double soft_block_z,
                          double target_force,
                          double theoretical_force,
                          int contact_state) {
  if (!log_initialized_ || !log_file_.is_open()) {
    return;
  }
  
  // 检查是否需要记录（每0.2秒记录一次）
  double elapsed = (time - last_log_time_).toSec();
  if (elapsed < 0.2) {
    return;
  }
  
  // 保存软体块表面高度
  soft_block_surface_z_ = soft_block_z;
  
  // 更新内部接触状态（用于统计）
  is_in_contact_ = (contact_state == 1);
  total_updates_++;
  if (is_in_contact_) {
    contact_detected_count_++;
  }
  
  try {
    // 计算力大小
    double force_magnitude = force.norm();
    
    // 计算与软块表面的相对距离，正值表示位于表面之下(接触深度)，负值表示位于表面之上
    double relative_distance = is_in_contact_ ? current_depth_ : -(position(2) - soft_block_z);
    
    // 写入数据
    log_file_ << std::fixed << std::setprecision(6)
              << time.toSec() << ","
              << phase << ","
              << position(0) << ","
              << position(1) << ","
              << position(2) << ","
              << force(0) << ","
              << force(1) << ","
              << force(2) << ","
              << force_magnitude << ","
              << current_depth_ << ","
              << target_force << ","
              << theoretical_force << ","
              << contact_state
              << std::endl;
    
    // 更新计数器和最后记录时间
    log_counter_++;
    last_log_time_ = time;
    
    // 每100条记录刷新一次文件
    if (log_counter_ % 100 == 0) {
      log_file_.flush();
    }
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Error recording data to log file: " << ex.what());
  }
}

/**
 * @brief 设置当前接触深度值（供外部调用）
 * @param depth 接触深度
 */
void LogGenerator::setCurrentDepth(double depth) {
  current_depth_ = depth;
  
  // 如果深度大于阈值，更新接触状态（仅用于日志）
  if (depth > 0.001) {  // 1mm
    is_in_contact_ = true;
    total_updates_++;
    contact_detected_count_++;
  } else {
    is_in_contact_ = false;
    total_updates_++;
  }
}

}  // namespace franka_example_controllers
