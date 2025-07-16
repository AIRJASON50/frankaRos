/**
 * @file log_generator.cpp
 * @brief 简化的数据记录工具 - 仅负责数据记录，不包含计算和绘图功能
 */
// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/log_generator.h>

#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <ros/package.h>
#include <ros/ros.h>

namespace franka_example_controllers {

/**
 * @brief LogGenerator析构函数
 */
LogGenerator::~LogGenerator() {
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

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
 */
void LogGenerator::initLogFile(const ContactParams& contact_params, double target_force) {
  // ✅ 允许重复初始化，如果已经初始化过则先关闭
  if (log_initialized_ && log_file_.is_open()) {
    ROS_WARN("LogGenerator: Re-initializing - closing previous log file");
    log_file_.close();
  }
  
  try {
    // 重置状态
    log_initialized_ = false;
    log_counter_ = 0;
    
    // 生成文件名
    std::string time_str = getFormattedTime();
    
    // 日志文件路径
    std::string log_dir = ros::package::getPath("franka_example_controllers") + "/data";
    log_file_path_ = log_dir + "/force_data_" + time_str + ".csv";
    
    // 确保data目录存在
    system(("mkdir -p " + log_dir).c_str());
    
    start_time_ = ros::Time::now();
    
    // ✅ 强制打开新的日志文件
    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    
    if (log_file_.is_open()) {
      // 写入CSV头和实验参数
      log_file_ << "# Experiment start time: " << std::fixed << std::setprecision(6) 
                << start_time_.toSec() << " seconds" << std::endl;
      log_file_ << "# Local time: " << time_str << std::endl;
      log_file_ << "# Control phases: 0=CALIBRATION, 4=TRAJECTORY, 5=PAUSE_AT_TARGET, 6=PROBE_DESCENT, 7=PROBE_PAUSE, 8=CIRCULAR_MOTION" << std::endl;
      log_file_ << "# Target force: " << target_force << "N" << std::endl;
      log_file_ << "# Logging frequency: real-time (no artificial limits)" << std::endl;
      log_file_ << "# ---------------------------------------" << std::endl;
      
      // 写入CSV头字段
      log_file_ << "time,phase,pos_x,pos_y,pos_z,force_x,force_y,force_z,raw_force_z,force_magnitude,"
                << "energy_level,energy_scale_factor" << std::endl;
      
      // ✅ 立即刷新确保头信息写入
      log_file_.flush();
      
      log_initialized_ = true;
      last_log_time_ = start_time_;
      ROS_INFO("LogGenerator: Successfully initialized log file: %s", log_file_path_.c_str());
    } else {
      ROS_ERROR("LogGenerator: Failed to open log file: %s", log_file_path_.c_str());
    }
  } catch (const std::exception& ex) {
    ROS_ERROR("LogGenerator: Exception during initialization: %s", ex.what());
  }
}

/**
 * @brief 关闭日志文件，记录实验结束时间和摘要信息
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
      
      // 关闭文件
      log_file_.close();
      ROS_INFO_STREAM("Log file closed: " << log_file_path_);
      ROS_INFO_STREAM("Experiment duration: " << duration << " seconds");
      ROS_INFO_STREAM("Recorded data points: " << log_counter_);
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
 * @param force 接触力向量（已调零的值）
 * @param phase 控制阶段
 * @param soft_block_z 软体块表面z坐标（未使用，保持接口兼容）
 * @param theoretical_force 理论力（未使用，保持接口兼容）
 * @param probe_length 探头长度（未使用，保持接口兼容）
 * @param contact_reference_z 接触参考Z坐标（未使用，保持接口兼容）
 * @param raw_force_z 未经校准的原始Z轴力值
 * @param energy_level 当前能量罐水平 (J)
 * @param energy_scale_factor 当前能量缩放因子 [0.0, 1.0]
 */
void LogGenerator::logData(const ros::Time& time, 
                          const Eigen::Vector3d& position, 
                          const Eigen::Vector3d& force, 
                          int phase,
                          double soft_block_z,
                          double theoretical_force,
                          double probe_length,
                          double contact_reference_z,
                          double raw_force_z,
                          double energy_level,
                          double energy_scale_factor) {
  // ✅ 增强的条件检查和错误诊断
  if (!log_initialized_) {
    static bool init_warning_shown = false;
    if (!init_warning_shown) {
      ROS_WARN("LogGenerator: Attempting to log data but logger not initialized");
      init_warning_shown = true;
    }
    return;
  }
  
  if (!log_file_.is_open()) {
    static bool file_warning_shown = false;
    if (!file_warning_shown) {
      ROS_WARN("LogGenerator: Attempting to log data but file not open");
      file_warning_shown = true;
    }
    return;
  }
  
  try {
    // 计算力大小
    double force_magnitude = force.norm();
    
    // ✅ 写入数据 - 确保立即写入和刷新
    log_file_ << std::fixed << std::setprecision(6)
              << time.toSec() << ","
              << phase << ","
              << position(0) << ","  // x位置
              << position(1) << ","  // y位置
              << position(2) << ","  // z位置
              << force(0) << ","
              << force(1) << ","
              << force(2) << ","
              << raw_force_z << ","
              << force_magnitude << ","
              << energy_level << ","
              << energy_scale_factor
              << std::endl;
    
    // ✅ 立即刷新，确保数据写入磁盘
    log_file_.flush();
    
    // 更新计数器和最后记录时间
    log_counter_++;
    last_log_time_ = time;
    
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Error recording data to log file: " << ex.what());
  }
}

/**
 * @brief 设置当前接触深度值（保持接口兼容，但功能简化）
 */
void LogGenerator::setCurrentDepth(double depth) {
  // 简化实现，仅保存深度值
  current_depth_ = depth;
}

}  // namespace franka_example_controllers
