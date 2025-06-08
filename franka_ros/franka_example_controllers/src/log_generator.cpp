/**
 * @file log_generator.cpp
 * @brief 实验数据记录工具的实现
 *
 * @架构：
 * - LogGenerator类：处理日志文件创建、数据记录和基本处理
 * - ForceDataVisualizer类：处理rqt力曲线绘制
 *
 * @数据流：
 * 输入：实验数据（时间、位置、力、控制阶段等） -> 
 * 处理：数据格式化、批量处理、力曲线绘制 -> 
 * 输出：CSV格式的实验日志文件和rqt力曲线图
 *
 * @功能分类：
 * 1. 核心日志功能：负责文件创建、数据写入和日志管理
 * 2. 力曲线可视化：实时绘制force_z和theoretical_force对比图
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
#include <std_msgs/Float64.h>

namespace franka_example_controllers {

//==============================================================================
// LogGenerator类实现
//==============================================================================

// LogGenerator析构函数实现 - 必须在ForceDataVisualizer完整定义后
LogGenerator::~LogGenerator() {
  // 确保日志文件正确关闭
  if (log_file_.is_open()) {
    log_file_.close();
  }
  // force_visualizer_ 会自动析构，因为这里ForceDataVisualizer类已经完全定义
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
 * @brief 初始化日志文件，创建CSV格式的数据记录文件，并启动力曲线绘制
 * 
 * 日志文件包含实验时间、控制阶段、机器人位置、接触力、深度等数据
 */
void LogGenerator::initLogFile(const ContactParams& contact_params, double target_force) {
  if (log_initialized_) {
    return;
  }
  
  try {
    // 生成文件名
    std::string time_str = getFormattedTime();
    
    // 修改日志文件路径为franka_gazebo/logs目录
    std::string log_dir = ros::package::getPath("franka_gazebo") + "/logs";
    log_file_path_ = log_dir + "/force_data_" + time_str + ".csv";
    
    // 确保logs目录存在
    system(("mkdir -p " + log_dir).c_str());
    
    start_time_ = ros::Time::now();
    
    // 初始化力曲线可视化器
    if (!force_visualizer_) {
      // 创建持久的节点句柄用于发布数据
      static ros::NodeHandle visualization_nh; 
      force_visualizer_ = std::make_unique<ForceDataVisualizer>(visualization_nh);
    }
    
    // 打开日志文件
    log_file_.open(log_file_path_);
    
    if (log_file_.is_open()) {
      // 写入CSV头和实验参数
      log_file_ << "# Experiment start time: " << std::fixed << std::setprecision(6) 
                << start_time_.toSec() << " seconds" << std::endl;
      log_file_ << "# Local time: " << time_str << std::endl;
      log_file_ << "# Control phase description: 0=CALIBRATION, 1=APPROACH, 2=CONTACT_CONFIRMATION, 3=DEPTH_CONTROL, 4=TRAJECTORY" << std::endl;
      log_file_ << "# Soft block physical properties: Young's modulus=" << contact_params.young_modulus 
                << "Pa, Poisson ratio=" << contact_params.poisson_ratio << std::endl;
      log_file_ << "# Target force: " << target_force << "N" << std::endl;
      log_file_ << "# Contact model parameters: Radius=" << contact_params.contact_radius 
                << "m, Depth threshold=" << contact_params.depth_threshold << "m" << std::endl;
      log_file_ << "# Logging frequency: 0.05 seconds" << std::endl;
      log_file_ << "# Force plot topics: /force_plot/force_z, /force_plot/theoretical_force" << std::endl;
      log_file_ << "# ---------------------------------------" << std::endl;
      
      // 写入CSV头字段 - 删除target_force和contact_state字段
      log_file_ << "time,phase,pos_z,force_x,force_y,force_z,raw_force_z,force_magnitude,"
                << "depth,surface_z,theoretical_force" << std::endl;
      
      log_initialized_ = true;
      log_counter_ = 0;
      last_log_time_ = start_time_;
      ROS_INFO_STREAM("Log file created: " << log_file_path_);
      ROS_INFO("Force plotting enabled - use 'rqt_plot /force_plot/force_z /force_plot/theoretical_force' to view");
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
 * @brief 记录数据到日志文件并绘制力曲线
 * 
 * @param time 当前时间
 * @param position 机器人末端执行器位置
 * @param force 接触力向量（已调零的值）
 * @param phase 控制阶段(0=APPROACH, 1=CONTACT, 2=TRAJECTORY)
 * @param soft_block_z 软体块表面z坐标
 * @param theoretical_force 理论力（用于对比）
 * @param probe_length 探头长度，用于计算探头末端位置
 * @param contact_reference_z 接触参考Z坐标（首次接触时探头末端Z坐标）
 * @param raw_force_z 未经校准的原始Z轴力值
 * 
 * 记录的数据包括时间、阶段、探头末端位置、力、力大小、深度、理论力
 * 为优化性能，数据每固定时间间隔写入一次，同时实时绘制力曲线
 */
void LogGenerator::logData(const ros::Time& time, 
                          const Eigen::Vector3d& position, 
                          const Eigen::Vector3d& force, 
                          int phase,
                          double soft_block_z,
                          double theoretical_force,
                          double probe_length,
                          double contact_reference_z,
                          double raw_force_z) {
  if (!log_initialized_ || !log_file_.is_open()) {
    return;
  }
  
  // 发布力曲线数据用于rqt绘图 - 确保安全调用
  if (force_visualizer_) {
    force_visualizer_->publishForceData(force(2), theoretical_force);
  }
  
  // 检查是否需要记录（每0.05秒记录一次）
  double elapsed = (time - last_log_time_).toSec();
  if (elapsed < 0.05) {
    return;
  }
  
  // 保存软体块表面高度
  soft_block_surface_z_ = soft_block_z;
  
    // 计算力大小
      double force_magnitude = force.norm();
      
    // 计算探头末端Z坐标 = 法兰Z坐标 - 探头长度
    double probe_tip_z = position(2) - probe_length;
    
    // 检查是否有一个有效的接触参考点
    bool has_valid_reference = (contact_reference_z > 0.0);
    
  // 计算深度值
    double reported_depth = 0.0;
  if (has_valid_reference) {
    // 使用与控制器一致的深度计算方法：接触参考点减去探头末端位置
    reported_depth = contact_reference_z - probe_tip_z;

      // 记录深度计算调试信息（每100帧记录一次）
      static int debug_counter = 0;
      if (debug_counter++ % 100 == 0) {
      ROS_DEBUG("Log: 深度计算: 接触参考点=%.4f, 探头位置=%.4f, 深度=%.4f mm", 
               contact_reference_z, probe_tip_z, reported_depth * 1000.0);
      }
    } else {
      // 未接触状态下使用内部深度值
      reported_depth = current_depth_;
    }
    
  try {
    // 写入数据 - 删除target_force和contact_state字段
      log_file_ << std::fixed << std::setprecision(6)
                << time.toSec() << ","
                << phase << ","
              << probe_tip_z << ","  // 仅输出探头末端Z坐标
                << force(0) << ","
                << force(1) << ","
                << force(2) << ","
              << raw_force_z << ","  // 添加原始Z轴力值
                << force_magnitude << ","
              << reported_depth << ","  // 使用计算的深度值
              << contact_reference_z << ","  // 使用接触参考Z坐标代替软块表面Z坐标
              << theoretical_force
                << std::endl;
      
    // 更新计数器和最后记录时间
    log_counter_++;
    last_log_time_ = time;
    
    // 每50条记录刷新一次文件 (降低刷新频率，减少文件I/O)
    if (log_counter_ % 50 == 0) {
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

//==============================================================================
// 核心日志功能：时间格式化和文件管理
//==============================================================================
