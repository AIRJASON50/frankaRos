// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/ds_robot_state.h>
#include <franka_ds/ds_utils.h>
#include <ros/ros.h>
#include <cmath>

namespace franka_ds {

DSRobotState::DSRobotState()
    : is_initialized_(false),
      is_calibrating_(true),
      is_in_contact_(false),
      force_bias_samples_(100),
      contact_force_threshold_(0.3),
      position_(Eigen::Vector3d::Zero()),
      velocity_(Eigen::Vector3d::Zero()),
      orientation_(Eigen::Quaterniond::Identity()),
      angular_velocity_(Eigen::Vector3d::Zero()),
      wrench_bias_(Eigen::Matrix<double, 6, 1>::Zero()),
      filtered_wrench_(Eigen::Matrix<double, 6, 1>::Zero()),
      contact_force_(Eigen::Vector3d::Zero()),
      normal_force_(0.0),
      surface_normal_(Eigen::Vector3d(0, 0, 1)) {
  // 初始化完成
}

void DSRobotState::updateRobotPose(const franka::RobotState& robot_state) {
  // 提取位置
  position_ = Eigen::Vector3d(
      robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);
  
  // 提取旋转矩阵并转换为四元数
  Eigen::Matrix3d rotation;
  rotation << robot_state.O_T_EE[0], robot_state.O_T_EE[4], robot_state.O_T_EE[8],
             robot_state.O_T_EE[1], robot_state.O_T_EE[5], robot_state.O_T_EE[9],
             robot_state.O_T_EE[2], robot_state.O_T_EE[6], robot_state.O_T_EE[10];
  orientation_ = Eigen::Quaterniond(rotation);
  
  // 提取速度
  velocity_ = Eigen::Vector3d(
      robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1], robot_state.O_dP_EE_c[2]);
      
  // 提取角速度
  angular_velocity_ = Eigen::Vector3d(
      robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4], robot_state.O_dP_EE_c[5]);
      
  // 标记为已初始化
  if (!is_initialized_ && !is_calibrating_) {
    is_initialized_ = true;
  }
}

void DSRobotState::updateForceReading(const geometry_msgs::WrenchStamped& msg) {
  // 获取原始力传感器数据
  Eigen::Matrix<double, 6, 1> raw_wrench;
  raw_wrench << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  
  if (is_calibrating_) {
    // 校准阶段：收集数据计算零偏
    force_bias_buffer_.push_back(raw_wrench);
    
    if (force_bias_buffer_.size() >= force_bias_samples_) {
      // 计算平均零偏
      wrench_bias_ = Eigen::Matrix<double, 6, 1>::Zero();
      for (const auto& sample : force_bias_buffer_) {
        wrench_bias_ += sample;
      }
      wrench_bias_ /= force_bias_buffer_.size();
      
      // 校准完成
      is_calibrating_ = false;
      is_initialized_ = true;
      
      ROS_INFO("Force sensor bias calibration complete: [%.3f, %.3f, %.3f]N",
               wrench_bias_(0), wrench_bias_(1), wrench_bias_(2));
    }
  } else {
    // 正常运行阶段：直接使用裸读数减去零偏
    filtered_wrench_ = raw_wrench - wrench_bias_;
    
    // 更新接触检测
    updateContactDetection();
  }
}

void DSRobotState::updateState(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                               const Eigen::Vector3d& velocity, const Eigen::Vector3d& angular_velocity) {
  position_ = position;
  orientation_ = orientation;
  velocity_ = velocity;
  angular_velocity_ = angular_velocity;
}

void DSRobotState::updateContactDetection() {
  // 使用Z轴力（假设Z轴朝下为正方向）作为接触检测依据
  double force_z = filtered_wrench_(2);  // Z轴力
  normal_force_ = force_z;  // 更新法向力

  // 提取接触力向量（前3个元素是力）
  contact_force_ << filtered_wrench_(0), filtered_wrench_(1), filtered_wrench_(2);

  // 检测接触状态变化
  bool was_in_contact = is_in_contact_;
  is_in_contact_ = (std::abs(force_z) > contact_force_threshold_);

  // 输出接触状态变化信息（添加时间间隔限制，避免频繁输出）
  static ros::Time last_contact_log_time = ros::Time::now();
  if (is_in_contact_ && !was_in_contact) {
    // 只有当距离上次日志超过1秒时才输出
    if ((ros::Time::now() - last_contact_log_time).toSec() > 1.0) {
      ROS_INFO("Contact detected: Force_Z=%.3fN > Threshold %.3fN",
               force_z, contact_force_threshold_);
      last_contact_log_time = ros::Time::now();
    }
  } else if (!is_in_contact_ && was_in_contact) {
    // 只有当距离上次日志超过1秒时才输出
    if ((ros::Time::now() - last_contact_log_time).toSec() > 1.0) {
      ROS_INFO("Contact lost: Force_Z=%.3fN < Threshold %.3fN",
               force_z, contact_force_threshold_);
      last_contact_log_time = ros::Time::now();
    }
  }
}

void DSRobotState::estimateSurfaceNormal() {
  // 简单实现：假设接触力方向与表面法向量相反
  if (contact_force_.norm() > 1e-6) {
    surface_normal_ = -contact_force_.normalized();
  }
}

}  // namespace franka_ds 