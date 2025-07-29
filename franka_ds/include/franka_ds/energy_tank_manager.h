// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license
#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>

namespace franka_ds {

/**
 * @brief 能量罐管理器，实现DS系统的无源性保证
 * 
 * 基于论文《接触任务中的运动和力生成：一种动力学系统方法》
 * 实现能量罐机制确保系统无源性和安全性
 * 
 * 核心原理：
 * - 能量罐动力学：ṡ = α*p_d - β*p_n - γ*p_f
 * - 无源性约束：当s≤0且功率流<0时，限制运动
 * - 功率项计算：耗散功率、标称DS功率、力反馈功率
 */
class EnergyTankManager {
 public:
  /**
   * @brief 构造函数
   * @param max_energy 最大能量容量 [J]
   */
  explicit EnergyTankManager(double max_energy = 4.0);
  
  ~EnergyTankManager() = default;

  // ========== 主要接口 ==========
  
  /**
   * @brief 更新能量罐动力学
   * @param current_velocity 当前速度 [m/s]
   * @param nominal_ds_velocity 标称DS速度 [m/s]
   * @param damping_matrix 阻尼矩阵
   * @param desired_force 期望接触力 [N]
   * @param surface_normal 表面法向量
   * @param dt 时间步长 [s]
   */
  void updateTankDynamics(const Eigen::Vector3d& current_velocity,
                         const Eigen::Vector3d& nominal_ds_velocity,
                         const Eigen::Matrix3d& damping_matrix,
                         double desired_force,
                         const Eigen::Vector3d& surface_normal,
                         double la,  // 添加la参数
                         double dt);
  
  /**
   * @brief 应用无源性约束到期望速度
   * @param desired_velocity 期望速度
   * @return 约束后的安全速度
   */
  Eigen::Vector3d constrainVelocity(const Eigen::Vector3d& desired_velocity);
  
  /**
   * @brief 检查是否允许执行指定运动
   * @param required_power 所需功率 [W]
   * @return 是否允许运动
   */
  bool isMotionAllowed(double required_power) const;
  
  // ========== 状态访问 ==========
  
  /**
   * @brief 获取当前能量水平
   * @return 当前能量 [J]
   */
  double getCurrentEnergy() const { return s_; }
  
  /**
   * @brief 获取最大能量容量
   * @return 最大能量 [J]
   */
  double getMaxEnergy() const { return s_max_; }
  
  /**
   * @brief 获取能量比例
   * @return 能量比例 [0,1]
   */
  double getEnergyRatio() const { return s_ / s_max_; }
  
  /**
   * @brief 获取能量缩放因子（用于速度调制）
   * @return 缩放因子 [0,1]
   */
  double getEnergyScaleFactor() const;
  
  /**
   * @brief 获取当前功率流
   * @return 功率流 [W]
   */
  double getCurrentPowerFlow() const { return power_flow_; }
  
  /**
   * @brief 是否违反无源性约束
   * @return 是否违反
   */
  bool isViolatingPassivity() const;
  
  // ========== 配置接口 ==========
  
  /**
   * @brief 设置能量罐参数
   * @param max_energy 最大能量 [J]
   * @param initial_energy 初始能量 [J]
   */
  void setEnergyParams(double max_energy, double initial_energy);
  
  /**
   * @brief 设置标量调节函数参数
   * @param alpha 耗散功率系数
   * @param beta 标称DS功率系数
   * @param gamma 力反馈功率系数
   * @param gamma_p 力调制增益
   */
  void setScalarParams(double alpha, double beta, double gamma, double gamma_p);
  
  /**
   * @brief 设置DS-阻抗增益
   * @param d1 增益参数
   */
  void setDSImpedanceGain(double d1) { d1_ = d1; }
  
  /**
   * @brief 重置能量罐到初始状态
   */
  void resetEnergyTank();
  
  /**
   * @brief 启用/禁用能量罐约束
   * @param enabled 是否启用
   */
  void setEnabled(bool enabled) { is_enabled_ = enabled; }
  
  // ========== 调试接口 ==========
  
  /**
   * @brief 获取功率分量（调试用）
   */
  struct PowerComponents {
    double dissipative_power;  // 耗散功率 p_d
    double nominal_power;      // 标称DS功率 p_n
    double force_power;        // 力反馈功率 p_f
    double total_power_flow;   // 总功率流
  };
  
  PowerComponents getPowerComponents() const;
  
  /**
   * @brief 打印能量罐状态（调试用）
   */
  void printStatus() const;

 private:
  // ========== 能量罐状态变量 ==========
  double s_max_;                // 最大能量容量 [J]
  double s_;                    // 当前能量水平 [J]
  double power_flow_;           // 当前功率流 [W]
  
  // ========== 功率项 ==========
  double p_d_;                  // 耗散功率：v^T D v [W]
  double p_n_;                  // 标称DS功率：d1 * v^T * f(x) [W]
  double p_f_;                  // 力反馈功率：F_d * v^T * e1 [W]
  
  // ========== 标量调节函数 ==========
  double alpha_;                // 耗散功率系数
  double beta_;                 // 标称DS功率系数
  double gamma_;                // 力反馈功率系数
  double gamma_p_;              // 力调制增益
  
  // ========== DS参数 ==========
  double d1_;                   // DS-阻抗增益
  
  // ========== 状态标志 ==========
  bool is_enabled_;             // 是否启用能量罐
  bool is_initialized_;         // 是否已初始化
  
  // ========== 私有方法 ==========
  
  /**
   * @brief 计算耗散功率 p_d = v^T D v
   */
  void computeDissipativePower(const Eigen::Vector3d& velocity,
                              const Eigen::Matrix3d& damping_matrix);
  
  /**
   * @brief 计算标称DS功率 p_n = d1 * v^T * f(x)
   */
  void computeNominalPower(const Eigen::Vector3d& velocity,
                          const Eigen::Vector3d& nominal_ds);
  
  /**
   * @brief 计算力反馈功率 p_f = F_d * v^T * e1
   */
  void computeForcePower(const Eigen::Vector3d& velocity,
                        double desired_force,
                        const Eigen::Vector3d& surface_normal);
  
  /**
   * @brief 更新标量调节函数（动态调节）
   */
  void updateScalarFunctions();
  
  /**
   * @brief 计算能量罐动力学 ṡ = α*p_d - β*p_n - γ*p_f
   */
  double computeEnergyDerivative() const;
  
  /**
   * @brief 应用无源性约束检查
   */
  bool checkPassivityConstraint(double energy_derivative) const;
  
  /**
   * @brief 计算耗散性速度（安全策略）
   */
  Eigen::Vector3d computeDissipativeVelocity(const Eigen::Vector3d& desired_velocity) const;
  
  // ========== 默认参数 ==========
  static constexpr double DEFAULT_MAX_ENERGY = 4.0;        // 默认最大能量 [J]
  static constexpr double DEFAULT_ALPHA = 1.0;             // 默认α系数
  static constexpr double DEFAULT_BETA = 1.0;              // 默认β系数
  static constexpr double DEFAULT_GAMMA = 1.0;             // 默认γ系数
  static constexpr double DEFAULT_GAMMA_P = 10.0;          // 默认γ_p增益
  static constexpr double DEFAULT_D1 = 150.0;                // 默认DS-阻抗增益
  static constexpr double ENERGY_EPSILON = 1e-3;           // 能量接近0的阈值 [J]
  static constexpr double MIN_SAFE_ENERGY_RATIO = 0.2;     // 最小安全能量比例 (20%)
};

}  // namespace franka_ds 