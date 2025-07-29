// Copyright (c) 2024 Franka DS Project
// Use of this source code is governed by the Apache-2.0 license

#include <franka_ds/energy_tank_manager.h>
#include <ros/ros.h>
#include <cmath>
#include <algorithm>

namespace franka_ds {

EnergyTankManager::EnergyTankManager(double max_energy) 
  : s_max_(max_energy), s_(max_energy * 0.8), power_flow_(0.0),
    p_d_(0.0), p_n_(0.0), p_f_(0.0),
    alpha_(DEFAULT_ALPHA), beta_(DEFAULT_BETA), gamma_(DEFAULT_GAMMA), gamma_p_(DEFAULT_GAMMA_P),
    d1_(DEFAULT_D1), is_enabled_(true), is_initialized_(true) {
  
  ROS_INFO("EnergyTankManager initialized: Max energy=%.2fJ, Initial energy=%.2fJ", s_max_, s_);
}

void EnergyTankManager::updateTankDynamics(const Eigen::Vector3d& current_velocity,
                                          const Eigen::Vector3d& nominal_ds_velocity,
                                          const Eigen::Matrix3d& damping_matrix,
                                          double desired_force,
                                          const Eigen::Vector3d& surface_normal,
                                          double la, // 新增参数
                                          double dt) {
  if (!is_enabled_ || dt <= 0.0) {
    return;
  }
  
  // Calculate power components
  computeDissipativePower(current_velocity, damping_matrix);
  computeNominalPower(current_velocity, nominal_ds_velocity);
  computeForcePower(current_velocity, desired_force, surface_normal);
  
  // Update scalar regulation functions
  updateScalarFunctions();
  
  // *** 核心修改：能量罐动力学：ṡ = α*p_d - β*(la-1.0f)*p_n - γ*p_f ***
  const double energy_derivative = alpha_ * p_d_ - beta_ * (la - 1.0) * p_n_ - gamma_ * p_f_;
  power_flow_ = energy_derivative;
  
  // Update energy level
  s_ = std::max(0.0, std::min(s_ + energy_derivative * dt, s_max_));
  
  // Debug output (low frequency)
  static int debug_counter = 0;
  if (++debug_counter % 100 == 0) {  // Output once every 100 updates
    ROS_DEBUG("Energy tank status: s=%.3fJ, ṡ=%.3fW, p_d=%.3f, p_n=%.3f, p_f=%.3f", 
              s_, power_flow_, p_d_, p_n_, p_f_);
  }
}

Eigen::Vector3d EnergyTankManager::constrainVelocity(const Eigen::Vector3d& desired_velocity) {
  if (!is_enabled_) {
    return desired_velocity;
  }
  
  // Check passivity constraint
  if (isViolatingPassivity()) {
    // Passivity violation: apply safety strategy
    return computeDissipativeVelocity(desired_velocity);
  }
  
  // Scale velocity based on energy level
  const double energy_scale = getEnergyScaleFactor();
  return energy_scale * desired_velocity;
}

bool EnergyTankManager::isMotionAllowed(double required_power) const {
  if (!is_enabled_) {
    return true;
  }
  
  // If required power is negative (dissipative), always allow
  if (required_power <= 0.0) {
    return true;
  }
  
  // Check if there's enough energy to support motion
  return (s_ > ENERGY_EPSILON) && (required_power <= s_ / 0.1);  // Assume 0.1 second time window
}

double EnergyTankManager::getEnergyScaleFactor() const {
  if (!is_enabled_) {
    return 1.0;
  }
  
  const double energy_ratio = getEnergyRatio();
  
  // Smooth scaling function: 1 when energy is sufficient, gradually decreases when energy is low
  if (energy_ratio > MIN_SAFE_ENERGY_RATIO) {
    return 1.0;
  } else {
    // Smooth transition to safe mode
    return energy_ratio / MIN_SAFE_ENERGY_RATIO;
  }
}

bool EnergyTankManager::isViolatingPassivity() const {
  if (!is_enabled_) {
    return false;
  }
  
  // Passivity constraint: violated when energy is near zero and power flow is negative
  return (s_ <= ENERGY_EPSILON) && (power_flow_ < 0.0);
}

void EnergyTankManager::setEnergyParams(double max_energy, double initial_energy) {
  s_max_ = std::max(max_energy, 0.1);  // Minimum 0.1J
  s_ = std::max(0.0, std::min(initial_energy, s_max_));
  
  ROS_INFO("Energy tank parameters updated: Max energy=%.2fJ, Current energy=%.2fJ", s_max_, s_);
}

void EnergyTankManager::setScalarParams(double alpha, double beta, double gamma, double gamma_p) {
  alpha_ = std::max(alpha, 0.0);
  beta_ = std::max(beta, 0.0);
  gamma_ = std::max(gamma, 0.0);
  gamma_p_ = std::max(gamma_p, 0.0);
  
  ROS_INFO("Scalar parameters updated: α=%.2f, β=%.2f, γ=%.2f, γ_p=%.2f", alpha_, beta_, gamma_, gamma_p_);
}

void EnergyTankManager::resetEnergyTank() {
  s_ = s_max_ * 0.8;  // Reset to 80%
  power_flow_ = 0.0;
  p_d_ = p_n_ = p_f_ = 0.0;
  
  ROS_INFO("Energy tank reset complete, current energy=%.2fJ", s_);
}

EnergyTankManager::PowerComponents EnergyTankManager::getPowerComponents() const {
  PowerComponents components;
  components.dissipative_power = p_d_;
  components.nominal_power = p_n_;
  components.force_power = p_f_;
  components.total_power_flow = power_flow_;
  return components;
}

void EnergyTankManager::printStatus() const {
  ROS_INFO("=== Energy Tank Status ===");
  ROS_INFO("Energy level: %.3fJ / %.3fJ (%.1f%%)", s_, s_max_, getEnergyRatio() * 100.0);
  ROS_INFO("Power flow: %.3fW", power_flow_);
  ROS_INFO("Power components: p_d=%.3f, p_n=%.3f, p_f=%.3f", p_d_, p_n_, p_f_);
  ROS_INFO("Scalar functions: α=%.2f, β=%.2f, γ=%.2f, γ_p=%.2f", alpha_, beta_, gamma_, gamma_p_);
  ROS_INFO("Passivity violation: %s", isViolatingPassivity() ? "Yes" : "No");
}

void EnergyTankManager::computeDissipativePower(const Eigen::Vector3d& velocity,
                                               const Eigen::Matrix3d& damping_matrix) {
  // Dissipative power: p_d = v^T D v
  p_d_ = velocity.transpose() * damping_matrix * velocity;
  p_d_ = std::max(p_d_, 0.0);  // Dissipative power should be non-negative
}

void EnergyTankManager::computeNominalPower(const Eigen::Vector3d& velocity,
                                           const Eigen::Vector3d& nominal_ds) {
  // Nominal DS power: p_n = d1 * v^T * f(x)
  p_n_ = d1_ * velocity.dot(nominal_ds);
}

void EnergyTankManager::computeForcePower(const Eigen::Vector3d& velocity,
                                         double desired_force,
                                         const Eigen::Vector3d& surface_normal) {
  // Force feedback power: p_f = F_d * v^T * e1
  const Eigen::Vector3d normal_unit = surface_normal.normalized();
  p_f_ = desired_force * velocity.dot(normal_unit);
}

void EnergyTankManager::updateScalarFunctions() {
  // *** 修复：更精确地模仿SurfacePolishing.cpp中的标量调整逻辑 ***
  const double energy_ratio = getEnergyRatio();
  
  // α 随能量比例平滑调整
  // 能量越低，耗散项权重越高，防止能量耗尽
  alpha_ = DEFAULT_ALPHA * std::max(1.0, 2.0 * (1.0 - energy_ratio)); // 能量低时alpha增大
  
  // β 和 γ 的条件判断 (硬切换)
  // 参考 SurfacePolishing.cpp 的 updateTankScalars()
  
  // β: 标称DS功率系数
  // 如果能量过低且标称功率为负 (表示DS在消耗能量，帮助能量罐恢复)，则 β=0 (关闭DS能量消耗，让能量罐恢复)
  // 如果能量过高且标称功率为正 (表示DS在注入能量，导致能量溢出)，则 β=0 (关闭DS能量注入，防止溢出)
  // 否则 β=1.0 (正常DS能量消耗/注入)
  if (s_ <= ENERGY_EPSILON && p_n_ < 0.0) { // 能量低且DS正在消耗能量 (帮助能量罐恢复)
    beta_ = 0.0; 
  } else if (s_ >= s_max_ - ENERGY_EPSILON && p_n_ > 0.0) { // 能量高且DS正在注入能量 (防止溢出)
    beta_ = 0.0; 
  } else {
    beta_ = DEFAULT_BETA; // 正常情况
  }

  // γ: 力反馈功率系数
  // 如果能量过低且力功率为正 (表示力反馈在注入能量，导致能量溢出)，则 γ=0 (关闭力反馈能量注入)
  // 如果能量过高且力功率为负 (表示力反馈在消耗能量，帮助能量罐恢复)，则 γ=0 (关闭力反馈能量消耗)
  // 否则 γ=1.0 (正常力反馈能量消耗/注入)
  if (s_ <= ENERGY_EPSILON && p_f_ > 0.0) { // 能量低且力反馈正在注入能量 (防止溢出)
    gamma_ = 0.0; 
  } else if (s_ >= s_max_ - ENERGY_EPSILON && p_f_ < 0.0) { // 能量高且力反馈正在消耗能量 (帮助能量罐恢复)
    gamma_ = 0.0; 
  } else {
    gamma_ = DEFAULT_GAMMA; // 正常情况
  }
}

double EnergyTankManager::computeEnergyDerivative() const {
  // Energy tank dynamics: ṡ = α*p_d - β*p_n - γ*p_f
  return alpha_ * p_d_ - beta_ * p_n_ - gamma_ * p_f_;
}

bool EnergyTankManager::checkPassivityConstraint(double energy_derivative) const {
  // Passivity constraint check
  if (s_ <= ENERGY_EPSILON && energy_derivative < 0.0) {
    return false;  // Passivity violated
  }
  return true;  // Passivity maintained
}

Eigen::Vector3d EnergyTankManager::computeDissipativeVelocity(const Eigen::Vector3d& desired_velocity) const {
  // Safety strategy when passivity is violated
  // Project velocity to ensure it doesn't decrease energy further
  
  // Simplified approach: scale down velocity
  const double scale = std::min(s_ / ENERGY_EPSILON, 1.0);
  return scale * desired_velocity;
}

}  // namespace franka_ds 