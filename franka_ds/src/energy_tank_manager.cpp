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
  
  // *** 核心修改：能量罐动力学：ṡ = α*p_d - β*p_n - γ*p_f ***
  const double energy_derivative = alpha_ * p_d_ - beta_ * p_n_ - gamma_ * p_f_;
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
  // 能量比例
  double energy_ratio = s_ / s_max_;
  double delta_ratio = 0.1;  // 固定δ比例，对应论文中的δs
  
  // 论文附录B公式25的Υ函数实现: Υ_{a,b}^-(x) 
  auto upsilon_func = [](double x, double a, double b) -> double {
    if (x < a) return 1.0;
    if (x > b) return 0.0;
    return 0.5 * (1.0 + std::cos(M_PI * (x - a) / (b - a)));
  };
  
  // α(s): 当能量接近满时从1平滑过渡到0
  alpha_ = upsilon_func(energy_ratio, 1.0 - delta_ratio, 1.0);
  
  // β(s,p_n): 根据能量水平和p_n符号动态调节
  if (p_n_ > 0.0) {
    // p_n > 0: DS在消耗能量，当能量低时限制流出
    beta_ = upsilon_func(energy_ratio, 0.0, delta_ratio);
  } else {
    // p_n < 0: 环境在补充能量，当能量高时限制流入
    beta_ = upsilon_func(energy_ratio, 1.0 - delta_ratio, 1.0);
  }
  
  // γ(s,p_f): 根据能量水平和p_f符号动态调节  
  if (p_f_ > 0.0) {
    // p_f > 0: 力项在消耗能量，当能量低时限制流出
    gamma_ = upsilon_func(energy_ratio, 0.0, delta_ratio);
  } else {
    // p_f < 0: 力项在补充能量，当能量高时限制流入
    gamma_ = upsilon_func(energy_ratio, 1.0 - delta_ratio, 1.0);
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