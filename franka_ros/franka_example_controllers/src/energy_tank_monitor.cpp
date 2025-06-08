#include <franka_example_controllers/energy_tank_monitor.h>
#include <cmath>

namespace franka_example_controllers {

EnergyTankMonitor::EnergyTankMonitor() 
  : energy_tank_level_(8.0),  // 初始能量
    current_safety_level_(NORMAL) {
  
  // FR3保守参数设置 (基于规格的60%安全系数)
  energy_tank_max_ = 15.0;
  energy_tank_min_ = 2.0;
  energy_warning_ = 12.0;
  energy_critical_ = 13.5;
  energy_emergency_ = 14.5;
  
  // 速度限制 (FR3: 线速度3.0 m/s, 角速度2.5 rad/s)
  vel_linear_warning_ = 1.2;   // 40%
  vel_linear_critical_ = 1.5;  // 50%
  vel_angular_warning_ = 1.0;  // 40%
  vel_angular_critical_ = 1.25; // 50%
  
  power_warning_ = 40.0;
  power_critical_ = 50.0;
  
  // 有效质量参数
  m_eff_ = 2.0;  // kg
  I_eff_ = 0.05; // kg⋅m²
}

bool EnergyTankMonitor::init(ros::NodeHandle& node_handle) {
  // 读取配置参数
  node_handle.param("energy_tank/capacity_max", energy_tank_max_, 15.0);
  node_handle.param("energy_tank/capacity_min", energy_tank_min_, 2.0);
  node_handle.param("energy_tank/threshold_warning", energy_warning_, 12.0);
  node_handle.param("energy_tank/threshold_critical", energy_critical_, 13.5);
  node_handle.param("energy_tank/threshold_emergency", energy_emergency_, 14.5);
  
  node_handle.param("energy_tank/vel_linear_warning", vel_linear_warning_, 1.2);
  node_handle.param("energy_tank/vel_linear_critical", vel_linear_critical_, 1.5);
  node_handle.param("energy_tank/vel_angular_warning", vel_angular_warning_, 1.0);
  node_handle.param("energy_tank/vel_angular_critical", vel_angular_critical_, 1.25);
  
  node_handle.param("energy_tank/power_warning", power_warning_, 40.0);
  node_handle.param("energy_tank/power_critical", power_critical_, 50.0);
  
  node_handle.param("energy_tank/effective_mass", m_eff_, 2.0);
  node_handle.param("energy_tank/effective_inertia", I_eff_, 0.05);
  
  // 初始化发布器
  energy_level_pub_ = node_handle.advertise<std_msgs::Float64>("energy_tank/level", 1);
  safety_status_pub_ = node_handle.advertise<std_msgs::String>("energy_tank/safety_status", 1);
  
  ROS_INFO("Energy Tank Monitor initialized with max energy: %.1f J", energy_tank_max_);
  return true;
}

double EnergyTankMonitor::updateAndGetScaleFactor(
    const Eigen::Matrix<double, 7, 1>& tau_d,
    const franka::RobotState& robot_state,
    const Eigen::Matrix<double, 6, 7>& jacobian,
    const ros::Duration& period) {
  
  // 计算末端速度
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Matrix<double, 6, 1> end_effector_velocity = jacobian * dq;
  Eigen::Vector3d linear_velocity = end_effector_velocity.head(3);
  Eigen::Vector3d angular_velocity = end_effector_velocity.tail(3);
  
  // 计算末端动能
  double kinetic_energy = calculateEndEffectorKineticEnergy(linear_velocity, angular_velocity);
  
  // 计算关节功率
  double joint_power = calculateJointPower(tau_d, dq);
  
  // 更新能量罐 (简化的能量平衡)
  double dt = period.toSec();
  if (dt > 1e-6) {
    // 耗散的能量进入能量罐
    double dissipated_power = std::max(0.0, -joint_power * 0.1); // 10%耗散率
    
    // 能量罐更新
    if (energy_tank_level_ < energy_tank_max_ && dissipated_power > 0) {
      energy_tank_level_ += dissipated_power * dt;
    }
    
    // 高能量操作消耗能量罐
    if (kinetic_energy > 0.5 && energy_tank_level_ > energy_tank_min_) {
      energy_tank_level_ -= kinetic_energy * dt * 0.5;
    }
    
    // 限制在范围内
    energy_tank_level_ = std::max(energy_tank_min_, std::min(energy_tank_level_, energy_tank_max_));
  }
  
  // 评估安全级别
  double linear_vel_norm = linear_velocity.norm();
  double angular_vel_norm = angular_velocity.norm();
  current_safety_level_ = evaluateSafetyLevel(
    energy_tank_level_, linear_vel_norm, angular_vel_norm, std::abs(joint_power));
  
  // 发布状态
  publishStatus(ros::Time::now());
  
  // 返回缩放因子
  return computeScaleFactor(current_safety_level_);
}

double EnergyTankMonitor::calculateEndEffectorKineticEnergy(
    const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity) {
  
  double linear_ke = 0.5 * m_eff_ * linear_velocity.squaredNorm();
  double angular_ke = 0.5 * I_eff_ * angular_velocity.squaredNorm();
  return linear_ke + angular_ke;
}

double EnergyTankMonitor::calculateJointPower(
    const Eigen::Matrix<double, 7, 1>& tau_d,
    const Eigen::Matrix<double, 7, 1>& dq) {
  
  return tau_d.dot(dq); // P = τ · ω
}

EnergyTankSafetyLevel EnergyTankMonitor::evaluateSafetyLevel(
    double energy_level, double linear_vel, double angular_vel, double power) {
  
  // 紧急情况
  if (energy_level >= energy_emergency_ || 
      linear_vel >= vel_linear_critical_ ||
      angular_vel >= vel_angular_critical_ ||
      power >= power_critical_) {
    return EMERGENCY;
  }
  
  // 临界情况
  if (energy_level >= energy_critical_ ||
      linear_vel >= vel_linear_warning_ ||
      angular_vel >= vel_angular_warning_ ||
      power >= power_warning_) {
    return CRITICAL;
  }
  
  // 警告情况
  if (energy_level >= energy_warning_ || energy_level <= energy_tank_min_ + 1.0) {
    return WARNING;
  }
  
  return NORMAL;
}

double EnergyTankMonitor::computeScaleFactor(EnergyTankSafetyLevel level) {
  switch (level) {
    case NORMAL:    return 1.0;   // 正常运行
    case WARNING:   return 0.8;   // 80%速度
    case CRITICAL:  return 0.5;   // 50%速度
    case EMERGENCY: return 0.1;   // 10%速度 (接近停止)
    default:        return 1.0;
  }
}

void EnergyTankMonitor::publishStatus(const ros::Time& time) {
  // 发布能量级别
  std_msgs::Float64 energy_msg;
  energy_msg.data = energy_tank_level_;
  energy_level_pub_.publish(energy_msg);
  
  // 发布安全状态
  std_msgs::String status_msg;
  switch (current_safety_level_) {
    case NORMAL:    status_msg.data = "NORMAL"; break;
    case WARNING:   status_msg.data = "WARNING"; break;
    case CRITICAL:  status_msg.data = "CRITICAL"; break;
    case EMERGENCY: status_msg.data = "EMERGENCY"; break;
  }
  safety_status_pub_.publish(status_msg);
}

void EnergyTankMonitor::reset() {
  energy_tank_level_ = 8.0;  // 重置到初始能量
  current_safety_level_ = NORMAL;
}

} // namespace franka_example_controllers 