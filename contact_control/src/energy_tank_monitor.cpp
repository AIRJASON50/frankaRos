#include <contact_control/energy_tank_monitor.h>
#include <cmath>

namespace contact_control {

EnergyTankMonitor::EnergyTankMonitor() 
  : energy_tank_level_(8.0),  // 初始能量
    current_safety_level_(NORMAL) {
  
  // 基于Franka Control Interface Specifications的阈值设置
  energy_tank_max_ = 12.0;    // 最大存储能量 (J)
  energy_tank_min_ = 0.5;     // 最小能量 (J) - 避免除零
  energy_warning_ = 9.0;      // 警告阈值
  energy_critical_ = 10.5;    // 临界阈值
  energy_emergency_ = 11.5;   // 紧急阈值
  
  // 基于Franka Panda规格的速度限制
  vel_linear_warning_ = 1.0;    // 1.0 m/s (约60%最大速度)
  vel_linear_critical_ = 1.4;   // 1.4 m/s (约80%最大速度)
  vel_angular_warning_ = 1.5;   // 1.5 rad/s (约60%最大速度)
  vel_angular_critical_ = 2.0;  // 2.0 rad/s (约80%最大速度)
  
  // 基于关节扭矩限制的功率阈值
  power_warning_ = 50.0;    // W
  power_critical_ = 80.0;   // W
  
  // 有效质量参数 (估算值，包括机器人惯性)
  m_eff_ = 3.0;    // kg (包括机器人质量)
  I_eff_ = 0.1;    // kg⋅m² (包括机器人转动惯量)
  
  // 阻尼系数 (用于计算耗散能量)
  damping_linear_ = 50.0;   // N⋅s/m
  damping_angular_ = 5.0;   // N⋅m⋅s/rad
}

bool EnergyTankMonitor::init(ros::NodeHandle& node_handle) {
  // 读取配置参数
  node_handle.param("energy_tank/capacity_max", energy_tank_max_, 12.0);
  node_handle.param("energy_tank/capacity_min", energy_tank_min_, 0.5);
  node_handle.param("energy_tank/threshold_warning", energy_warning_, 9.0);
  node_handle.param("energy_tank/threshold_critical", energy_critical_, 10.5);
  node_handle.param("energy_tank/threshold_emergency", energy_emergency_, 11.5);
  
  node_handle.param("energy_tank/vel_linear_warning", vel_linear_warning_, 1.0);
  node_handle.param("energy_tank/vel_linear_critical", vel_linear_critical_, 1.4);
  node_handle.param("energy_tank/vel_angular_warning", vel_angular_warning_, 1.5);
  node_handle.param("energy_tank/vel_angular_critical", vel_angular_critical_, 2.0);
  
  node_handle.param("energy_tank/power_warning", power_warning_, 50.0);
  node_handle.param("energy_tank/power_critical", power_critical_, 80.0);
  
  node_handle.param("energy_tank/effective_mass", m_eff_, 3.0);
  node_handle.param("energy_tank/effective_inertia", I_eff_, 0.1);
  node_handle.param("energy_tank/damping_linear", damping_linear_, 50.0);
  node_handle.param("energy_tank/damping_angular", damping_angular_, 5.0);
  
  // 初始化发布器
  energy_level_pub_ = node_handle.advertise<std_msgs::Float64>("energy_tank/level", 1);
  safety_status_pub_ = node_handle.advertise<std_msgs::String>("energy_tank/safety_status", 1);
  
  ROS_INFO("Energy Tank Monitor initialized with max energy: %.1f J", energy_tank_max_);
  ROS_INFO("Energy tank thresholds - Warning: %.1f, Critical: %.1f, Emergency: %.1f J", 
           energy_warning_, energy_critical_, energy_emergency_);
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
  
  // 计算关节功率 P = τ · ω
  double joint_power = calculateJointPower(tau_d, dq);
  
  // 根据能量罐理论更新能量罐
  double dt = period.toSec();
  if (dt > 1e-6) {
    // 1. 计算系统耗散的能量 (阻尼产生的耗散)
    double dissipated_power = calculateDissipatedPower(linear_velocity, angular_velocity);
    
    // 2. 检测非被动行为 (负功率输出)
    double non_passive_power = 0.0;
    if (joint_power < 0) {
      // 系统输出能量，这是非被动行为
      non_passive_power = -joint_power;
    }
    
    // 3. 能量罐更新逻辑
    updateEnergyTank(dissipated_power, non_passive_power, dt);
  }
  
  // 评估安全级别
  double linear_vel_norm = linear_velocity.norm();
  double angular_vel_norm = angular_velocity.norm();
  current_safety_level_ = evaluateSafetyLevel(
    energy_tank_level_, linear_vel_norm, angular_vel_norm, std::abs(joint_power));
  
  // 发布状态
  publishStatus(ros::Time::now());
  
  // 返回基于能量罐水平的缩放因子
  return computeScaleFactor(current_safety_level_);
}

double EnergyTankMonitor::calculateDissipatedPower(
    const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity) {
  
  // 计算阻尼耗散的功率 P_dissipated = D * v^2
  double linear_dissipation = damping_linear_ * linear_velocity.squaredNorm();
  double angular_dissipation = damping_angular_ * angular_velocity.squaredNorm();
  
  return linear_dissipation + angular_dissipation;
}

void EnergyTankMonitor::updateEnergyTank(double dissipated_power, 
                                        double non_passive_power, 
                                        double dt) {
  // 能量罐理论：
  // dE_tank/dt = P_dissipated - P_non_passive
  // 其中：
  // - P_dissipated > 0: 系统耗散的能量进入能量罐
  // - P_non_passive > 0: 非被动操作消耗能量罐中的能量
  
  double energy_change = dissipated_power * dt - non_passive_power * dt;
  
  // 更新能量罐水平
  energy_tank_level_ += energy_change;
  
  // 限制在允许范围内
  energy_tank_level_ = std::max(energy_tank_min_, 
                               std::min(energy_tank_level_, energy_tank_max_));
  
  // 调试信息
  static int debug_counter = 0;
  if (++debug_counter % 100 == 0) { // 每100个周期打印一次
    ROS_DEBUG("Energy Tank: level=%.2f, dissipated=%.3f, non_passive=%.3f", 
              energy_tank_level_, dissipated_power, non_passive_power);
  }
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
  
  // 基于能量罐水平和系统状态评估安全级别
  
  // 紧急情况：能量罐接近满或系统超过临界限制
  if (energy_level >= energy_emergency_ || 
      linear_vel >= vel_linear_critical_ ||
      angular_vel >= vel_angular_critical_ ||
      power >= power_critical_) {
    return EMERGENCY;
  }
  
  // 临界情况：能量罐较高或系统接近警告限制
  if (energy_level >= energy_critical_ ||
      linear_vel >= vel_linear_warning_ ||
      angular_vel >= vel_angular_warning_ ||
      power >= power_warning_) {
    return CRITICAL;
  }
  
  // 警告情况：能量罐较高或能量不足
  if (energy_level >= energy_warning_ || energy_level <= energy_tank_min_ + 1.0) {
    return WARNING;
  }
  
  return NORMAL;
}

double EnergyTankMonitor::computeScaleFactor(EnergyTankSafetyLevel level) {
  // 基于安全级别返回速度缩放因子
  // 这确保了在能量罐状态不佳时降低系统活跃度
  
  switch (level) {
    case NORMAL:    
      return 1.0;   // 正常运行，无限制
    case WARNING:   
      return 0.8;   // 轻微限制，80%速度
    case CRITICAL:  
      return 0.5;   // 显著限制，50%速度
    case EMERGENCY: 
      return 0.2;   // 严重限制，20%速度
    default:        
      return 1.0;
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
  ROS_INFO("Energy tank reset to initial level: %.1f J", energy_tank_level_);
}

} // namespace contact_control 