#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <franka/robot_state.h>

namespace contact_control {

enum EnergyTankSafetyLevel {
  NORMAL = 0,
  WARNING = 1, 
  CRITICAL = 2,
  EMERGENCY = 3
};

class EnergyTankMonitor {
public:
  EnergyTankMonitor();
  
  bool init(ros::NodeHandle& node_handle);
  
  /**
   * @brief 监控能量并返回安全缩放因子
   * @param tau_d 计算的期望关节力矩
   * @param robot_state 机器人状态
   * @param jacobian 雅可比矩阵
   * @param period 控制周期
   * @return 安全缩放因子 [0.0, 1.0]
   */
  double updateAndGetScaleFactor(
    const Eigen::Matrix<double, 7, 1>& tau_d,
    const franka::RobotState& robot_state,
    const Eigen::Matrix<double, 6, 7>& jacobian,
    const ros::Duration& period
  );
  
  void reset();
  EnergyTankSafetyLevel getCurrentSafetyLevel() const { return current_safety_level_; }
  double getEnergyLevel() const { return energy_tank_level_; }

private:
  // 能量罐参数 (基于Franka规格的保守设置)
  double energy_tank_max_;      // 12.0 J
  double energy_tank_min_;      // 0.5 J  
  double energy_warning_;       // 9.0 J
  double energy_critical_;      // 10.5 J
  double energy_emergency_;     // 11.5 J
  
  // 速度限制 (基于Franka Panda规格)
  double vel_linear_warning_;   // 1.0 m/s 
  double vel_linear_critical_;  // 1.4 m/s
  double vel_angular_warning_;  // 1.5 rad/s  
  double vel_angular_critical_; // 2.0 rad/s
  
  // 功率限制
  double power_warning_;        // 50.0 W
  double power_critical_;       // 80.0 W
  
  // 有效质量参数
  double m_eff_;               // 3.0 kg (包括机器人质量)
  double I_eff_;               // 0.1 kg⋅m²
  
  // 阻尼参数 (用于计算耗散能量)
  double damping_linear_;      // 50.0 N⋅s/m
  double damping_angular_;     // 5.0 N⋅m⋅s/rad
  
  // 状态变量
  double energy_tank_level_;
  EnergyTankSafetyLevel current_safety_level_;
  
  // ROS发布器
  ros::Publisher energy_level_pub_;
  ros::Publisher safety_status_pub_;
  
  // 内部方法
  double calculateEndEffectorKineticEnergy(
    const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity
  );
  
  double calculateJointPower(
    const Eigen::Matrix<double, 7, 1>& tau_d,
    const Eigen::Matrix<double, 7, 1>& dq
  );
  
  /**
   * @brief 计算系统耗散功率 (基于阻尼)
   */
  double calculateDissipatedPower(
    const Eigen::Vector3d& linear_velocity,
    const Eigen::Vector3d& angular_velocity
  );
  
  /**
   * @brief 根据能量罐理论更新能量罐水平
   */
  void updateEnergyTank(double dissipated_power, 
                       double non_passive_power, 
                       double dt);
  
  EnergyTankSafetyLevel evaluateSafetyLevel(
    double energy_level,
    double linear_vel,
    double angular_vel, 
    double power
  );
  
  double computeScaleFactor(EnergyTankSafetyLevel level);
  
  void publishStatus(const ros::Time& time);
};

} // namespace contact_control 